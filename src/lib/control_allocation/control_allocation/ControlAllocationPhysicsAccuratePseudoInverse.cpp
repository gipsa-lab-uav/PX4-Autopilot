/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ControlAllocationPhysicsAccuratePseudoInverse.cpp
 *
 * Simple Control Allocation Algorithm
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ControlAllocationPhysicsAccuratePseudoInverse.hpp"

#include <drivers/drv_hrt.h>
#include <cstring>

void
ControlAllocationPhysicsAccuratePseudoInverse::setRpmMax(const ActuatorVector &rpm_max)
{
	for (int i = 0; i < NUM_ACTUATORS; ++i) {
		// The physical effectiveness matrix uses CT in N/(krpm)^2, so the
		// allocator output is interpreted in krpm^2 as well.
		_rpm_max(i) = fmaxf(rpm_max(i) * 1e-3f, 1e-3f);
	}
}

void
ControlAllocationPhysicsAccuratePseudoInverse::setEffectivenessMatrix(
	const matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> &effectiveness,
	const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators,
	bool update_normalization_scale)
{
	ControlAllocation::setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, num_actuators,
			update_normalization_scale);
	_mix_update_needed = true;
	_normalization_needs_update = update_normalization_scale;

	if (_metric_allocation && update_normalization_scale) {
		// adding #include <px4_platform_common/log.h> + PX4_WARN leads to failed linking on test
		_normalization_needs_update = false;
	}
}

// void
// ControlAllocationPhysicsAccuratePseudoInverse::updatePseudoInverse()
// {
// 	if (_mix_update_needed) {
// 		matrix::geninv(_effectiveness, _mix);

// 		if (!_metric_allocation) {
// 			if (_normalization_needs_update && !_had_actuator_failure) {
// 				updateControlAllocationMatrixScale();
// 				_normalization_needs_update = false;
// 			}

// 			normalizeControlAllocationMatrix();
// 		}

// 		_mix_update_needed = false;
// 	}
// }

void
ControlAllocationPhysicsAccuratePseudoInverse::updatePseudoInverse()
{
	if (_mix_update_needed) {
		matrix::geninv(_effectiveness, _mix);

		// No normalization: keep raw physical effectiveness / pseudo-inverse scaling.
		_control_allocation_scale.setAll(1.f);

		_mix_update_needed = false;
	}
}


void
ControlAllocationPhysicsAccuratePseudoInverse::updateControlAllocationMatrixScale()
{
	// Same scale on roll and pitch
	if (_normalize_rpy) {

		int num_non_zero_roll_torque = 0;
		int num_non_zero_pitch_torque = 0;

		for (int i = 0; i < _num_actuators; i++) {

			if (fabsf(_mix(i, 0)) > 1e-3f) {
				++num_non_zero_roll_torque;
			}

			if (fabsf(_mix(i, 1)) > 1e-3f) {
				++num_non_zero_pitch_torque;
			}
		}

		float roll_norm_scale = 1.f;

		if (num_non_zero_roll_torque > 0) {
			roll_norm_scale = sqrtf(_mix.col(0).norm_squared() / (num_non_zero_roll_torque / 2.f));
		}

		float pitch_norm_scale = 1.f;

		if (num_non_zero_pitch_torque > 0) {
			pitch_norm_scale = sqrtf(_mix.col(1).norm_squared() / (num_non_zero_pitch_torque / 2.f));
		}

		_control_allocation_scale(0) = fmaxf(roll_norm_scale, pitch_norm_scale);
		_control_allocation_scale(1) = _control_allocation_scale(0);

		// Scale yaw separately
		_control_allocation_scale(2) = _mix.col(2).max();

	} else {
		_control_allocation_scale(0) = 1.f;
		_control_allocation_scale(1) = 1.f;
		_control_allocation_scale(2) = 1.f;
	}

	// Scale thrust by the sum of the individual thrust axes, and use the scaling for the Z axis if there's no actuators
	// (for tilted actuators)
	_control_allocation_scale(THRUST_Z) = 1.f;

	for (int axis_idx = 2; axis_idx >= 0; --axis_idx) {
		int num_non_zero_thrust = 0;
		float norm_sum = 0.f;

		for (int i = 0; i < _num_actuators; i++) {
			float norm = fabsf(_mix(i, 3 + axis_idx));
			norm_sum += norm;

			if (norm > FLT_EPSILON) {
				++num_non_zero_thrust;
			}
		}

		if (num_non_zero_thrust > 0) {
			_control_allocation_scale(3 + axis_idx) = norm_sum / num_non_zero_thrust;

		} else {
			_control_allocation_scale(3 + axis_idx) = _control_allocation_scale(THRUST_Z);
		}
	}
}

void
ControlAllocationPhysicsAccuratePseudoInverse::normalizeControlAllocationMatrix()
{
	if (_control_allocation_scale(0) > FLT_EPSILON) {
		_mix.col(0) /= _control_allocation_scale(0);
		_mix.col(1) /= _control_allocation_scale(1);
	}

	if (_control_allocation_scale(2) > FLT_EPSILON) {
		_mix.col(2) /= _control_allocation_scale(2);
	}

	if (_control_allocation_scale(3) > FLT_EPSILON) {
		_mix.col(3) /= _control_allocation_scale(3);
		_mix.col(4) /= _control_allocation_scale(4);
		_mix.col(5) /= _control_allocation_scale(5);
	}

	// Set all the small elements to 0 to avoid issues
	// in the control allocation algorithms
	for (int i = 0; i < _num_actuators; i++) {
		for (int j = 0; j < NUM_AXES; j++) {
			if (fabsf(_mix(i, j)) < 1e-3f) {
				_mix(i, j) = 0.f;
			}
		}
	}
}


void
ControlAllocationPhysicsAccuratePseudoInverse::normaliseActuatorSp()
{
	for (int i = 0; i < _num_actuators; ++i) {
		if (_actuator_max(i) > _actuator_min(i)) {
			// The allocator solves in physical krpm^2. Convert back to a normalized
			// speed command so THR_MDL_FAC can remain disabled for this mode.
			const float omega_sq = fmaxf(_actuator_sp(i), 0.f);
			const float omega = sqrtf(omega_sq);
			const float actuator_sp_normalized = omega / _rpm_max(i);
			_actuator_sp(i) = fminf(fmaxf(actuator_sp_normalized, _actuator_min(i)), _actuator_max(i));
		}
	}
}

matrix::Vector<float, ControlAllocation::NUM_AXES>
ControlAllocationPhysicsAccuratePseudoInverse::normalisedControlToPhysical(
	const matrix::Vector<float, NUM_AXES> &control) const
{
	matrix::Vector<float, NUM_AXES> physical_control{};

	for (int axis = 0; axis < NUM_AXES; ++axis) {
		float axis_max = 0.f;
		float axis_min = 0.f;

		for (int actuator = 0; actuator < _num_actuators; ++actuator) {
			const float coeff = _effectiveness(axis, actuator);
			const float rpm_max_sq = _rpm_max(actuator) * _rpm_max(actuator);
			const float actuator_min_physical = _actuator_min(actuator) * rpm_max_sq;
			const float actuator_max_physical = _actuator_max(actuator) * rpm_max_sq;

			if (coeff >= 0.f) {
				axis_max += coeff * actuator_max_physical;
				axis_min += coeff * actuator_min_physical;

			} else {
				axis_max += coeff * actuator_min_physical;
				axis_min += coeff * actuator_max_physical;
			}
		}

		if (control(axis) >= 0.f) {
			physical_control(axis) = (axis_max > FLT_EPSILON) ? control(axis) * axis_max : 0.f;

		} else {
			physical_control(axis) = (axis_min < -FLT_EPSILON) ? control(axis) * -axis_min : 0.f;
		}
	}

	return physical_control;
}

ControlAllocation::ActuatorVector
ControlAllocationPhysicsAccuratePseudoInverse::actuatorSetpointToPhysical(const ActuatorVector &actuator) const
{
	ActuatorVector actuator_physical{};

	for (int i = 0; i < _num_actuators; ++i) {
		const float rpm_max_sq = _rpm_max(i) * _rpm_max(i);
		actuator_physical(i) = actuator(i) * rpm_max_sq;
	}

	return actuator_physical;
}

void
ControlAllocationPhysicsAccuratePseudoInverse::publishPhysicalControlSetpoints(
	const matrix::Vector<float, NUM_AXES> &control_sp_physical)
{
	vehicle_torque_newton_setpoint_s torque_setpoint{};
	torque_setpoint.timestamp = hrt_absolute_time();
	torque_setpoint.timestamp_sample = _timestamp_sample;
	torque_setpoint.xyz[0] = control_sp_physical(ROLL);
	torque_setpoint.xyz[1] = control_sp_physical(PITCH);
	torque_setpoint.xyz[2] = control_sp_physical(YAW);
	_vehicle_torque_newton_setpoint_pub.publish(torque_setpoint);

	vehicle_thrust_newton_setpoint_s thrust_setpoint{};
	thrust_setpoint.timestamp = torque_setpoint.timestamp;
	thrust_setpoint.timestamp_sample = _timestamp_sample;
	thrust_setpoint.xyz[0] = control_sp_physical(THRUST_X);
	thrust_setpoint.xyz[1] = control_sp_physical(THRUST_Y);
	thrust_setpoint.xyz[2] = control_sp_physical(THRUST_Z);
	_vehicle_thrust_newton_setpoint_pub.publish(thrust_setpoint);
}

void
ControlAllocationPhysicsAccuratePseudoInverse::publishDebugArray()
{
	debug_array_s debug_array{};
	debug_array.timestamp = hrt_absolute_time();
	debug_array.id = 42;
	std::strncpy(debug_array.name, "physalloc", sizeof(debug_array.name));

	// Layout:
	// [0..5]   control_sp
	// [6..29]  effectiveness for first 4 actuators, scaled by 1e9
	// [30..33] actuator_sp for first 4 actuators after normalization
	// [34..37] rpm_max for first 4 actuators
	// [38..41] thrust-axis z contribution, scaled by 1e9
	// [42..45] yaw contribution, scaled by 1e9
	// [46..49] pseudo-inverse thrust-z row contribution for first 4 actuators
	// [50..53] pseudo-inverse yaw row contribution for first 4 actuators
	// [54]     number of configured actuators
	// [55]     effectiveness scale factor (1e9)
	// [56]     actuator_sp(0)
	// [57]     actuator_sp(1)

	for (int i = 0; i < NUM_AXES; ++i) {
		debug_array.data[i] = _control_sp(i);
	}

	int debug_actuators = _num_actuators < 4 ? _num_actuators : 4;
	int idx = NUM_AXES;

	for (int actuator = 0; actuator < 4; ++actuator) {
		for (int axis = 0; axis < NUM_AXES; ++axis) {
			if (actuator < debug_actuators) {
				debug_array.data[idx++] = _effectiveness(axis, actuator) * 1e9f;

			} else {
				debug_array.data[idx++] = 0.f;
			}
		}
	}

	for (int actuator = 0; actuator < 4; ++actuator) {
		if (actuator < debug_actuators) {
			debug_array.data[idx++] = _actuator_sp(actuator);
			debug_array.data[idx++] = _rpm_max(actuator);
			debug_array.data[idx++] = _effectiveness(THRUST_Z, actuator) * 1e9f;
			debug_array.data[idx++] = _effectiveness(YAW, actuator) * 1e9f;
			debug_array.data[idx++] = _mix(actuator, THRUST_Z);
			debug_array.data[idx++] = _mix(actuator, YAW);

		} else {
			for (int j = 0; j < 6; ++j) {
				debug_array.data[idx++] = 0.f;
			}
		}
	}

	debug_array.data[54] = (float)_num_actuators;
	debug_array.data[55] = 1e9f;
	debug_array.data[56] = _actuator_sp(0);
	debug_array.data[57] = _actuator_sp(1);

	_debug_array_pub.publish(debug_array);
}


void
ControlAllocationPhysicsAccuratePseudoInverse::allocate()
{
	//Compute new gains if needed
	updatePseudoInverse();

	_prev_actuator_sp = _actuator_sp;

	const matrix::Vector<float, NUM_AXES> control_sp_physical = normalisedControlToPhysical(_control_sp);
	const matrix::Vector<float, NUM_AXES> control_trim_physical = _effectiveness * actuatorSetpointToPhysical(_actuator_trim);
	const ActuatorVector actuator_trim_physical = actuatorSetpointToPhysical(_actuator_trim);
	publishPhysicalControlSetpoints(control_sp_physical);

	// Allocate in physical actuator units (krpm^2), then convert back to a
	// normalized speed command in [0, 1].
	_actuator_sp = actuator_trim_physical + _mix * (control_sp_physical - control_trim_physical);

	// Convert krpm^2 to normalized speed using sqrt(omega^2) / omega_max.
	normaliseActuatorSp();
	// Disabled temporarily while debugging rotor geometry/effectiveness publication on debug_array.
}
