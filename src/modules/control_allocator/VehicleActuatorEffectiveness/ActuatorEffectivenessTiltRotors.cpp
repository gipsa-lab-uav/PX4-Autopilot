/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file ActuatorEffectivenessTiltRotors.cpp
 *
 * Actuator effectiveness computed from rotors position and orientation
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ActuatorEffectivenessTiltRotors.hpp"

using namespace matrix;

ActuatorEffectivenessTiltRotors::ActuatorEffectivenessTiltRotors(ModuleParams *parent, ThrustAxisConfiguration axis_config)
	: ModuleParams(parent), _axis_config(axis_config)
{
	for (int i = 0; i < NUM_ROTORS_MAX; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PX", i);
		_param_handles[i].position_x = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PY", i);
		_param_handles[i].position_y = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PZ", i);
		_param_handles[i].position_z = param_find(buffer);

		if (_axis_config == ThrustAxisConfiguration::Configurable) {
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AX", i);
			_param_handles[i].axis_x = param_find(buffer);
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AY", i);
			_param_handles[i].axis_y = param_find(buffer);
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AZ", i);
			_param_handles[i].axis_z = param_find(buffer);
		}

		snprintf(buffer, sizeof(buffer), "CA_SV_TL%u_AX", i);
		_param_handles[i].tilt_axis_x = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SV_TL%u_AY", i);
		_param_handles[i].tilt_axis_y = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SV_TL%u_AZ", i);
		_param_handles[i].tilt_axis_z = param_find(buffer);

		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_CT", i);
		_param_handles[i].thrust_coef = param_find(buffer);

		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_KM", i);
		_param_handles[i].moment_ratio = param_find(buffer);

		snprintf(buffer, sizeof(buffer), "CA_SV_TL%u_MINA", i);
		_param_handles[i].tilt_min_angle = param_find(buffer);

		snprintf(buffer, sizeof(buffer), "CA_SV_TL%u_MAXA", i);
		_param_handles[i].tilt_max_angle = param_find(buffer);

	}

	updateParams();
}

void ActuatorEffectivenessTiltRotors::updateParams()
{
	ModuleParams::updateParams();

	_geometry.num_rotors = math::min(NUM_ROTORS_MAX, static_cast<int>(_param_ca_rotor_count.get()));

	for (int i = 0; i < _geometry.num_rotors; ++i) {
		Vector3f &position = _geometry.rotors[i].position;
		param_get(_param_handles[i].position_x, &position(0));
		param_get(_param_handles[i].position_y, &position(1));
		param_get(_param_handles[i].position_z, &position(2));

		Vector3f &axis = _geometry.rotors[i].thrust_axis;

		switch (_axis_config) {
		case ThrustAxisConfiguration::Configurable:
			param_get(_param_handles[i].axis_x, &axis(0));
			param_get(_param_handles[i].axis_y, &axis(1));
			param_get(_param_handles[i].axis_z, &axis(2));
			break;

		case ThrustAxisConfiguration::FixedForward:
			axis = Vector3f(1.f, 0.f, 0.f);
			break;

		case ThrustAxisConfiguration::FixedUpwards:
			axis = Vector3f(0.f, 0.f, -1.f);
			break;
		}

		Vector3f &tilt_axis = _geometry.rotors[i].tilt_axis;
		param_get(_param_handles[i].tilt_axis_x, &tilt_axis(0));
		param_get(_param_handles[i].tilt_axis_y, &tilt_axis(1));
		param_get(_param_handles[i].tilt_axis_z, &tilt_axis(2));

		param_get(_param_handles[i].thrust_coef, &_geometry.rotors[i].thrust_coef);
		param_get(_param_handles[i].moment_ratio, &_geometry.rotors[i].moment_ratio);

		param_get(_param_handles[i].tilt_min_angle, &_geometry.rotors[i].tilt_min_angle);
		param_get(_param_handles[i].tilt_max_angle, &_geometry.rotors[i].tilt_max_angle);
		_geometry.rotors[i].tilt_min_angle = math::radians(_geometry.rotors[i].tilt_min_angle);
		_geometry.rotors[i].tilt_max_angle = math::radians(_geometry.rotors[i].tilt_max_angle);

		PX4_INFO("Add TiltRotor %d.", i);
	}
}

bool
ActuatorEffectivenessTiltRotors::addActuators(Configuration &configuration)
{
	if (configuration.num_actuators[(int)ActuatorType::SERVOS] > 0) {
		PX4_ERR("Wrong actuator ordering: servos need to be after motors");
		return false;
	}

	int num_actuators = computeEffectivenessMatrix(_geometry,
			    configuration.effectiveness_matrices[configuration.selected_matrix],
			    configuration.num_actuators_matrix[configuration.selected_matrix]);

	_actuator_start_index = configuration.num_actuators_matrix[configuration.selected_matrix];

	configuration.actuatorsAdded(ActuatorType::MOTORS, num_actuators/2);
	configuration.actuatorsAdded(ActuatorType::SERVOS, num_actuators/2);
	return true;
}

int
ActuatorEffectivenessTiltRotors::computeEffectivenessMatrix(const Geometry &geometry,
		EffectivenessMatrix &effectiveness, int actuator_start_index)
{
	int num_actuators = 0;

	for (int i = 0; i < geometry.num_rotors; i++) {

		if (2*i + actuator_start_index >= NUM_ACTUATORS) {
			break;
		}

		num_actuators+=2;

		// Get rotor and tilt axis
		Vector3f thrust_axis = geometry.rotors[i].thrust_axis;
		Vector3f tilt_axis = geometry.rotors[i].tilt_axis;

		// Normalize thrust axis
		float thrust_axis_norm = thrust_axis.norm();

		if (thrust_axis_norm > FLT_EPSILON) {
			thrust_axis /= thrust_axis_norm;

		} else {
			// Bad axis definition, ignore this rotor
			continue;
		}

		// Normalize tilt axis
		float tilt_axis_norm = tilt_axis.norm();

		if (tilt_axis_norm > FLT_EPSILON) {
			tilt_axis /= tilt_axis_norm;

		} else {
			// Bad axis definition, ignore this rotor
			continue;
		}

		// Test orthogonality

		if(fabsf(tilt_axis.dot(thrust_axis)) > FLT_EPSILON)
		{
			// Axis are not orthogonal, ignore this rotor
			continue;
		}

		// Get rotor position
		const Vector3f &position = geometry.rotors[i].position;

		// Get coefficients
		float ct = geometry.rotors[i].thrust_coef;
		float km = geometry.rotors[i].moment_ratio;

		if (fabsf(ct) < FLT_EPSILON) {
			continue;
		}

		matrix::Vector3f thrust_cost = ct * thrust_axis;
		matrix::Vector3f thrust_sint = ct * tilt_axis.cross(thrust_axis);;

		matrix::Vector3f moment_cost = position.cross(thrust_cost) - km * thrust_cost;
		matrix::Vector3f moment_sint = position.cross(thrust_sint) - km * thrust_sint;

		// Fill corresponding items in effectiveness matrix
		for (size_t j = 0; j < 3; j++) {
			effectiveness(j, i + actuator_start_index) = moment_cost(j);
			effectiveness(j, i + geometry.num_rotors + actuator_start_index) = moment_sint(j);
			effectiveness(j + 3, i + actuator_start_index) = thrust_cost(j);
			effectiveness(j + 3, i + geometry.num_rotors + actuator_start_index) = thrust_sint(j);
		}
		/*
		// Compute thrust generated by this rotor
		matrix::Vector3f thrust = ct * axis;

		// Compute moment generated by this rotor
		matrix::Vector3f moment = ct * position.cross(axis) - ct * km * axis;

		// Fill corresponding items in effectiveness matrix
		for (size_t j = 0; j < 3; j++) {
			effectiveness(j, i + actuator_start_index) = moment(j);
			effectiveness(j + 3, i + actuator_start_index) = thrust(j);
		}
		*/

	}

	return num_actuators;
}

uint32_t ActuatorEffectivenessTiltRotors::getMotors() const
{
	uint32_t motors = 0;

	for (int i = 0; i < _geometry.num_rotors; ++i) {
		motors |= 1u << i;
	}

	return motors;
}

bool
ActuatorEffectivenessTiltRotors::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	return addActuators(configuration);
}

void ActuatorEffectivenessTiltRotors::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
	int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
	const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
	// static int k=0;
	// bool display = !(k++ % 500);

	// if(display)
	// {
	// PX4_INFO("control_sp : %f %f %f %f %f %f",
	//  	(double)control_sp(0),
	//  	(double)control_sp(1),
	//  	(double)control_sp(2),
	//  	(double)control_sp(3),
	//  	(double)control_sp(4),
	//  	(double)control_sp(5));
	// }

	int num_rotors = _geometry.num_rotors;
	for(int i=0; i<num_rotors; i++)
	{
		int id_mot = _actuator_start_index + i;
		int id_sv = _actuator_start_index + num_rotors + i;
		float tct = actuator_sp(id_mot);
		float tst = actuator_sp(id_sv);


		float rotor_sp = hypotf(tst, tct);
		float tilt = atan2f(tst, tct + 0.05f);

		 // float sv_sp = actuator_min(id_sv) + (actuator_max(id_sv) - actuator_min(id_sv))
		//                 * (tilt - _geometry.rotors[i].tilt_min_angle)/(_geometry.rotors[i].tilt_max_angle-_geometry.rotors[i].tilt_min_angle);

		//float sv_sp = math::lerp((float)actuator_min(id_sv), (float)actuator_max(id_sv), (float)(tilt-_geometry.rotors[i].tilt_min_angle)/(_geometry.rotors[i].tilt_max_angle-_geometry.rotors[i].tilt_min_angle));
		float sv_sp = math::lerp(-1.f, 1.f, (float)(tilt-_geometry.rotors[i].tilt_min_angle)/(_geometry.rotors[i].tilt_max_angle-_geometry.rotors[i].tilt_min_angle));

		actuator_sp(id_mot) = rotor_sp;
		actuator_sp(id_sv) = sv_sp;

		// if(display)
		// {
		// 	PX4_INFO("tct/tst [%d] : %f %f", i, (double)tct, (double)tst);
		// 	PX4_INFO("tilt svp_sp [%d] : %f %f", i, (double)tilt, (double)sv_sp);

		// }

	}

	// {
	// PX4_INFO("control_sp : %f %f %f %f %f %f",
	//  	(double)control_sp(0),
	//  	(double)control_sp(1),
	//  	(double)control_sp(2),
	//  	(double)control_sp(3),
	//  	(double)control_sp(4),
	//  	(double)control_sp(5));

	// PX4_INFO("Min max sv : %f %f", (double)_geometry.rotors[3].tilt_min_angle, (double)_geometry.rotors[3].tilt_max_angle);
	//  PX4_INFO("Servos %d %f %f %f %f",
	// 	num_rotors,
	// 	(double) actuator_sp(_actuator_start_index + num_rotors),
	// 	(double) actuator_sp(_actuator_start_index + num_rotors+1),
	// 	(double) actuator_sp(_actuator_start_index + num_rotors+2),
	// 	(double) actuator_sp(_actuator_start_index + num_rotors+3));
	// }

}
