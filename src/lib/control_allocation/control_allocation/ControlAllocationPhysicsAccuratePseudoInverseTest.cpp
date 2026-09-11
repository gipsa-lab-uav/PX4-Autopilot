/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include <ControlAllocationPhysicsAccuratePseudoInverse.hpp>

using namespace matrix;

TEST(ControlAllocationPhysicsAccuratePseudoInverseTest, NewtonThrustMatchesRotorSpeedSetpoints)
{
	static constexpr int num_rotors = 4;
	static constexpr float thrust_coefficient = 0.015049f; // N/(krpm)^2
	static constexpr float moment_coefficient = 0.00023153f; // N m/(krpm)^2
	static constexpr float arm_x = 0.06f;
	static constexpr float arm_y = 0.09f;
	static constexpr float rpm_min = 2200.f;
	static constexpr float rpm_max = 22000.f;

	ControlAllocationPhysicsAccuratePseudoInverse allocation;
	ControlAllocation::ActuatorVector rpm_minimum;
	ControlAllocation::ActuatorVector rpm_maximum;
	rpm_minimum.setAll(rpm_min);
	rpm_maximum.setAll(rpm_max);
	allocation.setRpmMin(rpm_minimum);
	allocation.setRpmMax(rpm_maximum);

	Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> effectiveness;
	effectiveness.setZero();

	// Quad-X rotor positions are (+,+), (-,-), (+,-), (-,+). The first
	// two rotors turn one way and the last two turn the other way.
	const float position_x[num_rotors] {arm_x, -arm_x, arm_x, -arm_x};
	const float position_y[num_rotors] {arm_y, -arm_y, -arm_y, arm_y};
	const float reaction_torque[num_rotors] {
		moment_coefficient, moment_coefficient, -moment_coefficient, -moment_coefficient
	};

	for (int rotor = 0; rotor < num_rotors; ++rotor) {
		effectiveness(ControlAllocation::ROLL, rotor) = -thrust_coefficient * position_y[rotor];
		effectiveness(ControlAllocation::PITCH, rotor) = thrust_coefficient * position_x[rotor];
		effectiveness(ControlAllocation::YAW, rotor) = reaction_torque[rotor];
		effectiveness(ControlAllocation::THRUST_Z, rotor) = -thrust_coefficient;
	}

	ControlAllocation::ActuatorVector actuator_trim;
	ControlAllocation::ActuatorVector linearization_point;
	actuator_trim.setZero();
	linearization_point.setZero();
	allocation.setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, num_rotors, true);

	Vector<float, ControlAllocation::NUM_AXES> control_setpoint;
	control_setpoint.setZero();
	control_setpoint(ControlAllocation::ROLL) = 0.02f;
	control_setpoint(ControlAllocation::PITCH) = -0.015f;
	control_setpoint(ControlAllocation::YAW) = 0.01f;
	control_setpoint(ControlAllocation::THRUST_Z) = -0.4f;
	allocation.setControlSetpoint(control_setpoint);
	allocation.allocate();
	allocation.clipActuatorSetpoint(); // Standard ControlAllocator post-allocation clipping.

	const ControlAllocation::ActuatorVector &motor_command = allocation.getActuatorSetpoint();
	float reconstructed_thrust_z = 0.f;

	for (int rotor = 0; rotor < num_rotors; ++rotor) {
		ASSERT_GT(motor_command(rotor), 0.f);
		ASSERT_LT(motor_command(rotor), 1.f);

		// With THR_MDL_FAC=0, this normalized speed command is mapped
		// linearly by the output driver to the ESC speed interval.
		const float rpm_setpoint_krpm =
			(rpm_min + motor_command(rotor) * (rpm_max - rpm_min)) * 1e-3f;
		reconstructed_thrust_z -= thrust_coefficient * rpm_setpoint_krpm * rpm_setpoint_krpm;
	}

	const Vector<float, ControlAllocation::NUM_AXES> &physical_setpoint = allocation.getPhysicalControlSetpoint();
	const float expected_maximum_thrust = num_rotors * thrust_coefficient * (rpm_max * 1e-3f) * (rpm_max * 1e-3f);
	EXPECT_NEAR(physical_setpoint(ControlAllocation::THRUST_Z),
		    control_setpoint(ControlAllocation::THRUST_Z) * expected_maximum_thrust, 1e-5f);
	EXPECT_NEAR(reconstructed_thrust_z, physical_setpoint(ControlAllocation::THRUST_Z), 1e-5f);

	// The reconstructed physical wrench must also map back to the original
	// normalized controller setpoint when no actuator is saturated.
	const Vector<float, ControlAllocation::NUM_AXES> allocated_control = allocation.getAllocatedControl();
	EXPECT_NEAR(allocated_control(ControlAllocation::ROLL), control_setpoint(ControlAllocation::ROLL), 1e-5f);
	EXPECT_NEAR(allocated_control(ControlAllocation::PITCH), control_setpoint(ControlAllocation::PITCH), 1e-5f);
	EXPECT_NEAR(allocated_control(ControlAllocation::YAW), control_setpoint(ControlAllocation::YAW), 1e-5f);
	EXPECT_NEAR(allocated_control(ControlAllocation::THRUST_Z), control_setpoint(ControlAllocation::THRUST_Z), 1e-5f);
}
