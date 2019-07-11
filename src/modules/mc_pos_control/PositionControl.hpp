/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file PositionControl.hpp
 *
 * A cascaded position controller for position/velocity control only.
 */

#include <matrix/matrix/math.hpp>

#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_constraints.h>
#include <px4_module_params.h>
#pragma once

struct PositionControlStates {
	matrix::Vector3f position;
	matrix::Vector3f velocity;
	matrix::Vector3f acceleration;
	float yaw;
};

/**
 * 	Core Position-Control for MC.
 * 	This class contains P-controller for position and
 * 	PID-controller for velocity.
 * 	Inputs:
 * 		vehicle position/velocity/yaw
 * 		desired set-point position/velocity/thrust/yaw/yaw-speed
 * 		constraints that are stricter than global limits
 * 	Output
 * 		thrust vector and a yaw-setpoint
 *
 * 	If there is a position and a velocity set-point present, then
 * 	the velocity set-point is used as feed-forward. If feed-forward is
 * 	active, then the velocity component of the P-controller output has
 * 	priority over the feed-forward component.
 *
 * 	A setpoint that is NAN is considered as not set.
 * 	If there is a position/velocity- and thrust-setpoint present, then
 *  the thrust-setpoint is ommitted and recomputed from position-velocity-PID-loop.
 */
class PositionControl : public ModuleParams
{
public:

	PositionControl(ModuleParams *parent);
	~PositionControl() = default;

	/**
	 *	Overwrites certain parameters.
	 *	Overwrites are required for unit-conversion.
	 *	This method should only be called if parameters
	 *	have been updated.
	 */
	void overwriteParams();

	/**
	 * Update the current vehicle state.
	 * @param PositionControlStates structure
	 */
	void updateState(const PositionControlStates &states);

	/**
	 * Update the desired setpoints.
	 * @param setpoint a vehicle_local_position_setpoint_s structure
	 * @return true if setpoint has updated correctly
	 */
	bool updateSetpoint(const vehicle_local_position_setpoint_s &setpoint);

	/**
	 * Set constraints that are stricter than the global limits.
	 * @param constraints a PositionControl structure with supported constraints
	 */
	void updateConstraints(const vehicle_constraints_s &constraints);

	/**
	 * Apply P-position and PID-velocity controller that updates the member
	 * thrust, yaw- and yawspeed-setpoints.
	 * @see _thr_sp
	 * @see _yaw_sp
	 * @see _yawspeed_sp
	 * @param dt the delta-time
	 */
	void generateThrustYawSetpoint(const float dt);

	/**
	 * 	Set the integral term in xy to 0.
	 * 	@see _thr_int
	 */
	void resetIntegralXY() { _thr_int(0) = _thr_int(1) = 0.0f; }

	/**
	 * 	Set the integral term in z to 0.
	 * 	@see _thr_int
	 */
	void resetIntegralZ() { _thr_int(2) = 0.0f; }

	/**
	 * 	Get the
	 * 	@see _thr_sp
	 * 	@return The thrust set-point member.
	 */
	const matrix::Vector3f &getThrustSetpoint() { return _thr_sp; }

	/**
	 * 	Get the
	 * 	@see _yaw_sp
	 * 	@return The yaw set-point member.
	 */
	const float &getYawSetpoint() { return _yaw_sp; }

	/**
	 * 	Get the
	 * 	@see _yawspeed_sp
	 * 	@return The yawspeed set-point member.
	 */
	const float &getYawspeedSetpoint() { return _yawspeed_sp; }

	/**
	 * 	Get the
	 * 	@see _vel_sp
	 * 	@return The velocity set-point that was executed in the control-loop. Nan if velocity control-loop was skipped.
	 */
	const matrix::Vector3f getVelSp()
	{
		matrix::Vector3f vel_sp{};

		for (int i = 0; i <= 2; i++) {
			if (_ctrl_vel[i]) {
				vel_sp(i) = _vel_sp(i);

			} else {
				vel_sp(i) = NAN;
			}
		}

		return vel_sp;
	}

	/**
	 * 	Get the
	 * 	@see _pos_sp
	 * 	@return The position set-point that was executed in the control-loop. Nan if the position control-loop was skipped.
	 */
	const matrix::Vector3f getPosSp()
	{
		matrix::Vector3f pos_sp{};

		for (int i = 0; i <= 2; i++) {
			if (_ctrl_pos[i]) {
				pos_sp(i) = _pos_sp(i);

			} else {
				pos_sp(i) = NAN;
			}
		}

		return pos_sp;
	}

protected:

	void updateParams() override;

private:
	/**
	 * Maps setpoints to internal-setpoints.
	 * @return true if mapping succeeded.
	 */
	bool _interfaceMapping();

	void _positionController(const float &dt); /** applies the P-position-controller */
	void _velocityController(const float &dt); /** applies the PID-velocity-controller */
	float _adaptiveDirectMRACNormalized(float T, float r, float yp, bool dryrun); /** applies adaptive normalized direct MRAC for swinging payload */
	void _setCtrlFlag(bool value); /**< set control-loop flags (only required for logging) */

	matrix::Vector3f _pos{}; /**< MC position */
	matrix::Vector3f _vel{}; /**< MC velocity */
	matrix::Vector3f _vel_dot{}; /**< MC velocity derivative */
	matrix::Vector3f _acc{}; /**< MC acceleration */
	float _yaw{0.0f}; /**< MC yaw */
	matrix::Vector3f _pos_sp{}; /**< desired position */
	matrix::Vector3f _vel_sp{}; /**< desired velocity */
	matrix::Vector3f _acc_sp{}; /**< desired acceleration: not supported yet */
	matrix::Vector3f _thr_sp{}; /**< desired thrust */
	float _yaw_sp{}; /**< desired yaw */
	float _yawspeed_sp{}; /** desired yaw-speed */
	matrix::Vector3f _pos_int{}; /**< thrust integral term */
	matrix::Vector3f _thr_int{}; /**< thrust integral term */
	vehicle_constraints_s _constraints{}; /**< variable constraints */
	bool _skip_controller{false}; /**< skips position/velocity controller. true for stabilized mode */
	bool _ctrl_pos[3] = {true, true, true}; /**< True if the control-loop for position was used */
	bool _ctrl_vel[3] = {true, true, true}; /**< True if the control-loop for velocity was used */

	// Adaptive Controller global variables.
	float r1, r2;
	float yp1, yp2;
	float u_prev, u_prev1, u_prev2;
	float ym1, ym2;
	float w11_1, w11_2, w12_1, w12_2, w21_1, w21_2, w22_1, w22_2;

	float uf1, uf2;
	float phi11_1, phi11_2, phi12_1, phi12_2, phi21_1, phi21_2, phi22_1, phi22_2, phi3_1, phi3_2, phi4_1, phi4_2;
	float rho;

	float the11, the12, the21, the22, the3, c0;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_THR_MAX>) _param_mpc_thr_max,
		(ParamFloat<px4::params::MPC_THR_HOVER>) _param_mpc_thr_hover,
		(ParamFloat<px4::params::MPC_THR_MIN>) _param_mpc_thr_min,
		(ParamFloat<px4::params::MPC_MANTHR_MIN>) _param_mpc_manthr_min,
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>) _param_mpc_xy_vel_max,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,
		(ParamFloat<px4::params::MPC_TILTMAX_AIR>)
		_param_mpc_tiltmax_air, // maximum tilt for any position controlled mode in degrees
		(ParamFloat<px4::params::MPC_MAN_TILT_MAX>)
		_param_mpc_man_tilt_max, // maximum til for stabilized/altitude mode in degrees
		(ParamFloat<px4::params::MPC_Z_P>) _param_mpc_z_p,
		(ParamFloat<px4::params::MPC_Z_VEL_P>) _param_mpc_z_vel_p,
		(ParamFloat<px4::params::MPC_Z_VEL_I>) _param_mpc_z_vel_i,
		(ParamFloat<px4::params::MPC_Z_VEL_D>) _param_mpc_z_vel_d,
		(ParamFloat<px4::params::MPC_XY_P>) _param_mpc_xy_p,
		(ParamInt<px4::params::MPC_X_ADAPTIVE>) MPC_X_ADAPTIVE,
		(ParamFloat<px4::params::MPC_X_ADAPTIVE_P>) MPC_X_ADAPTIVE_P,
		(ParamFloat<px4::params::MPC_X_ADAPTIVE_I>) MPC_X_ADAPTIVE_I,
		(ParamFloat<px4::params::MRAC_WN>) MRAC_WN,
		(ParamFloat<px4::params::MRAC_ZETA>) MRAC_ZETA,
		(ParamFloat<px4::params::MRAC_ZERO>) MRAC_ZERO,
		(ParamInt<px4::params::MRAC_SGN_K>) MRAC_SGN_K,
		(ParamFloat<px4::params::MRAC_LAMBDA>) MRAC_LAMBDA,
		(ParamFloat<px4::params::MRAC_GAIN_W11>) MRAC_GAIN_W11,
		(ParamFloat<px4::params::MRAC_GAIN_W12>) MRAC_GAIN_W12,
		(ParamFloat<px4::params::MRAC_GAIN_W21>) MRAC_GAIN_W21,
		(ParamFloat<px4::params::MRAC_GAIN_W22>) MRAC_GAIN_W22,
		(ParamFloat<px4::params::MRAC_GAIN_YP>) MRAC_GAIN_YP,
		(ParamFloat<px4::params::MRAC_GAIN_R>) MRAC_GAIN_R,
		(ParamFloat<px4::params::MRAC_GAIN_RHO>) MRAC_GAIN_RHO,
		(ParamFloat<px4::params::MRAC_INIT_THE11>) MRAC_INIT_THE11,
		(ParamFloat<px4::params::MRAC_INIT_THE12>) MRAC_INIT_THE12,
		(ParamFloat<px4::params::MRAC_INIT_THE21>) MRAC_INIT_THE21,
		(ParamFloat<px4::params::MRAC_INIT_THE22>) MRAC_INIT_THE22,
		(ParamFloat<px4::params::MRAC_INIT_THE3>) MRAC_INIT_THE3,
		(ParamFloat<px4::params::MRAC_INIT_C0>) MRAC_INIT_C0,
		(ParamFloat<px4::params::MRAC_ADAP_MARGIN>) MRAC_ADAP_MARGIN,
		(ParamFloat<px4::params::MRAC_LEAKAGE>) MRAC_LEAKAGE,
		(ParamFloat<px4::params::MPC_XY_VEL_P>) _param_mpc_xy_vel_p,
		(ParamFloat<px4::params::MPC_XY_VEL_I>) _param_mpc_xy_vel_i,
		(ParamFloat<px4::params::MPC_XY_VEL_D>) _param_mpc_xy_vel_d
	)
};
