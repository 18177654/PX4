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
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include "Utility/ControlMath.hpp"
#include <px4_defines.h>

using namespace matrix;

PositionControl::PositionControl(ModuleParams *parent) :
	ModuleParams(parent)
{
	// X
	the11_x = _param_mpc_mrac_init_the11.get();
	the12_x = _param_mpc_mrac_init_the12.get();
	the21_x = _param_mpc_mrac_init_the21.get();
	the22_x = _param_mpc_mrac_init_the22.get();
	the3_x = _param_mpc_mrac_init_the3.get();
	c0_x = _param_mpc_mrac_init_c0.get();
	rho_x = 1.0f/_param_mpc_mrac_init_c0.get();

	w11_1_x = 0.0f;
    w12_1_x = 0.0f;
    w21_1_x = 0.0f;
    w22_1_x = 0.0f;
    w11_2_x = 0.0f;
    w12_2_x = 0.0f;
    w21_2_x = 0.0f;
    w22_2_x = 0.0f;

    ym_1_x = 0.0f;
    uf_1_x = 0.0f;
    phi11_1_x = 0.0f;
    phi12_1_x = 0.0f;
    phi21_1_x = 0.0f;
    phi22_1_x = 0.0f;
    phi3_1_x = 0.0f;
    phi4_1_x = 0.0f;

    ua_1_x = 0.0f;
    e1_1_x = 0.0f;

    r_1_x = 0.0f;
    up_1_x = 0.0f;
    up_2_x = 0.0f;
    up_3_x = 0.0f;
    yp_1_x = 0.0f;
    yp_2_x = 0.0f;

	// Y
	the11_y = _param_mpc_mrac_init_the11.get();
	the12_y = _param_mpc_mrac_init_the12.get();
	the21_y = _param_mpc_mrac_init_the21.get();
	the22_y = _param_mpc_mrac_init_the22.get();
	the3_y = _param_mpc_mrac_init_the3.get();
	c0_y = _param_mpc_mrac_init_c0.get();
	rho_y = 1.0f/_param_mpc_mrac_init_c0.get();

	w11_1_y = 0.0f;
    w12_1_y = 0.0f;
    w21_1_y = 0.0f;
    w22_1_y = 0.0f;
    w11_2_y = 0.0f;
    w12_2_y = 0.0f;
    w21_2_y = 0.0f;
    w22_2_y = 0.0f;

    ym_1_y = 0.0f;
    uf_1_y = 0.0f;
    phi11_1_y = 0.0f;
    phi12_1_y = 0.0f;
    phi21_1_y = 0.0f;
    phi22_1_y = 0.0f;
    phi3_1_y = 0.0f;
    phi4_1_y = 0.0f;

    ua_1_y = 0.0f;
    e1_1_y = 0.0f;

    r_1_y = 0.0f;
    up_1_y = 0.0f;
    up_2_y = 0.0f;
    up_3_y = 0.0f;
    yp_1_y = 0.0f;
    yp_2_y = 0.0f;
}

void PositionControl::updateState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void PositionControl::_setCtrlFlag(bool value)
{
	for (int i = 0; i <= 2; i++) {
		_ctrl_pos[i] = _ctrl_vel[i] = value;
	}
}

bool PositionControl::updateSetpoint(const vehicle_local_position_setpoint_s &setpoint)
{
	// by default we use the entire position-velocity control-loop pipeline (flag only for logging purpose)
	_setCtrlFlag(true);

	_pos_sp = Vector3f(setpoint.x, setpoint.y, setpoint.z);
	_vel_sp = Vector3f(setpoint.vx, setpoint.vy, setpoint.vz);
	_acc_sp = Vector3f(setpoint.acc_x, setpoint.acc_y, setpoint.acc_z);
	_thr_sp = Vector3f(setpoint.thrust);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
	bool mapping_succeeded = _interfaceMapping();

	// If full manual is required (thrust already generated), don't run position/velocity
	// controller and just return thrust.
	_skip_controller = PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1))
			   && PX4_ISFINITE(_thr_sp(2));

	return mapping_succeeded;
}

void PositionControl::generateThrustYawSetpoint(const float dt)
{
	if (_skip_controller) {

		// Already received a valid thrust set-point.
		// Limit the thrust vector.
		float thr_mag = _thr_sp.length();

		if (thr_mag > _param_mpc_thr_max.get()) {
			_thr_sp = _thr_sp.normalized() * _param_mpc_thr_max.get();

		} else if (thr_mag < _param_mpc_manthr_min.get() && thr_mag > FLT_EPSILON) {
			_thr_sp = _thr_sp.normalized() * _param_mpc_manthr_min.get();
		}

		// Just set the set-points equal to the current vehicle state.
		_pos_sp = _pos;
		_vel_sp = _vel;
		_acc_sp = _acc;

	} else {
		_positionController();
		_velocityController(dt);
	}
}

bool PositionControl::_interfaceMapping()
{
	// if nothing is valid, then apply failsafe landing
	bool failsafe = false;

	// Respects FlightTask interface, where NAN-set-points are of no interest
	// and do not require control. A valid position and velocity setpoint will
	// be mapped to a desired position setpoint with a feed-forward term.
	// States and setpoints which are integrals of the reference setpoint are set to 0.
	// For instance: reference is velocity-setpoint -> position and position-setpoint = 0
	//               reference is thrust-setpoint -> position, velocity, position-/velocity-setpoint = 0
	for (int i = 0; i <= 2; i++) {

		if (PX4_ISFINITE(_pos_sp(i))) {
			// Position control is required

			if (!PX4_ISFINITE(_vel_sp(i))) {
				// Velocity is not used as feedforward term.
				_vel_sp(i) = 0.0f;
			}

			// thrust setpoint is not supported in position control
			_thr_sp(i) = NAN;

			// to run position control, we require valid position and velocity
			if (!PX4_ISFINITE(_pos(i)) || !PX4_ISFINITE(_vel(i))) {
				failsafe = true;
			}

		} else if (PX4_ISFINITE(_vel_sp(i))) {

			// Velocity controller is active without position control.
			// Set integral states and setpoints to 0
			_pos_sp(i) = _pos(i) = 0.0f;
			_ctrl_pos[i] = false; // position control-loop is not used
			_pos_int(i) = 0.0f;

			// thrust setpoint is not supported in velocity control
			_thr_sp(i) = NAN;

			// to run velocity control, we require valid velocity
			if (!PX4_ISFINITE(_vel(i))) {
				failsafe = true;
			}

		} else if (PX4_ISFINITE(_thr_sp(i))) {

			// Thrust setpoint was generated from sticks directly.
			// Set all integral states and setpoints to 0
			_pos_sp(i) = _pos(i) = 0.0f;
			_vel_sp(i) = _vel(i) = 0.0f;
			_ctrl_pos[i] = _ctrl_vel[i] = false; // position/velocity control loop is not used

			// Reset the Integral term.
			_thr_int(i) = 0.0f;
			_pos_int(i) = 0.0f;
			// Don't require velocity derivative.
			_vel_dot(i) = 0.0f;

		} else {
			// nothing is valid. do failsafe
			failsafe = true;
		}
	}

	// ensure that vel_dot is finite, otherwise set to 0
	if (!PX4_ISFINITE(_vel_dot(0)) || !PX4_ISFINITE(_vel_dot(1))) {
		_vel_dot(0) = _vel_dot(1) = 0.0f;
	}

	if (!PX4_ISFINITE(_vel_dot(2))) {
		_vel_dot(2) = 0.0f;
	}

	if (!PX4_ISFINITE(_yawspeed_sp)) {
		// Set the yawspeed to 0 since not used.
		_yawspeed_sp = 0.0f;
	}

	if (!PX4_ISFINITE(_yaw_sp)) {
		// Set the yaw-sp equal the current yaw.
		// That is the best we can do and it also
		// agrees with FlightTask-interface definition.
		if (PX4_ISFINITE(_yaw)) {
			_yaw_sp = _yaw;

		} else {
			failsafe = true;
		}
	}

	// check failsafe
	if (failsafe) {
		// point the thrust upwards
		_thr_sp(0) = _thr_sp(1) = 0.0f;
		// throttle down such that vehicle goes down with
		// 70% of throttle range between min and hover
		_thr_sp(2) = -(_param_mpc_thr_min.get() + (_param_mpc_thr_hover.get() - _param_mpc_thr_min.get()) * 0.7f);
		// position and velocity control-loop is currently unused (flag only for logging purpose)
		_setCtrlFlag(false);
	}

	return !(failsafe);
}

void PositionControl::_positionController()
{
	// Position controller
	Vector3f vel_sp_position;

	float gain_xp = _param_mpc_x_p.get();
	float gain_yp = _param_mpc_y_p.get();

	if(_param_mpc_x_adap_en.get() > 0)
	{
		gain_xp = _param_mpc_x_adap_p.get();
	}
	if(_param_mpc_y_adap_en.get() > 0)
	{
		gain_yp = _param_mpc_y_adap_p.get();
	}

	// Proportional
	vel_sp_position = (_pos_sp - _pos).emult(Vector3f(gain_xp, gain_yp, _param_mpc_z_p.get()));
	_vel_sp = vel_sp_position + _vel_sp;

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	const Vector2f vel_sp_xy = ControlMath::constrainXY(Vector2f(vel_sp_position),
				   Vector2f(_vel_sp - vel_sp_position), _param_mpc_xy_vel_max.get());
	_vel_sp(0) = vel_sp_xy(0);
	_vel_sp(1) = vel_sp_xy(1);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_constraints.speed_up, _constraints.speed_down);
}

void PositionControl::_velocityController(const float &dt)
{
	// Generate desired thrust setpoint.
	// PID
	// u_des = P(vel_err) + D(vel_err_dot) + I(vel_integral)
	// Umin <= u_des <= Umax
	//
	// Anti-Windup:
	// u_des = _thr_sp; r = _vel_sp; y = _vel
	// u_des >= Umax and r - y >= 0 => Saturation = true
	// u_des >= Umax and r - y <= 0 => Saturation = false
	// u_des <= Umin and r - y <= 0 => Saturation = true
	// u_des <= Umin and r - y >= 0 => Saturation = false
	//
	// 	Notes:
	// - PID implementation is in NED-frame
	// - control output in D-direction has priority over NE-direction
	// - the equilibrium point for the PID is at hover-thrust
	// - the maximum tilt cannot exceed 90 degrees. This means that it is
	// 	 not possible to have a desired thrust direction pointing in the positive
	// 	 D-direction (= downward)
	// - the desired thrust in D-direction is limited by the thrust limits
	// - the desired thrust in NE-direction is limited by the thrust excess after
	// 	 consideration of the desired thrust in D-direction. In addition, the thrust in
	// 	 NE-direction is also limited by the maximum tilt.

	const Vector3f vel_err = _vel_sp - _vel;

	// Consider thrust in D-direction.
	float thrust_desired_D = _param_mpc_z_vel_p.get() * vel_err(2) +  _param_mpc_z_vel_d.get() * _vel_dot(2) + _thr_int(
					 2) - _param_mpc_thr_hover.get();

	// The Thrust limits are negated and swapped due to NED-frame.
	float uMax = -_param_mpc_thr_min.get();
	float uMin = -_param_mpc_thr_max.get();

	// make sure there's always enough thrust vector length to infer the attitude
	uMax = math::min(uMax, -10e-4f);

	// Apply Anti-Windup in D-direction.
	bool stop_integral_D = (thrust_desired_D >= uMax && vel_err(2) >= 0.0f) ||
			       (thrust_desired_D <= uMin && vel_err(2) <= 0.0f);

	if (!stop_integral_D) {
		_thr_int(2) += vel_err(2) * _param_mpc_z_vel_i.get() * dt;

		// limit thrust integral
		_thr_int(2) = math::min(fabsf(_thr_int(2)), _param_mpc_thr_max.get()) * math::sign(_thr_int(2));
	}

	// Saturate thrust setpoint in D-direction.
	_thr_sp(2) = math::constrain(thrust_desired_D, uMin, uMax);

	if (PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1))) {
		// Thrust set-point in NE-direction is already provided. Only
		// scaling by the maximum tilt is required.
		float thr_xy_max = fabsf(_thr_sp(2)) * tanf(_constraints.tilt);
		_thr_sp(0) *= thr_xy_max;
		_thr_sp(1) *= thr_xy_max;

	} else {
		// PID-velocity controller for NE-direction.
		Vector2f thrust_desired_NE;
		if(_param_mpc_x_adap_en.get() > 0)
		{
			thrust_desired_NE(0) = _adaptiveDirectMRACNormalizedX(dt, _vel_sp(0), _vel(0));
		}
		else
		{
			thrust_desired_NE(0) = _param_mpc_x_vel_p.get() * vel_err(0) + _param_mpc_x_vel_d.get() * _vel_dot(0) + _thr_int(0);
		}

		if(_param_mpc_y_adap_en.get() > 0)
		{
			thrust_desired_NE(1) = _adaptiveDirectMRACNormalizedY(dt, _vel_sp(1), _vel(1));
		}
		else
		{
			thrust_desired_NE(1) = _param_mpc_y_vel_p.get() * vel_err(1) + _param_mpc_y_vel_d.get() * _vel_dot(1) + _thr_int(1);
		}

		// Get maximum allowed thrust in NE based on tilt and excess thrust.
		float thrust_max_NE_tilt = fabsf(_thr_sp(2)) * tanf(_constraints.tilt);
		float thrust_max_NE = sqrtf(_param_mpc_thr_max.get() * _param_mpc_thr_max.get() - _thr_sp(2) * _thr_sp(2));
		thrust_max_NE = math::min(thrust_max_NE_tilt, thrust_max_NE);

		// Saturate thrust in NE-direction.
		_thr_sp(0) = thrust_desired_NE(0);
		_thr_sp(1) = thrust_desired_NE(1);

		if (thrust_desired_NE * thrust_desired_NE > thrust_max_NE * thrust_max_NE) {
			float mag = thrust_desired_NE.length();
			_thr_sp(0) = thrust_desired_NE(0) / mag * thrust_max_NE;
			_thr_sp(1) = thrust_desired_NE(1) / mag * thrust_max_NE;
		}

		// Use tracking Anti-Windup for NE-direction: during saturation, the integrator is used to unsaturate the output
		// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
		float arw_gain_x = 2.f / _param_mpc_x_vel_p.get();
		float arw_gain_y = 2.f / _param_mpc_y_vel_p.get();

		Vector2f vel_err_lim;
		vel_err_lim(0) = vel_err(0) - (thrust_desired_NE(0) - _thr_sp(0)) * arw_gain_x;
		vel_err_lim(1) = vel_err(1) - (thrust_desired_NE(1) - _thr_sp(1)) * arw_gain_y;

		// Update integral
		_thr_int(0) += _param_mpc_x_vel_i.get() * vel_err_lim(0) * dt;
		_thr_int(1) += _param_mpc_y_vel_i.get() * vel_err_lim(1) * dt;
	}
}

float PositionControl::_adaptiveDirectMRACNormalizedX(float dt, float r, float yp)
{
	// Parameters
	float wm = _param_mpc_mrac_wn.get();
	float lambda_omega = _param_mpc_mrac_lambda_wn.get();
	float lambda_zeta = _param_mpc_mrac_lambda_zeta.get();
	float up_norm = _param_mpc_mrac_up_norm.get();
	int32_t sgn_k = _param_mpc_mrac_sgn_k.get();

	float adap_gain_w11 = _param_mpc_mrac_gain_w11.get();
	float adap_gain_w12 = _param_mpc_mrac_gain_w12.get();
	float adap_gain_w21 = _param_mpc_mrac_gain_w21.get();
	float adap_gain_w22 = _param_mpc_mrac_gain_w22.get();
	float adap_gain_yp = _param_mpc_mrac_gain_yp.get();
	float adap_gain_r = _param_mpc_mrac_gain_r.get();

	float adap_gain_rho = _param_mpc_mrac_gain_rho.get();

	float leak_bound11 = _param_mpc_mrac_leak11_bound.get();
	float leak_bound12 = _param_mpc_mrac_leak12_bound.get();
	float leak_bound21 = _param_mpc_mrac_leak21_bound.get();
	float leak_bound22 = _param_mpc_mrac_leak22_bound.get();
	float leak_bound3 = _param_mpc_mrac_leak3_bound.get();
	float leak_bound4 = _param_mpc_mrac_leak4_bound.get();

	float leak_gain11 = _param_mpc_mrac_leak11_gain.get();
	float leak_gain12 = _param_mpc_mrac_leak12_gain.get();
	float leak_gain21 = _param_mpc_mrac_leak21_gain.get();
	float leak_gain22 = _param_mpc_mrac_leak22_gain.get();
	float leak_gain3 = _param_mpc_mrac_leak3_gain.get();
	float leak_gain4 = _param_mpc_mrac_leak4_gain.get();

	float err_tau = _param_mpc_mrac_perf_tau.get();

	float bound11 = _param_mpc_mrac_bound11.get();
	float bound12 = _param_mpc_mrac_bound12.get();
	float bound21 = _param_mpc_mrac_bound21.get();
	float bound22 = _param_mpc_mrac_bound22.get();
	float bound3 = _param_mpc_mrac_bound3.get();
	float bound4 = _param_mpc_mrac_bound4.get();

	float the11_init = _param_mpc_mrac_init_the11.get();
	float the12_init = _param_mpc_mrac_init_the12.get();
	float the21_init = _param_mpc_mrac_init_the21.get();
	float the22_init = _param_mpc_mrac_init_the22.get();
	float the3_init = _param_mpc_mrac_init_the3.get();
	float c0_init = _param_mpc_mrac_init_c0.get();

	// Variables
    float w11, w12, w21, w22;
    float rho_dot;

    float ym, up, uf;
    float phi11, phi12, phi21, phi22, phi3, phi4;
    float eps, e1, e1_hat, err_norm, ns2;

    float the11_dot, the12_dot, the21_dot, the22_dot, the3_dot, c0_dot;
    float leak11, leak12, leak21, leak22, leak3, leak4;

    float ua;

    // Calculate reference model
    ym = ref_model(r, r_1_x, ym_1_x, wm, dt);

    // Calculate omega states
    w11 = omega1(up_1_x, up_3_x, w11_1_x, w11_2_x, lambda_omega, lambda_zeta, dt);
    w12 = omega2(up_1_x, up_2_x, up_3_x, w12_1_x, w12_2_x, lambda_omega, lambda_zeta, dt);
    w21 = omega1(yp, yp_2_x, w21_1_x, w21_2_x, lambda_omega, lambda_zeta, dt);
    w22 = omega2(yp, yp_1_x, yp_2_x, w22_1_x, w22_2_x, lambda_omega, lambda_zeta, dt);

    // Calculate filtered input
    uf = ref_model(up_1_x, up_2_x, uf_1_x, wm, dt);

    // Calculate filtered states
    phi11 = ref_model(w11, w11_1_x, phi11_1_x, wm, dt);
    phi12 = ref_model(w12, w12_1_x, phi12_1_x, wm, dt);
    phi21 = ref_model(w21, w21_1_x, phi21_1_x, wm, dt);
    phi22 = ref_model(w22, w22_1_x, phi22_1_x, wm, dt);
    phi3 = ref_model(yp, yp_1_x, phi3_1_x, wm, dt);
    phi4 = ref_model(r, r_1_x, phi4_1_x, wm, dt);

    // Calculate normalized error
    eps = uf - (the11_x*phi11 + the12_x*phi12 + the21_x*phi21 + the22_x*phi22 + the3_x*phi3 + c0_x*phi4);
    e1_hat = rho_x*eps;
    e1 = yp - ym;

    ns2 = phi11*phi11 + phi12*phi12 + phi21*phi21 + phi22*phi22 + phi3*phi3 + phi4*phi4 + uf*uf;
    err_norm = (e1 - e1_hat)/(ns2 + 1.0f);
    rho_dot = adap_gain_rho*err_norm*eps;
    rho_x = rho_x + rho_dot*dt;

    // Update leakage gains
    if(fabsf(the11_x) < leak_bound11)
        leak11 = 0;
    else if(fabsf(the11_x) > 2*leak_bound11)
        leak11 = leak_gain11;
    else
        leak11 = leak_gain11*(fabsf(the11_x)/leak_bound11 - 1.0f);

    if(fabsf(the12_x) < leak_bound12)
        leak12 = 0;
    else if(fabsf(the12_x) > 2*leak_bound12)
        leak12 = leak_gain12;
    else
        leak12 = leak_gain12*(fabsf(the12_x)/leak_bound12 - 1.0f);

    if(fabsf(the21_x) < leak_bound21)
        leak21 = 0;
    else if(fabsf(the21_x) > 2*leak_bound21)
        leak21 = leak_gain21;
    else
        leak21 = leak_gain21*(fabsf(the21_x)/leak_bound21 - 1.0f);

    if(fabsf(the22_x) < leak_bound22)
        leak22 = 0;
    else if(fabsf(the22_x) > 2*leak_bound22)
        leak22 = leak_gain22;
    else
        leak22 = leak_gain22*(fabsf(the22_x)/leak_bound22 - 1.0f);

    if(fabsf(the3_x) < leak_bound3)
        leak3 = 0;
    else if(fabsf(the3_x) > 2*leak_bound3)
        leak3 = leak_gain3;
    else
        leak3 = leak_gain3*(fabsf(the3_x)/leak_bound3 - 1.0f);

    if(fabsf(c0_x) < leak_bound4)
        leak4 = 0;
    else if(fabsf(c0_x) > 2*leak_bound4)
        leak4 = leak_gain4;
    else
        leak4 = leak_gain4*(fabsf(c0_x)/leak_bound4 - 1.0f);

    // Update estimates
    the11_dot = -adap_gain_w11*(err_norm*phi11*sgn_k + leak11*(the11_x - the11_init));
    the12_dot = -adap_gain_w12*(err_norm*phi12*sgn_k + leak12*(the12_x - the12_init));
    the21_dot = -adap_gain_w21*(err_norm*phi21*sgn_k + leak21*(the21_x - the21_init));
    the22_dot = -adap_gain_w22*(err_norm*phi22*sgn_k + leak22*(the22_x - the22_init));
    the3_dot = -adap_gain_yp*(err_norm*phi3*sgn_k + leak3*(the3_x - the3_init));
    c0_dot = -adap_gain_r*(err_norm*phi4*sgn_k + leak4*(c0_x - c0_init));

    the11_x = the11_x + the11_dot*dt;
	if(fabsf(the11_x) > fabsf(bound11))
        the11_x = bound11;

    the12_x = the12_x + the12_dot*dt;
	if(fabsf(the12_x) > fabsf(bound12))
        the12_x = bound12;

    the21_x = the21_x + the21_dot*dt;
	if(fabsf(the21_x) > fabsf(bound21))
        the21_x = bound21;

    the22_x = the22_x + the22_dot*dt;
	if(fabsf(the22_x) > fabsf(bound22))
        the22_x = bound22;

    the3_x = the3_x + the3_dot*dt;
	if(fabsf(the3_x) > fabsf(bound3))
        the3_x = bound3;

    c0_x = c0_x + c0_dot*dt;
	if(fabsf(c0_x) > fabsf(bound4))
        c0_x = bound4;

    // Update performance term
    ua = ((1.0f/(2.0f*err_tau*wm))*(c0_x*(wm*dt + 2.0f)*e1 + c0_x*(wm*dt - 2.0f)*e1_1_x + 2.0f*err_tau*wm*ua_1_x));

    // Control law
    up = (the11_x*w11 + the12_x*w12 + the21_x*w21 + the22_x*w22 + the3_x*yp + c0_x*r - ua)/up_norm;

    // Store control signal
    up_3_x = up_2_x;
    up_2_x = up_1_x;
    up_1_x = up;
    yp_2_x = yp_1_x;
    yp_1_x = yp;

    ym_1_x = ym;
    r_1_x = r;
    uf_1_x = uf;

    w11_2_x = w11_1_x;
    w11_1_x = w11;
    w12_2_x = w12_1_x;
    w12_1_x = w12;
    w21_2_x = w21_1_x;
    w21_1_x = w21;
    w22_2_x = w22_1_x;
    w22_1_x = w22;

    phi11_1_x = phi11;
    phi12_1_x = phi12;
    phi21_1_x = phi21;
    phi22_1_x = phi22;
    phi3_1_x = phi3;
    phi4_1_x = phi4;

    e1_1_x = e1;
    ua_1_x = ua;

	return up;
}

float PositionControl::_adaptiveDirectMRACNormalizedY(float dt, float r, float yp)
{
	// Parameters
	float wm = _param_mpc_mrac_wn.get();
	float lambda_omega = _param_mpc_mrac_lambda_wn.get();
	float lambda_zeta = _param_mpc_mrac_lambda_zeta.get();
	float up_norm = _param_mpc_mrac_up_norm.get();
	int32_t sgn_k = _param_mpc_mrac_sgn_k.get();

	float adap_gain_w11 = _param_mpc_mrac_gain_w11.get();
	float adap_gain_w12 = _param_mpc_mrac_gain_w12.get();
	float adap_gain_w21 = _param_mpc_mrac_gain_w21.get();
	float adap_gain_w22 = _param_mpc_mrac_gain_w22.get();
	float adap_gain_yp = _param_mpc_mrac_gain_yp.get();
	float adap_gain_r = _param_mpc_mrac_gain_r.get();

	float adap_gain_rho = _param_mpc_mrac_gain_rho.get();

	float leak_bound11 = _param_mpc_mrac_leak11_bound.get();
	float leak_bound12 = _param_mpc_mrac_leak12_bound.get();
	float leak_bound21 = _param_mpc_mrac_leak21_bound.get();
	float leak_bound22 = _param_mpc_mrac_leak22_bound.get();
	float leak_bound3 = _param_mpc_mrac_leak3_bound.get();
	float leak_bound4 = _param_mpc_mrac_leak4_bound.get();

	float leak_gain11 = _param_mpc_mrac_leak11_gain.get();
	float leak_gain12 = _param_mpc_mrac_leak12_gain.get();
	float leak_gain21 = _param_mpc_mrac_leak21_gain.get();
	float leak_gain22 = _param_mpc_mrac_leak22_gain.get();
	float leak_gain3 = _param_mpc_mrac_leak3_gain.get();
	float leak_gain4 = _param_mpc_mrac_leak4_gain.get();

	float err_tau = _param_mpc_mrac_perf_tau.get();

	float bound11 = _param_mpc_mrac_bound11.get();
	float bound12 = _param_mpc_mrac_bound12.get();
	float bound21 = _param_mpc_mrac_bound21.get();
	float bound22 = _param_mpc_mrac_bound22.get();
	float bound3 = _param_mpc_mrac_bound3.get();
	float bound4 = _param_mpc_mrac_bound4.get();

	float the11_init = _param_mpc_mrac_init_the11.get();
	float the12_init = _param_mpc_mrac_init_the12.get();
	float the21_init = _param_mpc_mrac_init_the21.get();
	float the22_init = _param_mpc_mrac_init_the22.get();
	float the3_init = _param_mpc_mrac_init_the3.get();
	float c0_init = _param_mpc_mrac_init_c0.get();

	// Variables
    float w11, w12, w21, w22;
    float rho_dot;

    float ym, up, uf;
    float phi11, phi12, phi21, phi22, phi3, phi4;
    float eps, e1, e1_hat, err_norm, ns2;

    float the11_dot, the12_dot, the21_dot, the22_dot, the3_dot, c0_dot;
    float leak11, leak12, leak21, leak22, leak3, leak4;

    float ua;

    // Calculate reference model
    ym = ref_model(r, r_1_y, ym_1_y, wm, dt);

    // Calculate omega states
    w11 = omega1(up_1_y, up_3_y, w11_1_y, w11_2_y, lambda_omega, lambda_zeta, dt);
    w12 = omega2(up_1_y, up_2_y, up_3_y, w12_1_y, w12_2_y, lambda_omega, lambda_zeta, dt);
    w21 = omega1(yp, yp_2_y, w21_1_y, w21_2_y, lambda_omega, lambda_zeta, dt);
    w22 = omega2(yp, yp_1_y, yp_2_y, w22_1_y, w22_2_y, lambda_omega, lambda_zeta, dt);

    // Calculate filtered input
    uf = ref_model(up_1_y, up_2_y, uf_1_y, wm, dt);

    // Calculate filtered states
    phi11 = ref_model(w11, w11_1_y, phi11_1_y, wm, dt);
    phi12 = ref_model(w12, w12_1_y, phi12_1_y, wm, dt);
    phi21 = ref_model(w21, w21_1_y, phi21_1_y, wm, dt);
    phi22 = ref_model(w22, w22_1_y, phi22_1_y, wm, dt);
    phi3 = ref_model(yp, yp_1_y, phi3_1_y, wm, dt);
    phi4 = ref_model(r, r_1_y, phi4_1_y, wm, dt);

    // Calculate normalized error
    eps = uf - (the11_y*phi11 + the12_y*phi12 + the21_y*phi21 + the22_y*phi22 + the3_y*phi3 + c0_y*phi4);
    e1_hat = rho_y*eps;
    e1 = yp - ym;

    ns2 = phi11*phi11 + phi12*phi12 + phi21*phi21 + phi22*phi22 + phi3*phi3 + phi4*phi4 + uf*uf;
    err_norm = (e1 - e1_hat)/(ns2 + 1.0f);
    rho_dot = adap_gain_rho*err_norm*eps;
    rho_y = rho_y + rho_dot*dt;

    // Update leakage gains
    if(fabsf(the11_y) < leak_bound11)
        leak11 = 0;
    else if(fabsf(the11_y) > 2*leak_bound11)
        leak11 = leak_gain11;
    else
        leak11 = leak_gain11*(fabsf(the11_y)/leak_bound11 - 1.0f);

    if(fabsf(the12_y) < leak_bound12)
        leak12 = 0;
    else if(fabsf(the12_y) > 2*leak_bound12)
        leak12 = leak_gain12;
    else
        leak12 = leak_gain12*(fabsf(the12_y)/leak_bound12 - 1.0f);

    if(fabsf(the21_y) < leak_bound21)
        leak21 = 0;
    else if(fabsf(the21_y) > 2*leak_bound21)
        leak21 = leak_gain21;
    else
        leak21 = leak_gain21*(fabsf(the21_y)/leak_bound21 - 1.0f);

    if(fabsf(the22_y) < leak_bound22)
        leak22 = 0;
    else if(fabsf(the22_y) > 2*leak_bound22)
        leak22 = leak_gain22;
    else
        leak22 = leak_gain22*(fabsf(the22_y)/leak_bound22 - 1.0f);

    if(fabsf(the3_y) < leak_bound3)
        leak3 = 0;
    else if(fabsf(the3_y) > 2*leak_bound3)
        leak3 = leak_gain3;
    else
        leak3 = leak_gain3*(fabsf(the3_y)/leak_bound3 - 1.0f);

    if(fabsf(c0_y) < leak_bound4)
        leak4 = 0;
    else if(fabsf(c0_y) > 2*leak_bound4)
        leak4 = leak_gain4;
    else
        leak4 = leak_gain4*(fabsf(c0_y)/leak_bound4 - 1.0f);

    // Update estimates
    the11_dot = -adap_gain_w11*(err_norm*phi11*sgn_k + leak11*(the11_y - the11_init));
    the12_dot = -adap_gain_w12*(err_norm*phi12*sgn_k + leak12*(the12_y - the12_init));
    the21_dot = -adap_gain_w21*(err_norm*phi21*sgn_k + leak21*(the21_y - the21_init));
    the22_dot = -adap_gain_w22*(err_norm*phi22*sgn_k + leak22*(the22_y - the22_init));
    the3_dot = -adap_gain_yp*(err_norm*phi3*sgn_k + leak3*(the3_y - the3_init));
    c0_dot = -adap_gain_r*(err_norm*phi4*sgn_k + leak4*(c0_y - c0_init));

    the11_y = the11_y + the11_dot*dt;
	if(fabsf(the11_y) > fabsf(bound11))
        the11_y = bound11;

    the12_y = the12_y + the12_dot*dt;
	if(fabsf(the12_y) > fabsf(bound12))
        the12_y = bound12;

    the21_y = the21_y + the21_dot*dt;
	if(fabsf(the21_y) > fabsf(bound21))
        the21_y = bound21;

    the22_y = the22_y + the22_dot*dt;
	if(fabsf(the22_y) > fabsf(bound22))
        the22_y = bound22;

    the3_y = the3_y + the3_dot*dt;
	if(fabsf(the3_y) > fabsf(bound3))
        the3_y = bound3;

    c0_y = c0_y + c0_dot*dt;
	if(fabsf(c0_y) > fabsf(bound4))
        c0_y = bound4;

    // Update performance term
    ua = (1.0f/(2.0f*err_tau*wm))*(c0_y*(wm*dt + 2.0f)*e1 + c0_y*(wm*dt - 2.0f)*e1_1_y + 2.0f*err_tau*wm*ua_1_y);

    // Control law
    up = (the11_y*w11 + the12_y*w12 + the21_y*w21 + the22_y*w22 + the3_y*yp + c0_y*r - ua)/up_norm;

    // Store control signal
    up_3_y = up_2_y;
    up_2_y = up_1_y;
    up_1_y = up;
    yp_2_y = yp_1_y;
    yp_1_y = yp;

    ym_1_y = ym;
    r_1_y = r;
    uf_1_y = uf;

    w11_2_y = w11_1_y;
    w11_1_y = w11;
    w12_2_y = w12_1_y;
    w12_1_y = w12;
    w21_2_y = w21_1_y;
    w21_1_y = w21;
    w22_2_y = w22_1_y;
    w22_1_y = w22;

    phi11_1_y = phi11;
    phi12_1_y = phi12;
    phi21_1_y = phi21;
    phi22_1_y = phi22;
    phi3_1_y = phi3;
    phi4_1_y = phi4;

    e1_1_y = e1;
    ua_1_y = ua;

	return up;
}

float PositionControl::ref_model(float u, float u_1, float y_1, float wm, float dt)
{
    return (1.0f/(wm*dt + 2.0f))*(wm*dt*u + wm*dt*u_1 - (wm*dt - 2.0f)*y_1);
}

float PositionControl::omega1(float u, float u_2, float y_1, float y_2, float lambda_wn, float lambda_zeta, float dt)
{
    float lambda1 = 2.0f*lambda_wn*lambda_zeta;
    float lambda0 = lambda_wn*lambda_wn;

    return (1.0f/(4.0f + 2.0f*lambda1*dt + dt*dt*lambda0))*(2.0f*dt*u - 2.0f*dt*u_2 - (2.0f*dt*dt*lambda0 - 8.0f)*y_1 - (4.0f - 2.0f*dt*lambda1 + dt*dt*lambda0)*y_2);
}

float PositionControl::omega2(float u, float u_1, float u_2, float y_1, float y_2, float lambda_wn, float lambda_zeta, float dt)
{
    float lambda1 = 2.0f*lambda_wn*lambda_zeta;
    float lambda0 = lambda_wn*lambda_wn;

    return (1.0f/(4.0f + 2.0f*lambda1*dt + dt*dt*lambda0))*(dt*dt*u + 2.0f*dt*dt*u_1 + dt*dt*u_2 - (2.0f*dt*dt*lambda0 - 8.0f)*y_1 - (4.0f - 2.0f*dt*lambda1 + dt*dt*lambda0)*y_2);
}

void PositionControl::updateConstraints(const vehicle_constraints_s &constraints)
{
	_constraints = constraints;

	// For safety check if adjustable constraints are below global constraints. If they are not stricter than global
	// constraints, then just use global constraints for the limits.

	const float tilt_max_radians = math::radians(math::max(_param_mpc_tiltmax_air.get(), _param_mpc_man_tilt_max.get()));

	if (!PX4_ISFINITE(constraints.tilt)
	    || !(constraints.tilt < tilt_max_radians)) {
		_constraints.tilt = tilt_max_radians;
	}

	if (!PX4_ISFINITE(constraints.speed_up) || !(constraints.speed_up < _param_mpc_z_vel_max_up.get())) {
		_constraints.speed_up = _param_mpc_z_vel_max_up.get();
	}

	if (!PX4_ISFINITE(constraints.speed_down) || !(constraints.speed_down < _param_mpc_z_vel_max_dn.get())) {
		_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
	}

	if (!PX4_ISFINITE(constraints.speed_xy) || !(constraints.speed_xy < _param_mpc_xy_vel_max.get())) {
		_constraints.speed_xy = _param_mpc_xy_vel_max.get();
	}
}

void PositionControl::updateParams()
{
	ModuleParams::updateParams();
}
