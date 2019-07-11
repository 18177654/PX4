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
	r1 = 0;
	r2 = 0;
	yp1 = 0;
	yp2 = 0;
	u_prev = 0;
	u_prev1 = 0;
	u_prev2 = 0;
	ym1 = 0;
	ym2 = 0;
	w11_1 = 0;
	w11_2 = 0;
	w12_1 = 0;
	w12_2 = 0;
	w21_1 = 0;
	w21_2 = 0;
	w22_1 = 0;
	w22_2 = 0;

	uf1 = 0;
	uf2 = 0;
	phi11_1 = 0;
	phi11_2 = 0;
	phi12_1 = 0;
	phi12_2 = 0;
	phi21_1 = 0;
	phi21_2 = 0;
	phi22_1 = 0;
	phi22_2 = 0;
	phi3_1 = 0;
	phi3_2 = 0;
	phi4_1 = 0;
	phi4_2 = 0;

	the11 = MRAC_INIT_THE11.get();
	the12 = MRAC_INIT_THE12.get();
	the21 = MRAC_INIT_THE21.get();
	the22 = MRAC_INIT_THE22.get();
	the3 = MRAC_INIT_THE3.get();
	c0 = MRAC_INIT_C0.get();
	rho = 1/(MRAC_INIT_C0.get() * 125.5251f);
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
		_positionController(dt);
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

void PositionControl::_positionController(const float &dt)
{
	// P-position controller
	Vector3f vel_sp_position;
	if(MPC_X_ADAPTIVE.get() > 0)
	{
		// Proportional
		vel_sp_position = (_pos_sp - _pos).emult(Vector3f(MPC_X_ADAPTIVE_P.get(), _param_mpc_xy_p.get(), _param_mpc_z_p.get()));

		// Integral
		_pos_int(0) = _pos_int(0) + (_pos_sp(0) - _pos(0)) * MPC_X_ADAPTIVE_I.get() * dt;
		vel_sp_position(0) = vel_sp_position(0) + _pos_int(0);
	}
	else
	{
		vel_sp_position = (_pos_sp - _pos).emult(Vector3f(_param_mpc_xy_p.get(), _param_mpc_xy_p.get(),
				    _param_mpc_z_p.get()));
		_vel_sp = vel_sp_position + _vel_sp;
	}

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
		if(MPC_X_ADAPTIVE.get() > 0)
		{
			thrust_desired_NE(0) = _adaptiveDirectMRACNormalized(dt, _vel_sp(0), _vel(0), false);
		}
		else
		{
			thrust_desired_NE(0) = _param_mpc_xy_vel_p.get() * vel_err(0) + _param_mpc_xy_vel_d.get() * _vel_dot(0) + _thr_int(0);
			_adaptiveDirectMRACNormalized(dt, _vel_sp(0), _vel(0), true); // Initialize variables of adaptive controller
		}
		thrust_desired_NE(1) = _param_mpc_xy_vel_p.get() * vel_err(1) + _param_mpc_xy_vel_d.get() * _vel_dot(1) + _thr_int(1);

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
		float arw_gain = 2.f / _param_mpc_xy_vel_p.get();

		Vector2f vel_err_lim;
		vel_err_lim(0) = vel_err(0) - (thrust_desired_NE(0) - _thr_sp(0)) * arw_gain;
		vel_err_lim(1) = vel_err(1) - (thrust_desired_NE(1) - _thr_sp(1)) * arw_gain;

		// Update integral
		_thr_int(0) += _param_mpc_xy_vel_i.get() * vel_err_lim(0) * dt;
		_thr_int(1) += _param_mpc_xy_vel_i.get() * vel_err_lim(1) * dt;
	}
}

float PositionControl::_adaptiveDirectMRACNormalized(float T, float r, float yp, bool dryrun)
{
	// Parameters
	float w = MRAC_WN.get();
	float z = MRAC_ZETA.get();
	float zm = MRAC_ZERO.get();
	float l = MRAC_LAMBDA.get();
	int32_t sgn_k = MRAC_SGN_K.get();

	float adap_margin = MRAC_ADAP_MARGIN.get();
	float leak = MRAC_LEAKAGE.get();

	float adap_gain_w11 = MRAC_GAIN_W11.get();
	float adap_gain_w12 = MRAC_GAIN_W12.get();
	float adap_gain_w21 = MRAC_GAIN_W21.get();
	float adap_gain_w22 = MRAC_GAIN_W22.get();
	float adap_gain_yp = MRAC_GAIN_YP.get();
	float adap_gain_r = MRAC_GAIN_R.get();

	float the11_init = MRAC_INIT_THE11.get();
	float the12_init = MRAC_INIT_THE12.get();
	float the21_init = MRAC_INIT_THE21.get();
	float the22_init = MRAC_INIT_THE22.get();
	float the3_init = MRAC_INIT_THE3.get();
	float c0_init = MRAC_INIT_C0.get();

	// Variables
	float u;
	float ym;
	float e1, w11, w12, w21, w22;
	float uf, phi11, phi12, phi21, phi22, phi3, phi4;
	float ns2, m2;
	float e1_hat, err;
	float the11_dt, the12_dt, the21_dt, the22_dt, the3_dt, c0_dt;

	// Reference model output
	ym = (-(2*w*w*T*T-8)*ym1 - (4-4*z*w*T+w*w*T*T)*ym2 + (w*w/zm)*((2*T+zm*T*T)*r + (2*zm*T*T)*r1 + (zm*T*T-2*T)*r2))/(4+4*z*w*T+w*w*T*T);

	// Error
	if(!dryrun)
	{
    		e1 = yp - ym;
	}
	else
	{
		e1 = 0;
	}


	// States
	w11 = (-(2*l*zm*T*T-8)*w11_1 - (4-2*T*(l+zm)+l*zm*T*T)*w11_2 + (2*T)*u_prev + (-2*T)*u_prev2)/(4+2*T*(l+zm)+l*zm*T*T);
	w12 = (-(2*l*zm*T*T-8)*w12_1 - (4-2*T*(l+zm)+l*zm*T*T)*w12_2 + (T*T)*u_prev +(2*T*T)*u_prev1 + (T*T)*u_prev2)/(4+2*T*(l+zm)+l*zm*T*T);
	w21 = (-(2*l*zm*T*T-8)*w21_1 - (4-2*T*(l+zm)+l*zm*T*T)*w21_2 + (2*T)*yp + (-2*T)*yp2)/(4+2*T*(l+zm)+l*zm*T*T);
	w22 = (-(2*l*zm*T*T-8)*w22_1 - (4-2*T*(l+zm)+l*zm*T*T)*w22_2 + (T*T)*yp +(2*T*T)*yp1 + (T*T)*yp2)/(4+2*T*(l+zm)+l*zm*T*T);

	// Normalize
	uf = (-(2*w*w*T*T-8)*uf1 - (4-4*z*w*T+w*w*T*T)*uf2 + (w*w/zm)*((2*T+zm*T*T)*u_prev + (2*zm*T*T)*u_prev1 + (zm*T*T-2*T)*u_prev2))/(4+4*z*w*T+w*w*T*T);

	phi11 = (-(2*w*w*T*T-8)*phi11_1 - (4-4*z*w*T+w*w*T*T)*phi11_2 + (w*w/zm)*((2*T+zm*T*T)*w11 + (2*zm*T*T)*w11_1 + (zm*T*T-2*T)*w11_2))/(4+4*z*w*T+w*w*T*T);
	phi12 = (-(2*w*w*T*T-8)*phi12_1 - (4-4*z*w*T+w*w*T*T)*phi12_2 + (w*w/zm)*((2*T+zm*T*T)*w12 + (2*zm*T*T)*w12_1 + (zm*T*T-2*T)*w12_2))/(4+4*z*w*T+w*w*T*T);
	phi21 = (-(2*w*w*T*T-8)*phi21_1 - (4-4*z*w*T+w*w*T*T)*phi21_2 + (w*w/zm)*((2*T+zm*T*T)*w21 + (2*zm*T*T)*w21_1 + (zm*T*T-2*T)*w21_2))/(4+4*z*w*T+w*w*T*T);
	phi22 = (-(2*w*w*T*T-8)*phi22_1 - (4-4*z*w*T+w*w*T*T)*phi22_2 + (w*w/zm)*((2*T+zm*T*T)*w22 + (2*zm*T*T)*w22_1 + (zm*T*T-2*T)*w22_2))/(4+4*z*w*T+w*w*T*T);
	phi3 = (-(2*w*w*T*T-8)*phi3_1 - (4-4*z*w*T+w*w*T*T)*phi3_2 + (w*w/zm)*((2*T+zm*T*T)*yp + (2*zm*T*T)*yp1 + (zm*T*T-2*T)*yp2))/(4+4*z*w*T+w*w*T*T);
	phi4 = (-(2*w*w*T*T-8)*phi4_1 - (4-4*z*w*T+w*w*T*T)*phi4_2 + (w*w/zm)*((2*T+zm*T*T)*r + (2*zm*T*T)*r1 + (zm*T*T-2*T)*r2))/(4+4*z*w*T+w*w*T*T);

	ns2 = phi11*phi11 + phi12*phi12 + phi21*phi21 + phi22*phi22 + phi3*phi3 + phi4*phi4 + uf*uf;
	m2 = 1 + ns2;
	e1_hat = rho*(uf - the11*phi11 + the12*phi12 + the21*phi21 + the22*phi22 + the3*phi3 + c0*phi4);
	err = (e1 - e1_hat)/m2;

	// Dead zone
	if(err < -adap_margin)
	{
		err = err + adap_margin;
	}
	else if(err > adap_margin)
	{
		err = err - adap_margin;
	}
	else
	{
		err = 0;
	}

	// Parameters with leakage
	the11_dt = adap_gain_w11*(-err*phi11*sgn_k - leak*(the11 - the11_init));
	the12_dt = adap_gain_w12*(-err*phi12*sgn_k - leak*(the12 - the12_init));
	the21_dt = adap_gain_w21*(-err*phi21*sgn_k - leak*(the21 - the21_init));
	the22_dt = adap_gain_w22*(-err*phi22*sgn_k - leak*(the22 - the22_init));
	the3_dt = adap_gain_yp*(-err*phi3*sgn_k - leak*(the3 - the3_init));
	c0_dt = adap_gain_r*(-err*phi4*sgn_k - leak*(c0 - c0_init));

	// Integrate
	the11 = the11 + the11_dt*T;
	the12 = the12 + the12_dt*T;
	the21 = the21 + the21_dt*T;
	the22 = the22 + the22_dt*T;
	the3 = the3 + the3_dt*T;
	c0 = c0 + c0_dt*T;
	rho = rho + MRAC_GAIN_RHO.get()*err*(uf - the11*phi11 + the12*phi12 + the21*phi21 + the22*phi22 + the3*phi3 + c0*phi4)*T;

	// Control Signal
	u = the11*w11 + the12*w12 + the21*w21 + the22*w22 + the3*yp + c0*r;

	// Update delayed signals
	r2 = r1;
	r1 = r;
	yp2 = yp1;
	yp1 = yp;
	u_prev2 = u_prev1;
	u_prev1 = u_prev;
	u_prev = u/125.5251f;
	ym2 = ym1;
	ym1 = ym;
	w11_2 = w11_1;
	w11_1 = w11;
	w12_2 = w12_1;
	w12_1 = w12;
	w21_2 = w21_1;
	w21_1 = w21;
	w22_2 = w22_1;
	w22_1 = w22;

	uf2 = uf1;
	uf1 = uf;
	phi11_2 = phi11_1;
	phi11_1 = phi11;
	phi12_2 = phi12_1;
	phi12_1 = phi12;
	phi21_2 = phi21_1;
	phi21_1 = phi21;
	phi22_2 = phi22_1;
	phi22_1 = phi22;
	phi3_2 = phi3_1;
	phi3_1 = phi3;
	phi4_2 = phi4_1;
	phi4_1 = phi4;

	return u;
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
