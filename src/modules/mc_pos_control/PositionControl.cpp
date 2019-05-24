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
    
    the11 = the11_init;
    the12 = the12_init;
    the21 = the21_init;
    the22 = the22_init;
    the3 = the3_init;
    c0 = c0_init;
}

void PositionControl::updateState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void PositionControl::_setCtrlFlagTrue()
{
	for (int i = 0; i <= 2; i++) {
		_ctrl_pos[i] = _ctrl_vel[i] = true;
	}
}

void PositionControl::_setCtrlFlagFalse()
{
	for (int i = 0; i <= 2; i++) {
		_ctrl_pos[i] = _ctrl_vel[i] = false;
	}
}

bool PositionControl::updateSetpoint(const vehicle_local_position_setpoint_s &setpoint)
{
	// Only for logging purpose: by default we use the entire position-velocity control-loop pipeline
	_setCtrlFlagTrue();

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

		if (thr_mag > MPC_THR_MAX.get()) {
			_thr_sp = _thr_sp.normalized() * MPC_THR_MAX.get();

		} else if (thr_mag < MPC_MANTHR_MIN.get() && thr_mag > FLT_EPSILON) {
			_thr_sp = _thr_sp.normalized() * MPC_MANTHR_MIN.get();
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
		_thr_sp(2) = -(MPC_THR_MIN.get() + (MPC_THR_HOVER.get() - MPC_THR_MIN.get()) * 0.7f);
		// position and velocity control-loop is not used (note: only for logging purpose)
		_setCtrlFlagFalse(); // position/velocity control-loop is not used
	}

	return !(failsafe);
}

void PositionControl::_positionController()
{
	// P-position controller
	Vector3f vel_sp_position;
	if(MPC_X_ADAPTIVE.get() > 0)
	{
		vel_sp_position = (_pos_sp - _pos).emult(Vector3f(MPC_X_ADAPTIVE_P.get(), MPC_XY_P.get(), MPC_Z_P.get()));
	}
	else
	{
		vel_sp_position = (_pos_sp - _pos).emult(Vector3f(MPC_XY_P.get(), MPC_XY_P.get(), MPC_Z_P.get()));
	}
	
	_vel_sp = vel_sp_position + _vel_sp;

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	const Vector2f vel_sp_xy = ControlMath::constrainXY(Vector2f(vel_sp_position),
				   Vector2f(_vel_sp - vel_sp_position), _constraints.speed_xy);
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
	float thrust_desired_D = MPC_Z_VEL_P.get() * vel_err(2) +  MPC_Z_VEL_D.get() * _vel_dot(2) + _thr_int(
					 2) - MPC_THR_HOVER.get();

	// The Thrust limits are negated and swapped due to NED-frame.
	float uMax = -MPC_THR_MIN.get();
	float uMin = -MPC_THR_MAX.get();

	// Apply Anti-Windup in D-direction.
	bool stop_integral_D = (thrust_desired_D >= uMax && vel_err(2) >= 0.0f) ||
			       (thrust_desired_D <= uMin && vel_err(2) <= 0.0f);

	if (!stop_integral_D) {
		_thr_int(2) += vel_err(2) * MPC_Z_VEL_I.get() * dt;

		// limit thrust integral
		_thr_int(2) = math::min(fabsf(_thr_int(2)), MPC_THR_MAX.get()) * math::sign(_thr_int(2));
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
			thrust_desired_NE(0) = _adaptiveDirectMRACUnnormalized(dt, _vel_sp(0), _vel(0));
		}
		else
		{
			thrust_desired_NE(0) = MPC_XY_VEL_P.get() * vel_err(0) + MPC_XY_VEL_D.get() * _vel_dot(0) + _thr_int(0);	
		}
		thrust_desired_NE(1) = MPC_XY_VEL_P.get() * vel_err(1) + MPC_XY_VEL_D.get() * _vel_dot(1) + _thr_int(1);

		// Get maximum allowed thrust in NE based on tilt and excess thrust.
		float thrust_max_NE_tilt = fabsf(_thr_sp(2)) * tanf(_constraints.tilt);
		float thrust_max_NE = sqrtf(MPC_THR_MAX.get() * MPC_THR_MAX.get() - _thr_sp(2) * _thr_sp(2));
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
		float arw_gain = 2.f / MPC_XY_VEL_P.get();

		Vector2f vel_err_lim;
		vel_err_lim(0) = vel_err(0) - (thrust_desired_NE(0) - _thr_sp(0)) * arw_gain;
		vel_err_lim(1) = vel_err(1) - (thrust_desired_NE(1) - _thr_sp(1)) * arw_gain;

		// Update integral
		_thr_int(0) += MPC_XY_VEL_I.get() * vel_err_lim(0) * dt;
		_thr_int(1) += MPC_XY_VEL_I.get() * vel_err_lim(1) * dt;
	}
}

float PositionControl::_adaptiveDirectMRACUnnormalized(float T, float r, float yp)
{
	// Variables
	float ym, u;
    float e1, w11, w12, w21, w22;
    float the11_dt, the12_dt, the21_dt, the22_dt, the3_dt, c0_dt;

    // Reference model output
    ym = (-(2*w*w*T*T-8)*ym1 - (4-4*z*w*T+w*w*T*T)*ym2 + (w*w/zm)*((2*T+zm*T*T)*r + (2*zm*T*T)*r1 + (zm*T*T-2*T)*r2))/(4+4*z*w*T+w*w*T*T);

    // Error
    e1 = yp - ym;
    
    // States
    w11 = (-(2*l*zm*T*T-8)*w11_1 - (4-2*T*(l+zm)+l*zm*T*T)*w11_2 + (2*T)*u_prev + (-2*T)*u_prev2)/(4+2*T*(l+zm)+l*zm*T*T);
    w12 = (-(2*l*zm*T*T-8)*w12_1 - (4-2*T*(l+zm)+l*zm*T*T)*w12_2 + (T*T)*u_prev +(2*T*T)*u_prev1 + (T*T)*u_prev2)/(4+2*T*(l+zm)+l*zm*T*T);
    w21 = (-(2*l*zm*T*T-8)*w21_1 - (4-2*T*(l+zm)+l*zm*T*T)*w21_2 + (2*T)*yp + (-2*T)*yp2)/(4+2*T*(l+zm)+l*zm*T*T);
    w22 = (-(2*l*zm*T*T-8)*w22_1 - (4-2*T*(l+zm)+l*zm*T*T)*w22_2 + (T*T)*yp +(2*T*T)*yp1 + (T*T)*yp2)/(4+2*T*(l+zm)+l*zm*T*T);
    
    // Parameters
    the11_dt = -adap_gain_w11*e1*w11*sgn_k;
    the12_dt = -adap_gain_w12*e1*w12*sgn_k;
    the21_dt = -adap_gain_w21*e1*w21*sgn_k;
    the22_dt = -adap_gain_w22*e1*w22*sgn_k;
    the3_dt = -adap_gain_yp*e1*yp*sgn_k;
    c0_dt = -adap_gain_r*e1*r*sgn_k;
    
    // Integrate
    the11 = the11 + the11_dt*T;
    the12 = the12 + the12_dt*T;
    the21 = the21 + the21_dt*T;
    the22 = the22 + the22_dt*T;
    the3 = the3 + the3_dt*T;
    c0 = c0 + c0_dt*T;
    
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

	return u;
}

void PositionControl::updateConstraints(const vehicle_constraints_s &constraints)
{
	_constraints = constraints;

	// For safety check if adjustable constraints are below global constraints. If they are not stricter than global
	// constraints, then just use global constraints for the limits.

	if (!PX4_ISFINITE(constraints.tilt)
	    || !(constraints.tilt < math::max(MPC_TILTMAX_AIR_rad.get(), MPC_MAN_TILT_MAX_rad.get()))) {
		_constraints.tilt = math::max(MPC_TILTMAX_AIR_rad.get(), MPC_MAN_TILT_MAX_rad.get());
	}

	if (!PX4_ISFINITE(constraints.speed_up) || !(constraints.speed_up < MPC_Z_VEL_MAX_UP.get())) {
		_constraints.speed_up = MPC_Z_VEL_MAX_UP.get();
	}

	if (!PX4_ISFINITE(constraints.speed_down) || !(constraints.speed_down < MPC_Z_VEL_MAX_DN.get())) {
		_constraints.speed_down = MPC_Z_VEL_MAX_DN.get();
	}

	if (!PX4_ISFINITE(constraints.speed_xy) || !(constraints.speed_xy < MPC_XY_VEL_MAX.get())) {
		_constraints.speed_xy = MPC_XY_VEL_MAX.get();
	}
}

void PositionControl::updateParams()
{
	ModuleParams::updateParams();

	// Tilt needs to be in radians
	MPC_TILTMAX_AIR_rad.set(math::radians(MPC_TILTMAX_AIR_rad.get()));
	MPC_MAN_TILT_MAX_rad.set(math::radians(MPC_MAN_TILT_MAX_rad.get()));
}
