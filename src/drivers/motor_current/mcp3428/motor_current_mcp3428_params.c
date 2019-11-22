/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file motor_current_mcp3428_params.c
 * Motor current measurement parameters.
 *
 * @author Anton Erasmus <18177654@sun.ac.za>
 */

/**
 * Amps per volt.
 *
 * @min 0.0
 * @max 100.0
 * @decimal 4
 * @increment 0.001
 * @group Motor Currents
 */
PARAM_DEFINE_FLOAT(MOT_ACS_A_PER_V, 37.5f);

/**
 * The resolution of a bit.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 8
 * @increment 0.00000001
 * @group Motor Currents
 */
PARAM_DEFINE_FLOAT(MOT_MCP_LSB, 0.00025f);

/**
 * Bias the current of motor 1 to measure 0 amps when motors are off.
 *
 * @min -100.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Motor Currents
 */
PARAM_DEFINE_FLOAT(MOT_MCP_BIAS1, -0.525f);

/**
 * Bias the current of motor 2 to measure 0 amps when motors are off.
 *
 * @min -100.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Motor Currents
 */
PARAM_DEFINE_FLOAT(MOT_MCP_BIAS2, -0.525f);

/**
 * Bias the current of motor 3 to measure 0 amps when motors are off.
 *
 * @min -100.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Motor Currents
 */
PARAM_DEFINE_FLOAT(MOT_MCP_BIAS3, -0.525f);

/**
 * Bias the current of motor 4 to measure 0 amps when motors are off.
 *
 * @min -100.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Motor Currents
 */
PARAM_DEFINE_FLOAT(MOT_MCP_BIAS4, -0.525f);

/**
 * The cutoff frequency of the lowpass filter for the current measurements (in hertz).
 *
 * @min 0.1
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Motor Currents
 */
PARAM_DEFINE_FLOAT(MOT_CURR_CUTOFF, 0.1f);
