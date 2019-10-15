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
 * @file payload_angle_estimator_params.c
 * Payload angle parameters.
 *
 * @author Anton Erasmus <18177654@sun.ac.za>
 */

/**
 * Source voltage of the payload angle estimator circuit. Should be 5V.
 *
 * @min 0.0
 * @max 12.0
 * @decimal 2
 * @increment 0.01
 * @group Payload Angle
 */
PARAM_DEFINE_FLOAT(PAYLOAD_VS, 5.0f);

/**
 * Resistor of the payload angle estimator circuit.
 *
 * @min 0.0
 * @max 100000.0
 * @decimal 2
 * @increment 0.01
 * @group Payload Angle
 */
PARAM_DEFINE_FLOAT(PAYLOAD_R, 13.35f);

/**
 * Potentiometer of the payload angle estimator circuit.
 *
 * @min 0.0
 * @max 100000.0
 * @decimal 2
 * @increment 0.01
 * @group Payload Angle
 */
PARAM_DEFINE_FLOAT(PAYLOAD_POT, 20.0f);

/**
 * The maximum rotation of the payload angle (in degrees).
 *
 * @min 0.0
 * @max 360.0
 * @decimal 2
 * @increment 0.01
 * @group Payload Angle
 */
PARAM_DEFINE_FLOAT(PAYLOAD_RANGE, 315.0f);

/**
 * Bias the payload angle for the estimator to measure 0 degrees when payload has no angle.
 *
 * @min -180.0
 * @max 180.0
 * @decimal 2
 * @increment 0.01
 * @group Payload Angle
 */
PARAM_DEFINE_FLOAT(PAYLOAD_BIAS, -0.14f);

/**
 * The cutoff frequency of the lowpass filter for the angle measurements (in hertz).
 *
 * @min 0.1
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Payload Angle
 */
PARAM_DEFINE_FLOAT(PAYLOAD_CUTOFF, 0.2f);
