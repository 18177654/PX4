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

#include "payload_angle_estimator.h"

#include <drivers/drv_hrt.h>
#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/payload_angle.h>


int PayloadAngleEstimator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Calculate the payload angle, measured by a potentiometer connected to an ADC.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("payload_angle_estimator", "module");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int PayloadAngleEstimator::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int PayloadAngleEstimator::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	return print_usage("unknown command");
}


int PayloadAngleEstimator::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("payload_angle_estimator",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

PayloadAngleEstimator *PayloadAngleEstimator::instantiate(int argc, char *argv[])
{
	PayloadAngleEstimator *instance = new PayloadAngleEstimator();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

PayloadAngleEstimator::PayloadAngleEstimator()
	: ModuleParams(nullptr)
{
}

void PayloadAngleEstimator::run()
{
	// Subscribe to adc_report
	int adc_report_sub = orb_subscribe(ORB_ID(adc_report));

	px4_pollfd_struct_t fds[1];
	fds[0].fd = adc_report_sub;
	fds[0].events = POLLIN;

	// Get a publish handle on payload_angle
	struct payload_angle_s payload_angle_report = {};
	
	orb_advert_t payload_angle_topic = orb_advertise(ORB_ID(payload_angle), &payload_angle_report);
	if(payload_angle_topic == nullptr)
	{
		PX4_ERR("failed to create payload_angle object. Did you start uORB?");
	}

	// Constants
	float v_out_max = 2.8206f;
	float r2_max = 12.0f*v_out_max/(5.0f - v_out_max);
	float range_max = 270.0f;

	while (!should_exit()) {

		// wait for up to 10ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 10);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN) {

			struct adc_report_s adc_report;
			orb_copy(ORB_ID(adc_report), adc_report_sub, &adc_report);
			
			uint8_t channel = 3;
			payload_angle_report.payload_angle = (range_max/r2_max)*(12.0f*adc_report.channel_value[channel]/(5.0f-adc_report.channel_value[channel])) - range_max/2.0f;
			if(payload_angle_topic != nullptr)
			{
				payload_angle_report.timestamp = hrt_absolute_time();
				orb_publish(ORB_ID(payload_angle), payload_angle_topic, &payload_angle_report);
			}
		}
	}

	orb_unsubscribe(adc_report_sub);
}

int payload_angle_estimator_main(int argc, char *argv[])
{
	return PayloadAngleEstimator::main(argc, argv);
}
