/****************************************************************************
 *
 *   Copyright (c) 2013, 2014, 2017 PX4 Development Team. All rights reserved.
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
 * @file pwm_test.cpp
 *
 * PWM servo output configuration and monitoring tool.
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_getopt.h>
#include <px4_defines.h>
#include <px4_log.h>
#include <px4_module.h>
#include <px4_cli.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#ifdef __PX4_NUTTX
#include <nuttx/fs/ioctl.h>
#endif

#include <arch/board/board.h>

#include "systemlib/err.h"
#include <parameters/param.h>
#include "drivers/drv_pwm_output.h"

static void	usage(const char *reason);
__BEGIN_DECLS
__EXPORT int	pwm_test_main(int argc, char *argv[]);
__END_DECLS


static void
usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This command is used to test PWM outputs for servo and ESC control.

The default device `/dev/pwm_output0` are the Main channels, AUX channels are on `/dev/pwm_output1` (`-d` parameter).

Channels are assigned to a group. Due to hardware limitations, the update rate can only be set per group. Use
`pwm info` to display the groups. If the `-c` argument is used, all channels of any included group must be included.

### Examples
Test the outputs of eg. channels 1 and 3, and set the PWM value to 1200 us:
$ pwm arm
$ pwm test -c 13 -p 1200

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("pwm", "command");

	PRINT_MODULE_USAGE_COMMAND_DESCR("info", "Print current configuration of all channels");
	
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Set Output to a specific value until 'q' or 'c' or 'ctrl-c' pressed");

	PRINT_MODULE_USAGE_PARAM_COMMENT("The command 'test' requires a PWM value:");
	PRINT_MODULE_USAGE_PARAM_INT('p', -1, 0, 4000, "PWM value (eg. 1100)", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("steps", "Run 5 steps from 0 to 100%");

	PRINT_MODULE_USAGE_PARAM_COMMENT("The command 'steps' requires the following values:");
	PRINT_MODULE_USAGE_PARAM_INT('l', -1, 0, 4000, "Low PWM value (eg. 1100)", false);
	PRINT_MODULE_USAGE_PARAM_INT('h', -1, 0, 4000, "High PWM value (eg. 1700)", false);
	PRINT_MODULE_USAGE_PARAM_INT('s', -1, 0, 4000, "Step size PWM value (eg. 50)", false);
	PRINT_MODULE_USAGE_PARAM_INT('t', -1, 0, 4000, "Time (in seconds) between steps (eg. 50)", false);

	PRINT_MODULE_USAGE_PARAM_COMMENT("The commands 'test' and 'steps' "
					 "additionally require to specify the channels with one of the following commands:");
	PRINT_MODULE_USAGE_PARAM_STRING('c', nullptr, nullptr, "select channels in the form: 1234 (1 digit per channel, 1=first)",
					true);
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 4096, "Select channels via bitmask (eg. 0xF, 3)", true);
	PRINT_MODULE_USAGE_PARAM_INT('g', -1, 0, 10, "Select channels by group (eg. 0, 1, 2. use 'pwm info' to show groups)",
				     true);
	PRINT_MODULE_USAGE_PARAM_FLAG('a', "Select all channels", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('r', 1, 0, 2, "Scale the PWM value", false);

	PRINT_MODULE_USAGE_PARAM_COMMENT("These parameters apply to all commands:");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/pwm_output0", "<file:dev>", "Select PWM output device", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('v', "Verbose output", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('e', "Exit with 1 instead of 0 on error", true);

}

int
pwm_test_main(int argc, char *argv[])
{
	const char *dev = PWM_OUTPUT0_DEVICE_PATH;
	uint32_t alt_channel_groups = 0;
	bool print_verbose = false;
	bool error_on_warn = false;
	int ch;
	int ret;
	int rv = 1;
	char *ep;
	uint32_t set_mask = 0;
	unsigned group;
	unsigned long channels;
	unsigned single_ch = 0;
	int pwm_value = 0;
	int min_pwm = 0;
	int max_pwm = 0;
	float pwm_scale = 1.0;
	int step_pwm = 0;
	int step_time = 0;

	if (argc < 2) {
		usage(nullptr);
		return 1;
	}

	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:vec:g:m:p:l:h:s:t:r:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

		case 'd':
			if (nullptr == strstr(myoptarg, "/dev/")) {
				PX4_WARN("device %s not valid", myoptarg);
				usage(nullptr);
				return 1;
			}

			dev = myoptarg;
			break;

		case 'v':
			print_verbose = true;
			break;

		case 'e':
			error_on_warn = true;
			break;

		case 'c':
			/* Read in channels supplied as one int and convert to mask: 1234 -> 0xF */
			channels = strtoul(myoptarg, &ep, 0);

			while ((single_ch = channels % 10)) {

				set_mask |= 1 << (single_ch - 1);
				channels /= 10;
			}

			break;

		case 'g':
			group = strtoul(myoptarg, &ep, 0);

			if ((*ep != '\0') || (group >= 32)) {
				usage("bad channel_group value");
				return 1;
			}

			alt_channel_groups |= (1 << group);
			break;

		case 'm':
			/* Read in mask directly */
			set_mask = strtoul(myoptarg, &ep, 0);

			if (*ep != '\0') {
				usage("BAD set_mask VAL");
				return 1;
			}

			break;

		case 'a':
			for (unsigned i = 0; i < PWM_OUTPUT_MAX_CHANNELS; i++) {
				set_mask |= 1 << i;
			}

			break;

		case 'p':
			if (px4_get_parameter_value(myoptarg, pwm_value) != 0) {
				PX4_ERR("CLI argument parsing for PWM value failed");
				return 1;
			}
			break;

		case 'l':
			if (px4_get_parameter_value(myoptarg, min_pwm) != 0) {
				PX4_ERR("CLI argument parsing for min PWM value failed");
				return 1;
			}
			break;

		case 'h':
			if (px4_get_parameter_value(myoptarg, max_pwm) != 0) {
				PX4_ERR("CLI argument parsing for max PWM value failed");
				return 1;
			}
			break;

		case 's':
			if (px4_get_parameter_value(myoptarg, step_pwm) != 0) {
				PX4_ERR("CLI argument parsing for step PWM value failed");
				return 1;
			}
			break;

		case 't':
			if (px4_get_parameter_value(myoptarg, step_time) != 0) {
				PX4_ERR("CLI argument parsing for step time value failed");
				return 1;
			}
			break;

		case 'r':
			pwm_scale = strtof(myoptarg, &ep);
			PX4_INFO("Scale value: %f\n", (double)pwm_scale);
			break;

		default:
			usage(nullptr);
			return 1;
		}
	}

	if (myoptind >= argc) {
		usage(nullptr);
		return 1;
	}

	const char *command = argv[myoptind];

	if (print_verbose && set_mask > 0) {
		PX4_INFO("Channels: ");
		printf("    ");

		for (unsigned i = 0; i < PWM_OUTPUT_MAX_CHANNELS; i++) {
			if (set_mask & 1 << i) {
				printf("%d ", i + 1);
			}
		}

		printf("\n");
	}

	/* open for ioctl only */
	int fd = px4_open(dev, 0);

	if (fd < 0) {
		PX4_ERR("can't open %s", dev);
		return 1;
	}

	/* get the number of servo channels */
	unsigned servo_count;
	ret = px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);

	if (ret != OK) {
		PX4_ERR("PWM_SERVO_GET_COUNT");
		return error_on_warn;
	}

	if (!strcmp(command, "test")) {

		if (set_mask == 0) {
			usage("no channels set");
			return 1;
		}

		if (pwm_value == 0) {
			usage("no PWM provided");
			return 1;
		}

		/* get current servo values */
		struct pwm_output_values last_spos;

		for (unsigned i = 0; i < servo_count; i++) {


			ret = px4_ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&last_spos.values[i]);

			if (ret != OK) {
				PX4_ERR("PWM_SERVO_GET(%d)", i);
				return 1;
			}
		}

		/* perform PWM output */

		/* Open console directly to grab CTRL-C signal */
		struct pollfd fds;
		fds.fd = 0; /* stdin */
		fds.events = POLLIN;

		if (::ioctl(fd, PWM_SERVO_SET_MODE, PWM_SERVO_ENTER_TEST_MODE) < 0) {
				PX4_ERR("Failed to Enter pwm test mode");
				goto err_out_no_test;
		}

		PX4_INFO("Press CTRL-C or 'c' to abort.");

		while (1) {
			for (unsigned i = 0; i < servo_count; i++) {
				if (set_mask & 1 << i) {
					ret = px4_ioctl(fd, PWM_SERVO_SET(i), pwm_value * pwm_scale);

					if (ret != OK) {
						PX4_ERR("PWM_SERVO_SET(%d)", i);
						goto err_out;
					}
				}
			}

			/* abort on user request */
			char c;
			ret = poll(&fds, 1, 0);

			if (ret > 0) {

				ret = read(0, &c, 1);

				if (c == 0x03 || c == 0x63 || c == 'q') {
					/* reset output to the last value */
					for (unsigned i = 0; i < servo_count; i++) {
						if (set_mask & 1 << i) {
							ret = px4_ioctl(fd, PWM_SERVO_SET(i), last_spos.values[i]);

							if (ret != OK) {
								PX4_ERR("PWM_SERVO_SET(%d)", i);
								goto err_out;
							}
						}
					}

					PX4_INFO("User abort\n");
					rv = 0;
					goto err_out;
				}
			}

			/* Delay longer than the max Oneshot duration */

			px4_usleep(2542);

#ifdef __PX4_NUTTX
			/* Trigger all timer's channels in Oneshot mode to fire
			 * the oneshots with updated values.
			 */

			up_pwm_update();
#endif
		}
		rv = 0;
err_out:
			if (::ioctl(fd, PWM_SERVO_SET_MODE, PWM_SERVO_EXIT_TEST_MODE) < 0) {
					rv = 1;
					PX4_ERR("Failed to Exit pwm test mode");
			}

err_out_no_test:
		return rv;


	} else if (!strcmp(command, "steps")) {

		if (set_mask == 0) {
			usage("no channels set");
			return 1;
		}

		if (min_pwm == 0) {
			usage("no min PWM provided");
			return 1;
		}

		if (max_pwm == 0) {
			usage("no max PWM provided");
			return 1;
		}

		if (step_pwm == 0) {
			usage("no step PWM provided");
			return 1;
		}

		if (step_time == 0) {
			usage("no step time provided");
			return 1;
		}

		/* get current servo values */
		struct pwm_output_values last_spos;

		for (unsigned i = 0; i < servo_count; i++) {

			ret = px4_ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&last_spos.values[i]);

			if (ret != OK) {
				PX4_ERR("PWM_SERVO_GET(%d)", i);
				return 1;
			}
		}

		/* perform PWM output */

		/* Open console directly to grab CTRL-C signal */
		struct pollfd fds;
		fds.fd = 0; /* stdin */
		fds.events = POLLIN;

		PX4_WARN("Running steps. WARNING! Motors will be live in 5 seconds\nPress any key to abort now.");
		px4_sleep(5);

		if (::ioctl(fd, PWM_SERVO_SET_MODE, PWM_SERVO_ENTER_TEST_MODE) < 0) {
				PX4_ERR("Failed to Enter pwm test mode");
				goto err_out_no_test;
		}

		int num_steps = abs(max_pwm - min_pwm) / abs(step_pwm);
		int current_val = min_pwm; // set current val

		for (int steps_timing_index = 0;
		     steps_timing_index <= num_steps;
		     steps_timing_index++) {

			PX4_INFO("Step input: %d", (int)(current_val * pwm_scale));

			for (unsigned i = 0; i < servo_count; i++) {
				if (set_mask & 1 << i) {

					ret = px4_ioctl(fd, PWM_SERVO_SET(i), current_val * pwm_scale);

					if (ret != OK) {
						PX4_ERR("PWM_SERVO_SET(%d) = %d", i, (int)(current_val * pwm_scale));
						goto err_out;
					}
				}
			}

			/* abort on user request */
			for(unsigned stop_index = 0 ; stop_index < 100 ; stop_index++) {
				char c;
				ret = poll(&fds, 1, 0);

				if (ret > 0) {

					ret = read(0, &c, 1);

					if (ret > 0) {
						/* reset output to the last value */
						for (unsigned i = 0; i < servo_count; i++) {
							if (set_mask & 1 << i) {
								ret = px4_ioctl(fd, PWM_SERVO_SET(i), last_spos.values[i]);

								if (ret != OK) {
									PX4_ERR("PWM_SERVO_SET(%d) = %d", i, last_spos.values[i]);
									goto err_out;
								}
							}
						}

						PX4_INFO("User abort\n");
						rv = 0;
						goto err_out;
					}
				}

				// sleep
				px4_usleep(1e4 * step_time);
			}

			// update current PWM value
			current_val = current_val + step_pwm;
		}

		/* reset output to the last value */
		for (unsigned i = 0; i < servo_count; i++) {
			if (set_mask & 1 << i) {
				ret = px4_ioctl(fd, PWM_SERVO_SET(i), last_spos.values[i]);

				if (ret != OK) {
					PX4_ERR("PWM_SERVO_SET(%d) = %d", i, last_spos.values[i]);
					goto err_out;
				}
			}
		}

		PX4_INFO("Finished...");
		rv = 0;
		goto err_out;

	} else if (!strcmp(command, "info")) {

		printf("device: %s\n", dev);

		uint32_t info_default_rate;
		uint32_t info_alt_rate;
		uint32_t info_alt_rate_mask;

		ret = px4_ioctl(fd, PWM_SERVO_GET_DEFAULT_UPDATE_RATE, (unsigned long)&info_default_rate);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_GET_DEFAULT_UPDATE_RATE");
			return 1;
		}

		ret = px4_ioctl(fd, PWM_SERVO_GET_UPDATE_RATE, (unsigned long)&info_alt_rate);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_GET_UPDATE_RATE");
			return 1;
		}

		ret = px4_ioctl(fd, PWM_SERVO_GET_SELECT_UPDATE_RATE, (unsigned long)&info_alt_rate_mask);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_GET_SELECT_UPDATE_RATE");
			return 1;
		}

		struct pwm_output_values failsafe_pwm;

		struct pwm_output_values disarmed_pwm;

		struct pwm_output_values min_pwm_str;

		struct pwm_output_values max_pwm_str;

		struct pwm_output_values trim_pwm;

		ret = px4_ioctl(fd, PWM_SERVO_GET_FAILSAFE_PWM, (unsigned long)&failsafe_pwm);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_GET_FAILSAFE_PWM");
			return 1;
		}

		ret = px4_ioctl(fd, PWM_SERVO_GET_DISARMED_PWM, (unsigned long)&disarmed_pwm);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_GET_DISARMED_PWM");
			return 1;
		}

		ret = px4_ioctl(fd, PWM_SERVO_GET_MIN_PWM, (unsigned long)&min_pwm_str);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_GET_MIN_PWM");
			return 1;
		}

		ret = px4_ioctl(fd, PWM_SERVO_GET_MAX_PWM, (unsigned long)&max_pwm_str);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_GET_MAX_PWM");
			return 1;
		}

		ret = px4_ioctl(fd, PWM_SERVO_GET_TRIM_PWM, (unsigned long)&trim_pwm);

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_GET_TRIM_PWM");
			return 1;
		}

		/* print current servo values */
		for (unsigned i = 0; i < servo_count; i++) {
			servo_position_t spos;

			ret = px4_ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&spos);

			if (ret == OK) {
				printf("channel %u: %u us", i + 1, spos);

				if (info_alt_rate_mask & (1 << i)) {
					printf(" (alternative rate: %d Hz", info_alt_rate);

				} else {
					printf(" (default rate: %d Hz", info_default_rate);
				}


				printf(" failsafe: %d, disarmed: %d us, min: %d us, max: %d us, trim: %5.2f)",
				       failsafe_pwm.values[i], disarmed_pwm.values[i], min_pwm_str.values[i], max_pwm_str.values[i],
				       (double)((int16_t)(trim_pwm.values[i]) / 10000.0f));
				printf("\n");

			} else {
				printf("%u: ERROR\n", i);
			}
		}

		/* print rate groups */
		for (unsigned i = 0; i < servo_count; i++) {
			uint32_t group_mask;

			ret = px4_ioctl(fd, PWM_SERVO_GET_RATEGROUP(i), (unsigned long)&group_mask);

			if (ret != OK) {
				break;
			}

			if (group_mask != 0) {
				printf("channel group %u: channels", i);

				for (unsigned j = 0; j < 32; j++) {
					if (group_mask & (1 << j)) {
						printf(" %u", j + 1);
					}
				}

				printf("\n");
			}
		}

		return 0;

	}

	usage(nullptr);
	return 0;
}
