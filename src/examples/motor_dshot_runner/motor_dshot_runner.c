/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file motor_dshot_runner.cpp
 * Using dshot test app
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/esc_status.h>
#include <drivers/drv_hrt.h>


#define RUN_TIME_S 5.0

__EXPORT int motor_dshot_runner_main(int argc, char *argv[]);

int motor_dshot_runner_main(int argc, char *argv[])
{
	PX4_INFO("Hello Starting Module");

	/* subscribe to sensor_combined topic */
	int esc_sub_fd = orb_subscribe(ORB_ID(esc_status));
	/* limit the update rate to 5 Hz */
	orb_set_interval(esc_sub_fd, 10);

	// PX4_INFO("esc_sub_fd: %d\n", esc_sub_fd);
	// /* one could wait for multiple topics with this technique, just using one here */
	// px4_pollfd_struct_t fds[] = {
	// 	{ .fd = esc_sub_fd,   .events = POLLIN },
	// };

	/* advertise actuator_controls topic */
	struct actuator_controls_s act_ctrl;
	memset(&act_ctrl, 0, sizeof(act_ctrl));
	orb_advert_t act_contrls_pub = orb_advertise(ORB_ID(actuator_controls_0), &act_ctrl);

	/* advertise actuator_armed*/
	struct actuator_armed_s  act_armed;
	memset (&act_armed, 0, sizeof(act_armed));
	orb_advert_t act_armed_pub = orb_advertise(ORB_ID(actuator_armed), &act_armed);

	// int ctrl_sub_fd = orb_subscribe(ORB_ID(actuator_controls_0));
	// orb_set_interval(ctrl_sub_fd, 20);
	// px4_pollfd_struct_t ctrl_fds[]={{.fd=ctrl_sub_fd, .events=POLLIN},};

	// int error_counter = 0;
	// int count = 0;
	hrt_abstime start_time = hrt_absolute_time();
	// hrt_abstime period_prev_time = hrt_absolute_time();
	// hrt_abstime time_passed_in_period = hrt_absolute_time();


	// Arming motors
	act_armed.timestamp = hrt_absolute_time();
	act_armed.armed_time_ms = act_armed.timestamp/1000;
	act_armed.armed = true;
	act_armed.prearmed = false;
	act_armed.ready_to_arm = false;
	act_armed.lockdown = false;
	act_armed.manual_lockdown = false;
	act_armed.force_failsafe = false;
	act_armed.in_esc_calibration_mode = false;
	act_armed.soft_stop = false;
	orb_publish(ORB_ID(actuator_armed), act_armed_pub, &act_armed);


	while (hrt_elapsed_time(&start_time)*1e-6 < RUN_TIME_S) {
		// /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		// // int poll_ret = px4_poll(fds, 1, 1000);
		// int poll_ret = px4_poll(ctrl_fds, 1, 1);
		// /* handle the poll result */
		// if (poll_ret == 0) {
		// 	/* this means none of our providers is giving us data */
		// 	// PX4_ERR("Got no data within %ds", 1);

		// } else if (poll_ret < 0) {
		// 	/* this is seriously bad - should be an emergency */
		// 	if (error_counter < 10 || error_counter % 50 == 0) {
		// 		/* use a counter to prevent flooding (and slowing us down) */
		// 		PX4_ERR("ERROR return value from poll(): %d", poll_ret);
		// 	}

		// 	error_counter++;

		// } else {
		// 	if (ctrl_fds[0].revents & POLLIN) {
		// 	// if(true){
		// 		/* obtained data for the first file descriptor */
		// 		// struct esc_status_s raw;
		// 		// /* copy sensors raw data into local buffer */
		// 		// orb_copy(ORB_ID(esc_status), esc_sub_fd, &raw);
		// 		// if(count % 500 == 0){
		// 		// // if(true){
		// 		// 	for (int i=0; i < ESC_STATUS_CONNECTED_ESC_MAX; i++){
		// 		// 		PX4_INFO("RPM: %8d\tTemp: %8.4f\tVolt: %8.4f\n",
		// 		// 		(int32_t)raw.esc[i].esc_rpm,
		// 		// 		(double)raw.esc[i].esc_temperature,
		// 		// 		(double)raw.esc[i].esc_voltage);
		// 		// 	}
		// 		struct actuator_controls_s ctrl_raw;
		// 		orb_copy(ORB_ID(actuator_controls_0), ctrl_sub_fd, &ctrl_raw);
		// 		if (count%2 == 0){
		// 			PX4_INFO("Time: %8dms\tMotor2: %5f", (int)ctrl_raw.timestamp, (double)ctrl_raw.control[0]);
		// 			// PX4_INFO("Timestamp: %d", (int)ctrl_raw.timestamp);
		// 			// for (int i = 0; i< 8; i++){
		// 			// 	PX4_INFO("\tMotor %1d: %1.5f", i, (double)ctrl_raw.control[i]);
		// 			// }
		// 		}
		// 		count++;
		// 	}

		// }


		// act_ctrl.control[0] = (0.5-0.5*cos(2.0*(double)M_PI_F*(hrt_elapsed_time(&start_time) * 1e-6)/RUN_TIME_S));
		act_ctrl.control[0] = 0;
		act_ctrl.control[1] = 0;
		act_ctrl.control[2] = 0;
		act_ctrl.control[3] = 0;
		act_ctrl.control[4] = 0;
		act_ctrl.control[5] = 0;
		act_ctrl.control[6] = 0;
		act_ctrl.control[7] = 0;
		act_ctrl.timestamp = hrt_elapsed_time(&start_time);
		act_ctrl.timestamp_sample = hrt_elapsed_time(&start_time);

		// time_passed_in_period = hrt_absolute_time()- period_prev_time;
		// px4_usleep(1250-time_passed_in_period);c
		// period_prev_time = hrt_absolute_time();
		px4_usleep(1250);
		orb_publish(ORB_ID(actuator_controls_0), act_contrls_pub, &act_ctrl);

	}
	// PX4_INFO("Received data %d times", count);
	// Arming devices
	act_armed.timestamp = hrt_absolute_time();
	act_armed.armed_time_ms = act_armed.timestamp/1000;
	act_armed.armed = false;
	act_armed.prearmed = false;
	act_armed.ready_to_arm = false;
	act_armed.lockdown = false;
	act_armed.manual_lockdown = false;
	act_armed.force_failsafe = false;
	act_armed.in_esc_calibration_mode = false;
	act_armed.soft_stop = false;
	orb_publish(ORB_ID(actuator_armed), act_armed_pub, &act_armed);


	PX4_INFO("exiting");

	return 0;
}
