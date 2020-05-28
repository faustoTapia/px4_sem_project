/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#pragma once

#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/sensor_gyro_integrated.h>
#include <uORB/uORB.h>


class DshotController : public ModuleBase<DshotController>, public ModuleParams, public px4::WorkItem
{
public:
	DshotController();
	~DshotController() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);


	int print_status() override;

	bool init();


private:
	void Run() override;

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	// uORB::SubscriptionCallbackWorkItem _esc_status_sub{this, ORB_ID(esc_status)};

	// uORB::SubscriptionCallbackWorkItem _telemetry_status_sub{this, ORB_ID(telemetry_status)};


	// px4_pollfd_struct_t fds[2];
	// int _esc_status_sub_fd;
	// int _telemetry_status_sub_fd;


	uORB::Publication<actuator_controls_s>		_actuators_0_pub;

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

	hrt_abstime _last_run{0};

	actuator_controls_s _act_ctrl;

	struct actuator_armed_s  _act_arm;
	orb_advert_t _act_armed_pub;

	uORB::Subscription _gyro_integrated_sub{ORB_ID(sensor_gyro_integrated)};
	unsigned long _gyro_msgs_received_counter;
	bool _next_gyro_msg_available = false;
	sensor_gyro_integrated_s _current_gyro_integrated;
	bool get_gyro_msg(sensor_gyro_integrated_s &gyro_msg);

	uORB::Subscription _esc_info_sub{ORB_ID(esc_status)};
	unsigned long _esc_msgs_received_counter;
	bool _next_esc_msg_available = false;
	esc_status_s _current_esc_status;
	bool get_esc_msg(esc_status_s &esc_msg);

	void print_esc_status(esc_status_s esc_status, long int count);

	static void arm_motors();
	static void disarm_motors();

};
