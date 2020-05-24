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

#include "dshot_controller.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/getopt.h>


DshotController::DshotController():
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_actuators_0_pub(ORB_ID(actuator_controls_0)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
}

DshotController::~DshotController()
{
	perf_free(_loop_perf);
}


void
DshotController::arm_motors()
{
	/* advertise actuator_armed*/
	// struct actuator_armed_s  act_arm;
	// memset (&act_arm, 0, sizeof(act_arm));
	// orb_advert_t act_armed_pub = orb_advertise(ORB_ID(actuator_armed), &act_arm);

	get_instance()->_act_arm.armed_time_ms = (int)(hrt_absolute_time()/1000);
	get_instance()->_act_arm.armed = true;
	get_instance()->_act_arm.prearmed = false;
	get_instance()->_act_arm.ready_to_arm = true;
	get_instance()->_act_arm.lockdown = false;
	get_instance()->_act_arm.manual_lockdown = false;
	get_instance()->_act_arm.force_failsafe = false;
	get_instance()->_act_arm.in_esc_calibration_mode = false;
	get_instance()->_act_arm.soft_stop = false;

	orb_publish(ORB_ID(actuator_armed), get_instance()->_act_armed_pub, &get_instance()->_act_arm);
}

void
DshotController::disarm_motors()
{
	/* advertise actuator_armed*/
	// struct actuator_armed_s  act_arm;
	// memset (&act_arm, 0, sizeof(act_arm));
	// orb_advert_t act_armed_pub = orb_advertise(ORB_ID(actuator_armed), &act_arm);

	// uORB::Publication<actuator_armed_s> actuator_armed_pub(ORB_ID(actuator_armed));
	get_instance()->_act_arm.armed_time_ms = (int)(hrt_absolute_time()/1000);
	get_instance()->_act_arm.armed = false;
	get_instance()->_act_arm.prearmed = false;
	get_instance()->_act_arm.ready_to_arm = true;
	get_instance()->_act_arm.lockdown = false;
	get_instance()->_act_arm.manual_lockdown = false;
	get_instance()->_act_arm.force_failsafe = false;
	get_instance()->_act_arm.in_esc_calibration_mode = false;
	get_instance()->_act_arm.soft_stop = false;
	orb_publish(ORB_ID(actuator_armed), get_instance()->_act_armed_pub, &get_instance()->_act_arm);

}


bool
DshotController::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle_angular_velocity callback registration failed!");
		return false;
	}
	actuator_controls_s act_ctrl_instantaenous;
	act_ctrl_instantaenous.timestamp = hrt_absolute_time();
	act_ctrl_instantaenous.timestamp_sample = hrt_absolute_time();
	act_ctrl_instantaenous.control[0] = 0;
	act_ctrl_instantaenous.control[1] = 0;
	act_ctrl_instantaenous.control[2] = 0;
	act_ctrl_instantaenous.control[3] = 0;
	act_ctrl_instantaenous.control[4] = 0;
	act_ctrl_instantaenous.control[5] = 0;
	act_ctrl_instantaenous.control[6] = 0;
	act_ctrl_instantaenous.control[7] = 0;
	_act_ctrl = act_ctrl_instantaenous;

	memset (&_act_arm, 0, sizeof(_act_arm));
	_act_armed_pub = orb_advertise(ORB_ID(actuator_armed), &_act_arm);

	return true;
}


void
DshotController::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	vehicle_angular_velocity_s ang_vel;
	if (_vehicle_angular_velocity_sub.update(&ang_vel)){
		_act_ctrl.timestamp = hrt_absolute_time();
		_act_ctrl.timestamp_sample = _act_ctrl.timestamp_sample;
		_actuators_0_pub.publish(_act_ctrl);
	}


	perf_end(_loop_perf);
}

int DshotController::task_spawn(int argc, char *argv[])
{
	DshotController *instance = new DshotController();
	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int DshotController::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	if (!strcmp(verb, "arm")) {
		PX4_INFO("Arming");
		arm_motors();
		return 0;
	}

	if (!strcmp(verb, "disarm")){
		PX4_INFO("Disarming");
		disarm_motors();
		return 0;
	}

	if (!strcmp(verb, "throttle")){
		if (is_running()){
			actuator_controls_s act_ctrl_temp;
			act_ctrl_temp = get_instance()->_act_ctrl;
			PX4_INFO("Current throttle: ");
			PX4_INFO("Timestamp: %ld", (long)act_ctrl_temp.timestamp);
			for (int i=0 ; i < 8; i++){
				PX4_INFO("Motor %d: %f", i+1, (double)act_ctrl_temp.control[i]);
			}
			return 0;
		}
		print_usage("Module not started");
		return -1;

	}

	if (!strcmp(verb, "command")){
		if (is_running()){
			int motor_index =0;
			float motor_power = 0.0;
			int myoptind = 1;
			int ch;
			const char *myoptarg = nullptr;
			while ((ch = px4_getopt(argc, argv, "m:p:", &myoptind, &myoptarg)) != EOF) {
				switch (ch) {
				case 'm':

					motor_index = strtol(myoptarg, nullptr, 10) - 1;
					if (motor_index<0 || motor_index>7){
						return print_usage("invalid channel");
					}
					break;
				case 'p':
					motor_power = strtof(myoptarg, nullptr);
					if (motor_power<0 || motor_power>1){
						return print_usage("invalid throttle");
					}
					break;
				default:
					return print_usage("unrecognized flag");
				}
			}

			actuator_controls_s act_ctrl_temp;
			act_ctrl_temp.timestamp = hrt_absolute_time();
			act_ctrl_temp.timestamp_sample = hrt_absolute_time();
			act_ctrl_temp.control[0] = 0.0;
			act_ctrl_temp.control[1] = 0.0;
			act_ctrl_temp.control[2] = 0.0;
			act_ctrl_temp.control[3] = 0.0;
			act_ctrl_temp.control[4] = 0.0;
			act_ctrl_temp.control[5] = 0.0;
			act_ctrl_temp.control[6] = 0.0;
			act_ctrl_temp.control[7] = 0.0;
			act_ctrl_temp.control[motor_index] = motor_power;
			get_instance()->_act_ctrl=act_ctrl_temp;
			PX4_INFO("Sent speed motor %d: %f", motor_index+1, (double)get_instance()->_act_ctrl.control[motor_index]);
			return 0;
		}
		return print_usage("Module not started");

	}

	return print_usage("unknown command");
}

int DshotController::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements communication to esc via Dshot protocol

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	PRINT_MODULE_USAGE_COMMAND_DESCR("arm", "Sends arming command to actuator_armed topic");

	PRINT_MODULE_USAGE_COMMAND_DESCR("disarm", "Sends disarming command to actuator_armed topic");

	PRINT_MODULE_USAGE_COMMAND_DESCR("throttle", "Prints current throttle being sent");

	PRINT_MODULE_USAGE_COMMAND_DESCR("command", "Sends throttle command to specified motor (others 0)");
	PRINT_MODULE_USAGE_PARAM_INT('m', 1, 1, 8, "Motor selected (1-8)", false);
	PRINT_MODULE_USAGE_PARAM_FLOAT('p', 0.0, 0.0, 1.0, "Throttle value (0-1)", false);

	return 0;
}

extern "C" __EXPORT int dshot_controller_main(int argc, char *argv[])
{
	return DshotController::main(argc, argv);
}
