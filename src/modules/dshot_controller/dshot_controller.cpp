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
#include <math.h>
#include <stdlib.h>
#include <time.h>

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

	if (!_vehicle_odometry_sub.registerCallback()){
		PX4_ERR("vehicle_odometry callback registration failed!");
		return false;
	}

	if (!_vehicle_attitude_sub.registerCallback()){
		PX4_ERR("vehicle_attitude callback registration failed!");
		return false;
	}

	memset (&_act_arm, 0, sizeof(_act_arm));
	_act_armed_pub = orb_advertise(ORB_ID(actuator_armed), &_act_arm);
	set_all_ctrls(0);
	_esc_msgs_received_counter = 0;
	_actuator_controls_counter = 0;
	return true;
}


void
DshotController::Run()
{
	perf_begin(_loop_perf);
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		_vehicle_attitude_sub.unregisterCallback();
		_vehicle_odometry_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_esc_info_sub.updated()){
		_esc_info_sub.update(&_current_esc_status);
		_next_esc_msg_available = true;
		_esc_msgs_received_counter++;
	}

	// vehicle_angular_velocity_s ang_vel;
	// if (_vehicle_angular_velocity_sub.update(&ang_vel)){
	// 	_actuator_controls_counter++;
	// 	_act_ctrl.timestamp_sample = _actuator_controls_counter;
	// 	_act_ctrl.timestamp = hrt_absolute_time();
	// 	_act_ctrl.timestamp_sample = _act_ctrl.timestamp_sample;
	// 	_actuators_0_pub.publish(_act_ctrl);
	// }

	// vehicle_odometry_s vehicle_odom;
	// if (_vehicle_odometry_sub.update(&vehicle_odom)){
	// 	_actuator_controls_counter++;
	// 	_act_ctrl.timestamp_sample = _actuator_controls_counter;
	// 	_act_ctrl.timestamp = hrt_absolute_time();
	// 	_act_ctrl.timestamp_sample = _act_ctrl.timestamp_sample;
	// 	_actuators_0_pub.publish(_act_ctrl);
	// }

	vehicle_attitude_s vehicle_attitude;
	if (_vehicle_attitude_sub.update(&vehicle_attitude)){
		if (++_downsampling_count % 2 == 0 || !_actuator_controls_counter){
			_actuator_controls_counter++;
			_act_ctrl.timestamp_sample = _actuator_controls_counter;
			_act_ctrl.timestamp = hrt_absolute_time();
			_actuators_0_pub.publish(_act_ctrl);
			_prev_act_ctrl =_act_ctrl;
			_downsampling_count = 0;
		}else{
			_prev_act_ctrl.timestamp_sample = _actuator_controls_counter;
			_prev_act_ctrl.timestamp = hrt_absolute_time();
			_actuators_0_pub.publish(_prev_act_ctrl);
		}
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
			PX4_INFO("Instance Initialized");
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

	if (!strcmp(verb, "command")){
		if (is_running()){
			int motor_index =0;
			float motor_power = 0.0;
			bool set_all = false;
			int myoptind = 1;
			int ch;
			const char *myoptarg = nullptr;
			while ((ch = px4_getopt(argc, argv, "m:p:a", &myoptind, &myoptarg)) != EOF) {
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
				case 'a':
					set_all = true;
					break;
				default:
					return print_usage("unrecognized flag");
				}
			}

			if (set_all){
				set_all_ctrls(motor_power);
			}else{
				set_ctrl(motor_index, motor_power);
			}
			PX4_INFO("Set act_controls to:");
			print_act_ctrl(get_instance()->_act_ctrl);

			return 0;
		}
		return print_usage("Module not started");

	}

	if (!strcmp(verb, "listen_esc")){
		if (is_running()){
			int myoptind = 1;
			int ch;
			int num_of_samples = 1;
			const char *myoptarg = nullptr;
			while ((ch = px4_getopt(argc, argv, "n:", &myoptind, &myoptarg)) != EOF) {
				switch (ch) {
				case 'n':
					num_of_samples = strtol(myoptarg, nullptr, 10);
					if (num_of_samples<=1 || num_of_samples>=100){
						return print_usage("Invalind num of samples");
					}
					break;
				default:
					return print_usage("unrecognized flag");
				}
			}
			PX4_INFO("Listening esc %d samples", num_of_samples);
			int count = 0;
			hrt_abstime prev_time = hrt_absolute_time();
			esc_status_s esc_temp;
			while(count<num_of_samples){

				if (get_instance()->get_esc_msg(esc_temp)){
					prev_time = hrt_absolute_time();
					get_instance()->print_esc_status(esc_temp, count+1);
					count++;
				}else if (hrt_absolute_time()-prev_time > 5e5){
					prev_time = hrt_absolute_time();
					PX4_INFO("Esc status count: %d Timedout",count+1);
					count++;
				}else{
					px4_usleep(1000);
				}
			}
			return 0;
		}
		return print_usage("Module not started");

	}

	if (!strcmp(verb, "run_sine")){
		if (is_running()){
			int myoptind = 1;
			int ch;
			float signal_center = 0.5;
			float signal_amplitude = 0.1;
			int period_ms = 2000;
			int dur_s = 10;
			uint8_t motor_index = 0;
			const char *myoptarg = nullptr;
			while ((ch = px4_getopt(argc, argv, "o:a:p:d:m:", &myoptind, &myoptarg)) != EOF) {
				switch (ch) {
				case 'm':
					motor_index = (uint8_t)strtol(myoptarg, nullptr, 10)-1;
					if (motor_index > get_instance()->_act_ctrl.NUM_ACTUATOR_CONTROLS - 1){
						return print_usage("Invalid motor_index");
					}
					break;
				case 'p':
					period_ms = strtol(myoptarg, nullptr, 10);
					if (period_ms<20 || period_ms>10000){
						return print_usage("Invalind period_ms, must be in [20,10000]");
					}
					break;
				case 'd':
					dur_s = strtol(myoptarg, nullptr, 10);
					if (dur_s<2 || dur_s>20){
						return print_usage("Invalid duration_s, must be in [2,20]");
					}
					break;

				case 'o':
					signal_center = strtof(myoptarg, nullptr);
					if (signal_center<(float)0 ||  signal_center >(float)0.9){
						return print_usage("Offset out of range. Needs to be (0.01-0.9)");
					}
					break;
				case 'a':
					signal_amplitude = strtof(myoptarg, nullptr);
					if (signal_center<(float)0 || signal_center>(float)0.5){
						return print_usage("Invalid amplitude, needs to be (0-0.5)");
					}
					break;
				default:
					return print_usage("unrecognized flag");
				}
			}

			if (signal_amplitude+signal_center>(float)0.9 || signal_center-signal_amplitude<(float)0){
				return print_usage("Invalid Sinewave definition, the wave must be bounded by [0-0.9]");
			}

			PX4_INFO("Playing Sinewave. period: %d ms for %d secs",period_ms,dur_s);

			get_instance()->set_ctrl(motor_index, signal_center);
			px4_sleep(1);

			hrt_abstime start_time = hrt_absolute_time();
			float val_to_output = 0;
			while(hrt_elapsed_time(&start_time) < dur_s*1e6){
				val_to_output = signal_center+signal_amplitude*(float)sin(hrt_elapsed_time(&start_time)*M_PI_F*2/1000/period_ms);
				get_instance()->set_ctrl(motor_index, val_to_output);
				px4_usleep(1000);
			}

			get_instance()->set_ctrl(motor_index, 0);

			return 0;
		}
		return print_usage("Module not started");

	}

	if (!strcmp(verb, "sweep_sine")){
		if (is_running()){
			int myoptind = 1;
			int ch;
			float signal_center = 0.4;
			float signal_amplitude = 0.1;
			int period_us[] = {8000000,4000000,2000000,1000000,500000,250000,125000,62500,31250,15625};
			int dur_s = 8;
			uint8_t motor_index = 0;
			const char *myoptarg = nullptr;
			while ((ch = px4_getopt(argc, argv, "o:a:m:", &myoptind, &myoptarg)) != EOF) {
				switch (ch) {
				case 'm':
					motor_index = (uint8_t)strtol(myoptarg, nullptr, 10)-1;
					if (motor_index > get_instance()->_act_ctrl.NUM_ACTUATOR_CONTROLS - 1){
						return print_usage("Invalid motor_index");
					}
					break;
				case 'o':
					signal_center = strtof(myoptarg, nullptr);
					if (signal_center<(float)0 ||  signal_center >(float)0.9){
						return print_usage("Offset out of range. Needs to be (0.1-0.9)");
					}
					break;
				case 'a':
					signal_amplitude = strtof(myoptarg, nullptr);
					if (signal_amplitude<(float)0 || signal_amplitude>(float)0.6){
						return print_usage("Invalid amplitude, needs to be (0-0.6)");
					}
					break;
				default:
					return print_usage("unrecognized flag");
				}
			}

			if (signal_amplitude+signal_center>(float)0.9 || signal_center-signal_amplitude<(float)0){
				return print_usage("Invalid Sinewave definition, the wave must be bounded by [0-0.9]");
			}

			PX4_INFO("Playing sine sweep for %d seconds",80);

			get_instance()->set_ctrl(motor_index, signal_center);
			px4_sleep(1);

			for (int i = 0; i<10; i++){
				hrt_abstime start_time = hrt_absolute_time();
				float val_to_output = 0;
				while(hrt_elapsed_time(&start_time) < dur_s*1e6){
					val_to_output = signal_center+signal_amplitude*(float)sin(hrt_elapsed_time(&start_time)*M_PI_F*2/period_us[i]);
					get_instance()->set_ctrl(motor_index, val_to_output);
					px4_usleep(1000);
				}
			}
			get_instance()->set_ctrl(motor_index, signal_center);
			px4_sleep(1);
			get_instance()->set_ctrl(motor_index, 0);

			return 0;
		}
		return print_usage("Module not started");

	}

	if (!strcmp(verb, "run_random")){
		if (is_running()){
			int myoptind = 1;
			int ch;
			float signal_min = 0.1;
			float signal_max = 0.4;
			int dur_s = 5;
			uint8_t motor_index = 0;
			const char *myoptarg = nullptr;
			while ((ch = px4_getopt(argc, argv, "b:t:d:m:", &myoptind, &myoptarg)) != EOF) {
				switch (ch) {
				case 'm':
					motor_index = (uint8_t)strtol(myoptarg, nullptr, 10)-1;
					if (motor_index > get_instance()->_act_ctrl.NUM_ACTUATOR_CONTROLS - 1){
						return print_usage("Invalid motor_index");
					}
					break;
				case 'd':
					dur_s = strtol(myoptarg, nullptr, 10);
					if (dur_s<2 || dur_s>20){
						return print_usage("Invalid duration_s, must be in [2,20]");
					}
					break;

				case 'b':
					signal_min = strtof(myoptarg, nullptr);
					if (signal_min<(float)0.1 ||  signal_min >(float)0.6){
						return print_usage("Invalid signal minimum. Needs to be (0.1-0.6)");
					}
					break;
				case 't':
					signal_max = strtof(myoptarg, nullptr);
					if (signal_max<(float)0.1 || signal_max>(float)0.6){
						return print_usage("Invalid signal max, Needs to be (0.1-0.6)");
					}
					break;
				default:
					return print_usage("unrecognized flag");
				}
			}
			if (signal_min >= signal_max){
				return print_usage("Invalid signal extrema, must comply with signal_min<signal_max");
			}
			PX4_INFO("Playing Random signals for %d secs",dur_s);
			float signal_center = (signal_max+signal_min)/(float)2.0;
			get_instance()->set_ctrl(motor_index, signal_center);
			px4_sleep(1);

			hrt_abstime start_time = hrt_absolute_time();
			float val_to_output = 0;
			int max_rand = 1001;
			srand(time(NULL));
			while(hrt_elapsed_time(&start_time) < dur_s*1e6){

				val_to_output = signal_center + ((float)(rand()%max_rand)/((float)(max_rand-1)) - (float)0.5)*((signal_max-signal_min)) ;
				get_instance()->set_ctrl(motor_index, val_to_output);
				px4_usleep(1000);
			}

			get_instance()->set_ctrl(motor_index, signal_center);
			px4_sleep(1);
			get_instance()->set_ctrl(motor_index, 0);

			return 0;
		}
		return print_usage("Module not started");

	}

	return print_usage("unknown command");
}

void DshotController::set_all_ctrls(float new_val){
	if (new_val<=1 && new_val >=0){
		if(is_running()){
			actuator_controls_s act_ctrl_temp;
			for (int i = 0; i < get_instance()->_act_ctrl.NUM_ACTUATOR_CONTROLS; i++){
				act_ctrl_temp.control[i] = new_val;
			}
			act_ctrl_temp.timestamp = hrt_absolute_time();
			act_ctrl_temp.timestamp_sample = get_instance()->_actuator_controls_counter;
			get_instance()->_act_ctrl = act_ctrl_temp;
		}
	}
}

void DshotController::set_ctrl(int index, float new_val){
	if (index<get_instance()->_act_ctrl.NUM_ACTUATOR_CONTROLS && index>=0
	   && new_val>=0 && new_val <=1){
		   get_instance()->_act_ctrl.control[index] = new_val;
	}
}

bool DshotController::get_esc_msg(esc_status_s &esc_msg){
	if (_next_esc_msg_available){
		esc_msg = _current_esc_status;
		_next_esc_msg_available = false;
		return true;
	}
	return false;
}

void DshotController::print_esc_status(esc_status_s esc_status, long int count){
	PX4_INFO("Esc status: %ld", count);
	PX4_INFO("\tTimestamp: %llu", (unsigned long long)esc_status.timestamp);
	PX4_INFO("\tRPM\tVoltage\tCurrent\tTemperature");
	for (int i =0; i<esc_status.CONNECTED_ESC_MAX; i++){
		PX4_INFO("\t%ld\t%f\t%f\t%d",
		(long)esc_status.esc[i].esc_rpm,
		(double)esc_status.esc[i].esc_voltage,
		(double)esc_status.esc[i].esc_current,
		(int)esc_status.esc[i].esc_temperature);
	}
}

void DshotController::print_act_ctrl(actuator_controls_s act_ctrl_temp){
	PX4_INFO("act_control");
	PX4_INFO("Timestamp: %ld", (long)act_ctrl_temp.timestamp);
	for (int i=0 ; i < 8; i++){
		PX4_INFO("\tMotor %d: %f", i+1, (double)act_ctrl_temp.control[i]);
	}
}

int DshotController::print_status(){

	if (is_running()){
		PX4_INFO("Module Running");
		actuator_controls_s act_ctrl_temp;
		act_ctrl_temp = get_instance()->_act_ctrl;
		print_act_ctrl(act_ctrl_temp);

		esc_status_s esc_stat = get_instance()->_current_esc_status;
		print_esc_status(esc_stat,0);
		return 0;
	}
	print_usage("Module not started");
	return -1;

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

	PRINT_MODULE_USAGE_NAME("dshot_controller", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	PRINT_MODULE_USAGE_COMMAND_DESCR("arm", "Sends arming command to actuator_armed topic");

	PRINT_MODULE_USAGE_COMMAND_DESCR("disarm", "Sends disarming command to actuator_armed topic");

	PRINT_MODULE_USAGE_COMMAND_DESCR("command", "Sends throttle command to specified motor (others 0)");
	PRINT_MODULE_USAGE_PARAM_INT('m', 1, 1, 8, "Motor selected (1-8)", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('p', 0.0, 0.0, 1.0, "Throttle value (0-1)", false);
	PRINT_MODULE_USAGE_PARAM_FLAG('a', "Sets all motors to given power values", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("listen_esc", "Shows a number of esc datasets retrieved");
	PRINT_MODULE_USAGE_PARAM_INT('n', 1, 1, 100, "Number of datasets to receive (1-100)", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("run_sine", "Outputs a Sinusoid to a selected motor");
	PRINT_MODULE_USAGE_PARAM_INT('m', 1, 1, 8, "Motor selected (1-8)", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 2000, 10, 10000, "Period in ms (10-10000)", true);
	PRINT_MODULE_USAGE_PARAM_INT('d', 10, 2, 20, "Duration in s (2-20)", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('o', 0.5, 0.01, 0.9, "Wave Offset (0.01-0.9)", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('a', 0.1, 0, 0.5, "Amplitude (0-0.5)", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("run_random", "Outputs a random signal to a selected motor");
	PRINT_MODULE_USAGE_PARAM_INT('m', 1, 1, 8, "Motor selected (1-8)", true);
	PRINT_MODULE_USAGE_PARAM_INT('d', 5, 2, 20, "Duration in s (2-20)", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('b', 0.1, 0.1, 0.6, "Signal min (0.1-0.6)", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('t', 0.4, 0.1, 0.4, "Signal max (0.1-0.6)", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("sweep_sine", "Outputs sinusoid chirp signal (changing frequency)");
	PRINT_MODULE_USAGE_PARAM_INT('m', 1, 1, 8, "Motor selected (1-8)", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('o', 0.4, 0.1, 0.9, "Offset (0.1-0.9)", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('a', 0.1, 0.0, 0.6, "Amplitude (0.1-0.6)", true);

	return 0;
}

extern "C" __EXPORT int dshot_controller_main(int argc, char *argv[])
{
	return DshotController::main(argc, argv);
}
