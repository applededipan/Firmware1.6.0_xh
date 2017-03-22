/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>




#include <board_config.h>

#include <px4_adc.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <mathlib/mathlib.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_airspeed.h>
#include <drivers/drv_px4flow.h>

#include <systemlib/airspeed.h>
#include <systemlib/mavlink_log.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/battery.h>

#include <conversion/rotation.h>

#include <lib/ecl/validation/data_validator.h>
#include <lib/ecl/validation/data_validator_group.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/rc_parameter_map.h>
#include <uORB/topics/sensor_preflight.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>

#include <DevMgr.hpp>

#include "hybrid_main.h"

#define  engine_current_a_lim  0.5f

using namespace DriverFramework;

extern "C" __EXPORT int hybrid_main(int argc, char *argv[]);


Hybrid::Hybrid():
	_task_should_exit(true),
	_hybrid_task(-1),
	_battery_status_sub(0),
	_actuator_pub(nullptr),
	_armed_sub(-1),
	batt_v(0),
	batt_i(0)
{
	value_pwm = 1.0f;
	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	memset(&engine_state, 0, sizeof(engine_state));
}

Hybrid::~Hybrid()
{
	hybrid::g_Hybrid = nullptr;
}


void
Hybrid::arming_status_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
	}
}

void
Hybrid::task_main_trampoline(int argc, char *argv[])
{
	hybrid::g_Hybrid->task_main();
}

int Hybrid::start()
{
	ASSERT(_hybrid_task == -1);

	/* start the task*/
	_hybrid_task = px4_task_spawn_cmd("hybrid",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1700,
					   (px4_main_t)&Hybrid::task_main_trampoline,
					   nullptr);

	/* wait until the task is up and running or has failed */
	while (_hybrid_task > 0 && _task_should_exit) {
		usleep(100);
	}

	if (_hybrid_task < 0) {
		return -PX4_ERROR;
	}

	return OK;
}

void Hybrid::print_status()
{

}

int	Hybrid::adc_init()
{

	DevMgr::getHandle(ADC0_DEVICE_PATH, _h_adc);

	if (!_h_adc.isValid()) {
		PX4_ERR("no ADC found: %s (%d)", ADC0_DEVICE_PATH, _h_adc.getError());
		return PX4_ERROR;
	}

	return OK;
}

void Hybrid::engine_throttle_poll()
{

	static uint64_t last_run = 0;
	static float err_plus;
	float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
	float err = engine_state.engine_current_a - engine_current_a_lim;

	last_run = hrt_absolute_time();

	batt_orb_read(&batt_v, &batt_i);


	if (engine_state.engine_current_a < 0.01f) {
//		engine_state.engine_current_a += 0.1;
		engine_state.engine_throttle = engine_state.engine_idle_throttle;
	}

	err_plus += err;
	engine_state.engine_throttle += engine_state.engine_throttle_p * (err) + err_plus * dt;

	/*
	 * battery Voltage Current limit
	 */
	if (batt_v > 25.2f) {
		engine_state.engine_throttle -= 0.5f;
	}

	if (batt_i > 0.5f) {
		engine_state.engine_throttle -= 0.5f;
	}

	engine_state.engine_throttle += engine_state.engine_throttle_grad ;

	if (engine_state.ESC_EN) {
		engine_state.ESC_throttle += engine_state.ESC_throttle_grad ;

		engine_state.ESC_throttle += engine_state.engine_throttle_p * (err) + err_plus * dt;

	}

}

void Hybrid::engine_poll()
{

	if (_armed.armed && !engine_state.start_flag && !engine_state.start_already_flag) {
		engine_state.start_flag = 1;
		engine_state.ESC_EN = 1;
		engine_state.start_delay = 1000000;

		if (engine_state.engine_throttle < engine_state.engine_idle_throttle) {
			engine_state.engine_throttle = engine_state.engine_idle_throttle;
		}
	}

	if (engine_state.start_flag && (engine_state.engine_current_a > 0.1f)) {
		engine_state.start_already_flag = 1;
		engine_state.start_flag = 0;
		engine_state.ESC_throttle = 0; // ESC stop
		engine_state.ESC_EN = 0;

	} else if (engine_state.start_flag && !engine_state.start_already_flag) {
//		engine_state.engine_throttle += engine_state.engine_throttle_grad ;
//		if(engine_state.ESC_EN)
//			engine_state.ESC_throttle += engine_state.ESC_throttle_grad ;
		engine_throttle_poll();
	}

	if (engine_state.start_already_flag) {
		engine_throttle_poll();
	}

	static uint64_t last_run = 0;
	if (!_armed.armed && (engine_state.start_flag || engine_state.start_already_flag)) {
		if(hrt_absolute_time() - last_run > engine_state.stop_delay) {
			engine_state.ESC_throttle = 0; // ESC stop
			engine_state.engine_throttle = 0; // ESC stop
		}

	} else if (_armed.armed) {
		engine_state.stop_delay = 1000000;
		last_run = hrt_absolute_time();
	}
}


/**
 * Poll the ADC and update readings to suit.
 *
 * @param raw			Combined sensor data structure into which
 *				data should be returned.
 */
void Hybrid::adc_poll( )
{

//	hrt_abstime t = hrt_absolute_time();

	/* rate limit to 100 Hz */
//	if (t - _last_adc >= 10000)
	{
		/* make space for a maximum of twelve channels (to ensure reading all channels at once) */
		struct adc_msg_s buf_adc[12];
		/* read all channels available */
		int ret = _h_adc.read(&buf_adc, sizeof(buf_adc));

//		float bat_voltage_v = 0.0f;

		if (ret >= (int)sizeof(buf_adc[0])) {

			/* Read add channels we got */
			for (unsigned int i = 0; i < ret / sizeof(buf_adc[0]); i++) {

				/* look for specific channels and process the raw voltage to measurement data */
				if (ADC_HYBRID_CURRENT_CHANNEL == buf_adc[i].am_channel) {
					/* Voltage in volts */
					engine_state.engine_current_a = (buf_adc[i].am_data);

//					printf("bat_voltage_v  %.7f\n",(double)engine_state.engine_current_a);

				}
			}
		}
	}
}

void Hybrid::batt_orb_read(float *V,float *I)
{
	bool updated;

	if (_battery_status_sub) {
		orb_check(_battery_status_sub, &updated);

		if (updated) {
			struct battery_status_s	 _battery_status;	/**< battery status */
			orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
			*V = _battery_status.voltage_v ;
			*I = _battery_status.current_a ;
		}

	} else {
		_battery_status_sub = orb_subscribe(ORB_ID(battery_status));
	}

}
void Hybrid::task_main()
{
	_task_should_exit = false;
	printf("yes, this is hybrid_main.cpp\n");
	//	   return;

	float V, I;
	adc_init();
	while (!_task_should_exit) {
		/**
		 * read orb get arm state
		 * */
		arming_status_poll();
		/**
		 *  read orb get battery state
		 * */
		batt_orb_read(&V, &I);

		perf_begin(_loop_perf);
		/**
		 *  read engine currente state
		 *  */
		adc_poll();
		/**
		 * controll engine throttle and ESC throttle
		 * */
		engine_poll();

		/**
		 * orb publish to controll throttle
		 * */
		static  struct actuator_controls_s _actuators = {};
		_actuators.control[6] = engine_state.engine_throttle; // engine throttle
		_actuators.control[7] = engine_state.ESC_throttle;  // ESC throttle


		/**
		 *  for pwm output test
		 * */
		_actuators.control[6] = value_pwm;

		_actuators.control[7] = value_pwm;

//		if(fabsf(_actuators.control[6])>(float)value_pwm)
//			_actuators.control[6]=-1.0f;


		/*
		 * Range constraint
		 * */
		if (_actuators.control[6] < -1.0f) {
			_actuators.control[6] = -1.0f;

		} else if (_actuators.control[6] > 1.0f) {
			_actuators.control[6] = 1.0f;
		}

		if (_actuator_pub != nullptr) {
			_actuators.timestamp = hrt_absolute_time();
			orb_publish(ORB_ID(actuator_controls_2), _actuator_pub, &_actuators);

		} else {
			_actuator_pub = orb_advertise(ORB_ID(actuator_controls_2), &_actuators);
		}
//		printf("hybrid_main  %.7f   %.7f    %.7f\n", (double)V,(double)I,(double)_actuators.control[6]);
		perf_end(_loop_perf);
		PX4_INFO("hello, this is hybrid thread! \n");
		/* Thread sleep*/
		usleep(100000);
	}
}


int hybrid_main(int argc, char *argv[])
{
    PX4_INFO("Hello Hybrid!");

	if (argc < 2) {
		PX4_INFO("usage: hybrid {start|stop|status |value}");
		return 0;
	}

	if (!strcmp(argv[1], "start")) {

		if (hybrid::g_Hybrid != nullptr) {
			PX4_INFO("already running");
			return 0;
		}

		hybrid::g_Hybrid = new  Hybrid;

		if (hybrid::g_Hybrid == nullptr) {
			PX4_ERR("alloc failed");
			return 1;
		}

		if (OK != hybrid::g_Hybrid ->start()) {
			delete hybrid::g_Hybrid ;
			hybrid::g_Hybrid = nullptr;
			PX4_ERR("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (hybrid::g_Hybrid == nullptr) {
			PX4_INFO("not running");
			return 1;
		}

		hybrid::g_Hybrid->_task_should_exit = true;
		delete hybrid::g_Hybrid ;
		hybrid::g_Hybrid = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (hybrid::g_Hybrid) {
			hybrid::g_Hybrid->print_status();
			return 0;

		} else {
			PX4_INFO("not running");
			return 1;
		}
	}

	if (!strcmp(argv[1], "value")) {
		if (hybrid::g_Hybrid) {
			if (!strcmp(argv[2], "-1")) {
				hybrid::g_Hybrid->value_pwm = -1;

			} else if (!strcmp(argv[2], "0")) {
				hybrid::g_Hybrid->value_pwm = 0;

			} else if (!strcmp(argv[2], "1")) {
				hybrid::g_Hybrid->value_pwm = 1;

			} else if (!strcmp(argv[2], "-0.5")) {
				hybrid::g_Hybrid->value_pwm = -0.5;

			} else if (!strcmp(argv[2], "0.5")) {
				hybrid::g_Hybrid->value_pwm = 0.5;
			}
			return 0;

		} else {
			PX4_INFO("not running");
			return 1;
		}
	}

    return OK;
}
