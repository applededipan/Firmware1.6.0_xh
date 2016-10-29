/****************************************************************************
 *
 *   Copyright (c) 2015-2016 PX4 Development Team. All rights reserved.
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
 * @file camera_trigger.cpp
 *
 * External camera-IMU synchronisation and triggering via FMU auxiliary pins.
 *
 * Support for camera manipulation via PWM signal over servo pins.
 *
 * @author Mohammed Kabir <mhkabir98@gmail.com>
 * @author Kelly Steich <kelly.steich@wingtra.com>
 * @author Andreas Bircher <andreas@wingtra.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdbool.h>
#include <mathlib/mathlib.h>
#ifdef __PX4_NUTTX
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#endif 
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h> //added by dzp 2016/8/24
#include <uORB/topics/camera_feedback.h>         //added by dzp 2016/8/24 added a topic
#include <uORB/topics/vehicle_attitude.h>  //added by dzp 2016/9/12
#include <uORB/topics/airspeed.h>          //added by dzp 2016/9/12
#include <time.h>                          //
#include <sys/time.h>                      //
#include <poll.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>
#include <board_config.h>
#ifdef __PX4_NUTTX
#include "interfaces/src/pwm.h"
#include "interfaces/src/relay.h"
#endif
#define TRIGGER_PIN_DEFAULT 1

extern "C" __EXPORT int camera_trigger_main(int argc, char *argv[]);
int camera_trigger_reboot();

typedef enum {
	CAMERA_INTERFACE_MODE_NONE = 0,
	CAMERA_INTERFACE_MODE_RELAY,
	CAMERA_INTERFACE_MODE_SEAGULL_MAP2_PWM
} camera_interface_mode_t;

class CameraTrigger
{
public:
	/**
	 * Constructor
	 */
	CameraTrigger();

	/**
	 * Destructor, also kills task.
	 */
	~CameraTrigger();

	/**
	 * Set the trigger on / off
	 */
	void		control(bool on);

	/**
	 * Trigger just once
	 */
	void		shootOnce();

	/**
	 * Toggle keep camera alive functionality
	 */
	void		keepAlive(bool on);

	/**
	 * Toggle camera on/off functionality
	 */
	void        turnOnOff();

	/**
	 * Start the task.
	 */
	void		start();

	/**
	 * Stop the task.
	 */
	void		stop();

	/**
	 * Display info.
	 */
	void		info();

	/**
	 * Trigger one image
	 */
	void		test();

private:

	struct hrt_call		_engagecall;
	struct hrt_call		_disengagecall;
	struct hrt_call     _engage_turn_on_off_call;
	struct hrt_call     _disengage_turn_on_off_call;
	struct hrt_call		_keepalivecall_up;
	struct hrt_call		_keepalivecall_down;

	static struct work_s	_work;

	int 			_gpio_fd;

	int			_mode;
	float			_activation_time;
	float			_interval;
	float 			_distance;
	uint32_t 		_trigger_seq;
	bool			_trigger_enabled;
	math::Vector<2>		_last_shoot_position;
	bool			_valid_position;
	
	int			_params_sub;
	int			_vcommand_sub;
	int			_vlposition_sub;
	int         _vgposition_sub;       //added by dzp 2016/8/24
	int         _vattitude_sub;        //added by dzp 2016/9/12
	int         _airspeed_sub;
	int32_t  _lat;
	int32_t  _lng;
	float  _alt_msl;
	float  _alt_rel;
	float  _roll;
	float  _pitch;
	float  _yaw;
  float  _airspeed;
  uint64_t _gps_time_usec;

	orb_advert_t		_trigger_pub;
	orb_advert_t		_feedback_pub; //added by dzp 2016/8/24

	param_t			_p_mode;
	param_t			_p_activation_time;
	param_t			_p_interval;
	param_t			_p_distance;
	param_t			_p_pin;
	param_t			_p_interface;
#ifdef __PX4_NUTTX
	camera_interface_mode_t	_camera_interface_mode;
	CameraInterface		*_camera_interface;  ///< instance of camera interface
#else
	/**
	 * Trampoline to the worker task
	 */
	static void	task_main_trampoline(void *arg);
#endif
	/**
	 * Vehicle command handler
	 */
	static void	cycle_trampoline(void *arg);
	/**
	 * Fires trigger
	 */
	static void	engage(void *arg);
	/**
	 * Resets trigger
	 */
	static void	disengage(void *arg);
	/**
	 * Fires on/off
	 */
	static void engange_turn_on_off(void *arg);
	/**
	 * Resets  on/off
	 */
	static void disengage_turn_on_off(void *arg);
	/**
	 * Fires trigger
	 */
	static void	keep_alive_up(void *arg);
	/**
	 * Resets trigger
	 */
	static void	keep_alive_down(void *arg);
	/**
	 *	Update params 
	 */
	static void update_params(void *arg);


};

//struct work_s CameraTrigger::_work;

namespace camera_trigger
{

CameraTrigger	*g_camera_trigger;
}
#ifdef __PX4_NUTTX
struct work_s CameraTrigger::_work;
#else
CameraTrigger	*g_camera_trigger = nullptr;
#endif
CameraTrigger::CameraTrigger() :
	_engagecall {},
	_disengagecall {},
	_engage_turn_on_off_call {},
	_disengage_turn_on_off_call {},
	_keepalivecall_up {},
	_keepalivecall_down {},
	_gpio_fd(-1),
	_mode(0),
	_activation_time(0.5f /* ms */),
	_interval(100.0f /* ms */),
	_distance(25.0f /* m */),
	_trigger_seq(0),
	_trigger_enabled(false),
	_last_shoot_position(0.0f, 0.0f),
	_valid_position(false),
	_params_sub(-1),
	_vcommand_sub(-1),
	_vlposition_sub(-1),
	_vgposition_sub(-1),
	_vattitude_sub(-1),
	_airspeed_sub(-1),
	_lat(0),
	_lng(0),
	_alt_msl(0.0f),
	_alt_rel(0.0f),
	_roll(0.0f),
	_pitch(0.0f),
	_yaw(0.0f),
	_airspeed(0.0f),
	_gps_time_usec(0),
	_trigger_pub(nullptr),
#ifdef __PX4_NUTTX	
	_feedback_pub(nullptr),                             //added by dzp 2016/8/24
	_camera_interface_mode(CAMERA_INTERFACE_MODE_RELAY),
	_camera_interface(nullptr)
#else
	_feedback_pub(nullptr)
#endif
{
#ifdef __PX4_NUTTX
	//Initiate Camera interface basedon camera_interface_mode
	if (_camera_interface != nullptr) {
		delete(_camera_interface);
		/* set to zero to ensure parser is not used while not instantiated */
		_camera_interface = nullptr;
	}

	memset(&_work, 0, sizeof(_work));
#else
  	g_camera_trigger = this;
#endif
	// Parameters
	_p_interval = param_find("TRIG_INTERVAL");
	_p_distance = param_find("TRIG_DISTANCE");
	_p_activation_time = param_find("TRIG_ACT_TIME");
	_p_mode = param_find("TRIG_MODE");
	_p_interface = param_find("TRIG_INTERFACE");

	param_get(_p_activation_time, &_activation_time);
	param_get(_p_interval, &_interval);
	param_get(_p_distance, &_distance);
	param_get(_p_mode, &_mode);
#ifdef __PX4_NUTTX
	param_get(_p_interface, &_camera_interface_mode);

	switch (_camera_interface_mode) {
	case CAMERA_INTERFACE_MODE_RELAY:
		_camera_interface = new CameraInterfaceRelay;
		break;

	case CAMERA_INTERFACE_MODE_SEAGULL_MAP2_PWM:
		_camera_interface = new CameraInterfacePWM;
		break;

	default:
		break;
	}
#endif
	struct camera_trigger_s	report = {};

	_trigger_pub = orb_advertise(ORB_ID(camera_trigger), &report);
	
	struct camera_feedback_s	report2 = {};
	_feedback_pub = orb_advertise(ORB_ID(camera_feedback), &report2);
	
#ifndef __PX4_NUTTX	
	info();
#endif
}

CameraTrigger::~CameraTrigger()
{
#ifdef __PX4_NUTTX
	delete(_camera_interface);
#endif
	camera_trigger::g_camera_trigger = nullptr;
}

void
CameraTrigger::control(bool on)
{
	// always execute, even if already on
	// to reset timings if necessary

	if (on) {
		// schedule trigger on and off calls
		hrt_call_every(&_engagecall, 0, (_interval * 1000),
			       (hrt_callout)&CameraTrigger::engage, this);

		// schedule trigger on and off calls
		hrt_call_every(&_disengagecall, 0 + (_activation_time * 1000), (_interval * 1000),
			       (hrt_callout)&CameraTrigger::disengage, this);

	} else {
		// cancel all calls
		hrt_cancel(&_engagecall);
		hrt_cancel(&_disengagecall);
		// ensure that the pin is off
		hrt_call_after(&_disengagecall, 0,
			       (hrt_callout)&CameraTrigger::disengage, this);
	}

	_trigger_enabled = on;
}

void
CameraTrigger::keepAlive(bool on)
{
	if (on) {
		// schedule keep-alive up and down calls
		hrt_call_every(&_keepalivecall_up, 0, (60000 * 1000),
			       (hrt_callout)&CameraTrigger::keep_alive_up, this);

		// schedule keep-alive up and down calls
		hrt_call_every(&_keepalivecall_down, 0 + (30000 * 1000), (60000 * 1000),
			       (hrt_callout)&CameraTrigger::keep_alive_down, this);

	} else {
		// cancel all calls
		hrt_cancel(&_keepalivecall_up);
		hrt_cancel(&_keepalivecall_down);
	}

}

void
CameraTrigger::turnOnOff()
{
	// schedule trigger on and off calls
	hrt_call_after(&_engage_turn_on_off_call, 0,
		       (hrt_callout)&CameraTrigger::engange_turn_on_off, this);

	// schedule trigger on and off calls
	hrt_call_after(&_disengage_turn_on_off_call, 0 + (200 * 1000),
		       (hrt_callout)&CameraTrigger::disengage_turn_on_off, this);
}

void
CameraTrigger::shootOnce()
{
	// schedule trigger on and off calls
	hrt_call_after(&_engagecall, 0,
		       (hrt_callout)&CameraTrigger::engage, this);

	// schedule trigger on and off calls
	hrt_call_after(&_disengagecall, 0 + (_activation_time * 1000),
		       (hrt_callout)&CameraTrigger::disengage, this);
}

#ifndef __PX4_NUTTX
void
CameraTrigger::task_main_trampoline(void *arg)
{
	g_camera_trigger->cycle_trampoline(g_camera_trigger);
}
#endif

void
CameraTrigger::start()
{
	// enable immediate if configured that way
	if (_mode == 2) {
		control(true);
	}

	// Prevent camera from sleeping, if triggering is enabled
	if (_mode > 0 && _mode < 4) {
		turnOnOff();
		keepAlive(true);

	} else {
		keepAlive(false);
	}
#ifdef __PX4_NUTTX
	// start to monitor at high rate for trigger enable command
	work_queue(LPWORK, &_work, (worker_t)&CameraTrigger::cycle_trampoline, this, USEC2TICK(1));
#else
/* start the Camera_Trigger task */
	 int _task = px4_task_spawn_cmd("camera_trigger",
				   SCHED_DEFAULT,
				   SCHED_PRIORITY_DEFAULT,
				   1200,
				   (px4_main_t)&CameraTrigger::task_main_trampoline,
				   nullptr);

	if (_task < 0) {
		PX4_INFO("task start failed: %d", errno);
	}
#endif
}

void
CameraTrigger::stop()
{
#ifdef __PX4_NUTTX
	work_cancel(LPWORK, &_work);
#endif
	hrt_cancel(&_engagecall);
	hrt_cancel(&_disengagecall);
	hrt_cancel(&_engage_turn_on_off_call);
	hrt_cancel(&_disengage_turn_on_off_call);
	hrt_cancel(&_keepalivecall_up);
	hrt_cancel(&_keepalivecall_down);

	if (camera_trigger::g_camera_trigger != nullptr) {
		delete(camera_trigger::g_camera_trigger);
	}
}

void
CameraTrigger::test()
{
	struct vehicle_command_s cmd = {};
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL;
	cmd.param5 = 1.0f;

	orb_advert_t pub;
	pub = orb_advertise_queue(ORB_ID(vehicle_command), &cmd, vehicle_command_s::ORB_QUEUE_LENGTH);
	(void)orb_unadvertise(pub);
}

void
CameraTrigger::update_params(void *arg)
{
#ifndef __PX4_POSIX
	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);

	//PX4_INFO("camera trigger update params");
	param_get(trig->_p_activation_time, &trig->_activation_time);
	param_get(trig->_p_interval, &trig->_interval);
	param_get(trig->_p_distance, &trig->_distance);
	param_get(trig->_p_mode, &trig->_mode);
	param_get(trig->_p_interface, &trig->_camera_interface_mode);
	
	//PX4_INFO("activation_time:%f,interval_interval:%f,distance:%f,camera_interface_mode:%d,mode:%d",
																						//(double)trig->_activation_time,
																						//(double)trig->_interval,
																						//(double)trig->_distance,
																						// trig->_camera_interface_mode,trig->_mode);
	if (trig->_camera_interface_mode == 1) {
			int  pin_list = 0;
		  int  polarity = 0;
		
		  param_t _p_pin = param_find("TRIG_PINS");
			param_t _p_polarity = param_find("TRIG_POLARITY");
			param_get(_p_pin, &pin_list);
			param_get(_p_polarity, &polarity);
			//PX4_INFO("camera_interface: pin_list:%d,_polarity:%d",pin_list,	polarity);	
			trig->_camera_interface->setup(pin_list,polarity);																							
	}
	// need update trigger mode 
 
	if (trig->_mode == 2) {
		 	trig->control(true);
	} else {
		 trig->control(false);
	}
	
	// Prevent camera from sleeping, if triggering is enabled
	if (trig->_mode > 0 && trig->_mode < 4) {
		 trig->keepAlive(true);

	} else {
		 trig->keepAlive(false);
	}
#endif
}

void
CameraTrigger::cycle_trampoline(void *arg)
{

	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);

	if ( trig->_params_sub < 0) {
				trig->_params_sub = orb_subscribe(ORB_ID(parameter_update));
		}
		
	/* Parameter update */
	bool params_updated;
	orb_check(trig->_params_sub, &params_updated);
	
	if (params_updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), trig->_params_sub, &param_update); // XXX: is this actually necessary?

		update_params(trig);
	}
/*********************************************************************************/
	if (trig->_vgposition_sub < 0) {
		trig->_vgposition_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	}
	struct vehicle_global_position_s gpos;

	orb_copy(ORB_ID(vehicle_global_position), trig->_vgposition_sub, &gpos);
	
	/* set timestamp the instant before the trigger goes off */
	trig->_gps_time_usec = gpos.time_utc_usec;
//	printf("gps_usec:%llu \n",gpos.time_utc_usec);
	trig->_lat = gpos.lat*10000000;
	trig->_lng = gpos.lon*10000000;
	trig->_alt_msl = gpos.alt;
/*********************************************************************************/
	if (trig->_vattitude_sub < 0) {
			trig->_vattitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	}
	struct vehicle_attitude_s vatt;

	orb_copy(ORB_ID(vehicle_attitude), trig->_vattitude_sub, &vatt);

	trig->_roll = vatt.roll;
	trig->_pitch = vatt.pitch;
	trig->_yaw = vatt.yaw;
/*********************************************************************************/
	if (trig->_airspeed_sub < 0) {
				trig->_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
		}
	struct airspeed_s airs;

	orb_copy(ORB_ID(airspeed), trig->_airspeed_sub, &airs);

	trig->_airspeed = airs.true_airspeed_m_s;
/*********************************************************************************/
	if (trig->_vcommand_sub < 0) {
		trig->_vcommand_sub = orb_subscribe(ORB_ID(vehicle_command));
	}

	bool updated;
	orb_check(trig->_vcommand_sub, &updated);

	// while the trigger is inactive it has to be ready
	// to become active instantaneously
	int poll_interval_usec = 10000;
	#ifndef __PX4_NUTTX
	PX4_INFO("camera trigger thread start");
	while(trig->_mode){

		if (trig->_vgposition_sub < 0) {
				trig->_vgposition_sub = orb_subscribe(ORB_ID(vehicle_global_position));
		}
		orb_copy(ORB_ID(vehicle_global_position), trig->_vgposition_sub, &gpos);
		trig->_gps_time_usec = gpos.timestamp;
		//PX4_INFO("gps_usec:%llu",gpos.timestamp);
	
		/* set timestamp the instant before the trigger goes off */
	
		trig->_lat = gpos.lat*10000000;
		trig->_lng = gpos.lon*10000000;
		//PX4_INFO("lat:%llu,lng:%llu",trig->_lat,trig->_lng);
		orb_check(trig->_vcommand_sub, &updated);
#endif 

	if (trig->_mode < 3) {

		if (updated) {

			struct vehicle_command_s cmd;

			orb_copy(ORB_ID(vehicle_command), trig->_vcommand_sub, &cmd);

			if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL) {
				// Set trigger rate from command
				if (cmd.param2 > 0) {
					trig->_interval = cmd.param2;
					param_set(trig->_p_interval, &(trig->_interval));
				}

				if (cmd.param1 < 1.0f) {
					trig->control(false);

				} else if (cmd.param1 >= 1.0f) {
					trig->control(true);
					// while the trigger is active there is no
					// need to poll at a very high rate
					poll_interval_usec = 100000;
				}

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL) {
				if (cmd.param5 > 0) {
					// One-shot trigger, default 1 ms interval
					//trig->_interval = 1000;
					trig->control(true);
				}
			}
		}

	} else {

		// Set trigger based on covered distance
		if (trig->_vlposition_sub < 0) {
			trig->_vlposition_sub = orb_subscribe(ORB_ID(vehicle_local_position));
		}

		struct vehicle_local_position_s pos;

		orb_copy(ORB_ID(vehicle_local_position), trig->_vlposition_sub, &pos);
		trig->_alt_rel = pos.ref_alt;
		
		if (pos.xy_valid) {

			bool turning_on = false;

			if (updated && trig->_mode == 4) {

				// Check update from command
				struct vehicle_command_s cmd;
				orb_copy(ORB_ID(vehicle_command), trig->_vcommand_sub, &cmd);

				if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST) {
#ifndef __PX4_POSIX
					// Set trigger to disabled if the set distance is not positive
					if (cmd.param1 > 0.0f && !trig->_trigger_enabled) {
						trig->turnOnOff();
						trig->keepAlive(true);
						// Give the camera time to turn on, before starting to send trigger signals
						poll_interval_usec = 5000000;
						turning_on = true;

					} else if (cmd.param1 <= 0.0f && trig->_trigger_enabled) {
						hrt_cancel(&(trig->_engagecall));
						hrt_cancel(&(trig->_disengagecall));
						trig->keepAlive(false);
						trig->turnOnOff();
					}
#endif
					trig->_trigger_enabled = cmd.param1 > 0.0f;
					trig->_distance = cmd.param1;
				}
			}

			if ((trig->_trigger_enabled || trig->_mode < 4) && !turning_on) {

				// Initialize position if not done yet
				math::Vector<2> current_position(pos.x, pos.y);
				//printf("dis mode on\n");
				if (!trig->_valid_position) {
					// First time valid position, take first shot
					trig->_last_shoot_position = current_position;
					trig->_valid_position = pos.xy_valid;
					trig->shootOnce();
				}

				// Check that distance threshold is exceeded and the time between last shot is large enough
				if ((trig->_last_shoot_position - current_position).length() >= trig->_distance) {
					//trig->_last_shoot_position.print();
					trig->shootOnce();
					trig->_last_shoot_position = current_position;
				}
			}

		} else {
			poll_interval_usec = 100000;
		}
	}
#ifdef __PX4_NUTTX
	work_queue(LPWORK, &_work, (worker_t)&CameraTrigger::cycle_trampoline,
		   camera_trigger::g_camera_trigger, USEC2TICK(poll_interval_usec));
#else
	usleep(poll_interval_usec);
	}
#endif
}


void
CameraTrigger::engage(void *arg)
{
	
	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);

	struct camera_trigger_s	report = {};
	struct camera_feedback_s	report2 = {};
	
	/* set timestamp the instant before the trigger goes off */
	report.timestamp = hrt_absolute_time();
#ifdef __PX4_NUTTX
	trig->_camera_interface->trigger(true);
#endif
	report.seq = trig->_trigger_seq++;

	orb_publish(ORB_ID(camera_trigger), trig->_trigger_pub, &report);
	
	/*****************************************************************************************************************/

	uint64_t time_usec;
	uint64_t clock_usec;

  struct timespec ts;
  px4_clock_gettime(CLOCK_REALTIME, &ts);
  clock_usec = ts.tv_sec*1e6 + ts.tv_nsec/1e3 + 28800*1e6;
//    printf("_gps_time_usec:%llu \n",trig->_gps_time_usec);
#ifdef __PX4_NUTTX
  if((trig->_gps_time_usec/1e6) >= ((time_t)1234567890ULL)){
    	 if(trig->_gps_time_usec/1e6 - ts.tv_sec > 315360000){//10 years
    	    	return;
    	   }else{  time_usec = clock_usec;    }
  }else{  return;  }
#else
	time_usec = clock_usec;
#endif
/**********************************************************************************************************************/
	report2.timestamp = time_usec;
	report2.lat = trig->_lat;
	report2.lng = trig->_lng;
	report2.alt_msl = trig->_alt_msl;
	report2.alt_rel = trig->_alt_rel;
	report2.roll = trig->_roll;
	report2.pitch = trig->_pitch;
	report2.yaw = trig->_yaw;
	report2.foc_len = trig->_airspeed;
	
	if(!trig->_feedback_pub)
		trig->_feedback_pub = orb_advertise(ORB_ID(camera_feedback), &report2);
	else{ orb_publish(ORB_ID(camera_feedback), trig->_feedback_pub, &report2);	}
}

void
CameraTrigger::disengage(void *arg)
{
#ifdef __PX4_NUTTX
	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);

	trig->_camera_interface->trigger(false);
#endif
}

void
CameraTrigger::engange_turn_on_off(void *arg)
{

	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);

	trig->_camera_interface->turn_on_off(true);
}

void
CameraTrigger::disengage_turn_on_off(void *arg)
{
	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);

	trig->_camera_interface->turn_on_off(false);
}

void
CameraTrigger::keep_alive_up(void *arg)
{
#ifdef __PX4_NUTTX
	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);

	trig->_camera_interface->keep_alive(true);
#endif
}

void
CameraTrigger::keep_alive_down(void *arg)
{
#ifdef __PX4_NUTTX
	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);

	trig->_camera_interface->keep_alive(false);
#endif
}

void
CameraTrigger::info()
{
	PX4_INFO("state : %s", _trigger_enabled ? "enabled" : "disabled");
	PX4_INFO("mode : %i", _mode);
	PX4_INFO("interval : %.2f [ms]", (double)_interval);
	PX4_INFO("distance : %.2f [m]", (double)_distance);
	PX4_INFO("activation time : %.2f [ms]", (double)_activation_time);
#ifdef __PX4_NUTTX
	_camera_interface->info();
#endif
}

static int usage()
{
	PX4_ERR("usage: camera_trigger {start|stop|info|test}\n");
	return 1;
}

int camera_trigger_main(int argc, char *argv[])
{
	if (argc < 2) {
		return usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (camera_trigger::g_camera_trigger != nullptr) {
			PX4_WARN("already running");
			return 0;
		}

		camera_trigger::g_camera_trigger = new CameraTrigger();

		if (camera_trigger::g_camera_trigger == nullptr) {
			PX4_WARN("alloc failed");
			return 1;
		}

		camera_trigger::g_camera_trigger->start();
		return 0;
	}

	if (camera_trigger::g_camera_trigger == nullptr) {
		PX4_WARN("not running");
		return 1;

	} else if (!strcmp(argv[1], "stop")) {
		camera_trigger::g_camera_trigger->stop();

	} else if (!strcmp(argv[1], "info")) {
		camera_trigger::g_camera_trigger->info();

	} else if (!strcmp(argv[1], "enable")) {
		camera_trigger::g_camera_trigger->control(true);

	} else if (!strcmp(argv[1], "disable")) {
		camera_trigger::g_camera_trigger->control(false);

	} else if (!strcmp(argv[1], "test")) {
		camera_trigger::g_camera_trigger->test();

	} else {
		return usage();
	}

	return 0;
}
