

#pragma once
#include <DevMgr.hpp>

#include "hybrid_main.h"
using namespace DriverFramework;

class Hybrid
{
public:
	/**
	 * Constructor
	 */
	Hybrid();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~Hybrid();

	/**
	 * Start the sensors task.
	 *
	 * @return		OK on success.
	 */
	int		start();


	void	print_status();


	FILE *f_fp;
	bool test_flag=0,   write_file=0;
	float value_pwm;
	bool _task_should_exit;

private:

	struct engine
	{
		bool start_flag;
		bool start_already_flag;

		bool stop_flag;
		bool stop_already_flag;

		int start_delay;
		int stop_delay;

		float engine_throttle;
		float engine_throttle_p,engine_throttle_i;
		float engine_throttle_grad;

		bool ESC_EN;
		float ESC_throttle;
		float ESC_throttle_grad;

		float engine_current_a ;

		int engine_idle_throttle;

	}engine_state;

	int _hybrid_task;
	perf_counter_t	_loop_perf;			/**< loop performance counter */
	int 	_battery_status_sub;	/**< battery status subscription */
	DevHandle 	_h_adc;				/**< ADC driver handle */
	orb_advert_t	_actuator_pub;
	int		_armed_sub;				/**< arming status subscription */

	float batt_v,batt_i;
	struct actuator_armed_s				_armed;				/**< actuator arming status */

	/**
	 * Do adc-related initialisation.
	 */
	int		adc_init();


	/**
	 * Poll the ADC and update readings to suit.
	 *
	 * @param raw			Combined sensor data structure into which
	 *				data should be returned.
	 */
	void		adc_poll( );
	void		engine_poll( );
	void		engine_throttle_poll( );

	void  batt_orb_read(float *V,float *I);

	static void task_main_trampoline(int argc, char *argv[]);

	void arming_status_poll();

	void		task_main();
};



namespace hybrid
{
Hybrid	*g_Hybrid = nullptr;
}
