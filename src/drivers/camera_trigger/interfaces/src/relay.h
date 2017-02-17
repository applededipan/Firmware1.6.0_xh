/**
 * @file relay.h
 *
 * Interface with cameras via FMU auxiliary pins.
 *
 */
#pragma once

#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <board_config.h>

#include "camera_interface.h"


class CameraInterfaceRelay : public CameraInterface
{
public:
	CameraInterfaceRelay();
	virtual ~CameraInterfaceRelay();

	void trigger(bool enable);

	void info();

	int _pins[6];
	int _polarity;

private:

	void setup();

	param_t _p_pin;
	param_t _p_polarity;

	static constexpr uint32_t _gpios[2] = {
		GPIO_GPIO14_OUTPUT,
		GPIO_GPIO15_OUTPUT
	};

};
