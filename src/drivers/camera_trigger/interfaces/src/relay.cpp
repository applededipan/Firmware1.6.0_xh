#ifdef __PX4_NUTTX
#include "relay.h"

constexpr uint32_t CameraInterfaceRelay::_gpios[2];

CameraInterfaceRelay::CameraInterfaceRelay():
	CameraInterface(),
	_pins{},
	_polarity(0)
{
	_p_pin = param_find("TRIG_PINS");
	_p_polarity = param_find("TRIG_POLARITY");

	int pin_list;
	param_get(_p_pin, &pin_list);
	param_get(_p_polarity, &_polarity);

	// Set all pins as invalid
	for (unsigned i = 0; i < sizeof(_pins) / sizeof(_pins[0]); i++) {
		_pins[i] = -1;
	}

	// Convert number to individual channels
	unsigned i = 0;
	int single_pin;

	while ((single_pin = pin_list % 10)) {

		_pins[i] = single_pin - 1;

		if (_pins[i] < 0 || _pins[i] >= static_cast<int>(sizeof(_gpios) / sizeof(_gpios[0]))) {
			_pins[i] = -1;
		}

		pin_list /= 10;
		i++;
	}

	setup(1234,1234);
}

CameraInterfaceRelay::~CameraInterfaceRelay()
{
}

void CameraInterfaceRelay::setup(int pins, int polarity)
{
	if (pins != 1234 && polarity != 1234) {
			unsigned i = 0;
			int single_pin;
			_polarity = polarity;
			// Set all pins as invalid
			for ( i = 0; i < sizeof(_pins) / sizeof(_pins[0]); i++) {
				_pins[i] = -1;
			}
			i = 0;
			while ((single_pin = pins % 10)) {
				_pins[i] = single_pin - 1;
				printf("change: pins[%d]:%d\n",i,_pins[i]);
			pins /= 10;
			i++;
		}
	} else {
			for (unsigned i = 0; i < sizeof(_pins) / sizeof(_pins[0]); i++) {
				px4_arch_configgpio(_gpios[_pins[i]]);
				px4_arch_gpiowrite(_gpios[_pins[i]], !_polarity);
			}
	}
}

void CameraInterfaceRelay::trigger(bool enable)
{
	if (enable) {
		for (unsigned i = 0; i < sizeof(_pins) / sizeof(_pins[0]); i++) {
			if (_pins[i] >= 0) {
				// ACTIVE_LOW == 1
				px4_arch_gpiowrite(_gpios[_pins[i]], _polarity);
			}
		}

	} else {
		for (unsigned i = 0; i < sizeof(_pins) / sizeof(_pins[0]); i++) {
			if (_pins[i] >= 0) {
				// ACTIVE_LOW == 1
				px4_arch_gpiowrite(_gpios[_pins[i]], !_polarity);
			}
		}
	}
}

void CameraInterfaceRelay::info()
{
	warnx("Relay - camera triggering, pins 1-2 : %d,%d, polarity : %s", _pins[0], _pins[1],
	      _polarity ? "ACTIVE_HIGH" : "ACTIVE_LOW");
}
#endif