/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include "sensor_bridge.hpp"
#include <drivers/drv_mag.h>

#include <uavcan/equipment/ahrs/MagneticFieldStrength.hpp>

class UavcanMagnetometerBridge : public UavcanCDevSensorBridgeBase
{
public:
	static const char *const NAME;

	UavcanMagnetometerBridge(uavcan::INode &node);

	const char *get_name() const override { return NAME; }

	int init() override;

private:
	ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	int ioctl(struct file *filp, int cmd, unsigned long arg) override;
	/**
	 * Check the current calibration and update device status
	 *
	 * @return 0 if calibration is ok, 1 else
	 */
	int 			check_calibration();

	/**
	* Check the current scale calibration
	*
	* @return 0 if scale calibration is ok, 1 else
	*/
	int 			check_scale();

	/**
	* Check the current offset calibration
	*
	* @return 0 if offset calibration is ok, 1 else
	*/
	int 			check_offset();
	bool			_calibrated;		/**< the calibration is valid */
	
	uint32_t  mag_use_id;

	void mag_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::MagneticFieldStrength> &msg);

	typedef uavcan::MethodBinder < UavcanMagnetometerBridge *,
		void (UavcanMagnetometerBridge::*)
		(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::MagneticFieldStrength> &) >
		MagCbBinder;

	uavcan::Subscriber<uavcan::equipment::ahrs::MagneticFieldStrength, MagCbBinder> _sub_mag;
	struct mag_calibration_s _scale = {};
	mag_report _report =  {};
};
