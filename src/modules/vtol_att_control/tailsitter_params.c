/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file tailsitter_params.c
 * Parameters for vtol attitude controller.
 *
 * @author Roman Bapst <bapstroman@gmail.com>
 * @author David Vorsin     <davidvorsin@gmail.com>
 */

#include <systemlib/param/param.h>

/**
 * Duration of front transition phase 2
 *
 * Time in seconds it should take for the rotors to rotate forward completely from the point
 * when the plane has picked up enough airspeed and is ready to go into fixed wind mode.
 *
 * @unit s
 * @min 0.1
 * @max 5.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control

PARAM_DEFINE_FLOAT(VT_TRANS_P2_DUR, 0.5f);*/

/**
 * Back transition thrust
 *
 * Back transition thrust. 0 means disable this parameter.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_B_TRANS_THR, 0.0f); // apple 

/**
 * pitch angle to switch to FW. 
 *
 * pitch angle to switch to FW. 0 means disable this parameter and use default parameter
 *
 * @unit norm
 * @min 0.0
 * @max 90.0
 * @decimal 2
 * @increment 0.1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_PITCH_F_TRANS, 0.0f); // apple

/**
 * pitch angle to switch to MC 
 *
 * pitch angle to switch to MC. 0 means disable this parameter and use default parameter
 *
 * @unit norm
 * @min 0.0
 * @max 60.0
 * @decimal 2
 * @increment 0.1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_PITCH_B_TRANS, 0.0f); // apple

/**
 * The proportion of VTOL rudder output
 *
 * The proportion of VTOL rudder output. 0 means disable this parameter.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_FW_YAW_SCALE, 0.0f); 

/**
 * Throttle during front transition
 *
 * @unit norm
 * @min 0.0 
 * @max 1.0
 * @increment 0.1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_THR_TRANS_MAX, 0.25f);

/**
 * speed during back transition
 *
 * @unit norm
 * @min 0.0
 * @max 20.0
 * @increment 0.1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_BACK_VEL, 5.0f);

/**
 * descend during back transition
 *
 * @unit norm
 * @min 0.0
 * @max 20.0
 * @increment 0.1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_BACK_DESCEND, 3.0f);
