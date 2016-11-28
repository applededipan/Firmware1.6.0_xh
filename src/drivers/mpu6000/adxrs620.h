/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file .h
 *
 * Shared defines for the adxrs620 driver.
 */
 
#pragma once

#include <arch/stm32/chip.h>
#include <stm32.h>
#include <board_config.h>
 
 /*
 * Register accessors.
 * For now, no reason not to just use ADC1.
 */
#define REG(_reg)	(*(volatile uint32_t *)(STM32_ADC1_BASE + _reg))

#define rSR		    REG(STM32_ADC_SR_OFFSET)
#define rCR1		REG(STM32_ADC_CR1_OFFSET)
#define rCR2		REG(STM32_ADC_CR2_OFFSET)
#define rSMPR1		REG(STM32_ADC_SMPR1_OFFSET)
#define rSMPR2		REG(STM32_ADC_SMPR2_OFFSET)
#define rJOFR1		REG(STM32_ADC_JOFR1_OFFSET)
#define rJOFR2		REG(STM32_ADC_JOFR2_OFFSET)
#define rJOFR3		REG(STM32_ADC_JOFR3_OFFSET)
#define rJOFR4		REG(STM32_ADC_JOFR4_OFFSET)
#define rHTR		REG(STM32_ADC_HTR_OFFSET)
#define rLTR		REG(STM32_ADC_LTR_OFFSET)
#define rSQR1		REG(STM32_ADC_SQR1_OFFSET)
#define rSQR2		REG(STM32_ADC_SQR2_OFFSET)       //                2
#define rSQR3		REG(STM32_ADC_SQR3_OFFSET)       //                3
#define rJSQR		REG(STM32_ADC_JSQR_OFFSET)
#define rJDR1		REG(STM32_ADC_JDR1_OFFSET)
#define rJDR2		REG(STM32_ADC_JDR2_OFFSET)       //                2
#define rJDR3		REG(STM32_ADC_JDR3_OFFSET)       //                3
#define rJDR4		REG(STM32_ADC_JDR4_OFFSET)       //                4
#define rDR		REG(STM32_ADC_DR_OFFSET)
#ifdef STM32_ADC_CCR
# define rCCR		REG(STM32_ADC_CCR_OFFSET)
#endif

#define PX4_ANALOG_MAX_CHANNELS 16
#define ADXRS620_X_CHANNL      14
#define ADXRS620_Y_CHANNL				4
#define ADXRS620_Z_CHANNL    	 15
#define ADXRS620_SEN            0.006f

#define ADXRS620 620
#define HWV1_4

