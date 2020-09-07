/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "servo_simple.h"
#include "ch.h"
#include "hal.h"
#include "hw.h"
#include "conf_general.h"
#include "stm32f4xx_conf.h"
#include "utils.h"
#include "commands.h"
#include "terminal.h"
#include <stdio.h>

// Settings
#define TIM_CLOCK			1000000 // Hz

#if SERVO_OUT_ENABLE

static void terminal_servo_write(int argc, const char **argv) {
	if (argc == 3) {
		int val = -1;
		int idx = -1;
		sscanf(argv[1], "%d", &idx);
		sscanf(argv[2], "%d", &val);
		servo_simple_set_output(idx, 0.001f * val);
	} else {
		commands_printf("This command requires 2 arguments.\n");
	}
}

void servo_simple_init(void) {
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN,
		PAL_MODE_ALTERNATE(HW_ICU_GPIO_AF) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 5, // SERVO2@PA5: TIM2/CH1
		PAL_MODE_ALTERNATE(GPIO_AF_TIM2) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);

	HW_ICU_TIM_CLK_EN();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = (uint16_t)((uint32_t)TIM_CLOCK / (uint32_t)SERVO_OUT_RATE_HZ);
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)((168000000 / 2) / TIM_CLOCK) - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(HW_ICU_TIMER, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	if (HW_ICU_CHANNEL == ICU_CHANNEL_1) {
		TIM_OC1Init(HW_ICU_TIMER, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(HW_ICU_TIMER, TIM_OCPreload_Enable);
	} else if (HW_ICU_CHANNEL == ICU_CHANNEL_2) {
		TIM_OC2Init(HW_ICU_TIMER, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(HW_ICU_TIMER, TIM_OCPreload_Enable);
	}
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(HW_ICU_TIMER, ENABLE);
	TIM_ARRPreloadConfig(TIM2, ENABLE);

	servo_simple_set_output(0, 0.5);
	servo_simple_set_output(1, 0.5);
	
	TIM_Cmd(HW_ICU_TIMER, ENABLE);
	TIM_Cmd(TIM2, ENABLE);

	terminal_register_command_callback(
		"servo",
		"Set servo duty",
		"{0|1} [0,1000]",
		terminal_servo_write);
}

void servo_simple_set_output(int idx, float out) {
	utils_truncate_number(&out, 0.0, 1.0);

	float us = (float)SERVO_OUT_PULSE_MIN_US + out *
			(float)(SERVO_OUT_PULSE_MAX_US - SERVO_OUT_PULSE_MIN_US);
	us *= (float)TIM_CLOCK / 1000000.0;

	switch (idx) {
	case 0: // the original servo output
		if (HW_ICU_CHANNEL == ICU_CHANNEL_1) {
			HW_ICU_TIMER->CCR1 = (uint32_t)us;
		} else if (HW_ICU_CHANNEL == ICU_CHANNEL_2) {
			HW_ICU_TIMER->CCR2 = (uint32_t)us;
		}
		break;

	case 1: TIM2->CCR1 = (uint32_t)us; break;
	default: break;
	}
}

#endif
