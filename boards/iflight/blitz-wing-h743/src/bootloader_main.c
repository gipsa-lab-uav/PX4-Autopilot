/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
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
 * @file bootloader_main.c
 *
 * BLITZ Wing H743 early startup code for bootloader
*/

#include "board_config.h"
#include "bl.h"

#include <nuttx/config.h>
#include <nuttx/board.h>
#include <chip.h>
#include <stm32_uart.h>
#include <arch/board/board.h>
#include "arm_internal.h"
#include <px4_platform_common/init.h>

extern int sercon_main(int c, char **argv);

__EXPORT void board_on_reset(int status)
{
	const uint32_t pins[] = {
		GPIO_PORTB | GPIO_PIN0,
		GPIO_PORTB | GPIO_PIN1,
		GPIO_PORTA | GPIO_PIN0,
		GPIO_PORTA | GPIO_PIN1,
		GPIO_PORTA | GPIO_PIN2,
		GPIO_PORTA | GPIO_PIN3,
		GPIO_PORTD | GPIO_PIN12,
		GPIO_PORTD | GPIO_PIN13,
		GPIO_PORTD | GPIO_PIN14,
		GPIO_PORTD | GPIO_PIN15,
		GPIO_PORTE | GPIO_PIN5,
		GPIO_PORTE | GPIO_PIN6,
	};

	for (unsigned i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
		px4_arch_configgpio(GPIO_INPUT | GPIO_PULLDOWN | pins[i]);
	}
}

__EXPORT void stm32_boardinitialize(void)
{
	board_on_reset(-1);

	/* configure USB interfaces */
	stm32_usbinitialize();
}

__EXPORT int board_app_initialize(uintptr_t arg)
{
	return 0;
}

void board_late_initialize(void)
{
	sercon_main(0, NULL);
}

extern void sys_tick_handler(void);
void board_timerhook(void)
{
	sys_tick_handler();
}
