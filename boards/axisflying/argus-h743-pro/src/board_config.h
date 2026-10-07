/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file board_config.h
 * AxisFlying Argus H743 Pro, 4-8S, dual ICM42688P.
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <stm32_gpio.h>

#define BOARD_HAS_NBAT_V 1
#define BOARD_HAS_NBAT_I 1

#define GPIO_nLED_RED (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN3)
#define BOARD_HAS_CONTROL_STATUS_LEDS 1
#define BOARD_OVERLOAD_LED LED_RED

/* PC0/PC1 are ADC1 inputs 10/11. */
#define PX4_ADC_GPIO GPIO_ADC123_INP10, GPIO_ADC123_INP11
#define ADC_BATTERY_VOLTAGE_CHANNEL 10
#define ADC_BATTERY_CURRENT_CHANNEL 11
#define ADC_CHANNELS ((1 << ADC_BATTERY_VOLTAGE_CHANNEL) | (1 << ADC_BATTERY_CURRENT_CHANNEL))
#define BOARD_ADC_OPEN_CIRCUIT_V (5.6f)
#define BOARD_ADC_SERVO_VALID (1)
#define BOARD_ADC_BRICK1_VALID (1)

#define DIRECT_PWM_OUTPUT_CHANNELS 8
#define BOARD_NUM_IO_TIMERS 2

/* TIM5 is independent of TIM8/TIM2 motor outputs and TIM3 gyro clocks. */
#define HRT_TIMER 5
#define HRT_TIMER_CHANNEL 1

/* PC13 drives an active buzzer; it cannot generate pitched tones. */
#define GPIO_TONE_ALARM_IDLE (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN13)
#define GPIO_TONE_ALARM_GPIO (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_SET | GPIO_PORTC | GPIO_PIN13)

/* Leave switchable VTX power off and select camera 1 at startup. */
#define GPIO_VTX_POWER (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_SET | GPIO_PORTE | GPIO_PIN2)
#define GPIO_CAMERA_SELECT (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN8)

/* The pin map does not specify VBUS sensing. USB CDC is always made available.
 * The bootloader's common deinit requires a GPIO definition; PA9 is unused in
 * the published pin map, and is never treated as a cable detector.
 */
#define BOARD_USB_VBUS_SENSE_DISABLED 1
#define BOARD_DMA_ALLOC_POOL_SIZE 5120
#define BOARD_HAS_ON_RESET 1
#define BOARD_ENABLE_CONSOLE_BUFFER
#define FLASH_BASED_PARAMS

#define PX4_GPIO_INIT_LIST { PX4_ADC_GPIO, GPIO_TONE_ALARM_IDLE, GPIO_VTX_POWER, GPIO_CAMERA_SELECT }

__BEGIN_DECLS
extern void stm32_spiinitialize(void);
extern void stm32_usbinitialize(void);
extern void board_peripheral_reset(int ms);
#include <px4_platform_common/board_common.h>
__END_DECLS
