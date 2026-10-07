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

/** iFlight BLITZ Wing H743: pin data cross-checked against INAV and ArduPilot. */
#pragma once
#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <stm32_gpio.h>

#define BOARD_HAS_NBAT_V 1
#define BOARD_HAS_NBAT_I 1
#define GPIO_nLED_RED (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_SET | GPIO_PORTE | GPIO_PIN3)
#define GPIO_nLED_BLUE (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_SET | GPIO_PORTE | GPIO_PIN4)
#define BOARD_HAS_CONTROL_STATUS_LEDS 1
#define BOARD_OVERLOAD_LED LED_RED

#define PX4_ADC_GPIO GPIO_ADC123_INP10, GPIO_ADC123_INP11, GPIO_ADC12_INP4
#define ADC_BATTERY_VOLTAGE_CHANNEL 10
#define ADC_BATTERY_CURRENT_CHANNEL 11
#define ADC_AIRSPEED_VOLTAGE_CHANNEL 4
#define ADC_DP_V_DIV 2.0f
#define ADC_CHANNELS ((1 << ADC_BATTERY_VOLTAGE_CHANNEL) | (1 << ADC_BATTERY_CURRENT_CHANNEL) | (1 << ADC_AIRSPEED_VOLTAGE_CHANNEL))
#define BOARD_ADC_OPEN_CIRCUIT_V (5.6f)
#define BOARD_ADC_SERVO_VALID (1)
#define BOARD_ADC_BRICK1_VALID (1)

#define DIRECT_PWM_OUTPUT_CHANNELS 12
#define BOARD_NUM_IO_TIMERS 4
/* TIM3/4/5/15 are actuator outputs; TIM2 is the buzzer; TIM7 is DroneCAN. */
#define HRT_TIMER 8
#define HRT_TIMER_CHANNEL 3
#define TONE_ALARM_TIMER 2
#define TONE_ALARM_CHANNEL 1
#define GPIO_TONE_ALARM_IDLE (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN15)
#define GPIO_TONE_ALARM GPIO_TIM2_CH1OUT_2

/* PD10 enables VTX power when high. Default off; camera selection low. */
#define GPIO_VTX_POWER (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN10)
#define GPIO_CAMERA_SELECT (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN11)
#define GPIO_CAN1_SILENT (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN3)
#define UAVCAN_NUM_IFACES_RUNTIME 1

#define SDIO_SLOTNO 0
#define SDIO_MINOR 0
/* No published card-detect or VBUS GPIO. PA9 is UART1 TX, not VBUS. */
#define BOARD_USB_VBUS_SENSE_DISABLED 1
#define BOARD_DMA_ALLOC_POOL_SIZE 5120
#define BOARD_HAS_ON_RESET 1
#define BOARD_ENABLE_CONSOLE_BUFFER
#define FLASH_BASED_PARAMS
#define PX4_GPIO_INIT_LIST { PX4_ADC_GPIO, GPIO_TONE_ALARM_IDLE, GPIO_VTX_POWER, GPIO_CAMERA_SELECT, GPIO_CAN1_SILENT, GPIO_CAN1_TX, GPIO_CAN1_RX }

__BEGIN_DECLS
extern void stm32_spiinitialize(void);
extern void stm32_usbinitialize(void);
extern int stm32_sdio_initialize(void);
extern void board_peripheral_reset(int ms);
#include <px4_platform_common/board_common.h>
__END_DECLS
