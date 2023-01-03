/*
 *		MIT License
 *
 *	Copyright (c) 2022 SelfTide
 *
 *	Permission is hereby granted, free of charge, to any person obtaining a copy of this
 *	software and associated documentation files (the "Software"), to deal in the Software
 *	without restriction, including without limitation the rights to use, copy, modify, merge,
 *	publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons
 *	to whom the Software is furnished to do so, subject to the following conditions:
 *
 *	The above copyright notice and this permission notice shall be included in all copies or
 *	substantial portions of the Software.
 *
 *	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 *	BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *	NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 *	DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef MAIN_ARDUCAM_H_
#define MAIN_ARDUCAM_H_

#include <stdio.h>
#include "stdlib.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "time.h"
#include <sys/time.h>
#include "ArduCAM_esp_ov2640.h"
#include "mbedtls/base64.h"
#include "esp_heap_caps.h"

#include "interface_i2c.h"
#include "interface_spi.h"

#include "esp_log.h"

#include "common.h"
#include "FIFO_stack_tool.h"

#define BASE64_BUF_SIZE ((SPI_MAX_TRANS_SIZE / 3 + 2) * 4)
#define CS_PIN GPIO_NUM_5


void OV2640_valid_check();
void take_image(void *arg);

#endif /* MAIN_ARDUCAM_H_ */
