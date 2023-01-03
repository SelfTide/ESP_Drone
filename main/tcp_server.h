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

#ifndef TCP_SERVER
#define TCP_SERVER

#include <sdkconfig.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <sys/param.h>
#include <sys/select.h>

#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_event.h"

#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "common.h"
#include "interface_spi.h"
#include "FIFO_stack_tool.h"

#define NUMELEMS(array) (sizeof(array) / sizeof(array[0]))

#define TCP_QUICKACK   12

int tcp_server_init (server *sc);
void tcp_server_run(void *arg);

#endif
