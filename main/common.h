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

#ifndef COMMON_H
#define COMMON_H

#define MAX_SIZE 1000

#include "lwip/sockets.h"
#include <pthread.h>
#include "esp_pthread.h"


typedef struct {
	int listen_s, client_s[2], nclients, port;
	char ip[16];
	struct timeval tv;
	struct sockaddr_in server, client[2];
	bool connected;
}server;

typedef struct {
	bool start_image, data_image, end_image;
	int image_size;
}video_stream_packet_state;

typedef struct args {
	server sc;
}args;

typedef struct data_pack{
	void *data;
	size_t length;
}data_pack;

typedef struct {
	double roll;
	double pitch;
	double yaw;
	double throttle;
}Control_data;

Control_data controller;

extern bool start_capture;
extern pthread_mutex_t lock;
extern pthread_cond_t cond;

#endif /* COMMON_H */
