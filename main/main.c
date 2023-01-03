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

#include "network_wifi.h"
#include "tcp_server.h"
#include "ArduCam.h"
#include "common.h"
#include "Motor_control.h"

extern ArduCAM_OV2640_C *myCAM;

bool start_capture;
pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

// run communication and control functions
void cc_run(void *args)
{	
	while(1)
	{
		tcp_server_run(args);

		motor_control_run();
	}
}

void app_main(void)
{
	args *thread_args = (args *)malloc(sizeof(args ));

	start_capture = false;
	
/*
 *	Initialize FIFO_stack, ArduCAM, pthread, wifi, i2c, spi, motor_control and tcp_server
 */

	myCAM = (ArduCAM_OV2640_C *)ArduCAM_OV2640_init(CS_PIN);

	stack_init();
	assert(pthread_mutex_init(&lock, NULL) == 0);
	assert(pthread_cond_init(&cond, NULL) == 0);

	ESP_ERROR_CHECK(start_wifi());
	ESP_ERROR_CHECK(i2c_master_init());
	ESP_ERROR_CHECK(spi_master_init());
	
	motor_control_init();

	ESP_LOGI("MAIN", "Size of vsps: %d", sizeof(video_stream_packet_state));

	ArduCAM_write_reg(0x07, 0x80);
	vTaskDelay(pdMS_TO_TICKS(100));
	ArduCAM_write_reg(0x07, 0x00);
	vTaskDelay(pdMS_TO_TICKS(100));

	OV2640_valid_check();
	ESP_LOGI("ArduCAM", "Initializing Camera...\n");	// setup camera for use, these are important
	ArduCAM_set_format(JPEG);
	ArduCAM_OV2640_InitCAM(myCAM);
	ArduCAM_OV2640_set_JPEG_size(OV2640_320x240);
	// ArduCAM_set_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
	// ArduCAM_clear_fifo_flag(myCAM);
	// ArduCAM_write_reg(ARDUCHIP_FRAMES, FRAMES_NUM);

	ArduCAM_clear_fifo_flag(myCAM);
	vTaskDelay(pdMS_TO_TICKS(1000));
	ArduCAM_clear_fifo_flag(myCAM);
	vTaskDelay(pdMS_TO_TICKS(1000));

	if(tcp_server_init(&thread_args->sc) < 0)
		printf("Error starting sever!!!\n");

/*
 *	Start process threads for imaging, communications and control
 */
	xTaskCreatePinnedToCore(take_image, "take_image", 8192, NULL, tskIDLE_PRIORITY+3, NULL, 1);
	xTaskCreatePinnedToCore(cc_run, "tcp_server_run", 8192, (void *)thread_args, tskIDLE_PRIORITY+3, NULL, 0);
}
