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

#include "ArduCam.h"
#include "ov2640_regs.h"

static const char *TAG = "ArduCAM";
extern ArduCAM_OV2640_C *myCAM;

static int64_t eclipse_time_ms(bool startend)
{
    enum
    {
        START = 0,
        END,
    };
    static struct timeval tik, tok;
    if (startend == START)
    {
        gettimeofday(&tik, NULL);
        return 0;
    }
    else
    {
        gettimeofday(&tok, NULL);
        int64_t time_ms = (int64_t)(tok.tv_sec - tik.tv_sec) * 1000 + (tok.tv_usec - tik.tv_usec) / 1000;
        return time_ms;
    }
}

void OV2640_valid_check()
{
	//Check if the ArduCAM SPI bus is OK
	ArduCAM_write_reg(ARDUCHIP_TEST1, 0x55);
	uint8_t temp = ArduCAM_read_reg(ARDUCHIP_TEST1);
	if (temp != 0x55)
		ESP_LOGE(TAG, "SPI: interface Error 0x%2x!\n", temp);
	else
		ESP_LOGI(TAG, "SPI: interface OK: %x\n", temp);

	//Check if the camera module type is OV2640
	uint8_t vid, pid;
	ArduCAM_wrSensorReg8_8(0xff, 0x01);
	ArduCAM_rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
	ArduCAM_rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
	if ((vid != 0x26) && ((pid != 0x41) || (pid != 0x42)))
		ESP_LOGE(TAG, "I2C: Can't find OV2640 module!\n");
	else
		ESP_LOGI(TAG, "I2C: OV2640 detected.\n");
}

void capture_one_frame()
{
	ArduCAM_flush_fifo();
	ArduCAM_clear_fifo_flag();
	ArduCAM_start_capture();
	
	eclipse_time_ms(false);
	while (!(ArduCAM_get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))) taskYIELD();
	//		 vTaskDelay(pdMS_TO_TICKS(2));
	ESP_LOGI(TAG, "capture total_time used (in miliseconds): %lld\n", eclipse_time_ms(true));

	/*
	// test to see if this will be quicker -> https://github.com/ArduCAM/Arduino/issues/108
	// select register bank 1
	ArduCAM_wrSensorReg8_8(0xff, 0x01);

	// enable video mode (Register COM3)
	uint8_t temp = 0;
	ArduCAM_rdSensorReg8_8(0x0C, &temp);
	temp = temp & (~0x01);
	ArduCAM_wrSensorReg8_8(0x0C, temp);

	// wait till the OV2640 starts to output the frame
	ArduCAM_CS_LOW(myCAM);		
	while(!ArduCAM_get_bit(ARDUCHIP_TRIG , VSYNC_MASK));
	ArduCAM_CS_HIGH(myCAM);

	// disable video mode (Register COM3)
	ArduCAM_rdSensorReg8_8(0x0C, &temp);
	temp = temp | 0x01;
	ArduCAM_wrSensorReg8_8(0x0C, temp); 
	*/
}

void read_frame_buffer(void)
{
	video_stream_packet_state *vsps;
	data_pack *dpack;
	uint8_t *buffer;
	uint8_t temp_count = 0;
	size_t len = 0, batch_num = 0, tail_len = 0;

	// ESP_LOGI(TAG, "start_capture = %s\t", start_capture ? "true" : "false");
	// ESP_LOGI(TAG, "stack_count = %d\t", stack_count);
	
	if(stack_count < 5)
	{
		// get size of image buffer
		len = ArduCAM_read_fifo_length(myCAM);
		batch_num = len / SPI_MAX_TRANS_SIZE;
		tail_len = len % SPI_MAX_TRANS_SIZE;

		// ESP_LOGI(TAG, "in read_frame_buffer\n");

		if (len >= MAX_FIFO_SIZE)
		{
			ESP_LOGE(TAG, "Over size.\n");
			return;
		}else if (len == 0){
			ESP_LOGE(TAG, "Size is 0.\n");
			return;
		}
		 
		// send start
		vsps = (video_stream_packet_state *)calloc(1, sizeof(video_stream_packet_state));
		vsps->start_image = true, vsps->data_image = false, vsps->end_image = false;
		vsps->image_size = len;
			
		dpack = (data_pack *)calloc(1, sizeof(data_pack));
		dpack->data = vsps;
		dpack->length = sizeof(video_stream_packet_state);
		
		push_stack(dpack);
		temp_count++;

		// send data
		vsps = (video_stream_packet_state *)calloc(1, sizeof(video_stream_packet_state));
		vsps->start_image = false, vsps->data_image = true, vsps->end_image = false;
		vsps->image_size = len;
		
		dpack = (data_pack *)calloc(1, sizeof(data_pack));
		dpack->data = vsps;
		dpack->length = sizeof(video_stream_packet_state);
		
		push_stack(dpack);
		temp_count++;
		
		// read in bytes from camera till done, also make sure we don't have a small image
		for (int i = 0; i < batch_num && batch_num > 0; i++)
		{
			dpack = (data_pack *)calloc(1, sizeof(data_pack));
			buffer = (uint8_t *)calloc(SPI_MAX_TRANS_SIZE, sizeof(uint8_t));	// allocate buffer

			ArduCAM_CS_LOW(myCAM);
			dpack->length = spi_transfer_bytes(BURST_FIFO_READ, buffer, buffer, SPI_MAX_TRANS_SIZE);	// read in bytes to buffer
			ArduCAM_CS_HIGH(myCAM);
/*
			ESP_LOGI(TAG, "4 Byte header");
			ESP_LOGI(TAG, "Byte[0]: %x", buffer[0]);
			ESP_LOGI(TAG, "Byte[1]: %x", buffer[1]);
			ESP_LOGI(TAG, "Byte[2]: %x", buffer[2]);
			ESP_LOGI(TAG, "Byte[3]: %x", buffer[3]);
			ESP_LOGI(TAG, "Burst size: %d", dpack->length);
 */			
			dpack->data = buffer;
			
			push_stack(dpack);
			temp_count++;
		}
		
		// read in leftover or if length is under SPI_MAX_TRANS_SIZE
		if (tail_len != 0)	
		{
			dpack = (data_pack *)calloc(1, sizeof(data_pack));
			buffer = (uint8_t *)calloc(tail_len, sizeof(uint8_t));
			ArduCAM_CS_LOW(myCAM);
			dpack->length = spi_transfer_bytes(BURST_FIFO_READ, buffer, buffer, tail_len);	// read in bytes to buffer
			ArduCAM_CS_HIGH(myCAM);
			ESP_LOGI(TAG, "4 Byte header");
			ESP_LOGI(TAG, "Byte[0]: %x", buffer[0]);
			ESP_LOGI(TAG, "Byte[1]: %x", buffer[1]);
			ESP_LOGI(TAG, "Byte[2]: %x", buffer[2]);
			ESP_LOGI(TAG, "Byte[3]: %x", buffer[3]);
			ESP_LOGI(TAG, "Burst size: %d", dpack->length);
			dpack->data = buffer;
			
			push_stack(dpack);
			temp_count++;
		}
		 
		// send end
		vsps = (video_stream_packet_state *)calloc(1, sizeof(video_stream_packet_state));
		vsps->start_image = false, vsps->data_image = false, vsps->end_image = true;
		vsps->image_size = 0;
		
		dpack = (data_pack *)calloc(1, sizeof(data_pack));
		dpack->data = vsps;
		dpack->length = sizeof(video_stream_packet_state);
		
		push_stack(dpack);
		temp_count++;
		
		while(pthread_mutex_lock(&lock) != 0) taskYIELD();
		stack_count += temp_count;
		pthread_mutex_unlock(&lock);
		
	}
}

void take_image(void *arg)
{
	ESP_LOGI(TAG, "In take_image()");

	while(1)
	{	
		if(start_capture)
		{
			// ESP_LOGI(TAG, "getting image...\n");
			capture_one_frame();
			read_frame_buffer();
		}else{
			vTaskDelay(pdMS_TO_TICKS(10));
		}
	}
}
