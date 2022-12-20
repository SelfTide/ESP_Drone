#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "freertos/task.h"
#include "Motor_control.h"

extern "C" {
	void app_main(void);
}

void app_main(void)
{
    xTaskCreate(&motor_control_init, "motor_control_init", 2048, NULL, 5, NULL);
    vTaskDelay(500/portTICK_PERIOD_MS);
    xTaskCreate(&run_motor_control, "disp_task", 8192, NULL, 5, NULL);
}
