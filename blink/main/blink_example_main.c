#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char* TAG = "uart_task";
#define UART_NUM    UART_NUM_0
#define BUF_SIZE    256

/**
 * @brief Tarea que lee de UART y hace eco de lo recibido.
 */
static void uart_task(void* arg)
{
    uint8_t buf[BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(UART_NUM, buf, BUF_SIZE - 1, pdMS_TO_TICKS(1000));
        if (len > 0) {
            buf[len] = '\0';
            printf("Eco UART: %s", (char*)buf);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
    // 1) Configurar UART0
    uart_config_t uart_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));

    ESP_LOGI(TAG, "Creando tarea de UART...");
    // 2) Crear la tarea de UART con prioridad 5 y stack de 2048 bytes
    xTaskCreate(uart_task, "uart_task", 2048, NULL, 5, NULL);

    // 3) Aquí puedes inicializar más cosas o lanzar otras tareas...
    //    app_main no debe bloquearse (ni usar while(1) aquí).
}
