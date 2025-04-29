#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/gptimer.h>
#include <esp_log.h>

static const char *TAG = "AlarmExample";

typedef struct {
    uint64_t event_count;
} alarm_event_t;


// Callback de la alarma
static bool IRAM_ATTR alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_data;

    static uint64_t alarm_count = 0;

    alarm_event_t evt = {
        .event_count = ++alarm_count,
    };

    xQueueSendFromISR(queue, &evt, &high_task_awoken);
    return high_task_awoken == pdTRUE;
}

void app_main(void)
{

    alarm_event_t evt;
    QueueHandle_t queue = xQueueCreate(10, sizeof(alarm_event_t));
    
    if (queue == NULL) {
        ESP_LOGE(TAG, "Fallo al crear la cola");
        return;
    }
    ESP_LOGI(TAG, "Cola creada exitosamente");

    // Configurar temporizador
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,   // 1 MHz - 1 microsegundo por tick
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&config, &gptimer));

    // Configurar callback de alarma
    gptimer_event_callbacks_t cbs = {
        .on_alarm = alarm_cb,
    };

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, queue));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    // Configurar alarma de 1 segundo con autoreload
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 1000000,           // Dispara cuando llega a 1s
        .reload_count = 0,                // Recarga desde 0 cada vez
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

    // iniciar temporizador
    
    ESP_ERROR_CHECK(gptimer_start(gptimer));
    ESP_LOGI(TAG, "Temporizador iniciado exitosamente");



    while (true) {
        if (xQueueReceive(queue, &evt, pdMS_TO_TICKS(2000))) {
            ESP_LOGI(TAG, "Alarma: Conteo = %llu", evt.event_count);
        }
    }
}