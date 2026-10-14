#include "time_sync.h"
#include "esp_netif_sntp.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <time.h>

static const char *TAG = "sntp";

#define SNTP_SERVER     "pool.ntp.br"
#define SYNC_WAIT_MS    10000
#define TIME_VALID_MIN  1700000000

esp_err_t time_sync_init(void)
{
    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG(SNTP_SERVER);
    esp_err_t ret = esp_netif_sntp_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao iniciar SNTP: %s", esp_err_to_name(ret));
        goto err;
    }

    ret = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(SYNC_WAIT_MS));
    if (ret != ESP_OK) {
        /* Não é fatal: o cliente continua tentando e o histórico espera */
        ESP_LOGW(TAG, "Relogio ainda nao sincronizado (%s)", esp_err_to_name(ret));
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Relogio sincronizado (%s): %lld", SNTP_SERVER, (long long)time(NULL));
    return ESP_OK;

err:
    return ret;
}

bool time_sync_is_valid(void)
{
    return time(NULL) >= TIME_VALID_MIN;
}
