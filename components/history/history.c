#include "history.h"

#include "esp_partition.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>

static const char *TAG = "history";

#define HIST_PART_LABEL    "hist"
#define HIST_PART_SUBTYPE  0x40
#define SECTOR_SIZE        4096U
#define HDR_SIZE           8U
#define REPUBLISH_S        1800U      /* renova a retida mesmo sem dado novo (expira em 3 dias no plano free) */
#define TASK_PERIOD_MS     10000U
#define TASK_STACK         6144U      /* publish roda TLS no contexto desta task */
#define TASK_PRIO          3
#define TIME_VALID_MIN     1700000000U   /* antes disso o relógio não passou pelo SNTP */

#define NA_U16  0xFFFFU
#define NA_I16  INT16_MIN
#define NA_U8   0xFFU

typedef struct __attribute__((packed)) {
    uint32_t ts;
    int16_t  temp_c100;
    uint16_t hum_x10;
    uint16_t press_x10;
    uint16_t lux_2;
    uint16_t gas_kohm_x10;
    uint16_t dust_ugm3;
    uint16_t wind_x10;
    uint16_t gust_x10;
    uint16_t rain_mm_x10;
    uint8_t  rain_pct;
    uint8_t  crc8;
} hist_rec_t;

_Static_assert(sizeof(hist_rec_t) == HISTORY_REC_SIZE, "hist_rec_t fora do formato de history.h");

#define REC_PER_SECTOR  (SECTOR_SIZE / sizeof(hist_rec_t))   /* 170; sobram 16 bytes por setor */

/* Acumulador do intervalo corrente */
typedef struct {
    uint32_t start;            /* 0 = vazio */
    uint16_t n_thp, n_gas, n_lux, n_dust, n_wind, n_rain, n_wet;
    bool     gauge;
    double   temp, hum, press, gas, lux, dust, wind;
    float    gust_max;
    float    rain_mm;
} acc_t;

static const esp_partition_t *s_part;
static uint32_t s_slots;       /* slots de registro na partição */
static uint32_t s_wr;          /* próximo slot a gravar */

/* Snapshot já no formato publicado: cabeçalho + registros contíguos (~48 KB) */
static uint8_t     s_blob[HDR_SIZE + HISTORY_MAX_RECORDS * sizeof(hist_rec_t)];
static hist_rec_t *const s_rec = (hist_rec_t *)(s_blob + HDR_SIZE);
static uint16_t    s_count;

static acc_t   s_acc;
static float   s_prev_mm = NAN;  /* último acumulado do pluviômetro visto */
static bool    s_dirty;

static SemaphoreHandle_t    s_lock;
static TaskHandle_t         s_task;
static history_publish_fn_t s_publish;

/* ---------- Utilitários ---------- */

/* Semente = versão do formato: registros de formatos antigos na flash não validam */
static uint8_t crc8(const uint8_t *d, size_t n)
{
    uint8_t c = HISTORY_FMT_VERSION;
    while (n--) {
        c ^= *d++;
        for (int i = 0; i < 8; i++) {
            c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
        }
    }
    return c;
}

static bool rec_valid(const hist_rec_t *r)
{
    return r->ts != UINT32_MAX && r->ts >= TIME_VALID_MIN &&
           r->crc8 == crc8((const uint8_t *)r, sizeof(*r) - 1);
}

static bool rec_blank(const hist_rec_t *r)
{
    const uint8_t *p = (const uint8_t *)r;
    for (size_t i = 0; i < sizeof(*r); i++) {
        if (p[i] != 0xFF) { return false; }
    }
    return true;
}

static size_t slot_off(uint32_t slot)
{
    return (slot / REC_PER_SECTOR) * SECTOR_SIZE + (slot % REC_PER_SECTOR) * sizeof(hist_rec_t);
}

/** Média de n amostras escalada para uint16; 0xFFFF se não houve amostra. */
static uint16_t enc_u16(uint16_t n, double sum, double scale)
{
    if (n == 0) { return NA_U16; }
    double v = round(sum / n * scale);
    if (v < 0)              { v = 0; }
    if (v > NA_U16 - 1)     { v = NA_U16 - 1; }
    return (uint16_t)v;
}

/* ---------- Agregação ---------- */

static void acc_to_rec(const acc_t *a, hist_rec_t *r)
{
    r->ts = a->start;

    if (a->n_thp) {
        double t = round(a->temp / a->n_thp * 100.0);
        r->temp_c100 = (int16_t)fmax(-32767.0, fmin(32767.0, t));
    } else {
        r->temp_c100 = NA_I16;
    }
    r->hum_x10      = enc_u16(a->n_thp,  a->hum,   10.0);
    r->press_x10    = enc_u16(a->n_thp,  a->press, 10.0);
    r->gas_kohm_x10 = enc_u16(a->n_gas,  a->gas,   10.0);
    r->lux_2        = enc_u16(a->n_lux,  a->lux,   0.5);
    r->dust_ugm3    = enc_u16(a->n_dust, a->dust,  1.0);
    r->wind_x10     = enc_u16(a->n_wind, a->wind,  10.0);
    r->gust_x10     = a->n_wind ? enc_u16(1, a->gust_max, 10.0) : NA_U16;
    r->rain_mm_x10  = a->gauge  ? enc_u16(1, a->rain_mm, 10.0)  : NA_U16;
    r->rain_pct     = a->n_rain ? (uint8_t)lround(100.0 * a->n_wet / a->n_rain) : NA_U8;
    r->crc8         = crc8((const uint8_t *)r, sizeof(*r) - 1);
}

static void acc_sample(acc_t *a, const lora_packet_t *p)
{
    if (lora_packet_has(p, LORA_VALID_THP)) {
        a->n_thp++;
        a->temp  += p->temp_c100 / 100.0;
        a->hum   += p->hum_x100 / 100.0;
        a->press += p->press_x10 / 10.0;
    }
    if (lora_packet_has(p, LORA_VALID_GAS)) {
        a->n_gas++;
        a->gas += p->gas_ohm / 1000.0;
    }
    if (lora_packet_has(p, LORA_VALID_LUX)) {
        a->n_lux++;
        a->lux += p->lux_x10 / 10.0;
    }
    if (lora_packet_has(p, LORA_VALID_DUST)) {
        a->n_dust++;
        a->dust += p->dust_ugm3;
    }
    if (lora_packet_has(p, LORA_VALID_WIND)) {
        a->n_wind++;
        a->wind    += p->wind_cms / 100.0;
        a->gust_max = fmaxf(a->gust_max, p->gust_cms / 100.0f);
    }
    if (lora_packet_has(p, LORA_VALID_RAIN)) {
        a->n_rain++;
        a->n_wet += (p->flags & LORA_FLAG_RAIN_WET) ? 1 : 0;
    }
    if (lora_packet_has(p, LORA_VALID_GAUGE)) {
        const float total = p->rain_x100 / 100.0f;
        if (isfinite(s_prev_mm)) {
            /* Acumulado é monotônico desde o boot do nó; se caiu, o nó reiniciou */
            a->rain_mm += (total >= s_prev_mm) ? (total - s_prev_mm) : total;
        }
        s_prev_mm = total;
        a->gauge  = true;
    }
}

/* ---------- Armazenamento ---------- */

static esp_err_t flash_append(const hist_rec_t *r)
{
    esp_err_t ret;

    if (s_wr % REC_PER_SECTOR == 0) {
        ret = esp_partition_erase_range(s_part, (s_wr / REC_PER_SECTOR) * SECTOR_SIZE, SECTOR_SIZE);
        if (ret != ESP_OK) { goto err; }
    }
    ret = esp_partition_write(s_part, slot_off(s_wr), r, sizeof(*r));
    if (ret != ESP_OK) { goto err; }

    s_wr = (s_wr + 1) % s_slots;
    return ESP_OK;

err:
    ESP_LOGE(TAG, "Falha ao gravar slot %lu: %s", (unsigned long)s_wr, esp_err_to_name(ret));
    return ret;
}

static void ram_append(const hist_rec_t *r)
{
    if (s_count == HISTORY_MAX_RECORDS) {
        memmove(&s_rec[0], &s_rec[1], (HISTORY_MAX_RECORDS - 1) * sizeof(hist_rec_t));
        s_count--;
    }
    s_rec[s_count++] = *r;
}

static void ram_prune(uint32_t now)
{
    const uint32_t lim = now - HISTORY_DAYS * 86400U;
    uint16_t k = 0;

    while (k < s_count && s_rec[k].ts < lim) { k++; }
    if (k > 0) {
        memmove(&s_rec[0], &s_rec[k], (size_t)(s_count - k) * sizeof(hist_rec_t));
        s_count -= k;
    }
}

/** Fecha o intervalo corrente. Chamar com s_lock. */
static void acc_close(void)
{
    if (s_acc.start == 0) { return; }

    hist_rec_t r;
    acc_to_rec(&s_acc, &r);
    memset(&s_acc, 0, sizeof(s_acc));

    ram_append(&r);
    (void)flash_append(&r);   /* falha na flash não impede publicar o que está em RAM */
    s_dirty = true;

    ESP_LOGI(TAG, "Intervalo %lu fechado (%u registros em RAM)", (unsigned long)r.ts, s_count);
}

/**
 * @brief Recupera o histórico da flash.
 *
 * Os slots formam um anel em ordem de gravação. Acha o registro mais novo e
 * caminha para trás pelo anel inteiro, pulando slots apagados ou corrompidos
 * (fim de setor pulado após queda de energia), até encher a RAM ou achar uma
 * quebra na ordem temporal.
 */
static esp_err_t flash_load(void)
{
    hist_rec_t r;
    uint32_t   newest = UINT32_MAX, newest_ts = 0;
    esp_err_t  ret;

    for (uint32_t i = 0; i < s_slots; i++) {
        ret = esp_partition_read(s_part, slot_off(i), &r, sizeof(r));
        if (ret != ESP_OK) { goto err; }
        if (rec_valid(&r) && r.ts >= newest_ts) {
            newest_ts = r.ts;
            newest    = i;
        }
    }

    s_count = 0;
    if (newest == UINT32_MAX) {
        s_wr = 0;
        ESP_LOGI(TAG, "Nenhum historico na flash");
        return ESP_OK;
    }

    uint16_t n = 0;
    uint32_t prev_ts = UINT32_MAX, i = newest;
    do {
        ret = esp_partition_read(s_part, slot_off(i), &r, sizeof(r));
        if (ret != ESP_OK) { goto err; }
        if (rec_valid(&r)) {
            if (r.ts > prev_ts) { break; }            /* volta completa ou relógio retrocedeu */
            s_rec[HISTORY_MAX_RECORDS - 1 - n] = r;   /* preenche de trás para frente */
            prev_ts = r.ts;
            n++;
        }
        i = (i == 0) ? s_slots - 1 : i - 1;
    } while (n < HISTORY_MAX_RECORDS && i != newest);

    memmove(&s_rec[0], &s_rec[HISTORY_MAX_RECORDS - n], (size_t)n * sizeof(hist_rec_t));
    s_count = n;

    /* Próxima gravação: slot após o mais novo. Se estiver sujo (queda de energia
       no meio de uma escrita), pula para o início do setor seguinte. */
    s_wr = (newest + 1) % s_slots;
    if (s_wr % REC_PER_SECTOR != 0) {
        ret = esp_partition_read(s_part, slot_off(s_wr), &r, sizeof(r));
        if (ret != ESP_OK) { goto err; }
        if (!rec_blank(&r)) {
            s_wr = ((s_wr / REC_PER_SECTOR + 1) * REC_PER_SECTOR) % s_slots;
        }
    }

    ESP_LOGI(TAG, "%u registros recuperados da flash (proximo slot %lu)", n, (unsigned long)s_wr);
    return ESP_OK;

err:
    ESP_LOGE(TAG, "Falha ao ler a particao: %s", esp_err_to_name(ret));
    return ret;
}

/* ---------- Publicação ---------- */

static size_t snapshot_fill(void)
{
    s_blob[0] = 'H';
    s_blob[1] = 'S';
    s_blob[2] = HISTORY_FMT_VERSION;
    s_blob[3] = (uint8_t)sizeof(hist_rec_t);
    s_blob[4] = (uint8_t)(HISTORY_PERIOD_S & 0xFF);
    s_blob[5] = (uint8_t)(HISTORY_PERIOD_S >> 8);
    s_blob[6] = (uint8_t)(s_count & 0xFF);
    s_blob[7] = (uint8_t)(s_count >> 8);
    return HDR_SIZE + (size_t)s_count * sizeof(hist_rec_t);
}

static void history_task(void *arg)
{
    (void)arg;
    uint32_t last_pub = 0;

    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(TASK_PERIOD_MS));

        const time_t t = time(NULL);
        if (t < (time_t)TIME_VALID_MIN) { continue; }
        const uint32_t now = (uint32_t)t;

        xSemaphoreTake(s_lock, portMAX_DELAY);

        /* Fecha o intervalo vencido mesmo sem pacote novo (nó parado ou fora do ar) */
        if (s_acc.start != 0 && now >= s_acc.start + HISTORY_PERIOD_S) {
            acc_close();
        }

        if ((s_dirty || now - last_pub >= REPUBLISH_S) && s_count > 0) {
            ram_prune(now);
            const size_t len = snapshot_fill();
            if (s_count > 0 && s_publish(s_blob, len) == ESP_OK) {
                s_dirty  = false;
                last_pub = now;
                ESP_LOGI(TAG, "Snapshot publicado (%u registros, %u bytes)", s_count, (unsigned)len);
            }
        }

        xSemaphoreGive(s_lock);
    }
}

/* ---------- API ---------- */

esp_err_t history_init(history_publish_fn_t publish)
{
    esp_err_t ret;

    if (publish == NULL) {
        ret = ESP_ERR_INVALID_ARG;
        goto err;
    }
    s_publish = publish;

    s_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                      (esp_partition_subtype_t)HIST_PART_SUBTYPE, HIST_PART_LABEL);
    if (s_part == NULL) {
        ESP_LOGE(TAG, "Particao '%s' nao encontrada (veja partitions.csv)", HIST_PART_LABEL);
        ret = ESP_ERR_NOT_FOUND;
        goto err;
    }

    s_slots = (uint32_t)(s_part->size / SECTOR_SIZE) * REC_PER_SECTOR;
    if (s_slots < HISTORY_MAX_RECORDS + REC_PER_SECTOR) {
        ESP_LOGE(TAG, "Particao pequena: %lu slots, precisa de %u",
                 (unsigned long)s_slots, (unsigned)(HISTORY_MAX_RECORDS + REC_PER_SECTOR));
        ret = ESP_ERR_INVALID_SIZE;
        goto err;
    }

    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto err;
    }

    ret = flash_load();
    if (ret != ESP_OK) { goto err_lock; }

    if (xTaskCreate(history_task, "history", TASK_STACK, NULL, TASK_PRIO, &s_task) != pdPASS) {
        ret = ESP_ERR_NO_MEM;
        goto err_lock;
    }

    ESP_LOGI(TAG, "Historico ativo: %lu slots, %u registros de %u s",
             (unsigned long)s_slots, (unsigned)HISTORY_MAX_RECORDS, (unsigned)HISTORY_PERIOD_S);
    return ESP_OK;

err_lock:
    vSemaphoreDelete(s_lock);
    s_lock = NULL;
err:
    return ret;
}

void history_add(const lora_packet_t *pkt)
{
    if (s_lock == NULL || pkt == NULL) { return; }

    const time_t t = time(NULL);
    if (t < (time_t)TIME_VALID_MIN) {
        ESP_LOGW(TAG, "Relogio sem SNTP: amostra fora do historico");
        return;
    }
    const uint32_t bucket = (uint32_t)t - ((uint32_t)t % HISTORY_PERIOD_S);
    bool closed = false;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_acc.start != 0 && s_acc.start != bucket) {
        acc_close();
        closed = true;
    }
    if (s_acc.start == 0) {
        s_acc.start = bucket;
    }
    acc_sample(&s_acc, pkt);
    xSemaphoreGive(s_lock);

    if (closed) {
        xTaskNotifyGive(s_task);   /* publica já, sem esperar o próximo tick */
    }
}
