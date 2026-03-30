#include "display_ssd1306.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_manager.h"
#include "project_config.h"

static const char *TAG = "display_ssd1306";

#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64
#define SSD1306_PAGES (SSD1306_HEIGHT / 8)

static bool s_initialized;
static bool s_has_presented_frame;
static uint8_t s_i2c_addr = SSD1306_I2C_ADDR;
static uint8_t s_framebuffer[SSD1306_WIDTH * SSD1306_PAGES];
static uint8_t s_last_presented_frame[SSD1306_WIDTH * SSD1306_PAGES];

static const uint8_t s_candidate_addresses[] = {
    SSD1306_I2C_ADDR,
    0x3D,
};

typedef struct {
    char c;
    uint8_t rows[5];
} glyph_3x5_t;

static const glyph_3x5_t s_glyphs[] = {
    {' ', {0b000, 0b000, 0b000, 0b000, 0b000}},
    {'%', {0b101, 0b001, 0b010, 0b100, 0b101}},
    {'+', {0b000, 0b010, 0b111, 0b010, 0b000}},
    {'-', {0b000, 0b000, 0b111, 0b000, 0b000}},
    {'.', {0b000, 0b000, 0b000, 0b000, 0b010}},
    {'0', {0b111, 0b101, 0b101, 0b101, 0b111}},
    {'1', {0b010, 0b110, 0b010, 0b010, 0b111}},
    {'2', {0b111, 0b001, 0b111, 0b100, 0b111}},
    {'3', {0b111, 0b001, 0b111, 0b001, 0b111}},
    {'4', {0b101, 0b101, 0b111, 0b001, 0b001}},
    {'5', {0b111, 0b100, 0b111, 0b001, 0b111}},
    {'6', {0b111, 0b100, 0b111, 0b101, 0b111}},
    {'7', {0b111, 0b001, 0b010, 0b100, 0b100}},
    {'8', {0b111, 0b101, 0b111, 0b101, 0b111}},
    {'9', {0b111, 0b101, 0b111, 0b001, 0b111}},
    {'A', {0b010, 0b101, 0b111, 0b101, 0b101}},
    {'B', {0b110, 0b101, 0b110, 0b101, 0b110}},
    {'C', {0b011, 0b100, 0b100, 0b100, 0b011}},
    {'D', {0b110, 0b101, 0b101, 0b101, 0b110}},
    {'E', {0b111, 0b100, 0b110, 0b100, 0b111}},
    {'F', {0b111, 0b100, 0b110, 0b100, 0b100}},
    {'G', {0b011, 0b100, 0b101, 0b101, 0b011}},
    {'H', {0b101, 0b101, 0b111, 0b101, 0b101}},
    {'I', {0b111, 0b010, 0b010, 0b010, 0b111}},
    {'J', {0b111, 0b001, 0b001, 0b101, 0b111}},
    {'K', {0b101, 0b101, 0b110, 0b101, 0b101}},
    {'L', {0b100, 0b100, 0b100, 0b100, 0b111}},
    {'M', {0b101, 0b111, 0b101, 0b101, 0b101}},
    {'N', {0b101, 0b111, 0b111, 0b111, 0b101}},
    {'O', {0b111, 0b101, 0b101, 0b101, 0b111}},
    {'P', {0b110, 0b101, 0b110, 0b100, 0b100}},
    {'Q', {0b111, 0b101, 0b101, 0b111, 0b001}},
    {'R', {0b110, 0b101, 0b110, 0b101, 0b101}},
    {'S', {0b111, 0b100, 0b111, 0b001, 0b111}},
    {'T', {0b111, 0b010, 0b010, 0b010, 0b010}},
    {'U', {0b101, 0b101, 0b101, 0b101, 0b111}},
    {'W', {0b101, 0b101, 0b111, 0b111, 0b101}},
    {'X', {0b101, 0b101, 0b010, 0b101, 0b101}},
    {'Y', {0b101, 0b101, 0b010, 0b010, 0b010}},
};

static bool sensor_has_numeric_sample(const sensor_snapshot_t *snapshot, sensor_id_t sensor_id)
{
    if (snapshot == NULL) {
        return false;
    }

    sensor_state_t state = snapshot->status[sensor_id].state;
    return state == SENSOR_STATE_HEALTHY || state == SENSOR_STATE_WARMING_UP;
}

static const char *sensor_state_label(const sensor_snapshot_t *snapshot, sensor_id_t sensor_id)
{
    const sensor_status_t *status = &snapshot->status[sensor_id];

    switch (status->state) {
    case SENSOR_STATE_WAITING_FIRST_SAMPLE:
        return "WAIT";
    case SENSOR_STATE_WARMING_UP:
        return "WARM";
    case SENSOR_STATE_HEALTHY:
        return "OK";
    case SENSOR_STATE_STALE:
        return "STAL";
    case SENSOR_STATE_ERROR:
        return "ERR";
    case SENSOR_STATE_ABSENT_OPTIONAL:
        return "MISS";
    case SENSOR_STATE_INIT_FAILED:
    default:
        return status->initialized ? "ERR" : "OFF";
    }
}

static const uint8_t *glyph_for_char(char c)
{
    for (size_t i = 0; i < (sizeof(s_glyphs) / sizeof(s_glyphs[0])); ++i) {
        if (s_glyphs[i].c == c) {
            return s_glyphs[i].rows;
        }
    }
    return s_glyphs[0].rows;
}

static esp_err_t ssd1306_send_command(uint8_t cmd)
{
    uint8_t packet[2] = {0x00, cmd};
    return i2c_manager_write(s_i2c_addr, packet, sizeof(packet));
}

static esp_err_t ssd1306_send_data(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t packet[17] = {0};
    packet[0] = 0x40;

    size_t offset = 0;
    while (offset < len) {
        size_t chunk = (len - offset > 16) ? 16 : (len - offset);
        memcpy(&packet[1], &data[offset], chunk);
        esp_err_t err = i2c_manager_write(s_i2c_addr, packet, chunk + 1);
        if (err != ESP_OK) {
            return err;
        }
        offset += chunk;
    }

    return ESP_OK;
}

static void ssd1306_clear_buffer(void)
{
    memset(s_framebuffer, 0, sizeof(s_framebuffer));
}

static void ssd1306_set_pixel(int x, int y)
{
    if (x < 0 || y < 0 || x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        return;
    }
    size_t idx = (size_t)x + ((size_t)y / 8U) * SSD1306_WIDTH;
    s_framebuffer[idx] |= (1U << (y % 8));
}

static void ssd1306_draw_char_3x5(int x, int y, char c)
{
    char upper = (char)toupper((unsigned char)c);
    const uint8_t *rows = glyph_for_char(upper);

    for (int row = 0; row < 5; ++row) {
        for (int col = 0; col < 3; ++col) {
            if (rows[row] & (1 << (2 - col))) {
                ssd1306_set_pixel(x + col, y + row);
            }
        }
    }
}

static void ssd1306_draw_text_3x5(int x, int y, const char *text)
{
    if (text == NULL) {
        return;
    }

    int cursor_x = x;
    for (size_t i = 0; text[i] != '\0'; ++i) {
        ssd1306_draw_char_3x5(cursor_x, y, text[i]);
        cursor_x += 4;
        if (cursor_x > (SSD1306_WIDTH - 3)) {
            break;
        }
    }
}

static esp_err_t ssd1306_flush(void)
{
    for (int page = 0; page < SSD1306_PAGES; ++page) {
        esp_err_t err = ssd1306_send_command((uint8_t)(0xB0 | page));
        if (err != ESP_OK) {
            return err;
        }
        err = ssd1306_send_command(0x00);
        if (err != ESP_OK) {
            return err;
        }
        err = ssd1306_send_command(0x10);
        if (err != ESP_OK) {
            return err;
        }
        err = ssd1306_send_data(&s_framebuffer[page * SSD1306_WIDTH], SSD1306_WIDTH);
        if (err != ESP_OK) {
            return err;
        }
    }

    return ESP_OK;
}

bool display_ssd1306_is_ready(void)
{
    return s_initialized;
}

esp_err_t display_ssd1306_init(void)
{
    esp_err_t err = ESP_FAIL;
    bool found = false;

    for (size_t i = 0; i < (sizeof(s_candidate_addresses) / sizeof(s_candidate_addresses[0])); ++i) {
        s_i2c_addr = s_candidate_addresses[i];
        err = i2c_manager_probe_device(s_i2c_addr, pdMS_TO_TICKS(50));
        if (err == ESP_OK) {
            found = true;
            break;
        }
    }

    if (!found) {
        s_initialized = false;
        return err;
    }

    const uint8_t init_seq[] = {
        0xAE,
        0xD5, 0x80,
        0xA8, 0x3F,
        0xD3, 0x00,
        0x40,
        0x8D, 0x14,
        0x20, 0x00,
        0xA1,
        0xC8,
        0xDA, 0x12,
        0x81, 0xCF,
        0xD9, 0xF1,
        0xDB, 0x40,
        0xA4,
        0xA6,
        0x2E,
        0xAF,
    };

    for (size_t i = 0; i < sizeof(init_seq); ++i) {
        err = ssd1306_send_command(init_seq[i]);
        if (err != ESP_OK) {
            s_initialized = false;
            return err;
        }
    }

    s_initialized = true;
    s_has_presented_frame = false;
    ssd1306_clear_buffer();
    memset(s_last_presented_frame, 0, sizeof(s_last_presented_frame));
    err = ssd1306_flush();
    if (err != ESP_OK) {
        s_initialized = false;
        return err;
    }

    memcpy(s_last_presented_frame, s_framebuffer, sizeof(s_framebuffer));
    s_has_presented_frame = true;

    ESP_LOGI(TAG, "SSD1306 detected at 0x%02X", s_i2c_addr);
    return ESP_OK;
}

esp_err_t display_ssd1306_render_summary(const sensor_snapshot_t *snapshot)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (snapshot == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    char line[40] = {0};

    ssd1306_clear_buffer();

    if (sensor_has_numeric_sample(snapshot, SENSOR_ID_DHT22)) {
        snprintf(line, sizeof(line), "AMB %.1fC H%.0f%%", snapshot->dht_temp_c, snapshot->dht_humidity_pct);
    } else {
        snprintf(line, sizeof(line), "AMB --.-C H--%%");
    }
    ssd1306_draw_text_3x5(0, 0, line);

    if (sensor_has_numeric_sample(snapshot, SENSOR_ID_MLX90614)) {
        snprintf(line, sizeof(line), "OBJ %.1fC", snapshot->mlx_object_temp_c);
    } else {
        snprintf(line, sizeof(line), "OBJ --.-C");
    }
    ssd1306_draw_text_3x5(0, 8, line);

    if (sensor_has_numeric_sample(snapshot, SENSOR_ID_KY037)) {
        snprintf(line, sizeof(line), "SND %d%% P%d", snapshot->ky037_activity_pct, snapshot->ky037_peak_to_peak);
    } else {
        snprintf(line, sizeof(line), "SND --%% P---");
    }
    ssd1306_draw_text_3x5(0, 16, line);

    if (sensor_has_numeric_sample(snapshot, SENSOR_ID_MQ135)) {
        snprintf(line, sizeof(line), "MQ135 %d%% D%d", snapshot->mq135_response_pct, snapshot->mq135_delta_raw);
    } else {
        snprintf(line, sizeof(line), "MQ135 --%% D---");
    }
    ssd1306_draw_text_3x5(0, 24, line);

    snprintf(line,
             sizeof(line),
             "DHT %s MLX %s",
             sensor_state_label(snapshot, SENSOR_ID_DHT22),
             sensor_state_label(snapshot, SENSOR_ID_MLX90614));
    ssd1306_draw_text_3x5(0, 32, line);

    snprintf(line,
             sizeof(line),
             "MQ135 %s MAX %s",
             sensor_state_label(snapshot, SENSOR_ID_MQ135),
             sensor_state_label(snapshot, SENSOR_ID_MAX30102));
    ssd1306_draw_text_3x5(0, 40, line);

    snprintf(line, sizeof(line), "OLED %s", sensor_state_label(snapshot, SENSOR_ID_OLED));
    ssd1306_draw_text_3x5(0, 48, line);

    if (s_has_presented_frame &&
        memcmp(s_framebuffer, s_last_presented_frame, sizeof(s_framebuffer)) == 0) {
        return ESP_OK;
    }

    esp_err_t err = ssd1306_flush();
    if (err != ESP_OK) {
        return err;
    }

    memcpy(s_last_presented_frame, s_framebuffer, sizeof(s_framebuffer));
    s_has_presented_frame = true;
    return ESP_OK;
}
