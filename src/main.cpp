#include <Arduino.h>
#include <ArduinoJson.h>
#include "driver/uart.h"
#include "esp_camera.h"
#include "driver/ledc.h"

#define CAR_ID          "RC001"
#define MQTT_BROKER     "broker.emqx.io"
#define MQTT_PORT       1883

#define MODEM_TX        17
#define MODEM_RX        18
#define MOTOR_L_PWM     10
#define MOTOR_L_IN1     11
#define MOTOR_L_IN2     12
#define MOTOR_R_PWM     13
#define MOTOR_R_IN1     14
#define MOTOR_R_IN2     15
#define SERVO_PIN       16

bool armed = false;
float throttle = 0, steering = 0;
uint32_t lastCmdTime = 0;

void modemSend(const char* cmd) {
    uart_write_bytes(UART_NUM_2, cmd, strlen(cmd));
    uart_write_bytes(UART_NUM_2, "\r\n", 2);
}

bool modemWait(const char* ok, int ms = 3000) {
    char buf[256] = {0};
    int n = 0;
    unsigned long t = millis();
    while (millis() - t < ms) {
        n += uart_read_bytes(UART_NUM_2, (uint8_t*)buf + n, 255 - n, 50);
        buf[n] = 0;
        if (strstr(buf, ok)) return true;
        if (strstr(buf, "ERROR")) return false;
    }
    return false;
}

bool mqttConnect() {
    uart_config_t c = {.baud_rate = 115200, .data_bits = UART_DATA_8_BITS, 
                       .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
                       .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(UART_NUM_2, &c);
    uart_set_pin(UART_NUM_2, MODEM_TX, MODEM_RX, -1, -1);
    uart_driver_install(UART_NUM_2, 2048, 2048, 0, NULL, 0);
    
    delay(3000);
    modemSend("AT");
    if (!modemWait("OK")) return false;
    
    modemSend("AT+CGDCONT=1,\"IP\",\"cmnet\"");
    modemSend("AT+CGACT=1,1");
    
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+QMTOPEN=0,\"%s\",%d", MQTT_BROKER, MQTT_PORT);
    modemSend(cmd);
    if (!modemWait("+QMTOPEN: 0,0", 10000)) return false;
    
    snprintf(cmd, sizeof(cmd), "AT+QMTCONN=0,\"%s\"", CAR_ID);
    modemSend(cmd);
    if (!modemWait("+QMTCONN: 0,0,0", 10000)) return false;
    
    snprintf(cmd, sizeof(cmd), "AT+QMTSUB=0,1,\"rc/car/%s/cmd\",0", CAR_ID);
    modemSend(cmd);
    return modemWait("+QMTSUB: 0,1,0");
}

void mqttPublish(const char* topic, const char* msg) {
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "AT+QMTPUB=0,0,0,0,\"%s\",%d", topic, strlen(msg));
    modemSend(cmd);
    uart_write_bytes(UART_NUM_2, msg, strlen(msg));
    uart_write_bytes(UART_NUM_2, "\x1A", 1);
}

void motorInit() {
    ledc_timer_config_t t = {.speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_8_BIT,
                             .timer_num = LEDC_TIMER_0, .freq_hz = 5000, .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&t);
    
    ledc_channel_config_t ch = {.gpio_num = MOTOR_L_PWM, .speed_mode = LEDC_LOW_SPEED_MODE,
                                .channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0, .duty = 0};
    ledc_channel_config(&ch);
    ch.gpio_num = MOTOR_R_PWM; ch.channel = LEDC_CHANNEL_1;
    ledc_channel_config(&ch);
    
    pinMode(MOTOR_L_IN1, OUTPUT); pinMode(MOTOR_L_IN2, OUTPUT);
    pinMode(MOTOR_R_IN1, OUTPUT); pinMode(MOTOR_R_IN2, OUTPUT);
    
    ledc_timer_config_t st = {.speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_16_BIT,
                              .timer_num = LEDC_TIMER_1, .freq_hz = 50};
    ledc_timer_config(&st);
    ledc_channel_config_t sc = {.gpio_num = SERVO_PIN, .speed_mode = LEDC_LOW_SPEED_MODE,
                                .channel = LEDC_CHANNEL_2, .timer_sel = LEDC_TIMER_1, .duty = 4915};
    ledc_channel_config(&sc);
}

void motorSet(float thr, float steer) {
    if (!armed) { thr = 0; steer = 0; }
    float left = thr, right = thr;
    if (steer > 0) right *= (1 - steer * 0.5);
    else if (steer < 0) left *= (1 + steer * 0.5);
    
    int pwm = abs(left) * 255;
    digitalWrite(MOTOR_L_IN1, left > 0); digitalWrite(MOTOR_L_IN2, left < 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    
    pwm = abs(right) * 255;
    digitalWrite(MOTOR_R_IN1, right > 0); digitalWrite(MOTOR_R_IN2, right < 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, pwm);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    
    int us = 1500 + steer * 500;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, us * 65535 / 20000);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}

bool camInit() {
    camera_config_t cfg = {
        .pin_pwdn = -1, .pin_reset = -1, .pin_xclk = 15,
        .pin_sccb_sda = 6, .pin_sccb_scl = 7,
        .pin_d7 = 16, .pin_d6 = 17, .pin_d5 = 18, .pin_d4 = 12,
        .pin_d3 = 11, .pin_d2 = 10, .pin_d1 = 9, .pin_d0 = 8,
        .pin_vsync = 6, .pin_href = 7, .pin_pclk = 13,
        .xclk_freq_hz = 20000000, .frame_size = FRAMESIZE_VGA,
        .pixel_format = PIXFORMAT_JPEG, .jpeg_quality = 25, .fb_count = 2
    };
    return esp_camera_init(&cfg) == ESP_OK;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    motorInit();
    camInit();
    
    Serial.println("Connecting 4G...");
    while (!mqttConnect()) {
        Serial.println("Retry...");
        delay(5000);
    }
    Serial.println("Ready!");
}

void loop() {
    char buf[512];
    int n = uart_read_bytes(UART_NUM_2, (uint8_t*)buf, 511, 10);
    if (n > 0) {
        buf[n] = 0;
        if (strstr(buf, "+QMTRECV:")) {
            char* p = strchr(buf, '{');
            if (p) {
                StaticJsonDocument<256> doc;
                deserializeJson(doc, p);
                const char* cmd = doc["cmd"];
                if (cmd) {
                    if (!strcmp(cmd, "control")) {
                        throttle = doc["throttle"];
                        steering = doc["steering"];
                        lastCmdTime = millis();
                    }
                    else if (!strcmp(cmd, "arm")) armed = true;
                    else if (!strcmp(cmd, "disarm")) armed = false;
                    else if (!strcmp(cmd, "stop")) { armed = false; throttle = 0; steering = 0; }
                }
            }
        }
    }
    
    if (millis() - lastCmdTime > 1000) { throttle = 0; steering = 0; }
    motorSet(throttle, steering);
    
    static unsigned long lastFrame = 0;
    if (millis() - lastFrame > 100) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb && fb->len < 30000) {
            char topic[64];
            snprintf(topic, sizeof(topic), "rc/car/%s/video", CAR_ID);
            mqttPublish(topic, (char*)fb->buf);
        }
        esp_camera_fb_return(fb);
        lastFrame = millis();
    }
    
    static unsigned long lastStatus = 0;
    if (millis() - lastStatus > 500) {
        StaticJsonDocument<256> doc;
        doc["id"] = CAR_ID;
        doc["armed"] = armed;
        doc["throttle"] = throttle;
        doc["steering"] = steering;
        char msg[256], topic[64];
        serializeJson(doc, msg);
        snprintf(topic, sizeof(topic), "rc/car/%s/status", CAR_ID);
        mqttPublish(topic, msg);
        lastStatus = millis();
    }
    
    delay(10);
}
