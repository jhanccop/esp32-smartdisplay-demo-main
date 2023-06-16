#include <Arduino.h>
#include <esp32_smartdisplay.h>

#include "FS.h"
#include "SD.h"
#include "SPI.h"

// ADC Objects
#include "ADS1X15.h"
ADS1115 ADS(0x48);

// LVGL Objects
static lv_obj_t *Acquisition_label;
static lv_obj_t *loadChart;
static lv_chart_series_t *load_ser;

static lv_obj_t *posChart;
static lv_chart_series_t *pos_ser;

static lv_obj_t *label_box_load;
static lv_obj_t *label_box_pos;

/* ====================== DEEP SLEEP CONFIG ======================== */
#define uS_TO_S_FACTOR 1000000
int TIME_TO_SLEEP = 300; // 5 minutes sleep

RTC_DATA_ATTR unsigned int bootCount = 0;

#include <ESP32Time.h>
ESP32Time rtc;

/* ====================== MQTT CONFIG ======================== */
#include <WiFi.h>

//const char *ssid = "INTERNET D-13";
const char *ssid = "UNONU UM52";
//const char *password = "20600680901";
const char *password = "12345678";

WiFiClient client;
#include <PubSubClient.h>

PubSubClient mqtt(client);

const char *broker = "broker.emqx.io";
//const char *broker = "broker.hivemq.com";
const int mqtt_port = 1883;
const char *mqtt_id = "gle_client_234:::#122";
const char *mqtt_user = "gle";
const char *mqtt_pass = "glettxx";

const char *topicSubscribe = "jphOandG/";

// const char *topicSubscribe = "oilAIOT/field/Well01";
const char *topicPublish = "jphOandG/data";

/* ========================== TOOLS ========================== */
#include <Average.h>
#include <Separador.h>
#include <ArduinoJson.h>

Separador s;

// ================== GENERALS VARIABLES ======================
String WellName = "3310";
String DateTime = "aaaa/mm/dd hh:mm";
String Diagnosis = "--";
String FillPump = "--";
String SPM = "--";
String PeakLoad = "--";
String MinLoad = "--";
String status = "--";

String Load = "--";
String Pos = "--";
int mode = 0; // 0: standAlone 1: endPoint (internet connect)

const int8_t pinCC1W = 1;

long previousMillis = 0; 

float f; // adc to mV

int16_t i_start, i_end;

unsigned long start_time;
unsigned long end_time;

const int16_t chart_n_max = 700;
const int16_t read_n_max = 700; //700
float FillPumpList[300];
float dtFillPumpList[300];

float alpha_p = 0.05;
float alpha_l = 0.08;
float filter_value_pos = 284.4;
float filter_value_load = 155;
//float load_bottom[559] = {4.09, 4.14, 4.18, 4.22, 4.25, 4.29, 4.33, 4.37, 4.42, 4.47, 4.51, 4.57, 4.62, 4.66, 4.72, 4.76, 4.79, 4.83, 4.88, 4.92, 4.97, 5.01, 5.06, 5.12, 5.17, 5.21, 5.26, 5.3, 5.34, 5.38, 5.42, 5.46, 5.49, 5.52, 5.54, 5.55, 5.55, 5.54, 5.53, 5.52, 5.51, 5.5, 5.49, 5.47, 5.45, 5.43, 5.41, 5.39, 5.36, 5.34, 5.32, 5.3, 5.28, 5.26, 5.24, 5.23, 5.21, 5.19, 5.17, 5.16, 5.16, 5.16, 5.17, 5.18, 5.2, 5.22, 5.24, 5.26, 5.28, 5.3, 5.33, 5.35, 5.38, 5.41, 5.43, 5.45, 5.47, 5.49, 5.5, 5.5, 5.52, 5.53, 5.54, 5.55, 5.56, 5.56, 5.57, 5.57, 5.57, 5.56, 5.55, 5.53, 5.52, 5.49, 5.47, 5.44, 5.41, 5.38, 5.35, 5.32, 5.3, 5.28, 5.26, 5.25, 5.24, 5.23, 5.23, 5.22, 5.21, 5.21, 5.21, 5.21, 5.21, 5.21, 5.21, 5.22, 5.23, 5.24, 5.25, 5.26, 5.27, 5.29, 5.31, 5.33, 5.35, 5.38, 5.4, 5.42, 5.43, 5.44, 5.44, 5.44, 5.44, 5.44, 5.44, 5.44, 5.43, 5.42, 5.4, 5.39, 5.38, 5.37, 5.37, 5.37, 5.36, 5.35, 5.33, 5.31, 5.29, 5.27, 5.25, 5.22, 5.2, 5.18, 5.17, 5.16, 5.14, 5.13, 5.13, 5.13, 5.13, 5.14, 5.16, 5.18, 5.19, 5.21, 5.23, 5.25, 5.27, 5.28, 5.29, 5.31, 5.33, 5.35, 5.37, 5.38, 5.4, 5.42, 5.43, 5.45, 5.46, 5.48, 5.5, 5.51, 5.52, 5.53, 5.53, 5.53, 5.53, 5.51, 5.49, 5.48, 5.46, 5.45, 5.44, 5.43, 5.42, 5.4, 5.38, 5.37, 5.36, 5.35, 5.34, 5.34, 5.34, 5.34, 5.34, 5.33, 5.32, 5.31, 5.3, 5.31, 5.32, 5.33, 5.34, 5.36, 5.37, 5.39, 5.42, 5.45, 5.46, 5.48, 5.5, 5.52, 5.53, 5.54, 5.55, 5.56, 5.57, 5.57, 5.57, 5.57, 5.56, 5.57, 5.58, 5.58, 5.59, 5.59, 5.6, 5.6, 5.59, 5.59, 5.57, 5.57, 5.55, 5.54, 5.53, 5.53, 5.51, 5.5, 5.48, 5.47, 5.47, 5.47, 5.47, 5.47, 5.48, 5.49, 5.5, 5.51, 5.52, 5.52, 5.53, 5.53, 5.53, 5.53, 5.53, 5.53, 5.53, 5.53, 5.52, 5.5, 5.48, 5.45, 5.41, 5.36, 5.31, 5.27, 5.22, 5.17, 5.13, 5.08, 5.04, 5.01, 4.96, 4.93, 4.89, 4.86, 4.83, 4.8, 4.76, 4.73, 4.71, 4.66, 4.63, 4.6, 4.56, 4.54, 4.51, 4.48, 4.45, 4.42, 4.39, 4.36, 4.34, 4.32, 4.3, 4.28, 4.27, 4.26, 4.26, 4.26, 4.25, 4.25, 4.26, 4.25, 4.25, 4.25, 4.26, 4.26, 4.26, 4.27, 4.27, 4.28, 4.28, 4.29, 4.29, 4.3, 4.3, 4.3, 4.3, 4.3, 4.3, 4.3, 4.29, 4.29, 4.28, 4.27, 4.27, 4.26, 4.26, 4.25, 4.25, 4.25, 4.24, 4.24, 4.24, 4.24, 4.23, 4.23, 4.22, 4.21, 4.21, 4.21, 4.21, 4.21, 4.21, 4.22, 4.22, 4.23, 4.24, 4.25, 4.26, 4.28, 4.29, 4.31, 4.33, 4.35, 4.36, 4.37, 4.38, 4.39, 4.4, 4.41, 4.42, 4.42, 4.43, 4.45, 4.45, 4.46, 4.47, 4.48, 4.49, 4.5, 4.5, 4.5, 4.49, 4.48, 4.47, 4.47, 4.46, 4.46, 4.46, 4.45, 4.45, 4.44, 4.43, 4.43, 4.43, 4.43, 4.43, 4.43, 4.44, 4.45, 4.45, 4.46, 4.46, 4.47, 4.47, 4.48, 4.48, 4.49, 4.5, 4.51, 4.52, 4.53, 4.55, 4.56, 4.57, 4.58, 4.59, 4.6, 4.6, 4.62, 4.63, 4.65, 4.65, 4.66, 4.66, 4.66, 4.65, 4.64, 4.63, 4.62, 4.59, 4.56, 4.53, 4.49, 4.43, 4.37, 4.32, 4.25, 4.18, 4.1, 4.02, 3.95, 3.88, 3.82, 3.76, 3.7, 3.64, 3.59, 3.53, 3.48, 3.42, 3.36, 3.31, 3.26, 3.21, 3.16, 3.11, 3.06, 3.0, 2.96, 2.92, 2.9, 2.9, 2.89, 2.9, 2.92, 2.94, 2.96, 2.98, 3.0, 3.02, 3.04, 3.05, 3.06, 3.08, 3.1, 3.13, 3.16, 3.19, 3.22, 3.24, 3.26, 3.28, 3.3, 3.32, 3.34, 3.34, 3.36, 3.35, 3.34, 3.32, 3.29, 3.26, 3.22, 3.19, 3.17, 3.14, 3.12, 3.09, 3.08, 3.06, 3.04, 3.02, 2.99, 2.97, 2.94, 2.92, 2.91, 2.9, 2.89, 2.87, 2.87, 2.86, 2.86, 2.87, 2.88, 2.91, 2.94, 2.98, 3.01, 3.05, 3.09, 3.13, 3.17, 3.2, 3.23, 3.25, 3.26, 3.28, 3.3, 3.32, 3.35, 3.38, 3.41, 3.44, 3.46, 3.49, 3.52, 3.54, 3.56, 3.58, 3.6, 3.6, 3.59, 3.6, 3.61, 3.63, 3.65, 3.67, 3.69, 3.71, 3.73, 3.75, 3.79, 3.82};
//float pos_bottom[559] = {0.0, 0.0, 0.01, 0.01, 0.02, 0.03, 0.05, 0.07, 0.09, 0.11, 0.13, 0.16, 0.19, 0.22, 0.26, 0.3, 0.34, 0.38, 0.43, 0.48, 0.53, 0.58, 0.64, 0.7, 0.76, 0.83, 0.89, 0.96, 1.04, 1.11, 1.19, 1.27, 1.35, 1.43, 1.52, 1.61, 1.7, 1.8, 1.89, 1.99, 2.09, 2.2, 2.3, 2.41, 2.53, 2.64, 2.75, 2.87, 2.99, 3.12, 3.24, 3.37, 3.5, 3.63, 3.76, 3.9, 4.04, 4.18, 4.32, 4.47, 4.61, 4.76, 4.91, 5.07, 5.22, 5.38, 5.54, 5.7, 5.86, 6.03, 6.19, 6.36, 6.53, 6.7, 6.88, 7.05, 7.23, 7.41, 7.59, 7.78, 7.96, 8.15, 8.33, 8.52, 8.71, 8.91, 9.1, 9.3, 9.49, 9.69, 9.89, 10.1, 10.3, 10.5, 10.7, 10.9, 11.1, 11.3, 11.5, 11.8, 12.0, 12.2, 12.4, 12.6, 12.8, 13.0, 13.3, 13.5, 13.7, 13.9, 14.2, 14.4, 14.6, 14.8, 15.1, 15.3, 15.5, 15.7, 16.0, 16.2, 16.4, 16.7, 16.9, 17.1, 17.4, 17.6, 17.8, 18.1, 18.3, 18.5, 18.8, 19.0, 19.2, 19.5, 19.7, 19.9, 20.2, 20.4, 20.6, 20.9, 21.1, 21.4, 21.6, 21.8, 22.1, 22.3, 22.5, 22.8, 23.0, 23.2, 23.5, 23.7, 23.9, 24.2, 24.4, 24.6, 24.9, 25.1, 25.3, 25.6, 25.8, 26.0, 26.3, 26.5, 26.7, 26.9, 27.2, 27.4, 27.6, 27.8, 28.1, 28.3, 28.5, 28.7, 29.0, 29.2, 29.4, 29.6, 29.8, 30.0, 30.2, 30.5, 30.7, 30.9, 31.1, 31.3, 31.5, 31.7, 31.9, 32.1, 32.3, 32.5, 32.7, 32.9, 33.1, 33.3, 33.5, 33.7, 33.9, 34.0, 34.2, 34.4, 34.6, 34.8, 34.9, 35.1, 35.3, 35.5, 35.6, 35.8, 36.0, 36.1, 36.3, 36.5, 36.6, 36.8, 36.9, 37.1, 37.2, 37.4, 37.5, 37.7, 37.8, 38.0, 38.1, 38.2, 38.4, 38.5, 38.6, 38.8, 38.9, 39.0, 39.1, 39.2, 39.4, 39.5, 39.6, 39.7, 39.8, 39.9, 40.0, 40.1, 40.2, 40.3, 40.4, 40.5, 40.6, 40.7, 40.7, 40.8, 40.9, 41.0, 41.0, 41.1, 41.2, 41.2, 41.3, 41.4, 41.4, 41.5, 41.5, 41.6, 41.6, 41.7, 41.7, 41.7, 41.8, 41.8, 41.8, 41.9, 41.9, 41.9, 41.9, 42.0, 42.0, 42.0, 42.0, 42.0, 42.0, 42.0, 42.0, 42.0, 42.0, 42.0, 42.0, 42.0, 41.9, 41.9, 41.9, 41.9, 41.8, 41.8, 41.8, 41.7, 41.7, 41.7, 41.6, 41.6, 41.5, 41.5, 41.4, 41.4, 41.3, 41.2, 41.2, 41.1, 41.0, 41.0, 40.9, 40.8, 40.7, 40.7, 40.6, 40.5, 40.4, 40.3, 40.2, 40.1, 40.0, 39.9, 39.8, 39.7, 39.6, 39.5, 39.4, 39.2, 39.1, 39.0, 38.9, 38.8, 38.6, 38.5, 38.4, 38.2, 38.1, 38.0, 37.8, 37.7, 37.5, 37.4, 37.2, 37.1, 36.9, 36.8, 36.6, 36.5, 36.3, 36.1, 36.0, 35.8, 35.6, 35.5, 35.3, 35.1, 34.9, 34.8, 34.6, 34.4, 34.2, 34.0, 33.9, 33.7, 33.5, 33.3, 33.1, 32.9, 32.7, 32.5, 32.3, 32.1, 31.9, 31.7, 31.5, 31.3, 31.1, 30.9, 30.7, 30.5, 30.2, 30.0, 29.8, 29.6, 29.4, 29.2, 29.0, 28.7, 28.5, 28.3, 28.1, 27.8, 27.6, 27.4, 27.2, 26.9, 26.7, 26.5, 26.3, 26.0, 25.8, 25.6, 25.3, 25.1, 24.9, 24.6, 24.4, 24.2, 23.9, 23.7, 23.5, 23.2, 23.0, 22.8, 22.5, 22.3, 22.1, 21.8, 21.6, 21.4, 21.1, 20.9, 20.6, 20.4, 20.2, 19.9, 19.7, 19.5, 19.2, 19.0, 18.8, 18.5, 18.3, 18.1, 17.8, 17.6, 17.4, 17.1, 16.9, 16.7, 16.4, 16.2, 16.0, 15.7, 15.5, 15.3, 15.1, 14.8, 14.6, 14.4, 14.2, 13.9, 13.7, 13.5, 13.3, 13.0, 12.8, 12.6, 12.4, 12.2, 12.0, 11.8, 11.5, 11.3, 11.1, 10.9, 10.7, 10.5, 10.3, 10.1, 9.89, 9.69, 9.49, 9.3, 9.1, 8.91, 8.71, 8.52, 8.33, 8.15, 7.96, 7.78, 7.59, 7.41, 7.23, 7.05, 6.88, 6.7, 6.53, 6.36, 6.19, 6.03, 5.86, 5.7, 5.54, 5.38, 5.22, 5.07, 4.91, 4.76, 4.61, 4.47, 4.32, 4.18, 4.04, 3.9, 3.76, 3.63, 3.5, 3.37, 3.24, 3.12, 2.99, 2.87, 2.75, 2.64, 2.53, 2.41, 2.3, 2.2, 2.09, 1.99, 1.89, 1.8, 1.7, 1.61, 1.52, 1.43, 1.35, 1.27, 1.19, 1.11, 1.04, 0.96, 0.89, 0.83, 0.76, 0.7, 0.64, 0.58, 0.53, 0.48, 0.43, 0.38, 0.34, 0.3, 0.26, 0.22, 0.19, 0.16, 0.13, 0.11, 0.09, 0.07, 0.05, 0.03, 0.02, 0.01, 0.01, 0.0, 0.0};

float load_raw[read_n_max];
float pos_raw[read_n_max];

float load_surf[chart_n_max];
float pos_surf[chart_n_max];

float load_bottom[chart_n_max];
float pos_bottom[chart_n_max];

/* ************************************************************************************************************** */
/* ************************************************* FUNCTIONS ************************************************** */
/* ************************************************************************************************************** */

/* ====================== SDCARD FUNCTIONS ======================== */
void appendFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)){
      Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

/* ====================== MQTT FUNCTIONS ======================== */
void mqttCallback(char *topic, byte *payload, unsigned int length);
void reconnect();
void OperationScreen(int16_t numberData, int16_t numberDataFill);

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    if (String(topic) == String(topicSubscribe) + String(WellName) + "/config")
    {
        String msg_in = "";
        for (int i = 0; i < length; i++)
        {
            msg_in += String((char)payload[i]);
        }
        TIME_TO_SLEEP = msg_in.toInt();
    }

    if (String(topic) == String(topicSubscribe) + String(WellName) + "/data")
    {

        DynamicJsonDocument doc(8000);

        String json = "";
        for (int i = 0; i < length; i++)
        {
            json += String((char)payload[i]);
        }

        //Serial.println(json);

        DeserializationError error = deserializeJson(doc, json);

        if (error)
        {
            delay(10);
            return;
        }


        String WN = doc["well"]; WellName = WN;
        String DT = doc["DT"]; DateTime = DT;
        String diag = doc["diagnosis"]; Diagnosis = diag;
        String st = doc["status"]; status = st;

        String fillPump = doc["fillPump"];
        String dtFillPump = doc["dtFillPump"];

        int16_t numberDataFill = s.separa(fillPump, ',', 0).toInt();

        FillPump = s.separa(fillPump, ',', numberDataFill);

        for (uint32_t i = 0; i <= numberDataFill; i++)
        {
            FillPumpList[i] = s.separa(fillPump, ',', i + 1).toFloat();
            dtFillPumpList[i] = s.separa(dtFillPump, ',', i).toFloat();
        }

        int16_t numberData = 0;

        if(String(status) == "running"){
            String pos_s = doc["pos"];
            String load_s = doc["load"];
            String SPM_c = doc["SPM"]; SPM = String(SPM_c);

            numberData = s.separa(pos_s, ',', 0).toInt();
            for (uint32_t i = 0; i <= numberData; i++)
            {
                pos_bottom[i] = s.separa(pos_s, ',', i + 1).toFloat();
                load_bottom[i] = s.separa(load_s, ',', i).toFloat();
            }
        }

        //Serial.println(String(numberData) + ":" + String(str_pos_bottom));

        OperationScreen(numberData, numberDataFill);
        lv_timer_handler();
        smartdisplay_set_led_color(lv_color32_t({.ch = {.blue = 0, .green = 0, .red = 0}}));
        delay(20000);
        digitalWrite(pinCC1W,LOW);
        gpio_hold_en(GPIO_NUM_1);

        digitalWrite(17,LOW);
        gpio_hold_en(GPIO_NUM_17);

        digitalWrite(4,LOW);
        gpio_hold_en(GPIO_NUM_4);

        digitalWrite(16,LOW);
        gpio_hold_en(GPIO_NUM_16);

        gpio_deep_sleep_hold_en();
        esp_deep_sleep_start();
    }
}

/* ******************* RECONNECT  ********************** */
void reconnect()
{
    int count = 0;
    while (!mqtt.connected())
    {
        if (mqtt.connect(mqtt_id, mqtt_user, mqtt_pass, topicPublish, 0, false, "Device conected"))
        {
            char topic1[25];
            String topicSubscribe1 = topicSubscribe + WellName + "/data";
            topicSubscribe1.toCharArray(topic1, (topicSubscribe1.length() + 1));
            mqtt.subscribe(topic1);
            // Serial.println(topicSubscribe1);

            delay(100);

            char topic2[25];
            String topicSubscribe2 = topicSubscribe + WellName + "/config";
            topicSubscribe2.toCharArray(topic2, (topicSubscribe2.length() + 1));
            mqtt.subscribe(topic2);
            // Serial.println(topicSubscribe2);
        }
        else
        {
            smartdisplay_set_led_color(lv_color32_t({.ch = {.blue = 255, .green = 0, .red = 255}}));
            if (count == 5)
            {
                ESP.restart();
            }
            delay(5000);
        }
        count++;
    }
}

/* ************************* SETUP WIFI ************************** */
void setup_wifi()
{
    WiFi.begin(ssid, password);
    delay(100);
    int count = 0;
    while (WiFi.status() != WL_CONNECTED)
    {
        //Serial.print(ssid);
        delay(5000);
        if (count == 3)
        {
            ESP.restart();
        }
        count++;
    }

    delay(2000);
}

/* ************************* ADD DATA FUNCTION FOR ACQUISITION ************************** */
static void add_data(lv_timer_t *timer)
{
    LV_UNUSED(timer);
    static uint32_t cnt = 0;

    filter_value_pos = (alpha_p * ADS.readADC(0) * f) + ((1 - alpha_p) * filter_value_pos);
    filter_value_load = (alpha_l * (ADS.readADC(1) - 7270)) + ((1 - alpha_l) * filter_value_load);
    pos_raw[cnt] = filter_value_pos;
    load_raw[cnt] = filter_value_load;

    //pos_raw[cnt] = 5.5;
    //load_raw[cnt] = 6.6;
    
    if (cnt % 5 == 0)
    {
        char payloadPos[10];
        Pos = String(pos_raw[cnt],1);
        Pos.toCharArray(payloadPos, (Pos.length() + 1));
        lv_label_set_text(label_box_pos, payloadPos);

        char payloadLoad[10];
        Load = String(load_raw[cnt],1);
        Load.toCharArray(payloadLoad, (Load.length() + 1));
        lv_label_set_text(label_box_load, payloadLoad);
    }
    lv_chart_set_next_value(posChart, pos_ser, pos_raw[cnt] * 10);
    lv_chart_set_next_value(loadChart, load_ser, load_raw[cnt] * 10);

    cnt++;
}

/* ************************* ACQUISITION SCREEN ************************ */
void AcquisitionScreen()
{
    // Clear screen
    lv_obj_clean(lv_scr_act());

    // Create Header
    static lv_style_t style_shadow;
    auto Header = lv_obj_create(lv_scr_act());
    lv_obj_add_style(Header, &style_shadow, 0);
    lv_obj_set_pos(Header, 1, 1);
    lv_obj_set_size(Header, 318, 30);

    // Set label to WellName
    auto WellName_label = lv_label_create(lv_scr_act());
    lv_obj_set_pos(WellName_label, 7, 5);
    lv_obj_set_style_text_color(WellName_label, lv_palette_main(LV_PALETTE_BROWN), LV_STATE_DEFAULT);
    lv_label_set_long_mode(WellName_label, LV_LABEL_LONG_WRAP); /*Break the long lines*/
    lv_obj_set_style_text_font(WellName_label, &lv_font_montserrat_22, LV_STATE_DEFAULT);

    char payloadWN[15];
    WellName.toCharArray(payloadWN, (WellName.length() + 1));
    lv_label_set_text(WellName_label, payloadWN);

    // Set label to DateTime
    auto DateTime_label = lv_label_create(lv_scr_act());
    lv_obj_set_pos(DateTime_label, 120, 5);
    lv_obj_set_style_text_color(DateTime_label, lv_palette_main(LV_PALETTE_BROWN), LV_STATE_DEFAULT);
    lv_label_set_long_mode(DateTime_label, LV_LABEL_LONG_WRAP); /*Break the long lines*/
    lv_obj_set_style_text_font(DateTime_label, &lv_font_montserrat_22, LV_STATE_DEFAULT);

    char payloadDT[20];
    DateTime.toCharArray(payloadDT, (DateTime.length() + 1));
    lv_label_set_text(DateTime_label, payloadDT);

    // ***************** Set label to message *****************
    Acquisition_label = lv_label_create(lv_scr_act());
    lv_obj_set_pos(Acquisition_label, 10, 32);
    lv_obj_set_style_text_color(Acquisition_label, lv_palette_main(LV_PALETTE_DEEP_ORANGE), LV_STATE_DEFAULT);
    lv_label_set_long_mode(Acquisition_label, LV_LABEL_LONG_SCROLL_CIRCULAR); /*Circular scroll*/
    lv_obj_set_width(Acquisition_label, 300);
    lv_obj_set_style_text_font(Acquisition_label, &lv_font_montserrat_22, LV_STATE_DEFAULT);
    lv_label_set_text(Acquisition_label, "Acquiring data");

    // ***************** Create Load text *****************
    lv_obj_t *box_load = lv_textarea_create(lv_scr_act());
    lv_textarea_set_one_line(box_load, true);
    lv_textarea_set_password_mode(box_load, false);
    lv_obj_set_pos(box_load, 5, 100);
    lv_obj_set_size(box_load, 85, 45);

    label_box_load = lv_label_create(box_load);

    lv_obj_center(label_box_load);
    lv_obj_set_style_text_font(label_box_load, &lv_font_montserrat_22, LV_STATE_DEFAULT);

    lv_obj_t *load_label = lv_label_create(lv_scr_act());
    lv_label_set_text(load_label, "Load:");
    lv_obj_align_to(load_label, box_load, LV_ALIGN_OUT_TOP_LEFT, 0, 0);

    // ***************** Create chart load *****************
    loadChart = lv_chart_create(lv_scr_act());
    lv_obj_set_pos(loadChart, 95, 100);
    lv_obj_set_size(loadChart, 220, 125);
    //lv_obj_set_style_line_width(loadChart, 2, LV_PART_ITEMS);   /*Remove the lines*/
    lv_obj_set_style_size(loadChart, 2, LV_PART_INDICATOR);
    lv_chart_set_type(loadChart, LV_CHART_TYPE_LINE);
    lv_chart_set_update_mode(loadChart, LV_CHART_UPDATE_MODE_CIRCULAR);
    load_ser = lv_chart_add_series(loadChart, lv_palette_main(LV_PALETTE_BROWN), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_range(loadChart, LV_CHART_AXIS_PRIMARY_Y, 0, 3500);
    lv_chart_set_point_count(loadChart, read_n_max);

    // ***************** Create POS text *****************
    lv_obj_t *box_pos = lv_textarea_create(lv_scr_act());
    lv_textarea_set_one_line(box_pos, true);
    lv_textarea_set_password_mode(box_pos, false);
    lv_obj_set_pos(box_pos, 5, 250);
    lv_obj_set_size(box_pos, 85, 45);

    label_box_pos = lv_label_create(box_pos);

    char payloadPos[20];
    Pos.toCharArray(payloadPos, (Pos.length() + 1));
    lv_label_set_text(label_box_pos, payloadPos);
    lv_obj_center(label_box_pos);
    lv_obj_set_style_text_font(label_box_pos, &lv_font_montserrat_22, LV_STATE_DEFAULT);

    lv_obj_t *pos_label = lv_label_create(lv_scr_act());
    lv_label_set_text(pos_label, "Pos:");
    lv_obj_align_to(pos_label, box_pos, LV_ALIGN_OUT_TOP_LEFT, 0, 0);

    // ***************** Create chart pos *****************
    posChart = lv_chart_create(lv_scr_act());
    lv_obj_set_pos(posChart, 95, 250);
    lv_obj_set_size(posChart, 220, 125);
    lv_obj_set_style_size(posChart, 2, LV_PART_INDICATOR);
    lv_chart_set_type(posChart, LV_CHART_TYPE_LINE); /*Show lines and points too*/
    lv_chart_set_update_mode(posChart, LV_CHART_UPDATE_MODE_CIRCULAR);
    pos_ser = lv_chart_add_series(posChart, lv_palette_main(LV_PALETTE_BLUE_GREY), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_range(posChart, LV_CHART_AXIS_PRIMARY_Y, 2800, 2900);
    lv_chart_set_point_count(posChart, read_n_max);

    lv_timer_t *timer = lv_timer_create(add_data, 2, NULL);
    lv_timer_set_repeat_count(timer, read_n_max - 1);
}

/* ************************* OPERATION SCREEN ************************ */
void OperationScreen(int16_t numberData, int16_t numberDataFill)
{
    // Clear screen
    lv_obj_clean(lv_scr_act());

    // Create Header
    static lv_style_t style_shadow;
    auto Header = lv_obj_create(lv_scr_act());
    lv_obj_add_style(Header, &style_shadow, 0);
    lv_obj_set_pos(Header, 1, 1);
    lv_obj_set_size(Header, 318, 30);

    // Set label to WellName
    auto WellName_label = lv_label_create(lv_scr_act());
    lv_obj_set_pos(WellName_label, 7, 5);
    lv_obj_set_style_text_color(WellName_label, lv_palette_main(LV_PALETTE_BROWN), LV_STATE_DEFAULT);
    lv_label_set_long_mode(WellName_label, LV_LABEL_LONG_WRAP); /*Break the long lines*/
    lv_obj_set_style_text_font(WellName_label, &lv_font_montserrat_22, LV_STATE_DEFAULT);

    char payloadWN[15];
    WellName.toCharArray(payloadWN, (WellName.length() + 1));
    lv_label_set_text(WellName_label, payloadWN);

    // Set label to DateTime
    auto DateTime_label = lv_label_create(lv_scr_act());
    lv_obj_set_pos(DateTime_label, 120, 5);
    lv_obj_set_style_text_color(DateTime_label, lv_palette_main(LV_PALETTE_BROWN), LV_STATE_DEFAULT);
    lv_label_set_long_mode(DateTime_label, LV_LABEL_LONG_WRAP); /*Break the long lines*/
    lv_obj_set_style_text_font(DateTime_label, &lv_font_montserrat_22, LV_STATE_DEFAULT);

    char payloadDT[20];
    DateTime.toCharArray(payloadDT, (DateTime.length() + 1));
    lv_label_set_text(DateTime_label, payloadDT);

    // ***************** Set label to Diagnosis *****************
    auto Diagnosis_label = lv_label_create(lv_scr_act());
    lv_obj_set_pos(Diagnosis_label, 10, 32);
    lv_obj_set_style_text_color(Diagnosis_label, lv_palette_main(LV_PALETTE_DEEP_ORANGE), LV_STATE_DEFAULT);
    lv_label_set_long_mode(Diagnosis_label, LV_LABEL_LONG_SCROLL_CIRCULAR); /*Circular scroll*/
    lv_obj_set_width(Diagnosis_label, 300);
    lv_obj_set_style_text_font(Diagnosis_label, &lv_font_montserrat_22, LV_STATE_DEFAULT);

    char payloadDignosis[100];
    Diagnosis.toCharArray(payloadDignosis, (Diagnosis.length() + 1));
    lv_label_set_text(Diagnosis_label, payloadDignosis);

    // ***************** Create dynachart *****************
    lv_obj_t *dynachart = lv_chart_create(lv_scr_act());
    lv_obj_set_pos(dynachart, 45, 65);
    lv_obj_set_size(dynachart, 180, 250);
    // lv_obj_add_event_cb(dynachart, draw_event_cb, LV_EVENT_DRAW_PART_BEGIN, NULL);
    lv_obj_set_style_line_width(dynachart, 1, LV_PART_ITEMS); /*Remove the lines*/

    lv_chart_set_type(dynachart, LV_CHART_TYPE_SCATTER);

    lv_chart_set_axis_tick(dynachart, LV_CHART_AXIS_PRIMARY_X, 10, 5, 5, 4, true, 30);
    lv_chart_set_axis_tick(dynachart, LV_CHART_AXIS_PRIMARY_Y, 10, 5, 6, 5, true, 50);

    lv_chart_set_range(dynachart, LV_CHART_AXIS_PRIMARY_X, 0, 430);
    lv_chart_set_range(dynachart, LV_CHART_AXIS_PRIMARY_Y, -50, 650);

    lv_chart_set_point_count(dynachart, numberData);

    lv_chart_series_t *ser = lv_chart_add_series(dynachart, lv_palette_main(LV_PALETTE_BROWN), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_series_t *ser1 = lv_chart_add_series(dynachart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_SECONDARY_Y);

    for (uint32_t i = 0; i < numberData; i++)
    {
        lv_chart_set_next_value2(dynachart, ser, pos_bottom[i] * 10, load_bottom[i] * 100);
        //lv_chart_set_next_value2(dynachart, ser1, pos_surf[i] * 10, load_surf[i] * 100);
    }

    // ***************** Create fill text *****************
    lv_obj_t *box_fill = lv_textarea_create(lv_scr_act());
    lv_textarea_set_one_line(box_fill, true);
    lv_textarea_set_password_mode(box_fill, false);
    lv_obj_set_pos(box_fill, 230, 75);
    lv_obj_set_size(box_fill, 85, 45);

    auto label_box_fill = lv_label_create(box_fill);

    char payloadFill[20];
    FillPump.toCharArray(payloadFill, (FillPump.length() + 1));
    lv_label_set_text(label_box_fill, payloadFill);
    lv_obj_center(label_box_fill);
    lv_obj_set_style_text_font(label_box_fill, &lv_font_montserrat_22, LV_STATE_DEFAULT);

    lv_obj_t *fill_label = lv_label_create(lv_scr_act());
    lv_label_set_text(fill_label, "Fill (%):");
    lv_obj_align_to(fill_label, box_fill, LV_ALIGN_OUT_TOP_LEFT, 0, 0);

    // ***************** Create SPM text *****************
    lv_obj_t *box_SPM = lv_textarea_create(lv_scr_act());
    lv_textarea_set_one_line(box_SPM, true);
    lv_textarea_set_password_mode(box_SPM, false);
    lv_obj_set_pos(box_SPM, 230, 150);
    lv_obj_set_size(box_SPM, 85, 45);

    auto label_box_SPM = lv_label_create(box_SPM);

    char payloadSPM[20];
    SPM.toCharArray(payloadSPM, (SPM.length() + 1));
    lv_label_set_text(label_box_SPM, payloadSPM);
    lv_obj_center(label_box_SPM);
    lv_obj_set_style_text_font(label_box_SPM, &lv_font_montserrat_22, LV_STATE_DEFAULT);

    lv_obj_t *SPM_label = lv_label_create(lv_scr_act());
    lv_label_set_text(SPM_label, "SPM (Hz):");
    lv_obj_align_to(SPM_label, box_SPM, LV_ALIGN_OUT_TOP_LEFT, 0, 0);

    // ***************** Create PEAK LOAD *****************
    lv_obj_t *box_peak = lv_textarea_create(lv_scr_act());
    lv_textarea_set_one_line(box_peak, true);
    lv_textarea_set_password_mode(box_peak, false);
    lv_obj_set_pos(box_peak, 230, 225);
    lv_obj_set_size(box_peak, 85, 45);

    auto label_box_peak = lv_label_create(box_peak);

    char payloadPeak[20];
    PeakLoad.toCharArray(payloadPeak, (PeakLoad.length() + 1));
    lv_label_set_text(label_box_peak, payloadPeak);
    lv_obj_center(label_box_peak);
    lv_obj_set_style_text_font(label_box_peak, &lv_font_montserrat_22, LV_STATE_DEFAULT);

    lv_obj_t *Peak_label = lv_label_create(lv_scr_act());
    lv_label_set_text(Peak_label, "Peak (KLb):");
    lv_obj_align_to(Peak_label, box_peak, LV_ALIGN_OUT_TOP_LEFT, 0, 0);

    // ***************** Create MIN LOAD *****************
    lv_obj_t *box_min = lv_textarea_create(lv_scr_act());
    lv_textarea_set_one_line(box_min, true);
    lv_textarea_set_password_mode(box_min, false);
    lv_obj_set_pos(box_min, 230, 300);
    lv_obj_set_size(box_min, 85, 45);

    auto label_box_min = lv_label_create(box_min);

    char payloadMin[20];
    MinLoad.toCharArray(payloadMin, (MinLoad.length() + 1));
    lv_label_set_text(label_box_min, payloadMin);
    lv_obj_center(label_box_min);
    lv_obj_set_style_text_font(label_box_min, &lv_font_montserrat_22, LV_STATE_DEFAULT);

    lv_obj_t *Min_label = lv_label_create(lv_scr_act());
    lv_label_set_text(Min_label, "Min (KLb):");
    lv_obj_align_to(Min_label, box_min, LV_ALIGN_OUT_TOP_LEFT, 0, 0);

    // ***************** Create fillpump plot *****************
    lv_obj_t *fillchart = lv_chart_create(lv_scr_act());
    lv_obj_set_pos(fillchart, 45, 370);
    lv_obj_set_size(fillchart, 270, 80);
    // lv_obj_add_event_cb(fillchart, draw_event_cb, LV_EVENT_DRAW_PART_BEGIN, NULL);
    lv_obj_set_style_line_width(fillchart, 1, LV_PART_ITEMS); /*Remove the lines*/
    lv_obj_set_style_size(fillchart, 2, LV_PART_INDICATOR);

    lv_chart_set_type(fillchart, LV_CHART_TYPE_SCATTER);
    // lv_chart_set_update_mode(fillchart,LV_CHART_UPDATE_MODE_CIRCULAR);
    // lv_chart_set_range(fillchart, LV_CHART_AXIS_PRIMARY_Y, 0, 100);
    // lv_chart_set_point_count(fillchart, 250);

    lv_chart_set_axis_tick(fillchart, LV_CHART_AXIS_PRIMARY_X, 5, 5, 7, 5, true, 30);
    lv_chart_set_axis_tick(fillchart, LV_CHART_AXIS_PRIMARY_Y, 2, 1, 2, 2, true, 10);

    lv_chart_set_range(fillchart, LV_CHART_AXIS_PRIMARY_X, 0, 240);
    lv_chart_set_range(fillchart, LV_CHART_AXIS_PRIMARY_Y, 0, 100);

    lv_chart_set_point_count(fillchart, 250);

    lv_chart_series_t *ser2 = lv_chart_add_series(fillchart, lv_palette_main(LV_PALETTE_BROWN), LV_CHART_AXIS_PRIMARY_Y);

    // lv_chart_set_next_value(posChart, pos_ser, pos_raw[cnt]*10);
    for (uint32_t i = 1; i < numberDataFill; i++)
    {
        lv_chart_set_next_value2(fillchart, ser2, dtFillPumpList[i] * 10, FillPumpList[i]);
    }

    // Set label tittle plot
    auto plot_label = lv_label_create(lv_scr_act());
    lv_obj_set_pos(plot_label, 10, 350);
    lv_obj_set_style_text_color(plot_label, lv_palette_main(LV_PALETTE_BROWN), LV_STATE_DEFAULT);
    lv_label_set_long_mode(plot_label, LV_LABEL_LONG_WRAP); /*Break the long lines*/
    lv_obj_set_style_text_font(plot_label, &lv_font_montserrat_14, LV_STATE_DEFAULT);
    lv_label_set_text(plot_label, "Fill pump for this day");
}

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

/* ********************** SEND DATA *********************** */
void sendData(String mqtt_payload)
{
    char payload[6000];
    mqtt_payload.toCharArray(payload, (mqtt_payload.length() + 1));
    mqtt.publish(topicPublish, payload);
}

/* ********************** SETUP *********************** */
void setup()
{
    smartdisplay_init();
    if(!SD.begin(5)){
        smartdisplay_set_led_color(lv_color32_t({.ch = {.blue = 255, .green = 255, .red = 255}}));
        return;
    }

    if(bootCount == 0){
        rtc.setTime(0,0,0,16,6,2023);
    }

    ++bootCount;

    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    
    ADS.begin();
    f = ADS.toVoltage(1) * 100;
    ADS.setDataRate(5);

    pinMode(pinCC1W, OUTPUT);
    digitalWrite(pinCC1W, HIGH);
    delay(3000);

    AcquisitionScreen();

    smartdisplay_set_led_color(lv_color32_t({.ch = {.blue = 0, .green = 255, .red = 0}}));

    start_time = millis();
    for (uint32_t i = 0; i < read_n_max; i++)
    {
        lv_timer_handler();
        delay(25);
    }
    
    end_time = millis() - start_time;

    digitalWrite(pinCC1W, LOW);

    // ================================ PROCESS DATA ================================

    Average<float> rpos(read_n_max);
    Average<float> rload(read_n_max);

    float maxPos = 0;
    float minPos = 0;

    for (uint16_t i = 0; i < read_n_max - 1; i++)
    {
        rpos.push(pos_raw[i]);
        rload.push(load_raw[i]);
    }

    int max_pos_index = 0;
    int min_pos_index = 0;

    maxPos = rpos.maximum(&max_pos_index);
    minPos = rpos.minimum(&min_pos_index);

    if(mode == 1){
        setup_wifi();
        mqtt.setServer(broker, mqtt_port);
        mqtt.setCallback(mqttCallback);
        reconnect();
    }
    
    // ================================ DETECT PUMP STOPPED ================================
    String mqtt_payload;
    if (abs(maxPos - minPos) <= 0.5)
    {
        mqtt_payload = "{\"well\":\"" + WellName + "\",\"status\":\"stopped\"}";
        
        sendData(mqtt_payload);
        Diagnosis = "Stopped";
        
        lv_timer_handler();
        lv_timer_handler();
    }
    else
    {
        // ================================ FILTER STROKE ================================
        float value;
        int16_t i_start = 0;
        int16_t i_flag = 0;
        int16_t i_end = 0;
        float tp = 0;
        float diff;
        float range = maxPos - minPos;

        for (int16_t i = 0; i < read_n_max - 1; i++)
        {
            value = pos_raw[i];
            diff = value - minPos;
            if(diff > 0.8 * range && i_flag == 0 && i_end == 0){
                if(diff > tp){
                    tp = diff;
                    i_start = i;
                }
            }
            else if (i_start != 0 && diff < 0.2 * range && i_end == 0){
                if(diff < tp){
                    tp = diff;
                    i_flag = i;
                }
            }
            else if(i_flag != 0 && diff > 0.8 * range){
                if(diff > tp){
                    tp = diff;
                    i_end = i;
                }
            }
            else if(diff < 0.2 * range && i_end != 0){break;}
        }

        String tempPeakLoad = String(rload.get(i_start-1), 2);
        char payloadPeak[20];
        tempPeakLoad.toCharArray(payloadPeak, (tempPeakLoad.length() + 1));
        PeakLoad = payloadPeak;

        String tempMinLoad = String(rload.get(i_flag-1), 2);
        char payloadMin[20];
        tempMinLoad.toCharArray(payloadMin, (tempMinLoad.length() + 1));
        MinLoad = payloadMin;

        //Serial.println( String(SPM) + ", " + String(PeakLoad) + ", " + String(MinLoad));

        // ======== TRACE IN  STROKE ========        
        load_ser->y_points[i_start + 1] = 1600;
        load_ser->y_points[i_end + 1] = 1600;

        pos_ser->y_points[i_start + 1] = 0;
        pos_ser->y_points[i_end + 1] = 0;

        lv_chart_refresh(loadChart);
        lv_chart_refresh(posChart);

        lv_timer_handler();
        lv_timer_handler();

        //Serial.println("markers: " + String(i_start) + "," + String(i_end) + "->" + String(min_pos_index) + "," + String(max_pos_index));

        String rawdataPos = "";
        String rawdataLoad = "";
        int16_t cont = 0;
        float t_pos = 0;
        float t_load = 0;

        for (uint16_t i = i_start; i < i_end-1; i++)
        {
            // mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
            t_pos = mapfloat(rload.get(i),280,290,0,42);
            //t_pos = (cos(PI + mapfloat(cont,0,(i_end - i_start - 1),0,2*PI)) + 1) * 21;

            t_load = mapfloat(rload.get(i),0,300,0,6.5);
            //t_load = rload.get(i);
            rawdataPos += String(t_pos, 2) + ",";
            rawdataLoad += String(t_load, 2) + ",";

            pos_surf[cont] = t_pos;
            load_surf[cont] = t_load;

            cont++;
        }
        //rawdataPos += String(rpos.get(i_end), 2);
        t_pos = mapfloat(rload.get(i_end),280,290,0,42);
        rawdataPos += String(0.00, 2);

        t_load = mapfloat(rload.get(i_end),0,300,0,6.5);
        rawdataLoad += String(t_load, 2);

        mqtt_payload = "{\"well\":\"" + WellName + "\",\"status\":\"running\"" + ",\"rate\":\"" + String(end_time) + "\",\"pos\":\"" + rawdataPos + "\"" + ",\"load\":\"" + rawdataLoad + "\"}";
        sendData(mqtt_payload);
    }
    mqtt_payload = rtc.getDateTime() + "," + mqtt_payload + "\n";
    appendFile(SD, "/data.txt", mqtt_payload.c_str());
}

void loop()
{
    if(mode == 1){
        if (!mqtt.connected())
        {
            reconnect();
        }

        mqtt.loop();
    }
    
    delay(10);

    unsigned long currentMillis = millis();

    if(currentMillis - previousMillis > 30000) {
        digitalWrite(pinCC1W,LOW);
        gpio_hold_en(GPIO_NUM_1);

        digitalWrite(17,LOW);
        gpio_hold_en(GPIO_NUM_17);

        digitalWrite(4,LOW);
        gpio_hold_en(GPIO_NUM_4);

        digitalWrite(16,LOW);
        gpio_hold_en(GPIO_NUM_16);

        gpio_deep_sleep_hold_en();
        esp_deep_sleep_start();
        previousMillis = currentMillis; 
    }
}