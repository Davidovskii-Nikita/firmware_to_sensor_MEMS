#ifndef CORE_H
#define CORE_H
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <ESP8266HTTPUpdateServer.h>
#include <Ticker.h>
#include <PubSubClient.h>
// Блок настройки пользователем
//====================================================================================
// const char* ssid = "RUT230_9A75";// название WiFi сети
// const char* password = "z1x2c3v4b5" ;// пароль WiFi сети
const char* ssid = "Davidovskii";// название WiFi сети
const char* password = "4054414LabU" ;// пароль WiFi сети
const char* mqtt_server = "M5.WQTT.RU";
const char* login = "u_WUYIDH";
const char* pass_mqtt = "kVu4uMTX";
const int full_scale_range = 16; // диапазон измерений акселерометра( 2, 4, 8, 16)
const uint16_t period_a = 250; // Частота записи виброскорости
const uint16_t period_temp = 2500; // Частота записи темперартуры
const uint16_t period_v = 25; // Частота итегрирования виброускрорения
const uint16_t range_a = 100; // колличество значений виброскорости в 1 пакете
const uint16_t range_temp = 10; // колличество значений времени в 1 пакете
//====================================================================================
// Адреса регистров MPU6050
//====================================================================================
const uint8_t MPU6050SlaveAddress = 0x68; // адрес MPU6050 в сети I2C
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B; // настройка режима питания и источнок часов
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C; // настройка чувтсвительности акселерометра в 4 и 3 бит. 00 = 2, 01 = 4, 10 = 8, 11 = 16.
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38; // настройка прерываний
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B; // MSB ускорения по оси X
const uint8_t MPU6050_REGISTER_ACCEL_YOUT_H =  0x3D; // MSB ускорения по оси Y
const uint8_t MPU6050_REGISTER_ACCEL_ZOUT_H =  0x3F; // MSB ускорения по оси Z
const uint8_t MPU6050_REGISTER_TEMP =  0x41; // МSB значения температуры
//====================================================================================
// Функциональные переменные
//====================================================================================
String MAC; // MAC адресс устройсва
size_t capacity; // объем Json 
float vibro_speed_x; // содержит значение виброскорости по оси x
float vibro_speed_y; // содержит значение виброскорости по оси y
float vibro_speed_z; // содержит значение виброскорости по оси z
float old_x; // для дискретного интегрирования
float old_y; // для дискретного интегрирования
float old_z; // для дискретного интегрирования
float rms; // СКЗ
uint16_t scale_factor; // для расчета 
const float g = 9.81; // постоянная ускорения
String opros_axel [range_a]; // значения ускорения для json
String opros_axel_time [range_a];// значения времения по ускорения для json
String opros_temp [range_temp]; // значения температуры для json
String opros_temp_time [range_temp]; // значения времени по температуре для json
uint16_t count_a,count_temp; // счетчики точек температуры и вибросокорости
bool flag_a, flag_temp; // флаги достижения счетчиками range_a range_temp
const uint16_t for_scale = 32768; // для расчета делителя ускорения
uint16_t local_time_ms;
double sync_time; // глобальная переменная, содержащая UNIX время в момент старта ESP
const char* host_OTA = "esp-8266_E6_3C";// название устройства в локальной сети для прошивки через браузер
const char* serverIndex = "<title>Update ESP</title><h1> Update ESP8266 Update_why_not  </h1><img src = ""https://raw.githubusercontent.com/AchimPieters/ESP8266-12F---Power-Mode/master/ESP8266_01X.jpg""><form method='POST' action='/update' enctype='multipart/form-data'> <input type='file' name='update'><input type='submit' value='Update'></form>";
const char* update_path = "/firmware";
double offset_startup_time = 0;
// Фильтр бегущего среднего
//====================================================================================
float k = 0.05; // настройка фильтра
float filVal_x = 0; // содержит отфильтрованиие и нормализированное значение по оси x
float filVal_y = 0; // содержит отфильтрованиие и нормализированное значение по оси y
float filVal_z = 0; // содержит отфильтрованиие и нормализированное значение по оси z
float time_to_calibr = 100;// колличество точек фильтрования
//====================================================================================
ESP8266WebServer server(80);
ESP8266HTTPUpdateServer httpUpdater;
Ticker Ticker_A, Ticker_T, Ticker_V; // инициализация счетчиков

// Инициализация функций
//====================================================================================
void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data);// функция записи значений по I2C
int16_t I2C_Read(uint8_t deviceAddress, uint8_t regAddress); // функция чтения значений 2 байт по I2C
double update_ntp(); // функция получения времени
double get_time(double s_t, double offset); // функция синхронизации времени s_t - время UNIX (sync_time)
float get_value_from_reg (uint8_t dev_adress, uint8_t reg_adress, uint16_t scale_factor); // получение вещественного значения из регистров
float get_RMS(float a, float b, float c); // расчет СКЗ
void get_vibrospeed();//интешрирование ускорения, управляется таймером Ticker_V
void upate_vibrospeed_value(); //запись значения виброскорости в массив с меткой времени. Управляется таймером Ticker_A
float get_temp(int16_t temp); // получение вещественного значения температуры
void update_temperature_value(); // запись значения температуры в массив с меткой времени. Управляется таймером Ticker_T
void post_json(); // создает и отправляет json документ
void calibration(); // получает значения углов, для нивелирования постоянной 1g
void expRunningAverage(float x ,float y, float z); // бегущее среднее для 3 осей
float clamp(float v, float minv, float maxv); // функция clamp если больше maxv то v если меньше min то м
void discret_integral(float x, float y, float z); // дискретное интегрирование по 3 осям
//====================================================================================
#endif