#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266HTTPUpdateServer.h>
#include <Ticker.h>
// Блок настройки пользователем
// ====================================================================================
const char* ssid = "Keenetic-8735";// название WiFi сети
const char* password = "hj838RRe" ;// пароль WiFi сети
const int full_scale_range = 16; // диапазон измерений акселерометра( 2, 4, 8, 16)
const uint16_t period_a = 250; // Частота записи виброскоростиz
const uint16_t period_temp = 2500; // Частота записи темперартуры
const uint16_t period_v = 25; // Частота итегрирования виброускрорения
// const uint16_t period_c = 200; // Частота калибрования 
const uint16_t range_a = 100; // колличество значений виброскорости в 1 пакете
const uint16_t range_temp = 10; // колличество значений времени в 1 пакете
String URL1 = "http://192.168.0.24:8001/nkvm"; // адрес куда отправляются POST запросы
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
// Константы каллибровки
//====================================================================================
float norm_x = 0.0;
float norm_y = 0.0;
float norm_z = 0.0;
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
const char* host_OTA = "esp-8266_n1";// название устройства в локальной сети для прошивки через браузер
const char* serverIndex = "<title> Update ESP</title><h1> Number X (partia 14.05)  </h1><img src = ""https://raw.githubusercontent.com/AchimPieters/ESP8266-12F---Power-Mode/master/ESP8266_01X.jpg""><form method='POST' action='/update' enctype='multipart/form-data'> <input type='file' name='update'><input type='submit' value='Update'></form>";
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
float get_value_from_reg (uint8_t dev_adress, uint8_t reg_adress, uint16_t scale_factor, float const_to_norm); // получение вещественного значения из регистров
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


void setup() 
{
  extern double sync_time;
  extern String MAC;
  extern uint16_t scale_factor;
  WiFi.begin(ssid, password);
  delay(100);
  while (WiFi.status() != WL_CONNECTED)
  {
    offset_startup_time = (double)millis()/1000;
    delay(1500);
  }

  WiFi.mode(WIFI_AP_STA); 
  // Создание WEB-страницы
  //====================================================================================
if (WiFi.waitForConnectResult() == WL_CONNECTED) {
    MDNS.begin(host_OTA);
    server.on("/", HTTP_GET, []() {
      server.sendHeader("Connection", "close");
      server.send(200, "text/html", serverIndex);
    });
    server.on("/update", HTTP_POST, []() {
      server.sendHeader("Connection", "close");
      server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
      ESP.restart();
    }, []() {
      HTTPUpload& upload = server.upload();
      if (upload.status == UPLOAD_FILE_START) {

        Ticker_A.detach();
        Ticker_V.detach();
        Ticker_T.detach();

        Serial.setDebugOutput(true);
        WiFiUDP::stopAll();
        Serial.printf("Update: %s\n", upload.filename.c_str());
        uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        if (!Update.begin(maxSketchSpace)) { //start with max available size
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) { //true to set the size to the current progress
          Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        } else {
          Update.printError(Serial);
        }
        Serial.setDebugOutput(false);
      }
      yield();
    });
    server.begin();
    MDNS.addService("http", "tcp", 80);

    Serial.printf("Ready! Open http://%s.local in your browser\n", host_OTA);
  } 
  //====================================================================================
  Serial.begin(9600); // отладка по последовательному порту
  Wire.begin(2, 0);  // инициализация I2C на GPIO 2 и 0 (SDA, SCL)
  sync_time = update_ntp(); // получение UNIX-времени
  scale_factor = for_scale/full_scale_range; // расчет делителя в зависимости от выбранной чувтсвиттельности
  // Блок инициализации MPU6050;
  //====================================================================================
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1,0x01);
    switch (full_scale_range)
    {
    case 16:
      I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x18);
      break;
    case 8:
      I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x10);
      break;
    case 4:
      I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x08);
      break;
    case 2:
      I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);
      break;  
    default:
      I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x18);
      break;
    }
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  //====================================================================================
  MAC = WiFi.macAddress(); // Получение MAC адреса устройства
  capacity = 2 * JSON_ARRAY_SIZE(range_temp) + 2 * JSON_ARRAY_SIZE(range_a) + JSON_OBJECT_SIZE(5) + 3500; // вычисление объема JSON файла
  local_time_ms = millis();//????
  calibration();
  WiFi.setOutputPower(0); //???
  // Блок инициализации таймеров
  // Объект класса Ticker вызывает функцию attach_ms с параметрами 
  // периода вызова и функци вызова
  //====================================================================================
  Ticker_V.attach_ms(period_v,get_vibrospeed);
  Ticker_A.attach_ms(period_a,upate_vibrospeed_value);
  Ticker_T.attach_ms(period_temp,update_temperature_value);
  //====================================================================================
}

void loop() 
{
  // Отображение страницы для OTA-update
  server.handleClient();
  MDNS.update();
  extern bool flag_a, flag_temp;
  extern uint16_t count_temp, count_a;
  if(flag_a && flag_temp)
  {
    post_json(); // сбор и отправка данных
    count_temp = 0; // обнуление всех счетчиков и флагов для следующей иттерации
    count_a = 0;
    flag_a = false;
    flag_temp = false;
  }
 
}

  /**
   *  @brief  Считает значение виброскорости.
   *  @param  scale_factor  Внешняя переменная для получения нормального значения ускорения.
   *  @return Ничего не возвращает, обновляет глобальную перемнную rms.
  */
void get_vibrospeed()
{
  extern uint16_t scale_factor;
  //получение значений ускорений
  float X,Y,Z;
  X = get_value_from_reg(MPU6050SlaveAddress,MPU6050_REGISTER_ACCEL_XOUT_H,scale_factor,norm_x) - filVal_x;
  Y = get_value_from_reg(MPU6050SlaveAddress,MPU6050_REGISTER_ACCEL_YOUT_H,scale_factor,norm_y) - filVal_y;
  Z = get_value_from_reg(MPU6050SlaveAddress,MPU6050_REGISTER_ACCEL_ZOUT_H,scale_factor,norm_z) - filVal_z;
  // интегрирование по 3 осям
  discret_integral(X,Y,Z);
  // получение СКЗ
  rms = get_RMS(vibro_speed_x,vibro_speed_y,vibro_speed_z);
} 
 
  /**
   *  @brief  Наполняет глобальные массивы данных(opros_axel[], opros_axel_time[]), сигнализирует о наполненности последних.
   *  @param  flag_a  Глобальный флаг, сигнализирующий о заполненности массивов.
   *  @param  count_a  Колличество значений, записываемых в массивы
   *  @param  rms Глобальная переменная среднеквадратичного значения виброскорости по трем осям.
  */
void upate_vibrospeed_value()
{
  extern bool flag_a;
  extern uint16_t count_a;
  extern float rms;
  String time_to_json;
  String speed_to_json;
  calibration(); 
  if (count_a<range_a)
  {
    time_to_json = String(get_time(sync_time,offset_startup_time)*1000.0);
    speed_to_json =String(rms);
    // Serial.println(rms);
    opros_axel[count_a]=speed_to_json; // запись виброскорости и времени в массивы
    opros_axel_time[count_a]=time_to_json;
    ++count_a;
    flag_a = false;

    vibro_speed_x = 0; // неоднозначно
    vibro_speed_y = 0;
    vibro_speed_z = 0;
  }
  else
  {
    flag_a = true;
    vibro_speed_x = 0;
    vibro_speed_y = 0;
    vibro_speed_z = 0;
  }

}

  /**
   *  @brief  Наполняет глобальные массивы данных(opros_temp[], opros_temp_time[]), сигнализирует о наполненности последних.
   *  @param  flag_t  Глобальный флаг, сигнализирующий о заполненности массивов.
   *  @param  count_t  Колличество значений, записываемых в массивы
  */
void update_temperature_value()
{
  extern bool flag_temp;
  extern uint16_t count_temp;
  String time_to_json;
  String temp_to_json;
  int16_t res_from_i2c;

  if(count_temp<range_temp)
  {
    res_from_i2c = I2C_Read(MPU6050SlaveAddress, MPU6050_REGISTER_TEMP);
    time_to_json = String(get_time(sync_time,offset_startup_time)*1000.0);
    temp_to_json = String(get_temp(res_from_i2c));
    opros_temp[count_temp]=temp_to_json;// запись температуры и времени в массив
    opros_temp_time[count_temp]=time_to_json;
    ++count_temp;
    flag_temp = false;
  }
  else
  {
    flag_temp = true;
  }
}

  /**
   *  @brief  Формирование json- -документа и отправка его.
  */
void post_json()
{
  WiFiClient client;
  HTTPClient http;
  String buffer; // локальны буффер json документа
  String dat="data=";

  DynamicJsonDocument jsonDocument(capacity); // объявление динамического jsom документа
  jsonDocument ["MAC"] = MAC; // добавление MAC адресаа в JSON
  // добавление соответвствующих массивов в json
  JsonArray Axel_time = jsonDocument.createNestedArray("Axel_time");
  JsonArray Axel = jsonDocument.createNestedArray("Axel");
  JsonArray Temp_time = jsonDocument.createNestedArray("Temp_time");
  JsonArray Temp = jsonDocument.createNestedArray("Temp");

  int i = 0;
  int j = 0;

  for(i = 0; i < range_temp; i++)// наполнеие массивов данными
  {
    Temp.add(opros_temp[i]);
    Temp_time.add(opros_temp_time[i]);
  }
  for(j=0; j<range_a; j++)// наполнеие массивов данными
  {
    Axel.add(opros_axel[j]);
    Axel_time.add(opros_axel_time[j]);
  }
  
  serializeJson(jsonDocument, buffer); // создание заполненного json

  // String new_buffer = dat + buffer;

  // http.begin(client, URL1);// отправка
  // http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  // http.POST(buffer);
  // http.end();

  http.begin(client, URL1);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  http.POST(buffer);
  http.end();
}

  /**
   *  @brief  Калибровка данных, получаемых с каждой оси путем расчета експоненциального бегущего среднего.
  */
void calibration()
{
  float x,y,z;
  float i = 0;
  while( i < time_to_calibr)
  {
    x = get_value_from_reg(MPU6050SlaveAddress,MPU6050_REGISTER_ACCEL_XOUT_H,scale_factor, norm_x);
    y = get_value_from_reg(MPU6050SlaveAddress,MPU6050_REGISTER_ACCEL_YOUT_H,scale_factor, norm_y);
    z = get_value_from_reg(MPU6050SlaveAddress,MPU6050_REGISTER_ACCEL_ZOUT_H,scale_factor, norm_z);
    x = clamp(x,-1.0,1.0);
    y = clamp(y,-1.0,1.0);
    z = clamp(z,-1.0,1.0);
    expRunningAverage(x, y, z);
    ++i;
  }
  i=0;
}

  /**
   *  @brief  Расчет бегущего среднего.
   *  @param  x,y,z  Для каждой переменной считается бегущее среднее.
   *  @return Ничего не возвращает, обновляет глобальные переменные (filVal_x, filVal_y, filVal_z)
  */
void expRunningAverage(float x ,float y, float z)
{
  filVal_x += (x - filVal_x) * k;
  filVal_y += (y - filVal_y) * k;
  filVal_z += (z - filVal_z) * k;
}

  /**
   *  @brief  Дискретеое интегрирование.
   *  @param  x,y,z  Для каждой переменной считается дискрентное интегрирование.
   *  @return Ничего не возвращает, обновляет глобальные переменные (vibro_speed_x, vibro_speed_y, vibro_speed_z)
  */
void discret_integral(float x, float y, float z)
{
  vibro_speed_x = vibro_speed_x + ((x+old_x)*(period_v/2000.0)*g)*(1000.0/period_v); // дискретное интегрирование методом трапеций
  old_x=x;
  vibro_speed_y = vibro_speed_y + ((y+old_y)*(period_v/2000.0)*g)*(1000.0/period_v); // дискретное интегрирование методом трапеций
  old_y=y;
  vibro_speed_z = vibro_speed_z + ((z+old_z)*(period_v/2000.0)*g)*(1000.0/period_v); // дискретное интегрирование методом трапеций
  old_z=z;
}

  /**
   *  @brief  Запись значения в регистр шины I2C.
   *  @param  deviceAddress  Адресс устройства в шине I2C.
   *  @param  regAddress  Адрес регистра, в который записываеься значение.
   *  @param  data Записываемая переменная.
  */
void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data)
{
    Wire.beginTransmission(deviceAddress);
    Wire.write(regAddress);
    Wire.write(data);
    Wire.endTransmission();
}
  /**
   *  @brief  Чтение данных с шины I2C.
   *  @param  deviceAddress  Адресс устройства в шине I2C.
   *  @param  regAddress  Адрес регистра, с которого считываются значения.
   *  @return Возвращает значение, содержащееся в регистре.
  */
int16_t I2C_Read(uint8_t deviceAddress, uint8_t regAddress)
{
    int16_t responce;
    Wire.beginTransmission(deviceAddress);
    Wire.write(regAddress);
    Wire.endTransmission();
    Wire.requestFrom(deviceAddress, (uint8_t)2);
    responce = (((int16_t)Wire.read() << 8) | Wire.read());

    return responce;
}

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "by.pool.ntp.org");     // выбор сервера NTP("by.pool.ntp.org"), cмещение пояса(10800)

  /**
   *  @brief  Получение времени в UNIX формате
   *  @return Возвращает полученное время.
  */
double update_ntp() 
{   
    double s_t; 
    timeClient.begin(); 
    delay(100);
    timeClient.update();
    s_t = timeClient.getEpochTime();
    if ( s_t > 2000000000 || s_t < 15000.0)
        {
        update_ntp(); // рекурсия этой функции пока не получим "нормальное" время
        }
    return s_t;
}   
  /**
   *  @brief  Функия получения времени.
   *  @param  s_t Содержит время с момента начала работы контроллера.
   *  @param  offset  Смещение во времени ( за какое время контроллер прошел setup()).
   *  @return Возвращает время в UNIX формате с милисекундами.
  */
double get_time(double s_t, double offset)
{ 
  return  ((double)millis())/1000 + s_t - offset;
}
  /**
   *  @brief  Считает среднеквадратичное значение.
   *  @param  а,b,c Переменные, между которыми считается СКЗ.
   *  @return Возвращает СКЗ.
  */
float get_RMS(float a, float b, float c)
{
    float sum, root, RMS;
    a = powf(a,2);
    b = powf(b,2);
    c = powf(c,2);
    sum = a+b+c;
    root = sum/3;
    RMS = powf(root,0.5);

    return RMS;
}

  /**
   *  @brief  Считывает значения с определенного регистра, приводит к нормальному виду и содержит каллибровку.
   *  @param  dev_adress  Адресс устройства в шине I2C.
   *  @param  reg_adress  Адрес регистра, с которого считываются значения.
   *  @param  scale_factor Переменная, с помощью которой нормируют принимаемое из регистра значение.
   *  @param  const_to_norm ДОБАВЛЯЕТСЯ к полученному нормированному значению, используется для каллибровки.
   *  @return Возвращает откалиброванное, нормированное значение, содержащееся в регистре.
  */
float get_value_from_reg (uint8_t dev_adress, uint8_t reg_adress, uint16_t scale_factor, float const_to_norm)
{
    float val_accel;
    int16_t res;
    res = I2C_Read(dev_adress,reg_adress);
    val_accel=(float)res/scale_factor;

    return val_accel+const_to_norm;
}

  /**
   *  @brief  Получение времени в Цельсиях (по формуле из даташита MPU6050).
   *  @param  temp  "Сырая" температура, значение из регистра.
   *  @return Возвращает температуру в Цельсиях.
  */
float get_temp(int16_t temp)
{
    return ((float)temp)/340+36.53; // формула расчета температуры из datasheet
}

  /**
   *  @brief  Ппринимает в качестве аргументов три разделённых запятой выражения, указываемых в порядке: минимальное значение, предпочитаемое значение, максимальное значение.
   *  @param  v  Обрабатываемое значение
   *  @param  minv  Минимальный порог.
   *  @param  maxv  Максимальный порог.
   *  @return Нормированное значение.
  */
float clamp(float  v, float minv, float maxv)
{
  if( v>maxv )
      return maxv;
  else if( v<minv )
      return minv;
  return v;
}
