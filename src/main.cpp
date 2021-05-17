#include <Arduino.h>
#include "core.h"

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