# Датчик вибрации.


### Содержанание:
* Введение
* Прицип действия, описание прошивки.
* Сборка.
* Отладка.
* Калибровка.
* Возможные проблемы.

---
### Введение.

В основе устройства лежит чувствительный элемент в виде MEMS-датчика **MPU6050** в модуле GY-521.

![Внешний вид GY-521](https://github.com/Davidovskii-Nikita/firmware_to_sensor_MEMS/blob/master/docs/mpu-6050-01.jpg)

Его [даташит](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf) и [карта регистров](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf). Подключение к нему осуществляется по I2C шине. Для этого используется библиотека *Wire.h*. 


Сбором и хранением данных занимается микроконтроллер **ESP 8266** на базе модуля ESP-01. 

![Распиновка ESP-01](https://raw.githubusercontent.com/AchimPieters/ESP8266-12F---Power-Mode/master/ESP8266_01X.jpg)

Для организвации питания используется DC/DC преобразователь **Mini-360**

![Внешний вид Mini-360](https://github.com/Davidovskii-Nikita/firmware_to_sensor_MEMS/blob/master/docs/Super-Mini-DC-DC-module-Mini-360-1000x750-min.jpg)

Вся прошивка написана с помощью платформы [PlatformIO](https://platformio.org/) на базе VSC, фреймворк Arduino.

---
### Принцип действия, описание прошивки.

Прошивка состоит из трех файлов:
* main.cpp. Содержит основной код программы.
* core.h. Header-файл. Содержит определения переменных и функций, а так же *#include* нужных библиотек
* core.cpp Содержит вспомогалтельные функции.

Для первичной настройки, в файле *core.h* требуется изменить следующие переменные:
``` C
// Блок настройки пользователем
// ====================================================================================
const char* ssid = "RUT230_9A75";// название WiFi сети
const char* password = "z1x2c3v4b5" ;// пароль WiFi сети
const int full_scale_range = 16; // диапазон измерений акселерометра( 2, 4, 8, 16)
const uint16_t period_a = 250; // Частота записи виброскорости
const uint16_t period_temp = 2500; // Частота записи темперартуры
const uint16_t period_v = 25; // Частота итегрирования виброускрорения
const uint16_t range_a = 100; // колличество значений виброскорости в 1 пакете
const uint16_t range_temp = 10; // колличество значений времени в 1 пакете
String URL1 = "http://192.168.8.212:8001/nkvm"; // адрес куда отправляются POST запросы
```

Контроллер опрашивает регистры MEMS-датчика, обрабатывает их и отправляет, а именно:
* Собирает значения ускорения с 3-х осей.
* Интегрирует полученные ускорения для получения виброскорости.
* Расчитывает СКЗ.
* Собирает значения температуры.
* Точно считает время.
* Для каждого значения виброскорости и температуры добавляется своя метка времени.
* Формирует JSON-документ содержащий полученные значения.
* Отправляет документ POST запросом (или MQTT брокеру, такая реализация в ветке mqtt)
* Имеет возможность перепрошивки через браузер.

Файл *main.cpp* начинается с функции *void setup()* в которой происходит:

* Подключение к WiFi:
``` C
  WiFi.begin(ssid, password); // подключение к WiFI.
  delay(100);
  while (WiFi.status() != WL_CONNECTED) // проверка подключения.
  {
    offset_startup_time = (double)millis()/1000; // расчет времени подключения к WiFi, для получения точного времени.
    delay(1500);
  }
  WiFi.mode(WIFI_AP_STA); // настройка режима WiFi
```
**ssid** и **password** настраиваются в файле *core.h*

* Создание веб страницы, организация возможности OTA-обновлений.
``` C
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
        Ticker_A.detach(); // выключение таймеров
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
```
* Инициализация I2C шины, последовательного порта, получение времени, расчет делителя:
``` C
  Serial.begin(9600); // отладка по последовательному порту
  Wire.begin(2, 0);  // инициализация I2C на GPIO 2 и 0 (SDA, SCL)
  sync_time = update_ntp(); // получение UNIX-времени
  scale_factor = for_scale/full_scale_range; // расчет делителя в зависимости от выбранной чувтсвиттельности
```
Последователное подключение требуется для отладки, шина I2C для общения с датчиком.
Для получения начального времени вызывается функция *update_ntp()*, содержащаяся в файле *core.h*:
```C
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
```
Для получения времени в UNIX формате используется библиотека *NTPClient.h*. Для этого создается объект **timeClient** и с помощью встроенной функции *getEpochTime()* записывается колличество секунд, которые прошли с 1 января 1970 года. Иногда библиотека выдает неадекватное время, для этого стоит проверка и рекурсия этой функции. 

Делитель *scale_factor* нужен для правильного перевода значений ускорения, получаеммых из регистров датчика в "человеческий" вид, в зависимости от выбранного диапазона измерений.

* Далее идет блок инициализации MPU6050:
``` C
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
```
В котором вызывается функция *I2C_Write*, содержащаяся в файле *core.cpp*
 ``` C
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
```
Передаваемые в функцию параметры описаны в файле *core.h* и являются адресами регистров датчика.

Далее идет получение MAC-адреса:
``` C
  MAC = WiFi.macAddress(); // Получение MAC адреса устройства
```
Расчет объема JSON-файла:
```C 
capacity = 2 * JSON_ARRAY_SIZE(range_temp) + 2 * JSON_ARRAY_SIZE(range_a) + JSON_OBJECT_SIZE(5) + 3500; // вычисление объема JSON файла
```
Уменьшение мощность WiFi-передатчика и получеие колличества милисекунд, прошедших со времени включения контроллера:
``` C
  local_time_ms = millis();
  WiFi.setOutputPower(0);
```
**Далее идет одна из основных функций**, функция *calibration()*.
```C
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
```
Дело в том, что на акселерометр, установленный в корпусе датчика *MPU6050* постоянно действует сила притяжения в 1g (9.81 m/c2). При расчете СКЗ и интегрировании его на выходе получается большая ошибка (около 40 mm/s), даже в статическом режиме. Что бы убрать влияние постоянной силы притяжения используется функция эта функция. Для этого cчитваются значения, присутствующее на осях c помощью функции *get_value_from_reg*:
```C
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
```
Функцией *I2C_Read* используется для чтения данных из регистра.
``` C
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
```
Полученные значения отправляются в функцию *clamp* в которой приводятся к диапазону [-1.0; 1.0]. Если передаваемое число не входит в диапзпон, то оно приравнивается к соответсвующей границе.
``` C
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
```
После нормализации данные фильтруются экспоненциальным бегущим средним с помощью функции *expRunningAverage*.
``` C
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
```
Таким образом обновляя глобальные переменные **filVal_x**, **filVal_y** и **filVal_z**. Далее эти переменные используются для получения целеиых значений.

В конце *setup()* функции инициализируются и настраиваются таймеры:
``` C
  // Блок инициализации таймеров
  // Объект класса Ticker вызывает функцию attach_ms с параметрами 
  // периода вызова и функци вызова
  //====================================================================================
  Ticker_V.attach_ms(period_v,get_vibrospeed); // управляет фунцией получения виброскорости
  Ticker_A.attach_ms(period_a,upate_vibrospeed_value); // управляет функцией записи виброскорости
  Ticker_T.attach_ms(period_temp,update_temperature_value); // управляет функцией записи температуры
  //====================================================================================
```

В функции *loop* проверяются глобальные флаги, в случае истины вызывается функция *post_json*.
``` C
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

  http.begin(client, URL1);// отправка
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  http.POST(buffer);
  http.end();
}
```
В которой создается JSON-файл, наполнятеяс содержимым глобальных массивов *opros_axel*, *opros_axel_time*, *opros_temp*, *opros_temp_time* содержащие значения виброскорости, метки виброскорости, температуры и метки температуры соответственно. И отправляется полученный JSON, POST запросом.

**Ticer_V** управляет функцией *get_vibrospeed*:
``` C
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
```
Полученные значения функцией *get_value_from_reg* фильтруются с помощью глобальных переменных **filVal_x**, **filVal_y** и **filVal_z**. Вызывается функция *discret_integral*:
``` C
void discret_integral(float x, float y, float z)
{
  vibro_speed_x = vibro_speed_x + ((x+old_x)*(period_v/2000.0)*g)*(1000.0/period_v); // дискретное интегрирование методом трапеций
  old_x=x;
  vibro_speed_y = vibro_speed_y + ((y+old_y)*(period_v/2000.0)*g)*(1000.0/period_v); // дискретное интегрирование методом трапеций
  old_y=y;
  vibro_speed_z = vibro_speed_z + ((z+old_z)*(period_v/2000.0)*g)*(1000.0/period_v); // дискретное интегрирование методом трапеций
  old_z=z;
}
```
Которая интегрирует каждую переменную методом трапеций.     
А так же вызывает функцию *get_RMS*:
``` C
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
```
Которая считатет среднеквадратичное значение.

**Ticer_A** управляет функцией *update_vibrospeed_value*:
``` C
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
```
Которая обнавляет глобальные массивы *opros_axel* и *opros_axel_time*. А так же проверяет заполненость последних. В случае истины обнуляет переменные виброскорости и обновляет глобальный флаг. Так же в этой функции вызвается функция *calibration* что бы постоянно нивелировать постоянную силу приятжения.

**Ticer_V** управляет функцией *update_temperature_value*.
 ``` C
 
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
```
Которая обнавляет глобальные массивы *opros_temp* и *opros_temp_time*. А так же проверяет заполненость последних. В случае истины обнуляет переменные виброскорости и обновляет глобальный флаг.

---
### Сборка.

Принципиальная схема в общем случае может выглядеть так:
![Сехма устройства](https://github.com/Davidovskii-Nikita/firmware_to_sensor_MEMS/blob/master/docs/%D0%9F%D1%80%D0%B8%D0%BD%D1%86%D0%B8%D0%BF%D0%B8%D0%B0%D0%BB%D1%8C%D0%BD%D0%B0%D1%8F%20%D1%81%D1%85%D0%B5%D0%BC%D0%B0.jpg)

---
### Отладка.
Для отладки лучше использовать макетную плату и USB TO TTL конвертер для прошивки ESP через USB. Схема может выглядить так:
![Схема отладки](https://github.com/Davidovskii-Nikita/vibroanaliz/blob/master/docs/%D0%A1%D1%85%D0%B5%D0%BC%D0%B0%20%D0%BE%D1%82%D0%BB%D0%B0%D0%B4%D0%BA%D0%B8.png)

Важно помнить что при подключении GPIO0 к земле плата ESP входит в режим загрузки. После заргузки следует отсоиденить порт GPIO0 от земли. Если устройство работает адекватно, можно прошивать датчик через браузер.
Для этого, нужно сформировать BIN файл прошивки. В среде PlatformIO что бы найти BIN прошивку, нужно перейти по пути : firmware_to_sensor_MEMS/.pio/build/esp01_1/firmware.bin. Далее нужно открыть веб страницу контроллера. Для этого нужно быть подключеным к той же WiFi сети что и датчик и знать его локальный IP-адресс. Перейдя по нему нужно нажать кнопку "Выберете файл". Для прошивки нужно нажать кнопку "Update". 

**В случае, если была залита неправильная прошивка в уже собранное устройство, есть возможность подпоятся к контактам ESP. Для этого нужно припаяться к клеме GND и соеденить ее с GND програматора (USB TO TTL). Припаять провод к клеме TX и соеденить с TX, так же RX. Припоять провод к EN и присоеденить к 3V3. Таким образом если подать питание к DC DC преобразователю и вставть програматор к ноутбуку, можно прошить устройство через провода.**

Так же можно использовать POST запросы, или MQTT соединение, для получения значений интересующих переменных.

---
### Калибровка.
Калибровку лучше проводить уже собранного устройства. Для этого можно использовать POST запросы или MQTT, или подпаятся к клемам по вышеописанной методике.    
Используя функцию *get_value_from_reg* можно получить значения ускорения, испытываемого на осях датчика. Перемещая устройство так, что бы сила притяжения была параллельна каждой оси по и против движения собрать полученные значения. Они должны быть примерно равны 1 в случае противонаправленности и -1 в случае сонаправленности. Если значения отличаются от 1, тогда можно изменить следующие переменные:
``` C
// Константы каллибровки
//====================================================================================
float norm_x = 0.0;
float norm_y = 0.0;
float norm_z = 0.0;
//====================================================================================
```
Важно помнить что значения этих переменных **ДОБАВЛЯЮТСЯ** к полученным значениям ускорения.   
Если фильтрование постоянной силы притяжения происходит неудволитворительно можно "поиграться" с настройками фильтра бегущего среднего:
``` C
// Фильтр бегущего среднего
//====================================================================================
float k = 0.05; // настройка фильтра
float filVal_x = 0; // содержит отфильтрованиие и нормализированное значение по оси x
float filVal_y = 0; // содержит отфильтрованиие и нормализированное значение по оси y
float filVal_z = 0; // содержит отфильтрованиие и нормализированное значение по оси z
float time_to_calibr = 100;// колличество точек фильтрования
//====================================================================================
```

---
### Возможные проблемы.


