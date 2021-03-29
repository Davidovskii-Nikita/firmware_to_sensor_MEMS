## Прошивка для датчика вибрации на основе MEMS

В основе, лежит сенсор MPU6050, включающий в себя 3-х осевой акселерометр, 3-x осевой гироскоп и датчик температуры. Его [даташит](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf) и [карта регистров](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf).
Управляющем устройством выступает чип esp8266 на борту модуля ESP-01 1 Mb.    
Вся прошивка написана с помощью платформы [PlatformIO](https://platformio.org/) на базе VSC, фреймворк Arduino.
 
### Содержание:
* Как работает код.
* Схема подключения.
* Отладка.
* Возможные проблемы.
---
### Как рабоатет код.
Прошивка состоит из 3-x файлов:
* **core.h** 
Header-файл. Содержит инициализацию библиотек, переменных и функций. 
* **core.cpp** 
Содержит вспомошательные функции.
* **main.cpp**
Содержит основной код.   
После подключения основного header-файла в функции setup объявляется использование глобальных переменных, начинается подключение по WiFi.   
Дальше идет большой кусок кода, нужный для возможности прошивки устройства через WiFi:   
``` C
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
```    
Для изменения внешнего вида web-страницы, можно менять HTML-код, написанный в переменной *serverIndex*, объявленной в файле core.h.     
Далее инициализируется MDNS, последовательный порт и Wire - для подключения I2C на соответствующие пины. Вызывается функция **update_ntp** для получения времени с NTP серверов. 
``` C
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
В дальнейшем, для получения точного времени вызывается функция *get_time*, которая добовляет к полученному выше числу *millis()* - колличество миллисекунд, прошедших после включения ESP. Расчитывается переменная *scale_factor*, разделив на константу 32768 значение диапазона чувствительности акслереометра (2,4,8,16).Далее идет блок инициализации MPU6050, в котором можно выставить требуемую чувствительность акселерометра:
``` C
 I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x18);
```
0x18 - для +-16g; 0x10 - для +-10g; 0x08 - для +- 4g; 0x00 для 2g.    
Для уменьшения воздействия постоянной силы притяжения на акселерометр, вызвается функция calibration():
``` C
void calibration()
{
  float x,y,z;
  float i = 0;
  while( i < time_to_calibr)
  {
    x = get_value_from_reg(MPU6050SlaveAddress,MPU6050_REGISTER_ACCEL_XOUT_H,scale_factor);
    y = get_value_from_reg(MPU6050SlaveAddress,MPU6050_REGISTER_ACCEL_YOUT_H,scale_factor);
    z = get_value_from_reg(MPU6050SlaveAddress,MPU6050_REGISTER_ACCEL_ZOUT_H,scale_factor);
    x = clamp(x,-1.0,1.0);
    y = clamp(y,-1.0,1.0);
    z = clamp(z,-1.0,1.0);
    expRunningAverage(x, y, z);
    ++i;
  }
  i=0;
}
```    
Вызываемая функция get_value_from_reg отдает значение, находящееся в регистре *reg_adress*, устройства *dev_adress* в сети I2C, а так же нормализует полученное значение разделив на переменную *scale_factor*:
``` C
float get_value_from_reg (uint8_t dev_adress, uint8_t reg_adress, uint16_t scale_factor)
{
    float val_accel;
    int16_t res;
    res = I2C_Read(dev_adress,reg_adress);
    val_accel=(float)res/scale_factor;

    return val_accel;
}
```   
Функция clamp отдает передаваемое значение если оно в диапазоне от -1.0 до 1.0, если значение выходит за границы, то функция выдает -1.0 или 1.0 соответственно:
``` C
float clamp(float  v, float minv, float maxv)
{
  if( v>maxv )
      return maxv;
  else if( v<minv )
      return minv;
  return v;
}
```    
Функция void expRunningAverage(float, float, float), расчитывает експоненциальное бегущее среднее по 3-м осям и обновляет глобальные переменные.
``` C
void expRunningAverage(float x ,float y, float z)
{
  filVal_x += (x - filVal_x) * k;
  filVal_y += (y - filVal_y) * k;
  filVal_z += (z - filVal_z) * k;
}
```   
Таким образом функция *calibration* фильтрует значения ускорения по трем осям, приведенные к диапазону от -1.0 до 1.0. Полученные глобальные перменные *filVal_x*, *filVal_y* и *filVal_z*, используются при расчете виброскорости.   
Далее идет блок инициализации таймеров:
``` C
Ticker_V.attach_ms(period_v,get_vibrospeed);
Ticker_A.attach_ms(period_a,upate_vibrospeed_value);
Ticker_T.attach_ms(period_temp,update_temperature_value);
```   
Время работы таймеров определяется в файле *core.h*.   
В функции *get_vibrospeed* получаются значения ускорения по трем осям с учетом калибрующих переменных *filVal*, вызывются функции *discret_integral* и *get_RMS*. 
```C
void get_vibrospeed()
{
  extern uint16_t scale_factor;
  //получение значений ускорений
  float X,Y,Z;
  X = get_value_from_reg(MPU6050SlaveAddress,MPU6050_REGISTER_ACCEL_XOUT_H,scale_factor) - filVal_x;
  Y = get_value_from_reg(MPU6050SlaveAddress,MPU6050_REGISTER_ACCEL_YOUT_H,scale_factor) - filVal_y;
  Z = get_value_from_reg(MPU6050SlaveAddress,MPU6050_REGISTER_ACCEL_ZOUT_H,scale_factor) - filVal_z;
  // интегрирование по 3 осям
  discret_integral(X,Y,Z);
  // получение СКЗ
  rms = get_RMS(vibro_speed_x,vibro_speed_y,vibro_speed_z);
}
```    
Функция *discret_integral* интегрирует передаваемые переменные *x*, *y* и *z* обновляя глобальные переменные *vibro_speed_x/_y/_z*.  
```C
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
Функция *get_RMS* расчитывает среднеквадратичное значения передаваемых переменных *a*, *b* и *c* возвращая СКЗ в переменной RMS. Функция содержится в файле *core.c*   
```C
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
Таким образом за каждый период работы функции *get_vibrospeed* мы получаем откалиброванное среднеквадратичное значение виброскорости с 3-х осей.  
В функции *update_vibrospeed_value* заполняются глобальные массивы *opros_axel* и *opros_axel_time* добавлением глобальной переменной *rms* и вызовом функции *get_time* соответственно.   
```C
double get_time(double s_t)
{ 
  return  ((double)millis())/1000 + s_t;
}
```   
А так же обновлятся глобальный флаг *flag_a* если колличество записываемых в массив значений больше переменной *range_a*(задается в файле *core.h*) и обнуляются переменные виброскорости (*vibro_speed_x/_y/_z*). Так же в этой функции вызывается *calibration*, таким образом осуществляется постоянная компенсация постоянной силы притяжения.   
Функция *update_temperature_value*, делает тоже самое, однако вместо значений виброскорости записываются значения температуры из функции *get_temp*(находится в файле *core.c*).
```C
float get_temp(int16_t temp)
{
    return ((float)temp)/340+36.53; // формула расчета температуры из datasheet
}
```    
В функции *loop()* постоянно обновляется содержимое web-сервера и MDNS а так же проверяются флаги, сигнализирующие о наполненности массивов.
```C
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
```    
Если массивы заполнены, обнуляются глобальные счетчики *count_a* и *count_temp*, сбрасываются флаги и вызывается функция post_json(). В которой создается json-файл, заполняется данными из глобальных массивов и отправляется POST-запросом в адреса *URL1* и *URL2*.   
Так каждые 25 секунд отправляется json-файл, содержащий 100 значений виброскорости и 10 значений температуры со своими временными ветками в UNIX-формате включая миллисекунды. Каждые 25 миллисекунд собирается и интегрируются значения ускорения.    
   
### Схема подключения.
