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
Содержит основной код:
После подключения основного header-файла в функции setup объявляется использование глобальных переменных, начинается подключение по WiFi.   
Дальше идет большой кусок кода, нужный для возможности прошивки устройства через WiFi:   
``` CPP
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
``` CPP
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
Расчитывается переменная *scale_factor*, разделив на константу 32768 значение диапазона чувствительности акслереометра (2,4,8,16).    
В дальнейшем, для получения точного времени вызывается функция *get_time*, которая добовляет к полученному выше числу *millis()* - колличество миллисекунд, прошедших после включения ESP. Далее идет блок инициализации MPU6050, в котором можно выставить требуемую чувствительность акселерометра:
``` CPP
 I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x18);
```
0x18 - для +-16g; 0x10 - для +-10g; 0x08 - для +- 4g; 0x00 для 2g.    
Для уменьшения воздействия постоянной силы притяжения на акселерометр, вызвается функция calibration():
``` CPP
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
``` CPP
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
``` CPP
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
``` CPP
void expRunningAverage(float x ,float y, float z)
{
  filVal_x += (x - filVal_x) * k;
  filVal_y += (y - filVal_y) * k;
  filVal_z += (z - filVal_z) * k;
}
```

