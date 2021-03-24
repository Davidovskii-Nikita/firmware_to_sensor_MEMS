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

В дальнейшем, для получения точного времени вызывается функция *get_time*, которая добовляет к полученному выше числу *millis()* - колличество миллисекунд, прошедших после включения ESP
