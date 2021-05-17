#include <Arduino.h>
#include <Wire.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

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
float get_value_from_reg (uint8_t dev_adress, uint8_t reg_adress, uint16_t scale_factor)
{
    float val_accel;
    int16_t res;
    res = I2C_Read(dev_adress,reg_adress);
    val_accel=(float)res/scale_factor;

    return val_accel;
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