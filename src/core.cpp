#include <Arduino.h>
#include <Wire.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data)
{
    Wire.beginTransmission(deviceAddress);
    Wire.write(regAddress);
    Wire.write(data);
    Wire.endTransmission();
}

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

double get_time(double s_t, double offset)
{ 
  return  ((double)millis())/1000 + s_t - offset;
}

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

float get_value_from_reg (uint8_t dev_adress, uint8_t reg_adress, uint16_t scale_factor)
{
    float val_accel;
    int16_t res;
    res = I2C_Read(dev_adress,reg_adress);
    val_accel=(float)res/scale_factor;

    return val_accel;
}

float get_temp(int16_t temp)
{
    return ((float)temp)/340+36.53; // формула расчета температуры из datasheet
}

float clamp(float  v, float minv, float maxv)
{
  if( v>maxv )
      return maxv;
  else if( v<minv )
      return minv;
  return v;
}