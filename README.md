## Версия для передачи данных по MQTT
---
Отличается от версии представленной в ветке master методом отправки. Используется протокол MQTT? библиотека *PubSubClient.h*.   
Контроллер подключается к брокеру и отправляет JSON - файлы.   
Для настройки следует изменить переменные в файле *core.h*.
~~~ C 
const char* mqtt_server = "M5.WQTT.RU"; // адрес MQTT брокера
const uint16_t mqtt_port = 2938; // порт MQTT брокера
const char* topic = "TEST_SENSOR";
const char* login = "u_WUYIDH"; // логин для топика MQTT
const char* pass_mqtt = "kVu4uMTX"; // пароль для топика MQTT
~~~

**В случае использования NODE RED**.   
Функция, которая парсит:
~~~ JS
let t;
var v;
var input;
var output;
var i = 0;
var date;

input = msg.payload

while (i<input.Temp_time.length)
{
    t = input.Temp_time[i]*1000;
    date = new Date(t)
    v = input.Temp[i]
    output = [v, date.toISOString()];
    msg.payload = output;
    i++;
    node.send({payload:output})
}
~~~
Функция, которая отправляет в Postgres:
~~~ JS
let value = msg.payload[0];
let time = msg.payload[1];

msg.payload = "INSERT INTO opc_server.vibro001_temp (value_t, time) VALUES ("+value+", '"+ time +"'::timestamp);";


msg.QueryParameters = msg.payload;
return msg;
~~~
Их следует подключить последовательно.