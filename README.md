## Версия для передачи данных по MQTT
---
Отличается от версии представленной в ветке master методом отправки. Используется протокол MQTT? библиотека *PubSubClient.h*.   
Контроллер подключается к брокеру и отправляет JSON - файлы.   
Для настройки следует изменить переменные в файле *core.h*.
~~~ C 
const char* mqtt_server = "185.231.71.153"; // адрес MQTT брокера
const uint16_t mqtt_port = 1883; // порт MQTT брокера
const char* topic = "TEST_SENSOR";
const char* login = "admin"; // логин для топика MQTT
const char* pass_mqtt = "admin"; // пароль для топика MQTT
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
// Для значений вибрации следует заменить Тemp на Axel.
input = msg.payload

while (i<input.Temp_time.length) // цикл работает по колличеству значений в массиве. 
{
    t = input.Temp_time[i]*1000; // Temp_time заменить на Axel_time для значений вибрации
    date = new Date(t)
    v = input.Temp[i] // Temp заменить на Axel для значений вибрации
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

msg.payload = "INSERT INTO opc_server.vibro001_temp (value_t, time) VALUES ("+value+", '"+ time +"'::timestamp);"; // vibro001_temp - название таблицы


msg.QueryParameters = msg.payload;
return msg;
~~~
Их следует подключить последовательно.