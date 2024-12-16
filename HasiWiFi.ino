// NodeMCU v3     http://s.click.aliexpress.com/e/6AYN33fYB
// Модуль часов реального времени DS3231 подключение по I2C  http://s.click.aliexpress.com/e/bIiMvZjYB
// Семи сегментный индикатор, модуль на TM1637  http://s.click.aliexpress.com/e/EyZ3juBmU    или  http://s.click.aliexpress.com/e/jIQB6ubyr

// Настройки Arduino 1.6.8 -> "Дополнительные ссылки для менеджера плат:"-> http://arduino.esp8266.com/stable/package_esp8266com_index.json
// Инструменты -> плата -> NodeMCU1.0(ESP-12E Module)
// Код написан на Arduino 1.6.8

/*
Подключение:

Модуль часов реального времени DS3231 подключение по I2C -> NodeMCU v3
pin SKL -> D1;
pin SDA -> D2;
GND     -> GND;
VCC     -> 3V3;

Семи сегментный индикатор, модуль на TM1637 -> NodeMCU v3
pin CLK -> D6;
pin DIO -> D5;
GND     -> GND;
VCC     -> 3V3;

Светодиод для индикации успешной синхронизации -> NodeMCU v3
Плюс светодиода  -> D0;
Минус светодиода -> GND;

Питание платы NodeMCU v3:
Родное напряжение модуля — 3,3 вольта. На плате есть регулятор напряжения, поэтому питать её можно через USB или подвести питание от 3,7 до 20 вольт к пину 5V.
 */

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "TM1637.h"  //рабочая библиотека https://yadi.sk/d/hT73-chfuuJDF

#define DS3231_I2C_ADDRESS 0x68    // pin SKL -> D1; SDA -> D2   Модуль часов реального времени DS3231 подключение по I2C

#define CLK 12           // CLK -> pin D6
#define DIO 14           // DIO -> pin D5
TM1637 tm1637(CLK,DIO);  // Семи сегментный индикатор, модуль на TM1637
#define brightness 0  // яркость индикатора, от 0 до 1

#define GMT 3         // часовой пояс

int apdate = 0;
char ssid[] = "TP-Link_D2DA";    //  Имя сети SSID (name)
char pass[] = "74931699";        // Пароль для сети WiFi
unsigned int localPort = 2390;   // local port to listen for UDP packets

int8_t TimeDisp[4]; 
byte second, minute, hour, dayOfWeek, dayOfMonth, month, year; 

/* Don't hardwire the IP address or we won't get the benefits of the pool. *  Lookup the IP address for the host name instead */
//IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
IPAddress timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName = "time.nist.gov";
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;



/// работа с модулем часов реального времени ---------------------------------------------------
byte decToBcd(byte val){
  return ( (val/10*16) + (val%10) );
}

byte bcdToDec(byte val){
  return ( (val/16*10) + (val%16) );
}
//................................................................................................



///-- Установка времени и даты в модуль часов-----------------------------------------------------
void setDateDs3231(byte second,        // 0-59
                   byte minute,        // 0-59
                   byte hour,          // 1-23
                   byte dayOfWeek,     // 1-7
                   byte dayOfMonth,    // 1-28/29/30/31
                   byte month,         // 1-12
                   byte year)          // 0-99
{
   Wire.beginTransmission(DS3231_I2C_ADDRESS);
   Wire.write(0);
   Wire.write(decToBcd(second));    
   Wire.write(decToBcd(minute));
   Wire.write(decToBcd(hour));     
   Wire.write(decToBcd(dayOfWeek));
   Wire.write(decToBcd(dayOfMonth));
   Wire.write(decToBcd(month));
   Wire.write(decToBcd(year));
   Wire.endTransmission();
}
//.................................................................................................


///-----Запрос времени и даты с модуля часов:-----------------------------------------------------
void getDateDs3231(byte *second,   
          byte *minute,
          byte *hour,
          byte *dayOfWeek,
          byte *dayOfMonth,
          byte *month,
          byte *year)
{

  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();

  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);

  *second     = bcdToDec(Wire.read() & 0x7f);
  *minute     = bcdToDec(Wire.read());
  *hour       = bcdToDec(Wire.read() & 0x3f); 
  *dayOfWeek  = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month      = bcdToDec(Wire.read());
  *year       = bcdToDec(Wire.read());
}
//...............................................................................................

/// Получение время с модуля часов точного времени и вывод на дисплей----------------------------
void TimeOnDispley(){

getDateDs3231(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year); 
  TimeDisp[0] = hour / 10;
  TimeDisp[1] = hour % 10;
  TimeDisp[2] = minute / 10;
  TimeDisp[3] = minute % 10;
tm1637.display(TimeDisp); // воводим на индикатор
tm1637.point(second % 2 ? POINT_ON : POINT_OFF); //включаем точки в зависимости от четная секунда или нет
}
//.............................................................................


//// Синхронизация часов реального времени с временим из интернета-------------
int q = 0;
void oldloop()
{  
  //get a random server from the pool
  WiFi.hostByName(ntpServerName, timeServerIP); 

  sendNTPpacket(timeServerIP); // отправляем запрос на сервер 
  delay(500); // ждем пол секунды
  int cb = udp.parsePacket();
  if (!cb) {  // если не получили время
    digitalWrite(BUILTIN_LED, LOW); // пин D0 = LOW (встроенный светодиод горит)
    apdate = 0;
    q = q+1;
    if (q>3){ // за три попытки
      q = 0;
      return ; // выходим из oldloop()
    }
    Serial.println("no packet yet");   
    oldloop(); // получаем время еще раз     
  }
  else {  // если получили пакет с сервера
    Serial.print("packet received, length=");
    Serial.println(cb);
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print("Seconds since Jan 1 1900 = " );
    Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    Serial.println(epoch);      
   /// корректировка часового пояса
    epoch = epoch + GMT * 3600;         
    hour = (epoch  % 86400L) / 3600;
    minute = (epoch  % 3600) / 60;
    second = epoch % 60;  
    setDateDs3231(second, minute, hour, dayOfWeek, dayOfMonth, month, year); // записываем время из интернета в модуль точного времени
    digitalWrite(BUILTIN_LED, HIGH); // пин D0 = HIGH (встроенный светодиод гаснет)
    apdate = 1;
    delay(200);

    // print the hour, minute and second:
    Serial.print("The UTC time is ");       // UTC is the time at Greenwich Meridian (GMT)
    Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
    Serial.print(':');
    if ( ((epoch % 3600) / 60) < 10 ) {
      // In the first 10 minutes of each hour, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
    Serial.print(':');
    if ( (epoch % 60) < 10 ) {
      // In the first 10 seconds of each minute, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.println(epoch % 60); // print the second
  }
}

void sendNTPpacket(IPAddress& address)   //отправление запроса на сервер
{
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}
//............................................................................



/// подключение к WIFI -----------------------------------------------------
void ConnectingWiFi(){
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass); // устанавливаем имя сети и пароль
  delay(500); // ждем пол секунды
  while (WiFi.status() != WL_CONNECTED) { // если нет подключения продолжаем ждать
    Serial.println("");
    Serial.print("NO Connecting WiFie ");
    delay(500);
    TimeOnDispley(); // А пока ждем выводим время на дисплей из модуля часов реального времени
   delay(500);
  }     
  Serial.println("");  
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort()); 
}
//...........................................................................





///-----  При запуске платы-------------------------------------------------
void setup() {

  pinMode(BUILTIN_LED, OUTPUT); //пин D0 он же встроенный светодиод (если на выходе LOW встроенный светодиод горит)
   
 Serial.begin(115200);   // скорость последовательного порта
  Wire.begin();

 tm1637.init();           // инициализируем индикатор
 tm1637.set(brightness);  // включаем подсветку индикатора
 tm1637.point(POINT_ON);  // включаем точки
 ConnectingWiFi();        // Подключаемся к WiFi

  if (WiFi.status() == WL_CONNECTED){ // если связь установили
     oldloop();        // синхронизируем модуль времени если подключились к WiFi
  }
  delay(1000);
 TimeOnDispley(); // воводим время на индикатор
}
//..........................................................................




///-- Основной цикл---------------------------------------------------------
void loop() {
  
 TimeOnDispley(); // выводим время на индикатор из модуля часов
 
if(apdate == 0){//если последняя синхронизация не удалась
 if (second == 30){ // если секунды из часов  = 30 (т.е. получается раз в минуту)
 delay(1000); // ждем секунду
  if (WiFi.status() == WL_CONNECTED){ //если есть соединение с WiFi
     oldloop(); // синхронизируем модуль часов с интернетом
    }else{
       digitalWrite(BUILTIN_LED, LOW); // пин D0 = LOW (встроенный светодиод горит, подключенный светодиод на пине D0 не горит)
                                       // говорит нам о том что синхронизация не удалась
       apdate = 0;                                
      Serial.println(" No conected");
         }
    }
    }
if(apdate == 1){//если последняя синхронизация удачная
 if ((minute == 30) && (second == 30)){ //синхронизируемся раз в час 
 delay(1000); // ждем секунду
  if (WiFi.status() == WL_CONNECTED){ //если есть соединение с WiFi
     oldloop(); // синхронизируем модуль часов с интернетом
    }else{
       digitalWrite(BUILTIN_LED, LOW); // пин D0 = LOW (встроенный светодиод горит, подключенный светодиод на пине D0 не горит)
                                       // говорит нам о том что синхронизация не удалась
       apdate = 0;                                
      Serial.println(" No conected");
         }
    }
    }
    
}
//..........................................................................


