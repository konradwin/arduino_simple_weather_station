/* Program obslugujacy "Prosta stacje pogodowa" na platforme arduino, z wykorzystaniem nastepujacych komponentow:
Arduino Nano V3, DS3231, BMP180, DHT11, DS18B20, czujnik opadow, LCD 4x20 I2c
Piny Arduino:
7-LED, 8-czujnik opadow-stan, 9-DHT11, 10-DS18B20-dom, 11-DS18B20-dwor
12-DS18B20-grzejnik, A3-czujnik opadow-wilgotnosc, A4-I2C SDA, A5-I2C SCL (DS3231, BMP180, LCD)
*/

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include "DS3231.h"
#include "DHT.h"
#include <BMP180.h>
#include <OneWire.h>
#include <DallasTemperature.h>


LiquidCrystal_I2C lcd(0x27,20,4); //set the LCD address to 0x27 for a 20 chars and 4 line display

DS3231 RTC; //create the R8025 object

char weekDay[][14] = {"niedziela", "wtorek", "xxx", "sroda", "czwartek", "piatek", "sobota", "poniedzialek"}; //  uint8_t dayOfWeek() const   { return wday;}  /*Su=0 Mo=1 Tu=3 We=4 Th=5 Fr=6 Sa=7 *

#define DHTPIN11 9 // what pin we're connected to
#define DHTTYPE11 DHT11 //DHT 11  (AM2302)

DHT dht11(DHTPIN11, DHTTYPE11); //initialize DHT sensor for normal 16mhz Arduino

BMP180 barometer; //store an instance of the BMP180 sensor

float seaLevelPressure = 102460; //store the current sea level pressure at your location in Pascals.

#define ONE_WIRE_BUS1 11 //data wire is plugged into port 11 on the Arduino
#define ONE_WIRE_BUS2 12 //data wire is plugged into port 12 on the Arduino
#define ONE_WIRE_BUS3 10 //data wire is plugged into port 10 on the Arduino

OneWire oneWire1(ONE_WIRE_BUS1); //setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire2(ONE_WIRE_BUS2); //setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire3(ONE_WIRE_BUS3); //setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)

DallasTemperature sensors1(&oneWire1); //pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors2(&oneWire2); //pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors3(&oneWire3); //pass our oneWire reference to Dallas Temperature.

#define LEDPIN 7
#define RAINPIN 8
#define RAIN2PIN A3


void setup()
{
  lcd.init(); //initialize the lcd  
  lcd.backlight(); //switch on backlight on lcd 
 
  Serial.begin(9600);
  
  Wire.begin();
  RTC.begin();
 
  RTC.convertTemperature(); //convert current temperature into registers
 
  dht11.begin();
  
  barometer = BMP180(); //we create an instance of our BMP180 sensor
  barometer.SoftReset(); //when we have connected, we reset the device to ensure a clean start
  barometer.Initialize(); //now we initialize the sensor and pull the calibration data
  
  sensors1.begin(); //start up the library
  sensors2.begin(); //start up the library
  sensors3.begin(); //start up the library
}


void lcdWelcome()
{
  lcd.setCursor(4,3);
  lcd.print("#powitanie#");
  
  lcd.setCursor(0,0);
  lcd.print("Stacja pogodowa z"); 
  lcd.setCursor(0,1);
  lcd.print("wykorzystaniem"); 
  lcd.setCursor(0,2);
  lcd.print("Arduino :)"); 
}


void lcdTimeDate()
{  
  DateTime now = RTC.now(); //get the current date-time
   
  lcd.setCursor(7,3);
  lcd.print("#czas i data#");
  
  lcd.setCursor(0,0);
  if (now.hour()<10) lcd.print("0"); 
  lcd.print(now.hour(), DEC);
  lcd.print(':');
  if (now.minute()<10) lcd.print("0"); 
  lcd.print(now.minute(), DEC);
  lcd.print(':');
  if (now.second()<10) lcd.print("0"); 
  lcd.print(now.second(), DEC);
    
  lcd.setCursor(0,1);
  if (now.date()<10) lcd.print("0"); 
  lcd.print(now.date(), DEC);
  lcd.print('.');
  if (now.month()<10) lcd.print("0"); 
  lcd.print(now.month(), DEC);
  lcd.print('.');
  lcd.print(now.year(), DEC);
    
  lcd.setCursor(0,2);
  lcd.print(weekDay[now.dayOfWeek()]);
}


void lcdTemp()
{
  sensors1.requestTemperatures(); //send the command to get temperatures
  sensors2.requestTemperatures(); //send the command to get temperatures
  sensors3.requestTemperatures(); //send the command to get temperatures

  DateTime now = RTC.now(); //get the current date-time
  
  RTC.convertTemperature(); //convert current temperature into registers
  
  lcd.setCursor(7,3);
  lcd.print("#temperatury#");
    
  lcd.setCursor(0,1); 
  lcd.print("t2=");
  lcd.print(sensors1.getTempCByIndex(0)); //temperature for the device 1
  lcd.print("C");

  lcd.setCursor(0,2); 
  lcd.print("t3=");
  lcd.print(sensors2.getTempCByIndex(0)); //temperature for the device 2
  lcd.print("C");

  lcd.setCursor(0,0); 
  lcd.print("t1=");
  lcd.print(sensors3.getTempCByIndex(0)); //temperature for the device 3
  lcd.print("C");
 
  lcd.setCursor(13,0);
  lcd.print('(');
  if (now.hour()<10) lcd.print("0"); 
  lcd.print(now.hour(), DEC);
  lcd.print(':');
  if (now.minute()<10) lcd.print("0"); 
  lcd.print(now.minute(), DEC);
  lcd.print(')');

  
 /* 
  lcd.setCursor(12,0);   
  lcd.print("t=");    
  lcd.print(RTC.getTemperature()); //read registers and display the temperature from rtc
  lcd.print("C");

  float t11 = dht11.readTemperature();
    
  lcd.setCursor(12,0);
  lcd.print("t=");
  lcd.print(t11); //print temp from dht11
  lcd.print("C");

  float currentTemperature = barometer.GetTemperature(); //retrive the current temperature in degrees celcius. 
  lcd.setCursor(12,1); 
  lcd.print("t=");
  lcd.print(currentTemperature); //print out the Temperature from bmp180
  lcd.print("C");
*/
}


void lcdHumPres()
{
  DateTime now = RTC.now(); //get the current date-time
  
  lcd.setCursor(6,3);
  lcd.print("#cis. i wilg.#");  
  
  float h11 = dht11.readHumidity();
  float t11 = dht11.readTemperature(); //read temperature as Celsiusa
  float f11 = dht11.readTemperature(true);  //read temperature as Fahrenheit
  
  float hi111 = dht11.computeHeatIndex(f11, h11); //must send in temp in Fahrenheit
  float hi11 = (hi111-32) / 1.8;
    
  lcd.setCursor(0,0);
  lcd.print("h=");
  lcd.print(h11); //print out humiditi from dht11
  lcd.print("%");

  lcd.setCursor(0,1);
  lcd.print("hi=");
  lcd.print(hi11); //print out Heat index from dht11
  lcd.print("C");

  float currentPressure = barometer.GetPressure(); //retrive the current pressure in Pascals
  float currentPressurehp = (currentPressure /100);
  float altitude = barometer.GetAltitude(seaLevelPressure); //retrive the current altitude (in meters). Current Sea Level Pressure is required for this
  float currentTemperature = barometer.GetTemperature(); //retrive the current temperature in degrees celcius
   
  lcd.setCursor(0,2); 
  lcd.print("p=");
  lcd.print(currentPressurehp); //print out the Pressure
  lcd.print("hPa");
  
  lcd.setCursor(13,0);
  lcd.print('(');
  if (now.hour()<10) lcd.print("0"); 
  lcd.print(now.hour(), DEC);
  lcd.print(':');
  if (now.minute()<10) lcd.print("0"); 
  lcd.print(now.minute(), DEC);
  lcd.print(')');
}


void lcdRain()
{
  DateTime now = RTC.now(); //get the current date-time
  
  lcd.setCursor(7,3);
  lcd.print("#cz. opadow#");  
  
  int sensorA = analogRead(RAIN2PIN);
  int sensorD = digitalRead(RAINPIN);
  
  int humi = map (sensorA, 1023, 0, 0, 100);

  lcd.setCursor(0,1);
  lcd.print("int=");
  lcd.print(humi);
  lcd.print("%");
  
  lcd.setCursor(0,0);
  lcd.print("stan=");
  if (sensorD==1) 
  {
  lcd.print("sucho");
  digitalWrite(LEDPIN, LOW);
  }
  else
  {
  lcd.print("pada!!!");
  digitalWrite(LEDPIN, HIGH);
  }
  
  lcd.setCursor(13,0);
  lcd.print('(');
  if (now.hour()<10) lcd.print("0"); 
  lcd.print(now.hour(), DEC);
  lcd.print(':');
  if (now.minute()<10) lcd.print("0"); 
  lcd.print(now.minute(), DEC);
  lcd.print(')');
} 


void loop () 
{
 
/*
lcdWelcome();
delay(5000);
lcd.clear();
*/

for(int i=0; i<5;i++)
{
lcdTimeDate();
delay(1000);
}
lcd.clear();

lcdTemp();
delay(5000);
lcd.clear();

lcdHumPres();
delay(5000);
lcd.clear();

lcdRain();
delay(5000);
lcd.clear();
}
