#include "Arduino.h"
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4); //sometimes the adress is not 0x3f. Change to 0x27 if it dosn't work.
/*    i2c LCD Module  ==>   Arduino
      SCL             ==>     A5
      SDA             ==>     A4
      Vcc             ==>     Vcc (5v)
      Gnd             ==>     Gnd      */

#include "WString.h"
#define F(string_literal) (reinterpret_cast<const __FlashStringHelper *>(PSTR(string_literal)))
#define DHTPIN 12
#define DHTTYPE DHT22   // DHT 22  (AM2302)
#define pwm_fan_pin     3
#define FULLY_ON        0
#define OFF             255
#define INTERVAL        10   // interval of printing humid limit, in seconds

DHT dht(DHTPIN, DHTTYPE);

bool disp_flag = false;
uint8_t pwm_fan;
uint8_t fan_trigger;
const int disp_interval = int(INTERVAL) / 5;   
unsigned int hours, five_min_counter, disp_counter;
float humid_limit = 96.5;
float hourly_humidity_avg;
unsigned long time_;
unsigned long time_2;
typedef enum {idle, operating, take_data} FSM;
FSM state = idle;


void setup() {
  Serial.begin(9600);
  pinMode(pwm_fan_pin, OUTPUT);
  lcd.init();
  lcd.backlight();
  TCCR2B = (TCCR2B & 0b11111000) | 0x07;  // setting pin 3 and 11 frequency 16MHz / 1024 = 31Hz
  dht.begin();
  analogWrite(pwm_fan_pin, OFF); // initialize fan to off
}

void loop() {
  static float humidity, temperature;

  switch (state) {
    case idle:
      if (humidity > humid_limit) {
        // trigger 7 times to enter operating mode
        fan_trigger = (fan_trigger + 1) % 7;
        if (fan_trigger == 6) {
          state = operating;
          time_2 = millis();
        }
      }
      break;
    case operating:
      if (humidity < 75 || millis() - time_2 > 2700000UL) { // if the fan is on for 45 mins
        time_2 = millis();
        analogWrite(pwm_fan_pin, OFF);  // turn the fan off
        state = idle;
      }
      else {
        // full power if H > 90% else 65-90% increase fan speed linearly
        pwm_fan = (humidity > 90) ? FULLY_ON : map(humidity, 65, 90, OFF, FULLY_ON);
        analogWrite(pwm_fan_pin, pwm_fan);
      }
      break;
    case take_data:
      if (hours >= 144)   {  // 12 hours = 12hours * 60mins / 5 mins = 144;
        hours = 1;
        hourly_humidity_avg = 0;
      }
      else hours++;
      hourly_humidity_avg = hourly_humidity_avg + humidity / hours;    // save current humidity
      state = idle;
      break;
  }

  delay(500);
  humidity = dht.readHumidity();
  if (humidity >= 40 || humidity <= 90) humid_limit = map(humidity, 39, 91, 75, 99.5);
  else if (humidity < 40) humid_limit = 75;
  else humid_limit = 99.5;
  if (millis() - time_ > 5000UL ) {
    fan_trigger = 0;
    disp_flag = true;
    temperature = dht.readTemperature();
    time_ = millis();
    disp_counter = (disp_counter + 1) % disp_interval;  // control interval of updating humid limit, each count = 5s
    five_min_counter++;
    // take data every 5mins, ignore data in operating mode
    if (five_min_counter >= 60U) {
      if (state == idle && humidity < 90) state = take_data;
      five_min_counter = 0;
    }
    lcd.setCursor(0, 0);
    lcd.print( F("TEMP:  "));
    lcd.setCursor(7, 0);
    lcd.print( (String)  temperature + (char)223 + "C" );
    lcd.setCursor(0, 1);
    if (disp_counter == disp_interval - 1)  lcd.print( (String) "LIMIT: " + humid_limit + " %");  // print humidity limit
  }

  // print humidity 1s after humid limit is displayed
  if (millis() - time_ > 1000UL && disp_flag == true) {
    lcd.setCursor(0, 1);
    lcd.print( (String) "HUMID: " + humidity + " %");  // print humidity
    disp_flag = false;  // use flag to prevent contiuous updates 
  }
}
