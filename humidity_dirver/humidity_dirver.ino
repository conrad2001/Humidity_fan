#include "Arduino.h"
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4); //sometimes the adress is not 0x3f. Change to 0x27 if it dosn't work.
/*    i2c LCD Module  ==>   Arduino
      SCL             ==>     A5
      SDA             ==>     A4
      Vcc             ==>     Vcc (5v)
      Gnd             ==>     Gnd      */

#include "WString.h"
#define F(string_literal) (reinterpret_cast<const __FlashStringHelper *>(PSTR(string_literal)))
#define temperature_pin A0
#define humidity_pin    A1
#define pwm_fan_pin     3
#define FULLY_ON        0
#define OFF             255

long time_;
float VDD = 5;
uint16_t  R2 = 2000;

void setup() {
  Serial.begin(9600);
  pinMode(temperature_pin, INPUT);
  pinMode(humidity_pin, INPUT);
  pinMode(pwm_fan_pin, OUTPUT);
  analogReference(DEFAULT);
  lcd.init();
  lcd.backlight();
  TCCR2B = (TCCR2B & 0b11111000) | 0x07;  //setting pin 3 and 11 frequency 16MHz / 1024 = 31Hz

}

void loop() {
  float humidity_bit = analogRead(humidity_pin);
  float humidity = 33.33 * humidity_bit / ( ( 1 << 10) - 1 ) * VDD;  //humidity transfer function: humidity = 33.33 * humidity_voltage , humidity_voltage =  humidity_bit / ( 1024 - 1 ) * Vref
  float temperature_bit = analogRead(temperature_pin);
  float Vin = temperature_bit / ( ( 1 << 10) - 1 ) * VDD;
  float Thermistor_res  = R2 * ( VDD / Vin - 1 ); 
  float temperature = 77.21*exp(-0.001217*Thermistor_res) + 86.24*exp(-0.0001248*Thermistor_res);    //B3435K NTC curve
  int pwm_fan = ( humidity > 70 ) ? ( humidity > 85 ) ? FULLY_ON : map(humidity, 60, 85, OFF, FULLY_ON) : OFF ; 
  analogWrite(pwm_fan_pin, pwm_fan);

  delay(1000);
  if (millis() - time_ > 5000) {
    time_ = millis();
    lcd.clear();
    lcd.print( F("TEMP:  "));
    lcd.print( (String)  temperature + (char)223 + "C" );
    lcd.setCursor(0, 1);
    lcd.print( (String) "HUMID: " + humidity + " %");
  }



}
