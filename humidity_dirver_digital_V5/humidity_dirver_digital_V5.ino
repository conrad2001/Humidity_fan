

#include "Arduino.h"
#include <DHT.h>

/*    i2c LCD Module  ==>   Arduino
      SCL             ==>     A5
      SDA             ==>     A4
      Vcc             ==>     Vcc (5v)
      Gnd             ==>     Gnd      */

#include "WString.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


#define F(string_literal) (reinterpret_cast<const __FlashStringHelper *>(PSTR(string_literal)))
#define DHTPIN 2
#define DHTTYPE DHT22   // DHT 22  (AM2302)
#define pwm_fan_pin     3
#define FULLY_ON        255
#define OFF             0
#define INTERVAL        10   // interval of printing humid limit, in seconds

DHT dht(DHTPIN, DHTTYPE);


bool disp_flag = false, half_hr_flag = false;
uint8_t pwm_fan;
uint8_t fan_trigger;
const int disp_interval = int(INTERVAL) / 5;
unsigned int hours, min_counter, disp_counter;
float humid_limit = 96.5;
float hourly_humidity_avg;
unsigned long time_ = 0;
unsigned long time_humidity = 0;
typedef enum {idle, operating, take_data, adjust_limit} FSM;
FSM state = idle, prev_state = idle;
uint8_t const LOWER_LIMIT = 20, OFF_LIMIT = 75, HALF_HR_LIM = 90;

void setup() {
  prev_state = idle;
  state = adjust_limit;
  //Serial.begin(9600);
  pinMode(pwm_fan_pin, OUTPUT);
  TCCR2B = (TCCR2B & 0b11111000) | 0x07;  // setting pin 3 and 11 frequency 16MHz / 1024 = 31Hz
  dht.begin();
  analogWrite(pwm_fan_pin, OFF); // initialize fan to off
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds
  // Clear the buffer
  display.clearDisplay();
  // Draw a single pixel in white
  display.drawPixel(10, 10, SSD1306_WHITE);
  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
  delay(2000);
  // display.display() is NOT necessary after every single drawing command,
  // unless that's what you want...rather, you can batch up a bunch of
  // drawing operations and then update the screen all at once by calling
  // display.display(). These examples demonstrate both approaches...
}

void loop() {
  static float humidity, temperature;
  // half_hr_limit used as 30 mins counter also to adjust humid limit within 30 mins after entering operating mode
  static float half_hr_limit = HALF_HR_LIM;
  switch (state) {
    case idle:
      if (humidity >= humid_limit) {
        // trigger 7 times to enter operating mode
        fan_trigger = (fan_trigger + 1) % 7;
        if (fan_trigger == 6) {
          state = operating;
          min_counter = 0;
          time_ = millis();
          half_hr_flag = true;    // start counting for 30 mins
        }
      }
      break;
    case operating:
      // if the fan is on for 5 mins (5 mins interval to avoid sensor error trigger for too long when not showering but RH is too high)
      if (!min_counter) {
        if (half_hr_limit < HALF_HR_LIM + 6) humid_limit = half_hr_limit;
        if (humidity <= humid_limit) {
          analogWrite(pwm_fan_pin, OFF);  // turn the fan off
          state = idle;
        }
      }
      else {
        // full power if H > 90% else 65-90% increase fan speed linearly
        if (humidity <= 75) {
          pwm_fan = map(75, 65, 90, OFF, FULLY_ON);
        }
        else pwm_fan = (humidity > 90) ? FULLY_ON : map(humidity, 65, 90, OFF, FULLY_ON);
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
    case adjust_limit:
      if (humidity > LOWER_LIMIT + 20 && humidity < 91) humid_limit = map(humidity, LOWER_LIMIT + 20, 90, 75, 99.9);
      else if (humidity < LOWER_LIMIT + 21) humid_limit = 75;
      else humid_limit = 99.9;
      if (state == idle && humidity < 90) state = take_data;
      else state = prev_state;
      break;
  }

  if (millis() - time_humidity > 499UL) {
    humidity = dht.readHumidity();
    time_humidity = millis();
  }

  if (millis() - time_ > 5000UL ) {
    fan_trigger = 0;
    disp_flag = true;
    temperature = dht.readTemperature();
    time_ = millis();
    disp_counter = (disp_counter + 1) % disp_interval;  // control interval of updating humid limit, each count = 5s
    min_counter = (min_counter + 1) % 12;
    // take data every 5mins, ignore data in operating mode
    if (!min_counter) {    // if 1 min interval is reached
      if (state != operating) {
        static uint8_t five_min_count = 0;
        five_min_count = (five_min_count + 1) % 5;
        if (!five_min_count) {  // adjust limit every 5 mins
          prev_state = state;
          state = adjust_limit;
        }
      }
      if (half_hr_flag) half_hr_limit = half_hr_limit + (1 / 6);  // count for 30mins from entering operating mode
      if (half_hr_limit >= HALF_HR_LIM + 5) {
        half_hr_limit = HALF_HR_LIM; // reset 30 mins limit when 30 mins reached
        humid_limit = map(humidity, LOWER_LIMIT + 20, 90, 75, 99.9);    // reset humid limit based on current RH
        half_hr_flag = false; // turn off counter after 30 mins
      }
    }

    display.clearDisplay();
    display.setTextSize(1);             // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);        // Draw white text
    display.setCursor(0, 0);            // Start at top-left corner
    display.println(F("TEMP:  "));
    display.setCursor(32, 0);
    display.print( (String)  temperature + " C" );
    display.setCursor(0, 16);
    if (disp_counter == disp_interval - 1) {
      static bool disp_sel = (disp_sel) ? false : true;
      float disp_out = (disp_sel) ? humid_limit : hourly_humidity_avg;
      display.print( (String) "LIMIT: " + disp_out + " %");  // print humidity limit and average humidity alternatively
      display.display();
    }
  }

  // print humidity 1s after humid limit is displayed
  if (millis() - time_ > 1000UL && disp_flag == true) {
    display.clearDisplay();
    display.setCursor(0, 0);            // Start at top-left corner
    display.println(F("TEMP:  "));
    display.setCursor(32, 0);
    display.print( (String)  temperature + " C" );
    display.setCursor(0, 16);
    display.setTextColor(SSD1306_WHITE);
    display.print( (String) "HUMID: " + humidity + " %");  // print humidity
    disp_flag = false;  // use flag to prevent contiuous updates
    display.display();
  }
}


//void testdrawstyles(void) {
//  display.clearDisplay();
//
//  display.setTextSize(1);             // Normal 1:1 pixel scale
//  display.setTextColor(SSD1306_WHITE);        // Draw white text
//  display.setCursor(0,0);             // Start at top-left corner
//  display.println(F("Hello, world!"));
//
//  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
//  display.println(3.141592);
//
//  display.setTextSize(2);             // Draw 2X-scale text
//  display.setTextColor(SSD1306_WHITE);
//  display.print(F("0x")); display.println(0xDEADBEEF, HEX);
//
//  display.display();
//  delay(2000);
//}
