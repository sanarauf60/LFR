#include "NewPing.h"

//OLED
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

char colour;
int Last_turn;

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };

// IR connections 
# define Left 36
# define Center 39
# define Right 34

// Ir encoder Sensor
// Define variables for measuring speed
const int sensorPin = 12;

unsigned long start_time = 0;
unsigned long end_time = 0;
int steps = 0;
float steps_old = 0;
float temp = 0;
float rps = 0;

// motor driver connections
# define ENA 26
# define IN1 27
# define IN2 14
# define IN3 12
# define IN4 13
# define ENB 15

// Colour Sensor 
#define S2 16
#define S3 17
#define S0 1
#define S1 3 
#define sensorOut 4

int Red = 0;
int Green = 0;
int Blue = 0;
int Frequency = 0;
int Color = 0;

// ultrasonic sensor connections
# define TRIGGER_PIN  33
# define ECHO_PIN 32
# define MAX_DISTANCE 400
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE) ;

// Current / voltage 
# define current_pin 35
# define voltage_pin 25
float current = 0, voltage = 0;

// IR Encoder

void setup() 
{
   Serial.begin(9600);

   // ir sensor
   pinMode(Left, INPUT);
   pinMode(Center, INPUT);
   pinMode(Right, INPUT);

   pinMode(ENA, OUTPUT);
   pinMode(IN1, OUTPUT);
   pinMode(IN2, OUTPUT);
   pinMode(IN3, OUTPUT);
   pinMode(IN4, OUTPUT);
   pinMode(ENB, OUTPUT);
   // IR Encoder
  pinMode(sensorPin, INPUT);   // set sensor pin as input
  // colour sensor 
  pinMode(S2, OUTPUT); //Define S2 Pin as a OUTPUT/
  pinMode(S3, OUTPUT); //Define S3 Pin as a OUTPUT/
  pinMode(S1, OUTPUT); //Define S2 Pin as a OUTPUT/
  pinMode(S0, OUTPUT); //Define S3 Pin as a OUTPUT/
  digitalWrite(S0,HIGH);
  digitalWrite(S1,HIGH);
  // Current / voltage 
  pinMode (current_pin , INPUT);
  pinMode (voltage_pin , INPUT);
   // motor driver
       // MOTOR 1
   digitalWrite(IN1, HIGH);
   digitalWrite(IN2, LOW);
      // MOTOR 2
   digitalWrite(IN3, LOW);
   digitalWrite(IN4, HIGH);

   pinMode(TRIGGER_PIN,OUTPUT);
   pinMode(ECHO_PIN, INPUT);

   //finding OLED
   // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.display();
  display.clearDisplay();
  display.display();

}

void loop() {

    obstacle_avoid();
    disp1();
}

void Forward()
{
  analogWrite(ENA, 80);
  analogWrite(ENB, 85);
}
void Left_turn()
{
  analogWrite(ENA, 80);
  analogWrite(ENB, 0);
  Last_turn = 0;
}
void Right_turn()
{ 
  analogWrite(ENA, 0);
  analogWrite(ENB, 85);
  Last_turn = 1;
}
void Stop()
{
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void line_follow()
{
 
   if (analogRead (Left)> 500 && analogRead (Center)> 500 && analogRead (Right)> 500 )
   { Forward();}
   if (analogRead (Left)> 500 && analogRead (Center)> 500 && analogRead (Right)< 500 )
   { Left_turn();}
   if (analogRead (Left)> 500 && analogRead (Center)< 500 && analogRead (Right)> 500 )
   {Stop();}
   if (analogRead (Left)> 500 && analogRead (Center)< 500 && analogRead (Right)< 500 )
   {Left_turn();}
   if (analogRead (Left)< 500 && analogRead (Center)> 500 && analogRead (Right)> 500 )
    {Right_turn();}    
   if (analogRead (Left)< 500 && analogRead (Center)> 500 && analogRead (Right)< 500 )
    {Forward();}
   if (analogRead (Left)< 500 && analogRead (Center)< 500 && analogRead (Right)> 500 )
    {Right_turn();}
   if (analogRead (Left)< 500 && analogRead (Center)< 500 && analogRead (Right)< 500 )
   {if (Last_turn == 1)
    {
      Left_turn();
    }
    else 
    Right_turn();
   }
}
void obstacle_avoid()
{
//  Serial.print(sonar.ping_cm());
 if(sonar.ping_cm()<= 10)
 { 
  Stop();;
  if (colour == 'R')
  red_obs();
  else
  disp();
 // delay(300);

  // if (detect_color() == 'R')
  // {red_obs();
  // Left_turn();
  // delay(200);
  // while (!(analogRead (Left)> 500) || !(analogRead (Center)>500) || !(analogRead (Right)> 500))
  // {
  //   Left_turn();
  // }
 // }
  // else{ 
  // while (sonar.ping_cm()<= 10)
  //   {Stop ();
  //    disp();}
  //     }
  //display.clearDisplay();

 }
 else
 line_follow();
}

void disp()
{
  display.display();
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(10, 5);
  display.println("Obstacle Detected");
  display.display();
}

void red_obs()
{
  display.display();
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(10, 5);
  display.println("Red Obstacle");
  display.display();
}
void disp1()
{
  voltage_current();
  display.display();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(10, 5);
  display.print("Voltage = ");
  display.println(voltage);
  display.print("V");

  display.setCursor(10, 20);
  display.print("currnet = ");
  display.print(current);
  display.print("mA");
//speed();
  display.setCursor(10, 40);
  display.print("Speed = ");
  display.println(rps);
  display.print("rps");

  display.display();
}

void voltage_current()
{
  current = analogRead(current_pin);
  voltage = analogRead(voltage_pin);
  voltage = map(voltage ,3200,0,5.0,0 );
}

void speed() {
  start_time = millis();
  end_time = start_time + 1000;

  // measure steps taken within 1 second
  while (millis() < end_time) {
    if (digitalRead(sensorPin)) {
      steps = steps + 1;
      while (digitalRead(sensorPin))
        ;
    }
  }
  temp = steps - steps_old;
  steps_old = steps;
  rps = (temp / 20);  // calculate revolutions per second, adjust the denominator to match your current situation.

  // print the revolutions per second
  Serial.print("rps:");
  Serial.println(rps);
}

int getRed() {
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);

  Frequency = pulseIn(sensorOut, LOW); 
  return Frequency;
}
int getGreen() {
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  Frequency = pulseIn(sensorOut, LOW);
  return Frequency;
}

int getBlue() {
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  Frequency = pulseIn(sensorOut, LOW);
  return Frequency; }
void detect_color()
{ 
  if( getRed()< getBlue() && getRed() <getGreen() )
  {
     colour = 'R';
  }
   else if( getBlue() < getRed() && getBlue() < getGreen() )
  {
    colour = 'B';
  }
   else if( getGreen() < getBlue() && getGreen() < getRed() )
  {
   colour = 'G';
  }
 
}