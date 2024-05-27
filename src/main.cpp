#include <Arduino.h>
#include <BluetoothSerial.h>
#include "Wire.h"
#include <MPU6050_light.h>

#define COUNT_LOW 1638
#define COUNT_HIGH 7864

#define TIMER_WIDTH 16

BluetoothSerial ble;
MPU6050 mpu(Wire);

// Define buzzer pin
int buzzerPin = 4;

// Define DHT22 pin
int dhtPin = 15;

// Define Fog Light pin
int foglightPin = 16;

// Define Rain Sensor pin
int rainsensorPin = 34;

// Define Servo Pin
int servoPin = 32;

// Define motor control pins
int ENB = 12;
int In4 = 14;
int In3 = 27;
int In2 = 26;
int In1 = 25;
int ENA = 33;

// Define IR sensor pins
int LS = 35;
int RS = 39;

// Motor speed (0 to 255)
int speed = 170; // Initial speed set to half speed

// Define PWM channels
int pwmChannelA = 3;
int pwmChannelB = 4;

void moveForward();
void moveBackward();
void turnSlightLeft();
void turnSharpLeft();
void turnSlightRight();
void turnSharpRight();
void stopMotors();
void loop2(void *pvParameters);
void fogLight();
void outOfTrack();
void detectRotation();

void setup()
{
  Serial.begin(115200);
  ble.begin("esp32");
  Wire.begin();

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0)
  {
  } // stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true, true); // gyro and accelero
  Serial.println("Done!\n");

  // Set the motor control pins as outputs
  pinMode(ENB, OUTPUT);
  pinMode(In4, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(buzzerPin,OUTPUT);

  // Set the sensor pins as inputs
  pinMode(LS, INPUT);
  pinMode(RS, INPUT);

  pinMode(rainsensorPin, INPUT);

  // Configure PWM channels
  ledcSetup(pwmChannelA, 5000, 8); // Channel 0, 5kHz, 8-bit resolution
  ledcAttachPin(ENA, pwmChannelA);

  ledcSetup(pwmChannelB, 5000, 8); // Channel 1, 5kHz, 8-bit resolution
  ledcAttachPin(ENB, pwmChannelB);

  ledcSetup(1, 50, TIMER_WIDTH); // channel 3, 50 Hz, 16-bit width
  ledcAttachPin(servoPin, 1);

  pinMode(foglightPin,OUTPUT);

  xTaskCreatePinnedToCore(
      loop2,   // Function to implement the task
      "loop2", // Name of the task
      20000,   // Stack size in bytes
      NULL,    // Task input parameter
      1,       // Priority of the task
      NULL,    // Task handle.
      0        // Core where the task should run
  );
}

void loop()
{
  detectRotation();
  fogLight();
  outOfTrack();
  delay(50);
}

void loop2(void *pvParameters)
{
  while (1)
  {
    if (ble.available())
    {
      char c = ble.read();
      if (c == 'U')
      {
        moveForward();
      }
      else if (c == 'D')
      {

        moveBackward();
      }
      else if (c == 'R')
      {

        turnSharpRight();
      }
      else if (c == 'L')
      {

        turnSharpLeft();
      }
      else
      {

        stopMotors();
      }
    }
    delay(10);
  }
}

void detectRotation()
{
  mpu.update();
  float rotationalSpeed = mpu.getGyroZ();

  float mappedValue = map(mpu.getGyroZ(), -150, 150, COUNT_LOW, COUNT_HIGH);
  if (mappedValue >= COUNT_LOW && mappedValue <= COUNT_HIGH)
  {
    if (mappedValue < 4900 && mappedValue > 4600)
    {
      ledcWrite(1, 4751);
    }
    else
    {
      ledcWrite(1, mappedValue);
      Serial.println(mappedValue);
    }
  }
}

void fogLight()
{
  bool val = digitalRead(rainsensorPin);
  if (val == LOW)
  {
    digitalWrite(foglightPin,HIGH);
  }
  else
  {
    digitalWrite(foglightPin, LOW);
  }
}

void outOfTrack()
{
  if (digitalRead(LS) == HIGH && digitalRead(RS) == HIGH)
  {
    digitalWrite(buzzerPin, HIGH);
  }
  else
  {
    digitalWrite(buzzerPin, LOW);
  }
}

void moveForward()
{
  digitalWrite(In1, LOW); // Swap HIGH and LOW signals
  digitalWrite(In2, HIGH);
  ledcWrite(pwmChannelA, speed); // Set speed

  digitalWrite(In3, LOW); // Swap HIGH and LOW signals
  digitalWrite(In4, HIGH);
  ledcWrite(pwmChannelB, speed); // Set speed
}

void moveBackward()
{
  digitalWrite(In1, HIGH); // Swap HIGH and LOW signals
  digitalWrite(In2, LOW);
  ledcWrite(pwmChannelA, speed); // Set speed

  digitalWrite(In3, HIGH); // Swap HIGH and LOW signals
  digitalWrite(In4, LOW);
  ledcWrite(pwmChannelB, speed); // Set speed
}

void turnSlightRight()
{
  digitalWrite(In1, LOW); // Swap HIGH and LOW signals
  digitalWrite(In2, HIGH);
  ledcWrite(pwmChannelA, speed / 2); // Reduce speed

  digitalWrite(In3, LOW); // Swap HIGH and LOW signals
  digitalWrite(In4, HIGH);
  ledcWrite(pwmChannelB, speed); // Set speed
}

void turnSharpRight()
{
  digitalWrite(In1, HIGH); // Swap HIGH and LOW signals
  digitalWrite(In2, LOW);
  ledcWrite(pwmChannelA, speed); // Set speed

  digitalWrite(In3, LOW); // Swap HIGH and LOW signals
  digitalWrite(In4, HIGH);
  ledcWrite(pwmChannelB, speed); // Set speed
}

void turnSlightLeft()
{
  digitalWrite(In1, LOW); // Swap HIGH and LOW signals
  digitalWrite(In2, HIGH);
  ledcWrite(pwmChannelA, speed); // Set speed

  digitalWrite(In3, LOW); // Swap HIGH and LOW signals
  digitalWrite(In4, HIGH);
  ledcWrite(pwmChannelB, speed / 2); // Reduce speed
}

void turnSharpLeft()
{
  digitalWrite(In1, LOW); // Swap HIGH and LOW signals
  digitalWrite(In2, HIGH);
  ledcWrite(pwmChannelA, speed); // Set speed

  digitalWrite(In3, HIGH); // Swap HIGH and LOW signals
  digitalWrite(In4, LOW);
  ledcWrite(pwmChannelB, speed); // Set speed
}

void stopMotors()
{
  ledcWrite(pwmChannelA, 0);
  ledcWrite(pwmChannelB, 0);
}
