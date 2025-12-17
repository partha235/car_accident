// Basic demo for accelerometer readings from Adafruit MPU6050 with Ultrasonic sensor

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define senv 4

Adafruit_MPU6050 mpu;

// Declaring pins
int buz = 4, led = 13;
int trigPin = 9; // Ultrasonic Trigger pin
int echoPin = 10; // Ultrasonic Echo pin

void setup(void) {
  Serial.begin(115200);

  // Buzzer and LED setup
  pinMode(buz, OUTPUT);
  digitalWrite(buz, HIGH);
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
  pinMode(senv,OUTPUT);
  digitalWrite(senv,HIGH);


  // Ultrasonic sensor setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  while (!Serial)
    delay(10); // Wait for serial monitor

  Serial.println("Adafruit MPU6050 + Ultrasonic Sensor Test");

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
}

void loop() {
  digitalWrite(buz, HIGH);
  digitalWrite(led, LOW);
  digitalWrite(senv,HIGH);


  // Read MPU6050 data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  int x = g.gyro.x;
  Serial.print("Gyro X: ");
  Serial.println(x);

  // Read distance from ultrasonic sensor
  long duration;
  float distance;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2; // cm

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  

  // Condition for either motion or proximity
  if (x != 0 || distance < 15) {
    digitalWrite(buz, LOW);
    digitalWrite(led, HIGH);
    digitalWrite(senv,LOW);
    delay(1000);
  }

  delay(500);
}
