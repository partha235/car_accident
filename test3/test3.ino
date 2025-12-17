#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#define senv 4

Adafruit_MPU6050 mpu;
SoftwareSerial BTSerial(3, 2); // HC-05 on pins RX=2, TX=3

// General IO
int buz = 4, led = 13;
int trigPin = 9, echoPin = 10;

// Motor control pins
int IN1 = 5;  // Left Motor
int IN2 = 6;
int IN3 = 7;  // Right Motor
int IN4 = 8;
int ENA = 11; // Speed for Left Motor (optional)
int ENB = 12; // Speed for Right Motor (optional)

void setup(void) {
  Serial.begin(115200);
  BTSerial.begin(9600);

  pinMode(buz, OUTPUT); digitalWrite(buz, HIGH);
  pinMode(led, OUTPUT); digitalWrite(led, LOW);
  pinMode(senv, OUTPUT); digitalWrite(senv, HIGH);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  motorStop();

#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_MEGAAVR)
  while (!Serial) delay(10);
#endif


  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);
}

void loop() {
  digitalWrite(buz, HIGH);
  digitalWrite(led, LOW);
  digitalWrite(senv, HIGH);

  // Read MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  int x = g.gyro.x;
  Serial.print("Gyro X: "); Serial.println(x);

  // Read Ultrasonic
  long duration;
  float distance;
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  Serial.print("Distance: "); Serial.print(distance); Serial.println(" cm");

  if (x != 0 || distance < 15) {
    digitalWrite(buz, LOW);
    digitalWrite(led, HIGH);
    digitalWrite(senv, LOW);
    motorStop();
    BTSerial.println("Alert: Motion or Obstacle");
    delay(1000);
  } else {
    motorForward();
  }

  // Bluetooth Control
  if (BTSerial.available()) {
    char cmd = BTSerial.read();
    Serial.println(cmd);
    if (cmd == 'F') motorForward();
    else if (cmd == 'B') motorBackward();
    else if (cmd == 'R') motorRight();
    else if (cmd == 'L') motorLeft();
    else if (cmd == 'S') motorStop();
    // delay(200);
  }

  delay(300);
}

// ===== MOTOR FUNCTIONS =====
void motorForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); // Left motor forward
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); // Right motor forward
  analogWrite(ENA, 180);
  analogWrite(ENB, 180);
}

void motorBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); // Left motor backward
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); // Right motor backward
  analogWrite(ENA, 180);
  analogWrite(ENB, 180);
}

void motorLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); // Left motor backward
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); // Right motor forward
  analogWrite(ENA, 180);
  analogWrite(ENB, 180);
  // delay(500);
}

void motorRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); // Left motor forward
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); // Right motor backward
  analogWrite(ENA, 180);
  analogWrite(ENB, 180);
  // delay(500);
}

void motorStop() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
