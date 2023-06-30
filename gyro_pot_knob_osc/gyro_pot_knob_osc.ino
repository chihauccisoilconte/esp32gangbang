#include <WiFi.h>
#include <WiFiUDP.h>
#include <OSCMessage.h> // https://github.com/CNMAT/OSC
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;


const char* ssid = "your_SSID";
const char* password = "your_PASSWORD";

const IPAddress remoteIP(192, 168, 0, 100);  // IP address of the OSC receiver
const unsigned int remotePort = 1234;        // Port number of the OSC receiver

WiFiUDP udp;

const int buttonPin = 19;     // Pin connected to the button
const int potPin = 34;        // Pin connected to the potentiometer

int lastButtonState = HIGH;   // Previous state of the button
int buttonState;              // Current state of the button
int potValue;                 // Current value of the potentiometer

void setup() {
  Wire.begin(4, 5); // I2C communication using GPIO21 (SDA) and GPIO22 (SCL),
  Serial.begin(115200);

  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G); // Set accelerometer range to 2g
  mpu.setGyroRange(MPU6050_RANGE_250_DEG); // Set gyroscope range to 250 degrees per second
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ); // Set filter bandwidth to 5Hz

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Start UDP
  udp.begin(remotePort);
  Serial.println("UDP started");

  // Set button pin as input with internal pull-up resistor
  pinMode(buttonPin, INPUT_PULLUP);

  // Set potentiometer pin as input
  pinMode(potPin, INPUT);
}

void loop() {
  // Read button state
  buttonState = digitalRead(buttonPin);

  // Send OSC message when button is pressed
  if (buttonState == LOW && lastButtonState == HIGH) {
    OSCMessage message("/button");
    message.add("Button pressed!");
  
    udp.beginPacket(remoteIP, remotePort);
    message.send(udp);
    udp.endPacket();
  }


  

  // Read potentiometer value
  potValue = analogRead(potPin);

  // Map potentiometer value to a desired range (e.g., 0 to 100)
  int mappedValue = map(potValue, 0, 4095, 0, 100);

  OSCMessage message1("/potentiometer1");
  message1.add(mappedValue);

  udp.beginPacket(remoteIP, remotePort);
  message1.send(udp);
  udp.endPacket();

//read and process gyroscope values

    sensors_event_t accel, gyro, temp;

  mpu.getEvent(&accel, &gyro, &temp);

  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;

  // Calculate tilt angles using accelerometer data
  float roll = atan2(ay, az) * RAD_TO_DEG;
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

  // Map angles detected by the gyroscope to 0-180 range, you can edit it if you want a different mapping of the sensor
  roll = mapAngle(roll, 0, 180);
  pitch = mapAngle(pitch, 0, 180);


  OSCMessage message2("/roll");
  message2.add(roll);

  udp.beginPacket(remoteIP, remotePort);
  message2.send(udp);
  udp.endPacket();

    OSCMessage message3("/pitch");
  message3.add(pitch);

  udp.beginPacket(remoteIP, remotePort);
  message3.send(udp);
  udp.endPacket();



  // Update the last button state
  lastButtonState = buttonState;

  delay(100); // Adjust the delay as needed
}

float mapAngle(float angle, float fromMin, float fromMax) {
  // Map angle to specified range
  float mappedAngle = map(angle, -90, 90, fromMin, fromMax);
  if (mappedAngle < fromMin) {
    mappedAngle = fromMin;
  } else if (mappedAngle > fromMax) {
    mappedAngle = fromMax;
  }
  return mappedAngle;
}
