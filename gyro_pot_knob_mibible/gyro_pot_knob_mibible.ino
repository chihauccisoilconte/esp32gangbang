#include <Wire.h>
#include <Adafruit_MPU6050.h> //go to https://github.com/adafruit/Adafruit_MPU6050
#include <Adafruit_Sensor.h>
#include <BLEMidi.h> // https://github.com/max22-/ESP32-BLE-MIDI

Adafruit_MPU6050 mpu;

const int buttonPin = 19;  // GPIO pin connected to the button
const int potentiometerPin = 34;

void setup() {
  Wire.begin(4, 5); // I2C communication using GPIO21 (SDA) and GPIO22 (SCL),
  pinMode(buttonPin, INPUT);
  digitalWrite(buttonPin, LOW);  // Enable internal pull-down resistor


  Serial.begin(115200);

  BLEMidiServer.begin("Basic MIDI device");
  while (!Serial); // Wait for serial monitor to open

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G); // Set accelerometer range to 2g
  mpu.setGyroRange(MPU6050_RANGE_250_DEG); // Set gyroscope range to 250 degrees per second
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ); // Set filter bandwidth to 5Hz
}

void loop() {

  int potValue3 = analogRead(potentiometerPin); //we tell the program to read the analog value that comes from the potentiometer
  int buttonState = digitalRead(buttonPin); //we tell the program to read what happens in our digital pin
  int buttonvalue = 0;


  if (buttonState == HIGH) {
    // Button is pressed
    buttonvalue = 1;
    // Perform any action you desire when the button is pressed
    // For example, you can toggle an LED, trigger a function, etc.
  }





  sensors_event_t accel, gyro, temp;

  mpu.getEvent(&accel, &gyro, &temp);

  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;

  // Calculate tilt angles using accelerometer data
  float roll = atan2(ay, az) * RAD_TO_DEG;
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

  // Map angles detected by the gyroscope to 0-180 range, you can edit it if you want a different mapping of the sensor
  roll = mapAngle(roll, 0, 127);
  pitch = mapAngle(pitch, 0, 127);

 int knob3 = map(potValue3, 1, 4095, 0, 127);

  // Print what we receive from sensors so that we know what is going on

  Serial.print("Roll (X-axis) (°): ");
  Serial.print(roll);
  Serial.print(" | Pitch (Y-axis) (°): ");
  Serial.println(pitch);
  Serial.print("Potentiometer 3: ");
  Serial.println(potValue3);
  Serial.print("buttonval: ");
  Serial.println(buttonvalue);

  delay(100); // Adjust delay as needed

  if (BLEMidiServer.isConnected()) {

    if (roll > 15) {

      BLEMidiServer.controlChange(0, 1, roll);
    } else {
      roll == 0;
    }

    if (pitch > 15) {

      BLEMidiServer.controlChange(0, 2, pitch);
    } else {
      pitch == 0;
    }

    if (knob3 > 15) {

      BLEMidiServer.controlChange(0, 2, knob3);
    } else {
      knob3 == 0;
    }

    if (buttonState == 1) {
      ;
      //play note
      BLEMidiServer.noteOn(0, 69, 127);
      
      BLEMidiServer.noteOff(0, 69, 127);        // Then we make a delay of one second before returning to the beginning of the loop
      delay(1000);
    } else {
      ;
    }




  }



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
