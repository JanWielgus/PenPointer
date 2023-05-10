#include <Arduino.h>
#include <BleMouse.h>
#include <SimpleMPU6050.h>
#include <Wire.h>
#include <Fusion/Fusion.h>
#include <type_traits>

BleMouse bleMouse;
SimpleMPU6050 mpu;
FusionAhrs ahrs;
FusionOffset fusionOffset;

const float SampleRate_Hz = 300.f;
const float SamplePeriod_s = 1.f/SampleRate_Hz;
const uint32_t SamplePeriod_us = SamplePeriod_s * 1000000;

const float MouseSensitivity = 5000.f;

const int BTSendingFrequencyDivider = 6;

const FusionAhrsSettings fusionSettings = { // TODO: test different settings
  .gain = 0.5f,
  .accelerationRejection = 10.0f,
  .magneticRejection = 20.0f,
  .rejectionTimeout = (uint16_t)(5 * SampleRate_Hz), /* 5 seconds */
};

void printFloat3(const float* arr)
{
  Serial.printf("%0.1f\t%0.1f\t%0.1f\n", arr[0], arr[1], arr[2]);
}

template <class T>
void printInt3(const T* arr)
{
  // typename std::enable_if<std::is_integral<T>::value>::type;
  Serial.printf("%d\t%d\t%d\n", arr[0], arr[1], arr[2]);
}

class Derivative
{
  float lastValue = 0.f;
  float lastReturnValue = 0.f;

public:
  float update(float newValue, float dT_s)
  {
    float diff = newValue - lastValue;
    lastReturnValue = diff * dT_s;
    lastValue = newValue;
    return lastReturnValue;
  }

  float getLastDerivative()
  {
    return lastReturnValue;
  }
};

Derivative mouseX, mouseY;
float derivSumX = 0, derivSumY = 0;
int freqDividerCounter = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting BLE work!");
  bleMouse.begin();

  Wire.begin();
  Wire.setClock(400000L);
  mpu.initialize();

  FusionAhrsInitialise(&ahrs);
  FusionAhrsSetSettings(&ahrs, &fusionSettings);
  FusionOffsetInitialise(&fusionOffset, SampleRate_Hz);

  mpu.setGyroOffset(-194, 1, -53);
}

void loop() {
  uint32_t startTime_us = micros();

  mpu.readRawData();

  // auto acc = mpu.getNormalizedAcceleration();
  // Serial.println(acc.x);

  auto acc = mpu.getNormalizedAcceleration();
  auto gyro = mpu.getNormalizedRotation();

  FusionVector fusionGyro = {gyro.x, gyro.y, gyro.z};
  FusionVector fusionAcc = {acc.x, acc.y, acc.z};

  fusionGyro = FusionOffsetUpdate(&fusionOffset, fusionGyro);

  FusionAhrsUpdateNoMagnetometer(&ahrs, fusionGyro, fusionAcc, SamplePeriod_s);

  FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

  // Serial.printf("%0.1f\t%0.1f\t%0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
  // printFloat3(fusionGyro.array);
  // printFloat3(euler.array); /////
  // printFloat3(&gyro.x);
  // auto rawGyro = mpu.getRawRotation();
  // printInt3(&rawGyro.x);

  mouseX.update(-euler.angle.yaw, SamplePeriod_s);
  mouseY.update(euler.angle.pitch, SamplePeriod_s);

  float mouseXValue = mouseX.getLastDerivative() * MouseSensitivity;
  float mouseYValue = mouseY.getLastDerivative() * MouseSensitivity;
  // Serial.print(mouseXValue);
  // Serial.print('\t');
  // Serial.println(mouseYValue);

  derivSumX += mouseXValue;
  derivSumY += mouseYValue;
  

  // Every BTSendingFrequencyDivider execution
  if (freqDividerCounter == 0)
  {
    bleMouse.move(derivSumX, derivSumY);
    derivSumX = derivSumY = 0;
  }
  freqDividerCounter++;
  if (freqDividerCounter == BTSendingFrequencyDivider)
    freqDividerCounter = 0;


  // bleMouse.move(mouseXValue, mouseYValue);


  // int32_t timeToWait_us = SamplePeriod_us - (micros() - startTime_us);
  // // Serial.println(timeToWait_us);
  // delayMicroseconds(timeToWait_us < 0 ? 0 : timeToWait_us);
  while (micros() < (startTime_us + SamplePeriod_us));
  // END
}
