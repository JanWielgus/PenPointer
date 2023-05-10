#include <Arduino.h>
#include <BleMouse.h>
#include <SimpleMPU6050.h>
#include <Wire.h>
#include <Fusion/Fusion.h>
#include <type_traits>
#include <Tasker.h>

BleMouse bleMouse;
SimpleMPU6050 mpu;
FusionAhrs ahrs;
FusionOffset fusionOffset;

const float SampleRate_Hz = 300.f;
const float SamplePeriod_s = 1.f/SampleRate_Hz;
const uint32_t SamplePeriod_us = SamplePeriod_s * 1000000;

const float MouseSensitivity = 5000.f;


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


Tasker tasker(10);

float cursorDisplacementSumX = 0;
float cursorDisplacementSumY = 0;



class : public IExecutable
{
  FusionEuler euler;
  Derivative derivativeX, derivativeY;

  void execute() override
  {
    updateIMU();
    updateCalculations();
  }

  void updateIMU()
  {
    mpu.readRawData();
    auto acc = mpu.getNormalizedAcceleration();
    auto gyro = mpu.getNormalizedRotation();
    FusionVector fusionGyro = {gyro.x, gyro.y, gyro.z};
    FusionVector fusionAcc = {acc.x, acc.y, acc.z};

    fusionGyro = FusionOffsetUpdate(&fusionOffset, fusionGyro);
    FusionAhrsUpdateNoMagnetometer(&ahrs, fusionGyro, fusionAcc, SamplePeriod_s);
    euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

    // Serial.printf("%0.1f\t%0.1f\t%0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
    // printFloat3(fusionGyro.array);
    // printFloat3(euler.array); /////
    // printFloat3(&gyro.x);
    // auto rawGyro = mpu.getRawRotation();
    // printInt3(&rawGyro.x);
  }

  void updateCalculations()
  {
    derivativeX.update(-euler.angle.yaw, SamplePeriod_s);
    derivativeY.update(euler.angle.pitch, SamplePeriod_s);

    float mouseXValue = derivativeX.getLastDerivative() * MouseSensitivity;
    float mouseYValue = derivativeY.getLastDerivative() * MouseSensitivity;
    // Serial.print(mouseXValue);
    // Serial.print('\t');
    // Serial.println(mouseYValue);

    cursorDisplacementSumX += mouseXValue;
    cursorDisplacementSumY += mouseYValue;
  }
} updatePenPointer_task;


class : public IExecutable
{
  void execute() override
  {
    bleMouse.move(cursorDisplacementSumX, cursorDisplacementSumY);

    // Serial.print(cursorDisplacementSumX);
    // Serial.print('\t');
    // Serial.println(cursorDisplacementSumY);

    cursorDisplacementSumX = cursorDisplacementSumY = 0;
  }
} sendData_task;


void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");
  bleMouse.begin();

  Wire.begin();
  Wire.setClock(400000L);
  mpu.initialize();
  mpu.setGyroOffset(-194, 1, -53);

  FusionAhrsInitialise(&ahrs);
  FusionAhrsSetSettings(&ahrs, &fusionSettings);
  FusionOffsetInitialise(&fusionOffset, SampleRate_Hz);

  tasker.addTask_Hz(&updatePenPointer_task, SampleRate_Hz);
  tasker.addTask_Hz(&sendData_task, 50.f);
}


void loop() {
  tasker.loop();
}
