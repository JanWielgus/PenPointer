/*
Pomysły:
- pen pointer - wskazywanie na ekranie
- pen writer - długopis, ale ślad pisany na kartce jest też zapisywany cyfrowo

*/

#include <Arduino.h>
#include <BleMouse.h>
#include <SimpleMPU6050.h>
#include <Wire.h>
#include <Fusion/Fusion.h>
#include <type_traits>
#include <Tasker.h>
#include <VectorOffset.h>
#include <cmath>

const float SampleRate_Hz = 300.f; // TODO: try to get even more than 300
const float SamplePeriod_s = 1.f/SampleRate_Hz;
const uint32_t SamplePeriod_us = SamplePeriod_s * 1000000;

const float MouseRotationalSensitivityX = 6500.f;
const float MouseRotationalSensitivityY = 4500.f;
const float MousePositionalSensitivityX = 3.f;
const float MousePositionalSensitivityY = 3.f;

const FusionAhrsSettings fusionSettings = { // TODO: test different settings
  .gain = 0.5f,
  .accelerationRejection = 10.0f,
  .magneticRejection = 20.0f,
  .rejectionTimeout = (uint16_t)(5 * SampleRate_Hz), /* 5 seconds */
};


void printFloat3(const float* arr);

template <class T>
void printInt3(const T* arr);

class TimedDerivative
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

  void reset()
  {
    lastValue = 0;
    lastReturnValue = 0;
  }
};

class TimedIntegral
{
  float integral = 0;

public:
  float update(float newValue, float dT_s)
  {
    integral += newValue * dT_s;
    return integral;
  }

  float getLastIntegral()
  {
    return integral;
  }
};


BleMouse bleMouse;
SimpleMPU6050 mpu;
FusionAhrs fusionAhrs;
FusionOffset fusionOffset;
Tasker tasker(10);

float cursorDisplacementSumX = 0;
float cursorDisplacementSumY = 0;



class : public IExecutable
{
  
  TimedDerivative rotationDerivativeX, rotationDerivativeY;
  TimedIntegral earthAccIntegral[3];
  VectorOffset earthAccOffset = VectorOffset(SamplePeriod_s, 1.1, 4, 0.5);
  VectorOffset earthAccIntegralOffset = VectorOffset(SamplePeriod_s, 0.2, 5, 0.6);
  bool accelerationIntegralLatch = false; // only calculate acceleration integral when true
  const float AccelerationLatchThreshold = 0.01;

  void execute() override
  {
    updateIMU();
    updateRotationalDisplacement();
    // updatePositionalDisplacement(); // TODO: improve positional displacement calculation
  }

  void updateIMU()
  {
    mpu.readRawData();
    auto mpuAcc_G = mpu.getNormalizedAcceleration();
    auto mpuGyro_rps = mpu.getNormalizedRotation();
    FusionVector fusionGyro = {mpuGyro_rps.x, mpuGyro_rps.y, mpuGyro_rps.z};
    FusionVector fusionAcc = {mpuAcc_G.x, mpuAcc_G.y, mpuAcc_G.z};

    fusionGyro = FusionOffsetUpdate(&fusionOffset, fusionGyro);
    FusionAhrsUpdateNoMagnetometer(&fusionAhrs, fusionGyro, fusionAcc, SamplePeriod_s);
  }

  void updateRotationalDisplacement()
  {
    FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&fusionAhrs));
    rotationDerivativeX.update(-euler.angle.yaw, SamplePeriod_s);
    rotationDerivativeY.update(euler.angle.pitch, SamplePeriod_s);

    float cursorXDisplacement = rotationDerivativeX.getLastDerivative() * MouseRotationalSensitivityX;
    float cursorYDisplacement = rotationDerivativeY.getLastDerivative() * MouseRotationalSensitivityY;
    // Serial.print(cursorXDisplacement);
    // Serial.print('\t');
    // Serial.println(cursorYDisplacement);

    // accumulate displacement for sending with lower frequency
    cursorDisplacementSumX += cursorXDisplacement;
    cursorDisplacementSumY += cursorYDisplacement;
  }

  void updatePositionalDisplacement()
  {
    // calculate acceleration
    FusionVector earthAcc_G = FusionAhrsGetEarthAcceleration(&fusionAhrs);
    FusionVector earthAcc_mps2 = FusionVectorMultiplyScalar(earthAcc_G, 9.81f); // G to mps2
    earthAcc_mps2 = earthAccOffset.update(earthAcc_mps2);

    // printFloat3(earthAcc_mps2.array);

    /// TODO: make displacement calculation active when detected all axed within small range close to 0.
    /// Mainly activate it when offset will be calculated.
    /// This will cause that there won't be any unwanted displacement at the beginning due to the initial error.

    if (accelerationIntegralLatch)
    {
      FusionVector earthVelocity_mps = {
        earthAccIntegral[0].update(earthAcc_mps2.array[0], SamplePeriod_s),
        earthAccIntegral[1].update(earthAcc_mps2.array[1], SamplePeriod_s),
        earthAccIntegral[2].update(earthAcc_mps2.array[2], SamplePeriod_s)
      };

      earthVelocity_mps = earthAccIntegralOffset.update(earthVelocity_mps);

      float cursorXDisplacement = -earthVelocity_mps.axis.y * MousePositionalSensitivityX;
      float cursorYDisplacement = -earthVelocity_mps.axis.z * MousePositionalSensitivityY;

      // Serial.print(earthVelocity_mps.axis.y);
      // Serial.print('\t');
      // Serial.println(earthVelocity_mps.axis.z);

      cursorDisplacementSumX += cursorXDisplacement;
      cursorDisplacementSumY += cursorYDisplacement;
    }
    else
    {
      updateAccelerationIntegralLatch(earthAcc_mps2);
    }
  }

  void updateAccelerationIntegralLatch(const FusionVector& earthAcc_mps2)
  {
    // set latch to true if all values are in specified range
    if (
      fabs(earthAcc_mps2.axis.x) <= AccelerationLatchThreshold &&
      fabs(earthAcc_mps2.axis.y) <= AccelerationLatchThreshold &&
      fabs(earthAcc_mps2.axis.z) <= AccelerationLatchThreshold
    )
    {
      accelerationIntegralLatch = true;
    }
  }
} updatePenPointer_task;


class : public IExecutable
{
  void execute() override
  {
    bleMouse.move(cursorDisplacementSumX, cursorDisplacementSumY);
    cursorDisplacementSumX = 0;
    cursorDisplacementSumY = 0;
  }
} sendData_task;


void setup() {
  // Serial.begin(115200);
  Serial.begin(256000);
  Serial.println("Starting BLE work!");
  bleMouse.begin();

  Wire.begin();
  Wire.setClock(400000L);
  mpu.initialize();
  mpu.setGyroOffset(-194, 1, -53);

  FusionAhrsInitialise(&fusionAhrs);
  FusionAhrsSetSettings(&fusionAhrs, &fusionSettings);
  FusionOffsetInitialise(&fusionOffset, SampleRate_Hz);

  tasker.addTask_Hz(&updatePenPointer_task, SampleRate_Hz);
  tasker.addTask_Hz(&sendData_task, 50.f);
}


void loop() {
  tasker.loop();
}



void printFloat3(const float* arr)
{
  Serial.printf("%0.01f\t%0.01f\t%0.01f\n", arr[0], arr[1], arr[2]);
}

template <class T>
void printInt3(const T* arr)
{
  // typename std::enable_if<std::is_integral<T>::value>::type;
  Serial.printf("%d\t%d\t%d\n", arr[0], arr[1], arr[2]);
}
