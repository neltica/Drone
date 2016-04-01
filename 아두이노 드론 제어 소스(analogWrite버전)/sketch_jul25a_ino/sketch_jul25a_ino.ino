#include<Wire.h>

int mPin1 = 11;
int mPin2 = 10;
int mPin3 = 9;
int mPin4 = 3;
const float radianToDegree = 180 / 3.14159;
const unsigned int accelScale = 16384;
const unsigned int gyroScale = 131;
int speedInfo;
const int MPU = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float acXFloat, acYFloat, acZFloat;
float aBiasX = -1.8680, aBiasY = 0.8570, aBiasZ, gBiasX = -319.5506, gBiasY = -858.9249, gBiasZ = -439.5857;
unsigned long preTime, nowTime, startTime;
float gapTime;
int count = 0;
unsigned int testTime;
unsigned int startFlag = 0;
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
float pidGain[3] = {0.2, 0, 5};
float errorRoll, errorPitch;
float preErrorRoll, preErrorPitch;
float d;
float iRoll = 0, iPitch = 0;
int dstRoll = 0, dstPitch = 0;
float motorRollSpeed = 0, motorPitchSpeed = 0;
int baseSpeed = 126;
float finalMotor1Speed, finalMotor2Speed, finalMotor3Speed, finalMotor4Speed;
int baseSpeedBias1 = 0;
int baseSpeedBias2 = 0;
int baseSpeedBias3 = 0;
int baseSpeedBias4 = 0;
////////////////////////////////////////////////////////
const int kpX = 30, kpY = 30, kpZ = 30; //complementary filter gain
const int kiX = 10, kiY = 10, kiZ = 10; //complementary filter gain
float preComplementaryAngleX = 0, preComplementaryAngleY = 0, preComplementaryAngleZ = 0;
float preAccelAngleX = 0, preAccelAngleY = 0, preAccelAngleZ = 0;
float temp1X, temp1Y, temp1Z;
float complementaryAngleX = 0, complementaryAngleY = 0, complementaryAngleZ = 0;
float temp2X, temp2Y, temp2Z;
float intTemp1X = 0;
float intTemp1Y = 0;
float intTemp1Z = 0;
////////////////////////////////////////////////////////
const float HDRIcX = 0.001, HDRIcY = 0.001, HDRIcZ = 0.001; //HDR alorithm gain
float finalGyroZ = 0; //gyro final value
float HDResultX = 0, HDResultY = 0, HDResultZ = 0; //gyro HDR algorithm final value
float preHDRX, preHDRY, preHDRZ;
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

void initSensor()
{
  unsigned int count = 0;
  int16_t acX, acY, acZ;
  int16_t gyX, gyY, gyZ;

  unsigned long startTime = millis();
  unsigned long preTime = 0;
  float gapTime = 0, nowTime;
  while (millis() - startTime <= 5000)
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 14, true); // request a total of 14 registers
    acX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    acY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    acZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    gyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    if (preTime != 0)
    {
      nowTime = millis();
      gapTime = (nowTime - preTime) / 1000.0;
      preTime = nowTime;

      count++;
      aBiasX += (atan( (float)acY / acZ ) * radianToDegree);
      aBiasY += (atan( (float)acX / acZ ) * radianToDegree);
      gBiasX += gyX;
      gBiasY += gyY;
      gBiasZ += gyZ;
    }
    else
    {
      preTime = millis();
    }
  }

  aBiasX /= count;
  aBiasY /= count;
  gBiasX /= count;
  gBiasY /= count;
  gBiasZ /= count;

}

inline int sign(float preW)
{
  if (preW > 0)
  {
    return 1;
  }
  else if (preW < 0)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}


void initSettingValue()
{
  Serial.print("destRoll:"); Serial.print(dstRoll);
  Serial.print(" | destPitch:"); Serial.println(dstPitch);

  Serial.print(" | iRoll:"); Serial.print(iRoll);
  Serial.print(" | iPitch:"); Serial.println(iPitch);

  Serial.print(" | finalGyroZ:"); Serial.println(finalGyroZ);
  Serial.print(" | HDResultX:"); Serial.print(HDResultX);
  Serial.print(" | HDResultY:"); Serial.print(HDResultY);
  Serial.print(" | HDResultZ:"); Serial.println(HDResultZ);

  Serial.print(" | preComplementaryAngleX:"); Serial.print(preComplementaryAngleX);
  Serial.print(" | preComplementaryAngleY:"); Serial.print(preComplementaryAngleY);
  Serial.print(" | preComplementaryAngleZ:"); Serial.println(preComplementaryAngleZ);

  Serial.print(" | preAccelAngleX:"); Serial.print(preAccelAngleX);
  Serial.print(" | preAccelAngleY:"); Serial.print(preAccelAngleY);
  Serial.print(" | preAccelAngleZ:"); Serial.println(preAccelAngleZ);

  Serial.print(" | complementaryAngleX:"); Serial.print(complementaryAngleX);
  Serial.print(" | complementaryAngleY:"); Serial.print(complementaryAngleY);
  Serial.print(" | complementaryAngleZ:"); Serial.println(complementaryAngleZ);

  Serial.print(" | intTempX:"); Serial.print(intTemp1X);
  Serial.print(" | intTempY:"); Serial.print(intTemp1Y);
  Serial.print(" | intTempZ:"); Serial.println(intTemp1Z);

  Serial.print(" | preErrorRoll:"); Serial.print(preErrorRoll);
  Serial.print(" | preErrorPitch:"); Serial.println(preErrorPitch);
  preErrorRoll = 0, preErrorPitch = 0;
  dstRoll = 0, dstPitch = 0;
  iRoll = 0, iPitch = 0;
  finalGyroZ = 0;
  HDResultX = 0, HDResultY = 0, HDResultZ = 0;
  preComplementaryAngleX = 0, preComplementaryAngleY = 0, preComplementaryAngleZ = 0;
  preAccelAngleX = 0, preAccelAngleY = 0, preAccelAngleZ = 0;
  complementaryAngleX = 0, complementaryAngleY = 0, complementaryAngleZ = 0;

  intTemp1X = 0;
  intTemp1Y = 0;
  intTemp1Z = 0;

}

inline float restrictSpeed(float value, unsigned int min, unsigned int max)
{
  if (value > max)
  {
    return max;
  }
  else if (value < min)
  {
    return min;
  }
  else
  {
    return value;
  }
}


void setDLPF()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x1A);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.write(6);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);
  Wire.write(0x1A);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 1, true); // request a total of 14 registers
  //Serial.println( Wire.read());
}


void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);

  pinMode(mPin1, OUTPUT);
  pinMode(mPin2, OUTPUT);
  pinMode(mPin3, OUTPUT);
  pinMode(mPin4, OUTPUT);
  analogWrite(mPin1, 126);
  analogWrite(mPin2, 126);
  analogWrite(mPin3, 126);
  analogWrite(mPin4, 126);

  setDLPF();
  initSensor();
  Serial.write(210);  //init end signal
}

void loop() {

  if (!Serial.available())
  {
    if (startFlag == 1)
    {
      Wire.beginTransmission(MPU);
      Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 14, true); // request a total of 14 registers
      AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
      AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
      GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
      GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
      //Serial.print(AcX);Serial.print("|");Serial.print(AcY);Serial.print("|");Serial.print(AcZ);Serial.println("|");
      if (preTime != 0)
      {
        //gapTime compute
        nowTime = millis();
        gapTime = (nowTime - preTime) / 1000.0;
        //Serial.println(gapTime,3);
        preTime = nowTime;


        /*
              //system loop count print
              testTime=nowTime-startTime;
              if(testTime>=1000)
              {
                //Serial.print(testTime);
                //Serial.print(":");
                Serial.println(count);
                count=0;
                startTime=millis();
              }
        */



        //gyro hdr algorithm
        preHDRX = (GyX - gBiasX) + HDResultX;               //I
        preHDRY = (GyY - gBiasY) + HDResultY;               //I
        preHDRZ = (GyZ - gBiasZ) + HDResultZ;               //I



        HDResultX = HDResultX - (sign(preHDRX) * HDRIcX);   //I(i-1)
        HDResultY = HDResultY - (sign(preHDRY) * HDRIcY);   //I(i-1)
        HDResultZ = HDResultZ - (sign(preHDRZ) * HDRIcZ);   //I(i-1)

        //finalGyroX+=((preHDRX/gyroScale)*gapTime);          //hdr end value
        //finalGyroY+=((preHDRY/gyroScale)*gapTime);          //hdr end value
        finalGyroZ += ((preHDRZ / gyroScale) * gapTime);    //hdr end value



        //complementary filter
        temp1X = preComplementaryAngleX - preAccelAngleX;
        temp1Y = preComplementaryAngleY - preAccelAngleY;


        intTemp1X += (temp1X * gapTime);
        intTemp1Y += (temp1Y * gapTime);


        temp2X = (temp1X * (kpX * -1)) + (intTemp1X * (kiX * -1)) + ((preHDRX) / gyroScale);
        temp2Y = (temp1Y * (kpY * -1)) + (intTemp1Y * (kiY * -1)) + ((preHDRY) / gyroScale);


        complementaryAngleX += (temp2X * gapTime);
        complementaryAngleY += (temp2Y * gapTime);
        complementaryAngleZ = finalGyroZ;


        //PID compute
        errorRoll = dstRoll - complementaryAngleX;
        iRoll += (errorRoll);
        d = (errorRoll - preErrorRoll);


        motorRollSpeed = errorRoll * pidGain[0] + iRoll * pidGain[1] + d * pidGain[2];


        errorPitch = dstPitch - complementaryAngleY;
        iPitch += (errorPitch);
        d = (errorPitch - preErrorPitch);


        motorPitchSpeed = errorPitch * pidGain[0] + iPitch * pidGain[1] + d * pidGain[2];


        finalMotor1Speed = ((baseSpeed + baseSpeedBias1) - motorRollSpeed + motorPitchSpeed);
        finalMotor2Speed = ((baseSpeed + baseSpeedBias2) + motorRollSpeed + motorPitchSpeed);
        finalMotor3Speed = ((baseSpeed + baseSpeedBias3) - motorRollSpeed - motorPitchSpeed);
        finalMotor4Speed = ((baseSpeed + baseSpeedBias4) + motorRollSpeed - motorPitchSpeed);
        
        finalMotor1Speed = restrictSpeed(finalMotor1Speed, 126, 250);
        finalMotor2Speed = restrictSpeed(finalMotor2Speed, 126, 250);
        finalMotor3Speed = restrictSpeed(finalMotor3Speed, 126, 250);
        finalMotor4Speed = restrictSpeed(finalMotor4Speed, 126, 250);



        //esc control
        analogWrite(mPin1, finalMotor1Speed);
        analogWrite(mPin2, finalMotor2Speed);
        analogWrite(mPin3, finalMotor3Speed);
        analogWrite(mPin4, finalMotor4Speed);




        preErrorRoll = errorRoll;
        preErrorPitch = errorPitch;
        preComplementaryAngleX = complementaryAngleX;
        preComplementaryAngleY = complementaryAngleY;
        preAccelAngleX = (atan( (float)AcY / AcZ ) * radianToDegree) - aBiasX;
        preAccelAngleY = (atan( (float)AcX / AcZ ) * radianToDegree) - aBiasY;

        /*
            Serial.print("finalAccelX:"); Serial.print(finalAccelX);
            Serial.print(" | finalAccelY:"); Serial.print(finalAccelY);
            Serial.print(" | finalGyroX:"); Serial.print(finalGyroX);
            Serial.print(" | finalGyroY:"); Serial.print(finalGyroY);
            Serial.print(" | finalGyroZ:"); Serial.print(finalGyroZ);
            */

        //Serial.println(baseSpeed);
        //Serial.print(" | ");
        //Serial.print(complementaryAngleX); Serial.print(" | ");
        //Serial.print(" | ");
        //Serial.print(complementaryAngleY); Serial.print(" | ");
        //Serial.print(" | ");
        //Serial.println(complementaryAngleZ);



        //Serial.print(finalMotor1Speed); Serial.print(" | ");

        //Serial.print(finalMotor2Speed); Serial.print(" | ");

        //Serial.print(finalMotor3Speed); Serial.print(" | ");

        //Serial.println(finalMotor4Speed);



        //Serial.write((int)(finalMotor1Speed));
        //Serial.write((int)(finalMotor2Speed));
        //Serial.write((int)(finalMotor3Speed));
        //Serial.write((int)(finalMotor4Speed));

        //Serial.write(10);
        //Serial.write(complementaryAngleX);
        //Serial.write(20);
        //Serial.write(complementaryAngleY);
        //Serial.write(30);
        //Serial.write(complementaryAngleZ);
        //Serial.write(40);

      }
      else
      {
        preTime = millis();
        //startTime=millis();

      }
      //delay(333);
      count++;
    }
  }
  else
  {
    
    int value = Serial.read();
    if (126 <= value && value <= 250)
    {
      baseSpeed = value;
    }
    switch (value)
    {
      case 1://stop
        startFlag = 0;
        baseSpeed = 126;
        analogWrite(mPin1, 126);
        analogWrite(mPin2, 126);
        analogWrite(mPin3, 126);
        analogWrite(mPin4, 126);
        initSettingValue();
        break;

      case 2://start
        startFlag = 1;
        break;

      case 4://up
        baseSpeed += 1;
        if (baseSpeed > 250)
        {
          baseSpeed = 250;
        }
        break;

      case 5://down
        baseSpeed -= 1;
        if (baseSpeed < 126)
        {
          baseSpeed = 126;
        }
        break;

      case 's':
        startFlag = 1;
        break;
      case 'h':
        startFlag = 0;
        baseSpeed = 126;
        analogWrite(mPin1, 126);
        analogWrite(mPin2, 126);
        analogWrite(mPin3, 126);
        analogWrite(mPin4, 126);
        initSettingValue();
        break;
      case '[':
        baseSpeed -= 1;
        if (baseSpeed < 126)
        {
          baseSpeed = 126;
        }
        break;
      case ']':
        baseSpeed += 1;
        if (baseSpeed > 250)
        {
          baseSpeed = 250;
        }
        break;

    }
    Serial.write(value);
    
  }




}

