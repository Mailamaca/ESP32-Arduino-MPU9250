
// ref: https://github.com/yelvlab/ESP32-Arduino-MPU9250 and mailamaca fork
#include <MPU9250.h>

#include <stdint.h>
#include "driver/gpio.h"

#define I2C_MASTER_SDA_PIN 18
#define I2C_MASTER_SCL_PIN 19

i2cbus::I2C i2cMaster(i2c1);
MPU9250 mpu9250(i2cMaster, 0x68); //i2c_port_1
int imureadings = 0;

union unionfloat{
  float i;
  uint8_t c[4];
};

unionfloat imuValues[9];

void setup() {
  // put your setup code here, to run once:

  // start i2c-1 (master)
  i2cMaster.begin((gpio_num_t)I2C_MASTER_SDA_PIN, (gpio_num_t)I2C_MASTER_SCL_PIN, GPIO_PULLUP_ENABLE, GPIO_PULLUP_ENABLE, 400000);
  printf(">> Setting up I2C-1 (master)");
  i2cMaster.scanner();
  // start communication with IMU
  int stat = mpu9250.begin(false);
  if (stat < 0)
  {
    printf("mpu9250 initialization unsuccessful. status: %d \n", stat);
    while (1) {;}
  }  
  mpu9250.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  mpu9250.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  mpu9250.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_184HZ);
  mpu9250.setSrd(0); //Data Output Rate = 1000 / (1 + SRD)
  
}

void readImu(MPU9250 *sensor) {
  sensor->readSensor();
  imuValues[0].i += sensor->getAccelX_mss();
  imuValues[1].i += sensor->getAccelY_mss();
  imuValues[2].i += sensor->getAccelZ_mss();
  imuValues[3].i += sensor->getGyroX_rads();
  imuValues[4].i += sensor->getGyroY_rads();
  imuValues[5].i += sensor->getGyroZ_rads();
  imuValues[6].i += sensor->getMagX_uT();
  imuValues[7].i += sensor->getMagY_uT();
  imuValues[8].i += sensor->getMagZ_uT();
  imureadings++;
}

void loop() {
  // put your main code here, to run repeatedly:

  //delay(1000);
  //printf("e1 delta: %06d value: %06d\n", encoderMotor[0].getDelta(), encoderMotor[0].getValue());


  // read imu for mean porpuses
  readImu(&mpu9250);
  printf("ax-ay-az %04.1f %04.1f %04.1f \n", imuValues[0].i, imuValues[1].i, imuValues[2].i);
  for (int i=0; i < 9; i++) {
    imuValues[i].i = 0;
  }
  delay(1);
  
}
