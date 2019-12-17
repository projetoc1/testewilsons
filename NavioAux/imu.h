#ifndef IMU_H
#define IMU_H
#include "Common/MPU9250.h"         // AccelGyroMag
#include "Navio2/LSM9DS1.h"         // AccelGyroMag
#include <unistd.h>                 //
#include "Common/Util.h"            //
#include <string>                   //
#include <memory>                   //
#include <stdio.h>                  // Nextion
#include <stdlib.h>                 // Nextion
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
#include "timer.h"

#include <fstream>                  // file manipulation

#include <pthread.h>                // multithread

//#include "NavioAux/timer.h"                  //temporizador

class imu{
private:
///
//	float q0=1;
//	float q1=0;
//	float q2=0;
//	float q3=0;
//	
//	float gyroOffset[3];//[2];
//	float twoKi=0;
//	float twoKp=2;
//	float integralGBx,integralGBy,integralGBz;
///

public:
    timer tm_1;
    //timer tm_2;
    MPU9250 imu1;
    LSM9DS1 imu2;
	unsigned long int loop_period=10000; // period in microseconds 
	unsigned long int loop_tolerance=90; // tolerance in microseconds
	float loop_delta=0;         // delta in seconds
    bool initialize();
    bool probe();
    void update();
    float read_temperature() {return temperature;};
    void read_accelerometer(float *ax, float *ay, float *az) {*ax = _ax; *ay = _ay; *az = _az;};
    void read_gyroscope(float *gx, float *gy, float *gz) {*gx = _gx; *gy = _gy; *gz = _gz;};
    void read_magnetometer(float *mx, float *my, float *mz) {*mx = _mx; *my = _my; *mz = _mz;};
    pthread_t imu_thread;
///
//	void updateIMU(float dt);
//	void setGyroOffset();
//	void getEuler( float* roll, float* pitch, float* yaw);
//	void invSqrt(float x);
//	float getW();
//	float getX();
//	float getY();
//	float getZ();
///
protected:
    float temperature_m;
    float _axm, _aym, _azm;
    float _gxm, _gym, _gzm;
    float _mxm, _mym, _mzm;

    float temperature_l;
    float _axl, _ayl, _azl;
    float _gxl, _gyl, _gzl;
    float _mxl, _myl, _mzl;

    float temperature_mh=0.5;
    float _axmh=0.5;
    float _aymh=0.5;
    float _azmh=0.5;
    
    float _gxmh=0.5;
    float _gymh=0.5;
    float _gzmh=0.5;
    
    float _mxmh=0.5;
    float _mymh=0.5;
    float _mzmh=0.5;

    float temperature_lh=0.5;
    float _axlh=0.5;
    float _aylh=0.5;
    float _azlh=0.5;
    
    float _gxlh=0.5;
    float _gylh=0.5;
    float _gzlh=0.5;
    
    float _mxlh=0.5;
    float _mylh=0.5;
    float _mzlh=0.5;

    float temperature;
    float _ax, _ay, _az;
    float _gx, _gy, _gz;
    float _mx, _my, _mz;

};

void init_imu(imu* imu_ptr);
void* imu_loop(void * ptr);
#endif
