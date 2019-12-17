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
#include "imu.h"

bool imu::initialize(){
	if(!imu1.probe()){
        printf("Sensor of IMU1 not enabled\n");
	}
	else{
		imu1.initialize();
	}
	if(!imu2.probe()){
        printf("Sensor of IMU2 not enabled\n");
	}
	else{
		imu2.initialize();
	}
	return 0;
}
bool imu::probe(){
	return(imu1.probe()&& imu2.probe());
}
void imu::update(){
	imu1.update();
	imu2.update();

	temperature_m=imu1.read_temperature();
	temperature_l=imu2.read_temperature();
	temperature=temperature_m*temperature_mh+temperature_l*temperature_lh;

    imu1.read_accelerometer(&_axm,&_aym,&_azm);
    imu2.read_accelerometer(&_axl,&_ayl,&_azl);
    _ax=_axm*_axmh+_axl*_axlh;
    _ay=_aym*_aymh+_ayl*_aylh;
    _az=_azm*_azmh+_azl*_azlh;

    imu1.read_gyroscope(&_gxm,&_gym,&_gzm);
    imu2.read_gyroscope(&_gxl,&_gyl,&_gzl);
    _gx=_gxm*_gxmh+_gxl*_gxlh;
    _gy=_gym*_gymh+_gyl*_gylh;
    _gz=_gzm*_gzmh+_gzl*_gzlh;

    imu1.read_magnetometer(&_mxm,&_mym,&_mzm);
    imu2.read_magnetometer(&_mxl,&_myl,&_mzl);
	_mx=_mxm*_mxmh+_mxl*_mxlh;
	_my=_mym*_mymh+_myl*_mylh;
	_mz=_mzm*_mzmh+_mzl*_mzlh;
}
void* imu_loop(void* ptr) {
	imu* imu_ptr=(imu*)ptr;
//	if( imu_ptr->loop_delta==0){
	imu_ptr->tm_1.get_reference();
	imu_ptr->update();
//	}
	while(true)	{
		
		if(imu_ptr->tm_1.get_period() >(1.0e-6*(imu_ptr->loop_period -imu_ptr->loop_tolerance )))
		{

			imu_ptr->loop_delta=imu_ptr->tm_1.forced_loop_interval(imu_ptr->loop_period -(imu_ptr->loop_tolerance>>1));
			imu_ptr->update();
			//usleep(imu_ptr->loop_period -imu_ptr->loop_tolerance);
		}
	}
	pthread_exit(NULL);
}

void init_imu(imu* imu_ptr) {
	imu_ptr->initialize();
	if(pthread_create(&(imu_ptr->imu_thread), NULL, imu_loop,(void *)imu_ptr) )
	{
		printf("Error: Failed to create IMU thread.\n");
	}
	
}
