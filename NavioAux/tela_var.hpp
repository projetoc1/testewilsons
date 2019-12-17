#ifndef TELA_VAR_HPP
#define TELA_VAR_HPP
#include <Common/MS5611.h>
#include <Common/Util.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
#include "Nextion.h"
//#include "tela_var.hpp"
class tela_var{
public:
	NexVariable setp_lat=NexVariable(0,37,"setp_lat");
	NexVariable setp_lon=NexVariable(0,38,"setp_lon");

	NexVariable lat_p=NexVariable(0,52,"lat_p");
	NexVariable lat_i=NexVariable(0,53,"lat_i");
	NexVariable lat_d=NexVariable(0,54,"lat_d");
	NexVariable lon_p=NexVariable(0,55,"lon_p");
	NexVariable lon_i=NexVariable(0,56,"lon_i");
	NexVariable lon_d=NexVariable(0,57,"lon_d");

	NexVariable error_lat=NexVariable(0,58,"error_lat");
	NexVariable error_lon=NexVariable(0,59,"error_lon");

	NexVariable flag=NexVariable(0,60,"flag");
	NexVariable flag_piloto=NexVariable(0,63,"flag_piloto");

	NexVariable lat_navio2=NexVariable(0,67,"lat_navio2");
	NexVariable lon_navio2=NexVariable(0,68,"lon_navio2");

	NexVariable adc1=NexVariable(0,69,"adc1");
	NexVariable adc2=NexVariable(0,70,"adc2");

	uint32_t setpoint_lat=0, setpoint_lon=0;
	uint32_t latp=0,lati=0,latd=0;
	uint32_t lonp=0,loni=0,lond=0;
	uint32_t errorlat=3,errorlon=4;
	uint32_t flag0=0,flagpiloto=0;
	uint32_t latnavio2=1,lonnavio2=2;
	uint32_t leituraadc1=0,leituraadc2=0;
	tela_var(void);
	~tela_var(void);
	pthread_t tela_thread;
    //void init_tela(tela_var* tela_ptr);
};
void init_tela(tela_var* tela_ptr);
void* tela_loop (void * tela_ptr);

//void * acquireBarometerData(void * barom);
#endif
