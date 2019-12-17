/*
Provided to you by Emlid Ltd (c) 2014.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Get pressure from MS5611 barometer onboard of Navio shield for Raspberry Pi
using a different thread, to update pressure info in the background

To run this example navigate to the directory containing it and run following commands:
make
sudo ./threaded_baro
*/

#include <Common/Util.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
#include "Nextion.h"
#include "tela_var.hpp"
tela_var::tela_var(void)
{ }
tela_var::~tela_var(void)
{ }
void init_tela(tela_var* tela_ptr)
{
	//(*tela_ptr).setp_lat = NexVariable(0,37,"setp_lat");
	//(*tela_ptr).setp_lon = NexVariable(0,38,"setp_lon");

	//(*tela_ptr).lat_p = NexVariable(0,52,"lat_p");
	//(*tela_ptr).lat_i = NexVariable(0,53,"lat_i");
	//(*tela_ptr).lat_d = NexVariable(0,54,"lat_d");

	//(*tela_ptr).lon_p = NexVariable(0,55,"lon_p");
	//(*tela_ptr).lon_i = NexVariable(0,56,"lon_i");
	//(*tela_ptr).lon_d = NexVariable(0,57,"lon_d");

	//(*tela_ptr).error_lat = NexVariable(0,58,"error_lat");
	//(*tela_ptr).error_lon = NexVariable(0,59,"error_lon");

	//(*tela_ptr).flag = NexVariable(0,60,"flag");
	//(*tela_ptr).flag_piloto = NexVariable(0,63,"flag_piloto");

	//(*tela_ptr).lat_navio2 = NexVariable(0,67,"lat_navio2");
	//(*tela_ptr).lon_navio2 = NexVariable(0,68,"lon_navio2");

	//(*tela_ptr).adc1 = NexVariable(0,69,"adc1");
	//(*tela_ptr).adc2 = NexVariable(0,70,"adc2");

	(*tela_ptr).setpoint_lat=0;
	(*tela_ptr).setpoint_lon=0;
	(*tela_ptr).latp=0;
	(*tela_ptr).lati=0;
	(*tela_ptr).latd=0;
	(*tela_ptr).lonp=0;
	(*tela_ptr).loni=0;
	(*tela_ptr).lond=0;
	(*tela_ptr).errorlat=0;
	(*tela_ptr).errorlon=0;
	(*tela_ptr).flag0=0;
	(*tela_ptr).flagpiloto=0;
	(*tela_ptr).latnavio2=0;
	(*tela_ptr).lonnavio2=0;
	(*tela_ptr).leituraadc1=0;
	(*tela_ptr).leituraadc2=0;
	if(!nexInit()){printf("Tela falhou\n");}
	if( pthread_create(&(tela_ptr->tela_thread), NULL, tela_loop, (void *)tela_ptr) )
    {
        printf("Error: Failed to create barometer thread\n");
    }

}
void* tela_loop (void * tela_ptr)
{
	tela_var* temp_ptr =(tela_var*) tela_ptr;

    int cont;
    while(1){
		for(cont=0;cont<15;cont++){
			switch(cont){
				//recebimento de variaveis da navio2
				case 0:
					temp_ptr->lat_navio2.setValue(temp_ptr->latnavio2);
				break;
				case 1:
					temp_ptr->lon_navio2.setValue(temp_ptr->lonnavio2);
				break;
				case 2:
					temp_ptr->adc1.setValue(temp_ptr->leituraadc1);
				break;
				case 3:
					temp_ptr->adc2.setValue(temp_ptr->leituraadc2);
				break;
				case 4:
					temp_ptr->error_lat.setValue(temp_ptr->errorlat);
				break;
				case 5:
					temp_ptr->error_lon.setValue(temp_ptr->errorlon);
				break;
//				//Leitura da tela
				case 6:
					temp_ptr->setp_lat.getValue(&(temp_ptr->setpoint_lat));
				break;
				case 7:
					temp_ptr->setp_lon.getValue(&(temp_ptr->setpoint_lon));
				break;
				case 8:
					temp_ptr->lat_p.getValue(&(temp_ptr->latp));
				break;
				case 9:
					temp_ptr->lat_i.getValue(&(temp_ptr->lati));
				break;
				case 10:
					temp_ptr->lat_d.getValue(&(temp_ptr->latd));
				break;
				case 11:
					temp_ptr->lon_p.getValue(&(temp_ptr->lonp));
				break;
				case 12:
					temp_ptr->lon_i.getValue(&(temp_ptr->loni));
				break;
				case 13:
					temp_ptr->lon_d.getValue(&(temp_ptr->lond));
				break;
				case 14:
					temp_ptr->flag_piloto.getValue(&(temp_ptr->flagpiloto));
				break;
			}
		} 
	}
}
