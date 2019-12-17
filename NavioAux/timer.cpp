#include <stdio.h>
#include <memory>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
#include "timer.h"
float timer::get_period ( void)
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
	return (((1000000 * tv.tv_sec + tv.tv_usec) - timer::previoustime)/1.0e6);
}

float timer::forced_loop_interval( unsigned long interval_us )
{
    struct timeval tv;
    unsigned long int dt_;
	float dt;

	gettimeofday(&tv,NULL);
    timer::previoustime=timer::currenttime;
    timer::currenttime= 1000000 * tv.tv_sec + tv.tv_usec;
	dt_= (timer::currenttime - timer::previoustime);

    if(dt_< interval_us)
	{usleep(interval_us-dt_);}

    gettimeofday(&tv,NULL);
    timer::currenttime= 1000000 * tv.tv_sec + tv.tv_usec;
    dt= (timer::currenttime - timer::previoustime)/1.0e6 ;
	return(dt);
}

unsigned long timer::get_reference( void )
{
    struct timeval tv;
	float dt;

	gettimeofday(&tv,NULL);
    timer::previoustime= 1000000 * tv.tv_sec + tv.tv_usec;
    timer::currenttime= timer::previoustime;
    return(timer::previoustime);
}
void timer::set_reference( unsigned long value )
{
    timer::previoustime= value;
}
