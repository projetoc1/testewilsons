#ifndef TIMER_H
#define TIMER_H

#include <sys/time.h>
class timer{
private:
	float dt;
	unsigned long previoustime=0, currenttime;
public:
	float get_period (void);
	float forced_loop_interval( unsigned long interval_us);
	unsigned long get_reference( void );
	void set_reference( unsigned long value );
};
#endif
