
CXX ?= g++
NAVIO = ./Navio2/C++/Navio
NAVIOAUX = ./NavioAux
PIGPIO_PATH := $(PIGPIO_PATH)

#LIB = -L$(PIGPIO_PATH)

SEGNIX= ./Segnix
#INCLUDES = -I /Navio2/C++/Navio 
INCLUDES := -I ./Navio2/C++/Navio 
INCLUDES += -I ./NavioAux
#INCLUDES += -I$(PIGPIO_PATH)
INCLUDES += -I ./Segnix/libraries/itead_Nextion
INCLUDES += -I/usr/include/python2.7 -I/usr/include/arm-linux-gnueabihf/python2.7
LINKERDATA= -L/usr/lib/python2.7/config-arm-linux-gnueabihf -L/usr/lib -lpython2.7 -ldl  -lutil -lm  -Xlinker -export-dynamic -Wl,-O1 -Wl,-Bsymbolic-functions
LIB_NEXTION = -liteadc -liteadcpp -liteadmodule
PIGPIO_DO =-lpigpio 
# || $(MAKE) pigpio
all:
	$(MAKE) -C ./Navio2/C++/Navio all
	$(CXX) -std=c++11 $(INCLUDES) $(LIB) main.cpp $(NAVIOAUX)/timer.cpp $(NAVIOAUX)/tela_var.cpp $(NAVIOAUX)/imu.cpp -L$(NAVIO) -lnavio -L$(NAVIOAUX) -lnavioaux $(LIB_NEXTION) -lrt -lpthread -o Pendulum $(LINKERDATA) $(PIGPIO_DO)

clean:
	rm -f Pendulum
