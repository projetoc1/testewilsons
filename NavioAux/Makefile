CXX ?= g++
NAVIO = ../../Navio2/C++/Navio
PIGPIO_PATH := $(PIGPIO_PATH)

LIB = -L$(PIGPIO_PATH)

PYTHON27_LDFLAGS= python2.7-config --ldflags
PYTHON27_INCLUDES=`python2.7-config --includes

SEGNIX= ../Segnix

INCLUDES := -I ../Navio2/C++/Navio -I$(PIGPIO_PATH)
INCLUDES += -I ../Segnix/include

CFLAGS = -std=c++11 -Wno-psabi -c $(INCLUDES) . -L$(NAVIO)  -lrt -lnavio -liteadc -liteadcpp -lpthread -liteadmodule

SRC=$(wildcard *.cpp)
OBJECTS = $(SRC:.cpp=.o) 

#INCLUDES := -I ../../Navio2-master/C++/Navio 
#INCLUDES += ../../Segnix-master/include
#INCLUDES += -I$(PIGPIO_PATH)
all:
#	$(MAKE) -C ../Navio2-master/C++/Navio all
#	$(MAKE) -C ./Segnix-master Raspberry_Pi_Rv2
#	$(MAKE) -C ../Segnix-master install
#	$(CXX) -std=c++11 $(includes) -L$(NAVIO)  -L$(SEGNIX) Pendulum.cpp -lnavio -o Pendulum
#	$(CXX) -std=c++11 $(INCLUDES) $(LIB) Pendulum.cpp -L$(NAVIO)  -lrt -lnavio -lpthread -o Pendulum -lpigpio || $(MAKE) pigpio

#	$(CXX) $(CFLAGS) $(INCLUDES) $(LIB) timer.cpp -L$(NAVIO)  -lrt -lnavio -liteadc -liteadcpp -lpthread -liteadmodule -o timer.o -lpigpio $(PYTHON27_LDFLAGS) $(PYTHON27_INCLUDES)|| $(MAKE) pigpio
#	$(CXX) -std=c++11 $(INCLUDES) $(LIB) tela_var.cpp -L$(NAVIO)  -lrt -lnavio -liteadc -liteadcpp -lpthread -liteadmodule -o tela_var.o -lpigpio $(PYTHON27_LDFLAGS) $(PYTHON27_INCLUDES)|| $(MAKE) pigpio
#	$(CXX) -std=c++11 $(INCLUDES) $(LIB) imu.cpp -L$(NAVIO)  -lrt -lnavio -liteadc -liteadcpp -lpthread -liteadmodule -o imu.o -lpigpio $(PYTHON27_LDFLAGS) $(PYTHON27_INCLUDES)|| $(MAKE) pigpio

#clean:
#	rm -f Pendulum


#CXX = g++
#CFLAGS = -std=c++11 -Wno-psabi -c -I .

#SRC=$(wildcard *.cpp)
#OBJECTS = $(SRC:.cpp=.o) 

%.o: %.cpp
	$(CXX) $(CFLAGS) -o $@ $< 

all: $(OBJECTS)
	ar rcs libnavioaux.a $(OBJECTS)

clean:
	rm -f */*.o *.a
