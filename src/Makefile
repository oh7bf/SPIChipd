# -*- mode: makefile -*-
# 
# Makefile for compiling 'spichipd' on Raspberry Pi. 
#
# Sat  3 Nov 20:27:45 CDT 2018
# Edit: 
# Jaakko Koivuniemi

CXX           = g++
CXXFLAGS      = -g -O2 -Wall -Wextra -std=c++11 
LD            = g++
LDFLAGS       = -O2

MODULES       = SPIChip.o 
MODULES      += Max31865.o
MODULES      += spichipd.o 



%.o : %.cpp
	$(CXX) $(CXXFLAGS) -c $<

all: spichipd

spichipd: $(MODULES) 
	$(LD) $(LDFLAGS) $^ -o $@

clean:
	rm -f *.o

