#!/bin/bash

/Users/evil/raspberrypi/bin/arm-willtm-linux-gnueabi-c++ -Wall -s -std=gnu++0x -g0 -O3 -lpthread -shared-libgcc -I/Users/evil/prg/nrf24l01/ -fno-use-linker-plugin ./main.cpp -o server
#/Users/evil/raspberrypi/bin/arm-willtm-linux-gnueabi-c++ -Wall  -std=gnu++0x -ggdb -g3 -O3 -lpthread -shared-libgcc -I/Users/evil/prg/nrf24l01/ -fno-use-linker-plugin ./main.cpp -o server
scp ./server evil@10.0.0.1:/home_rw/server


