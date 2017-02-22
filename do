#!/bin/bash

/Users/evil/raspberrypi/bin/arm-willtm-linux-gnueabi-g++ -Wall -std=gnu++0x -O3 -ggdb -pthread -lpthread -shared-libgcc -fno-use-linker-plugin main.cpp process_sock_cmd.cpp support.cpp cfg_files.cpp node_values.cpp uart.cpp -o server

#/Users/evil/raspberrypi/bin/arm-willtm-linux-gnueabi-g++ -Wall -std=gnu++0x -O3 -ggdb -pthread -lpthread -shared-libgcc -fno-use-linker-plugin -march=armv6zk -mcpu=arm1176jzf-s -mfloat-abi=hard -mfpu=vfp main.cpp process_sock_cmd.cpp support.cpp cfg_files.cpp uart.cpp node_values.cpp -o server

#/Users/evil/raspberrypi/bin/arm-willtm-linux-gnueabi-c++ -Wall -std=gnu++0x -ggdb -O3 -lpthread -shared-libgcc -I/Users/evil/prg/nrf24l01/ -fno-use-linker-plugin ./main.cpp ./cfg_files.cpp ./node_values.cpp ./support.cpp ./uart.cpp -o server
#/Users/evil/raspberrypi/bin/arm-willtm-linux-gnueabi-c++ -Wall -s -std=gnu++0x -g0 -O3 -lpthread -shared-libgcc -I/Users/evil/prg/nrf24l01/ -fno-use-linker-plugin ./main.cpp ./cfg_files.cpp ./node_values.cpp ./support.cpp ./uart.cpp -o server
#/Users/evil/raspberrypi/bin/arm-willtm-linux-gnueabi-c++ -Wall  -std=gnu++0x -ggdb -g3 -O3 -lpthread -shared-libgcc -I/Users/evil/prg/nrf24l01/ -fno-use-linker-plugin ./main.cpp -o server

scp -P 28802 ./server evil@sailtrack.eu:/tmp/server


