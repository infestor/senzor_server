//#define _GNU_SOURCE 1

#include <iostream>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netdb.h>
#include <signal.h>
#include <sys/poll.h>
#include <limits.h>

#include "main.h"
//#include "packet_defs.h"

//file and socket descriptors
int uart_fd;
FILE *out_fd; //just for saving temperatures (now unused)
int socketfd;
int signalInt;
int incoming_conns;
struct pollfd ufds[MAX_CONNS];

//#define DEBUG

//----------------- CONSTANTS -----------------------------------------
const char *druhy_senzoru_str[] = {
    "Vystup ZAP/VYP", "Teplota", "Dverni spinac ON/OFF", "Teplota procesoru",
    "Teplota DS1820", "Napeti baterie 1 clanek LION", "Napeti baterie 2 clanky LION",
	"PWM 1 Channel", "PWM 3 channels"};
const char low_power_str[] = "low power";

//const uchar sensorReturnLen[] = {1, 2, 1, 1, 2, 2, 2};
//const writableSensorTypes !!! originally writableSensorTypes was declared here, and used extern elsewhere
//but since it was not used anywhere in main.cpp, compiler optimalized it completely away which lead
//to undefined reference linker error (because other files still needed it)

const char str_port_name[] = "/dev/tty.wchusbserialfa130";
const char str_port_name_raspi[] = "/dev/ttyUSB0";
const char str_port_name_raspi1[] = "/dev/ttyUSB1";
//const char port_name[] = "/dev/tty.wchusbserialfd120";
//const char port_name[] = "/dev/ttyAMA0";


//----------------- VARIABLES -----------------------------------------
volatile NODE_VALUES_T * volatile nodeValues[MAX_NODES];

volatile uchar *nodeList;

extern const double TempCorrections[] = {99, 99, 0, 0};  //4.5

volatile uchar numNodes = 0;

//array of pointers to functions
//these functions process raw sensor data and stores them to nodeValues
//this is needed to handle different data types in every sensor
//parameters of all functions (NODE_VALUES_T *nodeP, uchar sensorNum, uchar *rawData, int rawLen)
//INFO - positions in array coresponds to sensor type defines
void (*countAndStoreSensorValue[]) STORE_VALUE_PARAMS = \
{decideOnOffValue, NULL, decideOnOffValue, countInternalProcTemp, countDS1820Temp, countBatteryVoltOneCell, countBatteryVoltTwoCell, storePwmValsOneToFourBytes, storePwmValsOneToFourBytes};

//array of pointers to functions
//these functions get sensor value from nodeValues and convert it to string in brackets, example [value_in_string_form] [1] [24.4]
//according to sensor type
//string is allocated inside function, so there is need to free it after use !!!
//INFO - positions in array coresponds to sensor type defines
void (*getSensorValStr[]) GET_VALUE_PARAMS = \
{getIntValStr, NULL, getIntValStr, getFloatValStr, getFloatValStr, getFloatValStr, getFloatValStr, getPwmValStr, getPwmValStr};

//array for storing information, how often read which sensor value over UART
//both these vectors must always be the same size!!
//and indexes from one must point to same sensor's data in the other vector
volatile SENSOR_INTERVAL_REC * volatile sensorIntervals[MAX_NODES];
std::vector<SENSOR_INTERVAL_VECT_REC> intervalVect;

std::queue<THREAD_QUEUE_REC> threadQueue;
std::mutex mutexQueue;
std::mutex mutexValues;
std::mutex mutexIntervals;

volatile bool threadRevealFinished = true; //should be true most of the time, false only during reveal
volatile bool threadProcessingPriotityCommand = false; //this is used, when we need urgently send something over uart
volatile int  threadProcessingPriotityCommand_result;
volatile uchar urgentReadResponsePacket[11];
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------


//=============================================================================
//====== Decreasing THREAD (once per second decreases all intervals in queue) =====
//=============================================================================
void threadDecreaseIntervals(void)
{
    printf("## QUEUE thread started..\n");
    while(1) {

        //cycle through intervalVect and decrease values by 1
        //if some value is 0, then it means to read real value over UART for that sensor
        //then reload its value to original value from sensorIntervals
        std::vector<SENSOR_INTERVAL_VECT_REC>::iterator it_vect;
        mutexIntervals.lock();
        for (it_vect = intervalVect.begin(); it_vect < intervalVect.end(); it_vect++)
        {
            sensorIntervals[it_vect->nodeNum][it_vect->sensorNum].countDown -= 1; //decrease value
            if (sensorIntervals[it_vect->nodeNum][it_vect->sensorNum].countDown == 0)
            {
                sensorIntervals[it_vect->nodeNum][it_vect->sensorNum].countDown = sensorIntervals[it_vect->nodeNum][it_vect->sensorNum].interval; //reload

                THREAD_QUEUE_REC rec;
                rec.cmd = CMD_READ;
                rec.nodeNum = it_vect->nodeNum;
                rec.sensorNum = it_vect->sensorNum;
                rec.intervalOk = sensorIntervals[it_vect->nodeNum][it_vect->sensorNum].interval;

                mutexQueue.lock();
                threadQueue.push(rec);
                mutexQueue.unlock();
            }
        }
        mutexIntervals.unlock();

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}


//=============================================================================
//====== UART thread (reads and saves data in backgroud) =====================
//=============================================================================
void duplicateRecAgainToQueueIfPossible(volatile THREAD_QUEUE_REC *rec)
{
	if (rec->intervalErr > 0) //we have some more tryouts available
	{
	  THREAD_QUEUE_REC newRec;
	  newRec.nodeNum = rec->nodeNum;
	  newRec.sensorNum = rec->sensorNum;
	  newRec.sensorVal = rec->sensorVal;
	  newRec.intervalErr = rec->intervalErr - 1;

	  //add again to self-queue with decreased max-try-counter
	  mutexQueue.lock();
	  threadQueue.push(newRec);
	  mutexQueue.unlock();
	}
}

void threadProcessQueue(void)
{
    printf("## UART thread started..\n");

    bool work;
    volatile THREAD_QUEUE_REC rec;

    while (1)
    {
        work = false;
        mutexQueue.lock();
        if (!threadQueue.empty())
        {
            THREAD_QUEUE_REC recTemporary = threadQueue.front();
            memcpy((void*)&rec, (const void*)&recTemporary, sizeof(THREAD_QUEUE_REC) );
            //rec = threadQueue.front();
            work = true;
        }
        mutexQueue.unlock();

        if (work == true)
        {
            //handle command type
            if (rec.cmd == CMD_READ)
            {
                //we must check if the value from UART sensor was read with success, else
                //if it wasnt and its interval is bigger than 35s, we choose
                //random interval 4-31sec to try it again
                if ( (performUartValueReadAndSave(rec.nodeNum, rec.sensorNum) != 1) ) //&& (rec.intervalOk > 60)
                {
                    //decide if normal node or low power node
                    //for normal node choose random new interval
                    //for low power node just decrease its alive interval by value in REC.INTERVAL (!! should be always 1sec for this type of node)
                    if (nodeValues[rec.nodeNum]->is_low_power == 0) //NORMAL powered
                    {
                        mutexIntervals.lock();
                        if (sensorIntervals[rec.nodeNum] != NULL) //only for sensors with valid intervals record
                        {
                            if (sensorIntervals[rec.nodeNum][rec.sensorNum].interval > 35)
                            {
                                sensorIntervals[rec.nodeNum][rec.sensorNum].countDown = randr(4, 31);
                                printf("Retry in %ds\n", sensorIntervals[rec.nodeNum][rec.sensorNum].countDown );
                            }
                        }
                        mutexIntervals.unlock();
                    }
                    else //LOW powered
                    {
                        //low power device AND sensor - decrease alive interval
                        if ( nodeValues[rec.nodeNum]->sensor_types[rec.sensorNum] >= LOW_POWER_NODE_SIGN)
                        {
							int temp_alive = nodeValues[rec.nodeNum]->low_power_alive;
							int temp_interval = sensorIntervals[rec.nodeNum][rec.sensorNum].interval;
							//to decrease proper time in case that previously it wasnt only 1 sec
							int temp_interval_before = temp_interval;

							//decrease the auto-read interval from 60 to 1 sec after unsuccesful try
							//we have to cover situation when it is only one-time triggered READ
							//and sensor is not in auto-reread queue. but this should not happen
							if( temp_interval > 0) temp_interval = 1;
							mutexIntervals.lock();
							sensorIntervals[rec.nodeNum][rec.sensorNum].interval = temp_interval;
							sensorIntervals[rec.nodeNum][rec.sensorNum].countDown = temp_interval;
							mutexIntervals.unlock();

							if (temp_interval_before > temp_alive)
							{
								temp_alive = 0;
							}
							else
							{
								temp_alive -= temp_interval_before;
							}
							nodeValues[rec.nodeNum]->low_power_alive = temp_alive;
                        }
                    }
                }
                else //read was successful
                {
					//FOR LOW POWER DEVICES:
                    //after success read - refresh the low_power alive interval
					//and change interval to 60 secs, because now the device will sleep for approx 64 secs
					//so it is not necessary to poll it every second for this period..
					//but after it yes - change interval back to 1 sec and poll it until it responds
                    if (nodeValues[rec.nodeNum]->is_low_power == 1)
					{
						nodeValues[rec.nodeNum]->low_power_alive = LOW_POWER_ALIVE_TIMEOUT;
						mutexIntervals.lock();
						sensorIntervals[rec.nodeNum][rec.sensorNum].interval = 60;
						sensorIntervals[rec.nodeNum][rec.sensorNum].countDown = 60;
						mutexIntervals.unlock();
						//TODO: save even the last-valid-value timestamp to node values
					}
                }
            }
            else if (rec.cmd == CMD_WRITE)
            {
                //we must check if the value to UART sensor was written with success
                //else we use rec.intervalErr as a MAX REPEAT number (must be set on command create)
                if ( writeUartSensorData(rec.nodeNum, rec.sensorNum, rec.sensorVal) != 1 ) //error writing, lets try it again
                {
					duplicateRecAgainToQueueIfPossible(&rec);
                }
                else //success writing
                {
                    //we have to save/mirror new value to internal array
                    mutexValues.lock();
                    nodeValues[rec.nodeNum]->sensors[rec.sensorNum]->int_val = rec.sensorVal;
                    mutexValues.unlock();
                }
            }
            else if (rec.cmd == CMD_WRITE_CALIB)
            {
                //we must check if the value to UART sensor was written with success
                //else we use rec.intervalErr as a MAX REPEAT number (must be set on command create)
                if ( writeUartSensorCalib(rec.nodeNum, rec.sensorNum, rec.sensorVal) != 1 ) //error writing, lets try it again
                {
					duplicateRecAgainToQueueIfPossible(&rec);
                }
                //no need to do anything on success writing
            }
             else if (rec.cmd == CMD_URGENT_READ)
            {
                //uchar sensor_read_packet[11] = {1, 99, 3, 255, 0, 1, 99, 0, 0, 0, 0};

                mirfPacket sensor_read_packet;

                sensor_read_packet.txAddr = 1;
                sensor_read_packet.rxAddr = rec.nodeNum;
                sensor_read_packet.type = (PACKET_TYPE) REQUEST;

                sensor_read_packet.payload.request_struct.cmd = (CMD_TYPE) rec.sensorVal; //command number (its value) - is stored to SensorVal field of thread record (which is not-standart)
                sensor_read_packet.payload.request_struct.for_sensor = rec.sensorNum;
                sensor_read_packet.payload.request_struct.len = 1;

                if (sendAndGetResponse((uchar *)&sensor_read_packet, (uchar*)urgentReadResponsePacket) == 1)
                {
                    //on success reading back
                    threadProcessingPriotityCommand = false;
                    threadProcessingPriotityCommand_result = 1;
                    //result is in urgentReadResponsePacket  (whole received packet)
                }
                else
                {
                  if (rec.intervalErr > 0) //we have some more tryouts available
                  {
					duplicateRecAgainToQueueIfPossible(&rec);
                  }
                  else
                  { //no more tryouts left.. end the command with fail
                    threadProcessingPriotityCommand = false;
                    threadProcessingPriotityCommand_result = -1;
                  }
                }
            }
            else if (rec.cmd == CMD_REVEAL)
            {
                revealNodes();
                threadRevealFinished = true;
            }

            //after processing record from queue whe have to throw it away!!!
            mutexQueue.lock();
            threadQueue.pop();
            mutexQueue.unlock();
        }
        else //queue empty
        {
            usleep(50000);
        }
    }
}

//==============================================================================================
int main(int argc, char ** argv)
{
    //all nodes are undefined by default (null pointers to their data store structures)
    for (int xx=0; xx < MAX_NODES; xx++) nodeValues[xx] = NULL;

    int i = 0;

    setvbuf (stdout, NULL, _IONBF, 0); //no caching of output to console
    printf("NRF24L01 SENSOR NET SERVER!\n");

    //try to open UART
    printf("Trying UART port on MAC (%s)\n", str_port_name);
    if (setup_uart(str_port_name) == -1)
    {
        printf("Trying UART port on RASPBERRY (%s)\n", str_port_name_raspi);
        if (setup_uart(str_port_name_raspi) == -1)
        {
			printf("Trying UART port on RASPBERRY (%s)\n", str_port_name_raspi1);
			if (setup_uart(str_port_name_raspi1) == -1)
			{
				printf("Cannot open neither MAC nor Raspi UART port!\n");
				return -1;
			}
        }
    }
    printf("UART Port opened..\n");

    //observe CTRL-C
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_ctrl_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    //create socket
    vytvoritSocket();
    //printf("\n");

    printf("## Reading config files..\n");
    readConfigFiles();

    //---------------------------------------
    printf("waiting 5s\n");
    sleep(5);
    tcflush(uart_fd, TCIOFLUSH);
    printf("Ready..\n");
    //---------------------------------------

    //find available nodes in network
    revealNodes();
    printf("\nFound nodes %d\n", numNodes);
    for (i=0; i < MAX_NODES; i++)
    {
        if (nodeValues[i] != NULL)
        {
            volatile NODE_VALUES_T *node = nodeValues[i];
            printf("Node %d, numOfSensors: %d", node->node, node->num_sensors);
            for (uchar x=0; x < node->num_sensors; x++)
            {
                if (node->sensor_types[x] >= LOW_POWER_NODE_SIGN)
                {
                    printf(" | %s (%s)", druhy_senzoru_str[ node->sensor_types[x]-LOW_POWER_NODE_SIGN ], low_power_str);
                }
                else
                {
                    printf(" | %s", druhy_senzoru_str[ node->sensor_types[x] ]);
                }

            }
            printf("\n");
        }
    }
    printf("\n");

    //open file for storing measured values (deprecated)
    out_fd = fopen("./temperatures.txt", "a");
    if (out_fd == NULL)
    {
        printf("Cannot open data file!\n");
        cleanup();
        exit(10);
    }

    printf("Starting background THREAD for handling UART comm\n");
    std::thread threadUART(threadProcessQueue);
	//usleep(50000);
    printf("Starting background THREAD for decreasing countdowns in queue\n");
    std::thread threadCOUNTDOWN(threadDecreaseIntervals);

    printf("\n");

    while (1) //infinite program loop
    {

        //handling existing socket connections
        if (incoming_conns > 0)
        {
            int rv = poll(ufds, incoming_conns, 20);
            if (rv > 0)
            {
                uchar ind = 0;
                do
                {
                    if (ufds[ind].revents & POLLIN)
                    {
                        uchar inBuff[500];
                        //instead of memset all buffer to 0 before receive,
                        //just set to 0 only byte laying right after received payload (like ending string with 0)
                        //but do it after receive when len of incoming stored data is known
                        //AND also do it after checking that len was not 0 or -1 which means error
                        //this will save a lot of cpu time by not zeroing the buffer again and again
                        ssize_t len = recv(ufds[ind].fd, inBuff, sizeof(inBuff), 0);
                        //TODO : if incoming message is bigger than inBuff size, read it repeatedly with realloc to read all data

                        //if len was 0 or -1 that means either socket close or error
                        //in both cases we must close the socket to get rid of it
                        if (len < 1)
                        {
                            if (len == -1) printf("Socket %d error (errno: %d)\n", ufds[ind].fd, errno);
                            printf("Socket %d closed\n", ufds[ind].fd);
                            close(ufds[ind].fd);
                            removeUfd(ind); //remove this descriptor from queue
                            if (ind < incoming_conns) continue; //without increasing index
                        }
                        else //real data received and stored in buffer
                        {
                            inBuff[len] = 0;
                            //printf("received %zd Bytes\n", len);
                            printf(">%s", inBuff);
                            uchar *outBuff;
                            int outBufLen = 200;
                            int outBufPos = -1; //0;
                            outBuff = (uchar *)malloc(outBufLen);
                            processSockCmd(inBuff, len, &outBuff, &outBufPos, &outBufLen);
                            if (outBufPos >= 0) //only if there is something added in outBuffer (something was processed from incoming message)
                            {
                                //at the end append linefeed
                                appendToBuffer(&outBuff, &outBufLen, &outBufPos, (uchar*)"\n\r", 2);
                                send(ufds[ind].fd, outBuff, outBufPos + 1, 0);
                                // TODO: check whether all data was sent in one iteration, else repeat with offset outbuf+sentBytes
                            }
                            free(outBuff);
                        }
                    }
                    else if ((ufds[ind].revents & POLLHUP) || (ufds[ind].revents & POLLERR))
                    {
                        printf("HangUp\n");
                        removeUfd(ind); //remove this descriptor from queue
                        close(ufds[ind].fd);
                        if (ind < incoming_conns) continue; //without increasing index
                    }
                    ind++;
                } while ( (ind < incoming_conns) && (incoming_conns > 0) );
            }
        }

        //accepting new socket connections (if any)
        if (incoming_conns < MAX_CONNS)
        {
            bool socketAccepted = false;
			volatile int incoming_sd;

            while ( (incoming_sd = accept(socketfd, NULL, NULL)) > 0 )
            {
                socketAccepted = true;
                printf("Connection accepted. Using new socketfd : %d\n", incoming_sd);
                fcntl(incoming_sd, F_SETOWN, getpid());
                int flags = fcntl(incoming_sd, F_GETFL, 0);
                fcntl(incoming_sd, F_SETFL, flags | O_NONBLOCK); //| O_ASYNC);
                ufds[incoming_conns].fd = incoming_sd;
                ufds[incoming_conns].events = POLLIN | POLLHUP;
                send(incoming_sd, "[hello]\n", 8, 0);
                incoming_conns++;

            	if (incoming_conns == MAX_CONNS) break;
            }

            //if some socked accepted - immediately return to begin of loop without sleeping, and poll incoming data
            if (socketAccepted == true) continue;
        }

        usleep(10000);

        //end of infinite program loop
        //tcflush(uart_fd, TCIOFLUSH); //we can only flush in separate thread, because only it can access uart
    }

    //cleanup file descriptors etc.
    cleanup();
    return 0;
}
