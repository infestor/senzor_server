#include <iostream>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <limits.h>

#include "main.h"

extern int uart_fd;
extern volatile NODE_VALUES_T * volatile nodeValues[MAX_NODES];
extern volatile SENSOR_INTERVAL_REC * volatile sensorIntervals[MAX_NODES];
extern std::vector<SENSOR_INTERVAL_VECT_REC> intervalVect;
extern volatile uchar *nodeList;
extern std::mutex mutexValues;
extern std::mutex mutexIntervals;
extern volatile uchar numNodes;
extern const char *druhy_senzoru_str[];

//=====================================================================================
//==== UART related functions =========================================================
//=====================================================================================
int setup_uart(const char *port_name)
{
    uart_fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_fd == -1) {
	    char bu[100];
	    sprintf(bu, "open_port: Unable to open %s - ", port_name);
	    perror(bu);
	    return(-1);
    }
    
    struct termios options;
    tcgetattr(uart_fd, &options);
    //cfmakeraw(&options);
    
    options.c_iflag = IGNBRK | IGNPAR | IXANY;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cflag = CLOCAL | CREAD | CS8;
    
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    cfsetispeed(&options, B57600);
    cfsetospeed(&options, B57600);
    
    tcsetattr(uart_fd, TCSANOW, &options);
    
    // Turn off blocking for reads, use (uart_fd, F_SETFL, FNDELAY) if you want that
    //fcntl(uart_fd, F_SETFL, FNDELAY);
    
    return 1;
}

int transceiveData(uchar *paket)
{
    ssize_t n;
    int i, repeats_send;
    uchar buf[3];
    uchar c;
    bool respReceived;
    
    repeats_send = 1;
    //tcflush(uart_fd, TCIOFLUSH);
    do
    {
        n = write(uart_fd, (void*)paket, 12);
        if (n < 0) {
            return -1;
        }
        
        respReceived = false;
        do
        {
            i = 0;
            memset(&buf, 0, sizeof(buf) );
            //tcflush(uart_fd, TCIFLUSH);
            while(i < 2) {
                n = read(uart_fd, (void*)&c, 1);
                if (n>0)
                {
                    buf[i] = c;
                    i++;
                }
                else
                {
                    usleep(500);
                }
            }
            
            buf[2] = 0;
#ifdef DEBUG
            printf("Vysledek: %s\n", buf);
#endif
            if ( strncmp((char*)buf, "OK", 2) != 0)
            {
                //eliminujeme nacteni realneho paketu tim ze ho rozpozname
                if ( (strncmp((char*)buf, "ER", 2) != 0) && (strncmp((char*)buf, "TO", 2) != 0) )
                {
                    //neni to odpoved na send paket, takze jsou to prvni 2 bajty z realneho paketu
                    //ten nepotrebujem, protoze zrejme prisel nejak opozdene, tudiz nactem zbylych 9 bajtu
                    //a tim se v bufferu posuneme na zacatek dalsiho mozneho paketu/potvrzeni
                    uchar u = 0;
                    do
                    {
                        n = read(uart_fd, (void*)&c, 1);
                        if (n > 0) {
                            u++;
                        }
                        else
                        {
                            usleep(500);
                        }
                    } while (u < 9);
#ifdef DEBUG
                    printf("wrong ack\n");
#endif
                }
                else //TO or ER received
                {
                    break;
                }
            }
            else
            {
                respReceived = true;
            }
        } while(respReceived == false);
        
        if (respReceived == true) break;
        if (repeats_send > 0) return -1; //prilis mnoho pokusu o odeslani
        repeats_send++;
        usleep(50000);
    }
    while( 1 );
    
    return 1;
}

//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
int sendAndGetResponse(uchar *paket, uchar *response_buffer, unsigned int timeout, unsigned int repeats)
{
    unsigned int i;
    unsigned int repeats_get;
    unsigned int klok;
    unsigned int counted_timeout;
    ssize_t n;
    bool was_timeout;
    uchar buf[11];
    uchar c;
    
    counted_timeout = timeout;
    
    repeats_get = 0;
    klok = 0;
    was_timeout = false;
    do
    {
        int result = transceiveData(paket);
        if (result < 0)
        {
            repeats_get++;
        }
        else
        {
            do
            {
                i = 0;
                while(i < 11) {
                    n = read(uart_fd, (void*)&c, 1);
                    if (n>0)
                    {
                        buf[i] = c;
                        i++;
                    }
                    else
                    {
                        usleep(1000);
                        klok++;
                        if (klok > counted_timeout)
                        {
                            repeats_get++;
                            was_timeout = true;
                            break;
                        }
                    }
                }
                
                if (was_timeout == true)
                {
                    was_timeout = false;
                    break;
                }
                
                if (i == 11) //whole packet was read from uart
                {
                    //decide if the packet is really from our node
                    if (buf[0] == paket[2]) //yes, it is our response
                    {
#ifdef DEBUG
                        printf("raw N:%d %d %d\n", buf[0], buf[7], buf[8]);
#endif
                        memcpy(response_buffer, buf, 11);
                        return 1; //return from function with success
                    }
                    else
                    { //no, it is from other node
#ifdef DEBUG
                        printf("Wrong node packet\n");
#endif
                        continue; //read next packet from uart buffer
                    }
                }
            } while (1);
        }
        usleep(5000);
    } while( !((was_timeout==true) || (repeats_get > (repeats-1))) );
    return -1;
}

//----------------------------------------------------------------------------------------------------
// --- higher level UART functions called by program
//----------------------------------------------------------------------------------------------------
int getNodePresentation(uchar nodeNnum, NodeSpecsT *paket)
{
//note: packet sent to master over uart must have 0xFE byte at the beginning
//so useful data of packet starts at index 1 !!
    uchar pres_req_packet[12] = {254, 1, 3, 5, 0, 0, 0, 0, 0, 0, 0, 0};
    pres_req_packet[2] = nodeNnum;
    uchar respPacket[11];
    
    memset(respPacket, 0, 11);
    
    if (sendAndGetResponse(pres_req_packet, respPacket, 50, 2) >= 0 )
    {
        paket->node = respPacket[0];
        paket->num_sensors = respPacket[4];
        for (unsigned int i = 0; i < respPacket[4]; i++)
        {
            paket->sensors[i] = respPacket[5+i];
        }
        return 1; //success
    }
    
    return -1; //node not revealed, return fail
}

int getSensorRawData(uchar nodeNum, uchar sensorNum, uchar *data, int *dataLen)
{
//note: packet sent to master over uart must have 0xFE byte at the beginning
//so useful data of packet starts at index 1 !!
    uchar sensor_read_packet[12] = {254, 1, 99, 3, 255, 0, 0, 99, 0, 0, 0, 0};
    uchar respPacket[11];

    sensor_read_packet[2] = nodeNum;
    sensor_read_packet[7] = sensorNum;

    if (sendAndGetResponse(sensor_read_packet, respPacket) >= 0)
    {
        *dataLen = respPacket[5]; //length value
        memcpy(data, &respPacket[7], *dataLen);
        return 1; //return from function with success
    }
    
    return -1; //node probably didnt response, return with fail
}

void revealNodes(void)
{
    for (int nodeNum=2; nodeNum<MAX_NODES; nodeNum++) //nodes starting from 2, because 0 is forbidden and 1 is master node
    {
        printf("Trying get info about node %i\n", nodeNum);
        NodeSpecsT node_specs;
        if (getNodePresentation(nodeNum, &node_specs) > 0 )
        {
            mutexValues.lock();
            //prepare node values array for this node
            if (nodeValues[nodeNum] == NULL)
            {
                //freeNodeValStruct(&nodeValues[nodeNum]);
                nodeValues[nodeNum] = (NODE_VALUES_T*)malloc(sizeof(NODE_VALUES_T) );
                memset((void*)nodeValues[nodeNum], 0, sizeof(NODE_VALUES_T) );
                nodeValues[nodeNum]->node = nodeNum;
                nodeValues[nodeNum]->num_sensors = node_specs.num_sensors;
                nodeValues[nodeNum]->is_low_power = 0; //first set it to 0 = NO, but during cycling through sensors, determine its really value
                nodeValues[nodeNum]->low_power_alive = LOW_POWER_ALIVE_TIMEOUT;
                memcpy((void*)nodeValues[nodeNum]->sensor_types, node_specs.sensors, MAX_SENSORS);
                nodeValues[nodeNum]->sensor_names = (volatile uchar**) malloc( sizeof(uchar*) * MAX_SENSORS); //alloc array of pointers to user names of sensors
                //int L;
                for (int x = 0; x < node_specs.num_sensors; x++ ) {
                    //fill all names with default values
                    //the IF is there only to silence compiler warning of not using result value of asprintf function
                    if ( asprintf((char**)&(nodeValues[nodeNum]->sensor_names[x]), "node_%d_%i_%s", nodeNum, x, druhy_senzoru_str[nodeValues[nodeNum]->sensor_types[x]]) );
                    ; //and to silence mac compiler, semicolon must be on other line than IF statement
                }
                //alloc num_sensors of value unions
                nodeValues[nodeNum]->sensors = (volatile SENSOR_VAL_T**) malloc( sizeof(SENSOR_VAL_T*) * node_specs.num_sensors);  //alloc array of pointers to sensors values
                //alloc also intervals structure for this node and init it to 0
                mutexIntervals.lock();
                if (sensorIntervals[nodeNum] == NULL)
                {
                    sensorIntervals[nodeNum] = (SENSOR_INTERVAL_REC *) malloc(sizeof(SENSOR_INTERVAL_REC) * MAX_SENSORS);
                    memset((void*)sensorIntervals[nodeNum], 0, sizeof(SENSOR_INTERVAL_REC) * MAX_SENSORS);
                }               
                //alloc sensor val structures
                for (int x = 0; x < node_specs.num_sensors; x++ ) 
                {
                    nodeValues[nodeNum]->sensors[x] = (SENSOR_VAL_T*) malloc( sizeof(SENSOR_VAL_T) );
                    memset((void*)nodeValues[nodeNum]->sensors[x], 255, sizeof(SENSOR_VAL_T) );
                    
                    //determine if node is low powered - if any sensor has vylue bigger than 128
                    //that means low power node
                    if (nodeValues[nodeNum]->sensor_types[x] >= LOW_POWER_NODE_SIGN) nodeValues[nodeNum]->is_low_power = 1;
                    
                    //set interval for reading of values from sensor if it is DS1820 - set interval automatically to 60sec
                    if (nodeValues[nodeNum]->sensor_types[x] == TEPLOTA_DS1820)
                    {
                        sensorIntervals[nodeNum][x].countDown = 1;
                        sensorIntervals[nodeNum][x].interval = 60;
                        //prepare new record to be pushed to vector
                        SENSOR_INTERVAL_VECT_REC rec;
                        rec.nodeNum = nodeNum;
                        rec.sensorNum = x;
                        //add record to interval vector
                        intervalVect.push_back(rec);
                    }
                    //set interval for reading of values from sensor if it is low powered DS1820 - set interval automatically to 1sec
                    else if (nodeValues[nodeNum]->sensor_types[x] == (TEPLOTA_DS1820 + LOW_POWER_NODE_SIGN) )
                    {
                        sensorIntervals[nodeNum][x].countDown = 1;
                        sensorIntervals[nodeNum][x].interval = 1;
                        //prepare new record to be pushed to vector
                        SENSOR_INTERVAL_VECT_REC rec;
                        rec.nodeNum = nodeNum;
                        rec.sensorNum = x;
                        //add record to interval vector
                        intervalVect.push_back(rec);
                    }                    
                    else
                    {
                        //else push ONE TIME request for read of sensor value to thread queue
                        /*
                        THREAD_QUEUE_REC rec;
                        rec.cmd = CMD_READ;
                        rec.nodeNum = nodeNum;
                        rec.sensorNum = x;
                        rec.intervalOk = 1; //dont know what was this interval meant for, so I load it with 1
                        
                        mutexQueue.lock();
                        threadQueue.push(rec);
                        mutexQueue.unlock(); 
                        */
                    }                   
                }
                mutexIntervals.unlock();
                
            }
            mutexValues.unlock();
        }
    }
    //create simple list (just node nums) of all nodes
    //used also for recount of complete number of sensors
    uchar listik[MAX_NODES];
    mutexValues.lock();
    numNodes = 0;
    for (int xxx=0; xxx < MAX_NODES; xxx++)
    {
        if (nodeValues[xxx] != NULL)
        {
            listik[numNodes] = nodeValues[xxx]->node; //add node num to temporary list
            numNodes++;
        }
    }
    //copy only valid values from temporary list to normal list (clearing the original list before)
    if (nodeList != NULL) free((void*)nodeList);
    if (numNodes > 0) //only if there are nodes
    {
        nodeList = (uchar *) malloc(numNodes);
        memcpy((void*)nodeList, listik, numNodes);
    }
    mutexValues.unlock();
}

int writeUartSensorData(uchar nodeNum, uchar sensorNum, int sensorData)
{
//note: packet sent to master over uart must have 0xFE byte at the beginning
//so useful data of packet starts at index 1 !!
  uchar paket[12] = {254, 1, 99, 3, 255, 0, 0, 99, 0, 0, 0, 0};
    paket[2] = nodeNum;
    paket[5] = 1; //write cmd
    paket[6] = 1; //length of data written
    paket[7] = sensorNum;
    paket[8] = uchar(sensorData);
  
  int result = transceiveData(paket);
  return result;
}

int writeUartSensorCalib(uchar nodeNum, uchar sensorNum, int calibData)
{
//note: packet sent to master over uart must have 0xFE byte at the beginning
//so useful data of packet starts at index 1 !!
  uchar paket[12] = {254, 1, 99, 3, 255, 5, 1, 99, 0, 0, 0, 0};
    paket[2] = nodeNum;
    paket[5] = CALIBRATION_WRITE; //calibration write cmd
    paket[6] = 1; //length of data written
    paket[7] = sensorNum;
    paket[8] = uchar(calibData);
  
  int result = transceiveData(paket);
  printf("## setCalib - r:%i, newVal:%d\n", result, calibData);
  return result;
}

int performUartValueReadAndSave(uchar nodeNum, uchar sensorNum)
{
    uchar rawData[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    int rawLen = 4;
    int result = getSensorRawData(nodeNum, sensorNum, (uchar *)&rawData, &rawLen);
    //if the getting of raw data fails, 0xFFs will remain in the rawData variable and forces
    //the count&store func to store FFs as sign of error of reading
    
    //there could not be any check whether the node and sensor exists during saving of new value
    //because this function is only called from inside program loop, where these
    //things were already checked

    volatile NODE_VALUES_T *node = nodeValues[nodeNum];
    uchar sensType = node->sensor_types[sensorNum];
    if (sensType >= LOW_POWER_NODE_SIGN) sensType -= LOW_POWER_NODE_SIGN;
    
    mutexValues.lock();
    
    //test for FFFF value as a sign of error reading from node, so store error value to array
    if (result <= 0) //( (int32_t)*rawData == 0xFFFFFFFF)
    {
        //we must differ normal node and low power node
        //for low power device we leave last valid value until the ALIVE counter is bigger than 0
        //because we are reading the sensor each second so there would be error value displayed most of the time
        //when the sensor is sleeping        
        if ( (node->is_low_power == 0) )
        {
            //put ERROR value inside sensor's place
            node->sensors[sensorNum]->uint_val = UINT_MAX;
        }
        else if ((node->is_low_power == 1) && (node->low_power_alive == 0))
        {   //for low power sensor the same
            node->sensors[sensorNum]->uint_val = UINT_MAX;
            node->low_power_voltage = UINT_MAX;
        }
    }
    else
    {
        //choose proper sensor value handle function (according to sensor type) from array of pointers to functions
        //but we have to take care of low power sensors (because they have number over 128 and that would
        //exceed bounds of the array of value handling functions
        //so we have to normalise the sensor type to 0-127
        (*countAndStoreSensorValue[ sensType ])(node, sensorNum, (uchar*)&rawData, rawLen);

        //now little addition - storing Vcc which comes along with DS18B20 value if low power sensor
        //TODO: move this somewhere else (probably right to function that counts DS18b20 temp
        if (node->is_low_power == 1) {
        	if (sensType == TEPLOTA_DS1820) {
        		countLowPowerVcc(node, sensorNum, (uchar*)&rawData, rawLen);
        	}
        }
    }
    mutexValues.unlock();
    
    if ( result > 0)
    {
        int valStrLen;
        uchar *valStr;
        char low_power_str[40] = "";

        (*getSensorValStr[ sensType ])(node, sensorNum, &valStr, &valStrLen);
        if (node->is_low_power == 1) sprintf(low_power_str, "(low pwr, Vcc = %.3f V)\n", (double)node->low_power_voltage / 1000.0);
        printf("## Node %d, sensor %d, value: %s\n%s", nodeNum,  sensorNum, valStr, low_power_str );
        free(valStr);
        return 1;
    }
    else
    {
        char low_power_str[40] = "";
        if (node->is_low_power == 1) sprintf(low_power_str, "(low pwr, alive:%d)", node->low_power_alive);
        printf("##!! ERR getting Node %d, sensor %d value %s\n", nodeNum,  sensorNum, low_power_str);
        return -1;
    }
}
