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
    cfsetispeed(&options, B2000000); //B57600);
    cfsetospeed(&options, B2000000); //B57600);

    tcsetattr(uart_fd, TCSANOW, &options);

    // Turn off blocking for reads, use (uart_fd, F_SETFL, FNDELAY) if you want that
    //fcntl(uart_fd, F_SETFL, FNDELAY);

    return 1;
}

int transmitData(uchar *paket)
{
	ssize_t n;
    unsigned char buf[3];
	unsigned char c;

    buf[0] = 0xFE; //use first byte of buf as place from where the start chanracter 0xFE will be transmitted to nrf_master
	n = write(uart_fd, buf, 1);
    if (n < 0) { return -1; }

    n = write(uart_fd, (void*)paket, 11); //transmit normal packet
    if (n < 0) { return -1; }

    memset(&buf, 0, sizeof(buf) );
	//tcflush(uart_fd, TCIOFLUSH);

    unsigned int i = 0;
    unsigned timeout = 1100;

    while (i < 2) {
    	n = read(uart_fd, (void*)&c, 1);
        if (n > 0)
        {
            buf[i] = c;
            i++;
        }
        else
        {
            timeout--; //1100 * 20us = 22ms
            if (timeout == 0) { return -1; }
            usleep(20);
        }
    }
    buf[2] = 0;

    // recognize response
    if ( strncmp((char*)buf, "OK", 2) != 0)
    {
        //eliminujeme nacteni realneho paketu tim ze ho rozpozname
        if ( (strncmp((char*)buf, "ER", 2) != 0) && (strncmp((char*)buf, "TO", 2) != 0) )
        {
            //neni to odpoved na send paket, takze jsou to prvni 2 bajty z realneho paketu
            //ten nepotrebujem, protoze zrejme prisel nejak opozdene, tudiz nactem zbylych 9 bajtu
            //a tim se v bufferu posuneme na zacatek dalsiho mozneho paketu/potvrzeni
            while ( (n = read(uart_fd, (void*)&c, 1)) > 0) {}

            return -1;
        }
        else //TO or ER received
        {
        	return 0;
        }
    }
    else
    {
    	return 1;
    }
}

//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
int sendAndGetResponse(uchar *paket, uchar *response_buffer, unsigned int repeats)
{
	ssize_t n;
	unsigned int cycles = 0;
	unsigned char buf[11];
	unsigned int timeout_internal = 100;

	while ( (cycles < repeats) && (timeout_internal > 0) )
	{
		int result = transmitData(paket);
		if (result == -1) {
			return -1;
		} else if (result == 0) {
			cycles++;
			continue;
		}

		unsigned int i = 0;
		while(i < 11) {
			unsigned char c;

			n = read(uart_fd, (void*)&c, 1);
			if (n > 0)
			{
				buf[i] = c;
				i++;
			}
			else
			{
				timeout_internal--; //100 * 100us = 10ms
				if (timeout_internal == 0) { break; }
				usleep(100);
			}
		}

		if (i == 11) //whole packet was read from uart
		{
			//decide if the packet is really from our node
			if (buf[0] == paket[1]) //yes, it is our response (transmitter is the same as target)
			{
				memcpy(response_buffer, buf, 11);
				return 1; //return from function with success
			}
			else
			{ //no, it is from other node
				tcflush(uart_fd, TCIFLUSH);
				cycles++;
				continue;
			}
		}
	} // end WHILE

	if (timeout_internal == 0) {return -10;} //timeout occured
	return -11; //repeats tryouts depleted
}

//----------------------------------------------------------------------------------------------------
// --- higher level UART functions called by program
//----------------------------------------------------------------------------------------------------
int getNodePresentation(uchar nodeNnum, NodeSpecsT *paket)
{
	//uchar pres_req_packet[11] = {1, 3, 5, 0, 0, 0, 0, 0, 0, 0, 0};

    mirfPacket pres_req_packet;
    uchar respPacket[11];

    pres_req_packet.txAddr = 1;
    pres_req_packet.rxAddr = nodeNnum;
    pres_req_packet.type = (PACKET_TYPE) PRESENTATION_REQUEST;

    memset(respPacket, 0, 11);

    if (sendAndGetResponse((uchar *)&pres_req_packet, respPacket, 2) == 1 )
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
    //uchar sensor_read_packet[11] = {1, 99, 3, 255, 0, 0, 99, 0, 0, 0, 0};

	uchar respPacket[11];
    mirfPacket sensor_read_packet;

    sensor_read_packet.txAddr = 1;
    sensor_read_packet.rxAddr = nodeNum;
    sensor_read_packet.type = (PACKET_TYPE) REQUEST;

    sensor_read_packet.payload.request_struct.cmd = (CMD_TYPE) READ;
    sensor_read_packet.payload.request_struct.for_sensor = sensorNum;
    sensor_read_packet.payload.request_struct.len = 1;

    if (sendAndGetResponse((uchar *)&sensor_read_packet, respPacket) == 1)
    {
        *dataLen = respPacket[5]; //length value
        memcpy(data, &respPacket[7], *dataLen);
        return 1; //return from function with success
    }

    return -1; //node probably didnt response, return with fail
}

void revealNodes(void)
{
    for (int nodeNum = 2; nodeNum < MAX_NODES; nodeNum++) //nodes starting from 2, because 0 is forbidden and 1 is master node
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
                    if ( asprintf((char**)&(nodeValues[nodeNum]->sensor_names[x]), "node_%d_%i_%s", nodeNum, x, druhy_senzoru_str[nodeValues[nodeNum]->sensor_types[x]]) )
                    ; //and to silence mac compiler, semicolon must be on other line than IF statement
                }
                //alloc num_sensors of value unions
                nodeValues[nodeNum]->sensors = (volatile SENSOR_VAL_T**) malloc( sizeof(SENSOR_VAL_T*) * node_specs.num_sensors);  //alloc array of pointers to sensors values
                nodeValues[nodeNum]->last_valid_values = (volatile SENSOR_VAL_T**) malloc( sizeof(SENSOR_VAL_T*) * node_specs.num_sensors);  //alloc array of pointers to sensors values
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
                    nodeValues[nodeNum]->last_valid_values[x] = (SENSOR_VAL_T*) malloc( sizeof(SENSOR_VAL_T) );
                    memset((void*)nodeValues[nodeNum]->last_valid_values[x], 255, sizeof(SENSOR_VAL_T) );

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
	//uchar paket[11] = {1, 99, 3, 255, 1, 1, 99, 255, 0, 0, 0};

    mirfPacket paket;

    paket.txAddr = 1;
    paket.rxAddr = nodeNum;
    paket.type = (PACKET_TYPE) REQUEST;

    paket.payload.request_struct.cmd = (CMD_TYPE) WRITE;
    paket.payload.request_struct.for_sensor = sensorNum;
    paket.payload.request_struct.payload[0] = uchar(sensorData);
    paket.payload.request_struct.len = 1;

    int result = transmitData((uchar *)&paket);
    return result;
}

int writeUartSensorCalib(uchar nodeNum, uchar sensorNum, int calibData)
{
	//uchar paket[11] = {1, 99, 3, 255, 5, 1, 99, 0, 0, 0, 0};

    mirfPacket paket;

    paket.txAddr = 1;
    paket.rxAddr = nodeNum;
    paket.type = (PACKET_TYPE) REQUEST;

    paket.payload.request_struct.cmd = (CMD_TYPE) CALIBRATION_WRITE;
    paket.payload.request_struct.for_sensor = sensorNum;
    paket.payload.request_struct.payload[0] = uchar(calibData);
    paket.payload.request_struct.len = 1;

    int result = transmitData((uchar *)&paket);
    printf("## setCalib - r:%i, newVal:%d\n", result, calibData);
    return result;
}

int performUartValueReadAndSave(uchar nodeNum, uchar sensorNum)
{
    uchar rawData[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    int rawLen = 4;
    auto start_time = std::chrono::high_resolution_clock::now();
    int result = getSensorRawData(nodeNum, sensorNum, (uchar *)&rawData, &rawLen);
    auto end_time = std::chrono::high_resolution_clock::now();
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

        //store also last alive related informations
        node->sensor_read_times[sensorNum] = time(NULL);
        copySensorValueToLastValid(node, sensorNum);

        //now little addition - storing Vcc which comes along with DS18B20 value if low power sensor
        //TODO: move this somewhere else (probably right to function that counts DS18b20 temp
        if (node->is_low_power == 1) {
        	if (sensType == TEPLOTA_DS1820) {
        		countLowPowerVcc(node, sensorNum, (uchar*)&rawData, rawLen);
        	}
        }
    }
    mutexValues.unlock();

    auto uart_stopwatch = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
#ifdef UART_DEBUG

        printf("Vysledek: %s\n", buf);
#endif

    if ( result > 0)
    {
        int valStrLen;
        uchar *valStr;
        char low_power_str[40] = "";

        (*getSensorValStr[ sensType ])(node, sensorNum, &valStr, &valStrLen);
        if (node->is_low_power == 1) sprintf(low_power_str, "(low pwr, Vcc = %.3f V)\n", (double)node->low_power_voltage / 1000.0);
        printf("## Node %d, sensor %d, value: %s (t=%dms)\n%s", nodeNum,  sensorNum, valStr, (int)uart_stopwatch, low_power_str );
        free(valStr);
        return 1;
    }
    else
    {
        char low_power_str[40] = "";
        if (node->is_low_power == 1) sprintf(low_power_str, "(low pwr, alive:%d)", node->low_power_alive);
        printf("##!! ERR getting Node %d, sensor %d value %s (t=%dms)\n", nodeNum,  sensorNum, low_power_str, (int)uart_stopwatch);
        return -1;
    }
}
