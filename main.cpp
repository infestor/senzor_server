//#define _GNU_SOURCE 1

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
#include <netinet/in.h>
#include <netdb.h>
#include <signal.h>
#include <sys/poll.h>
#include <limits.h>

typedef unsigned char uint8_t;
//#include "packet_defs.h"

//file and socket descriptors
int fd;
FILE *out_fd;
int socketfd;
int signalInt;
int incoming_sd;
int incoming_conns;
#define MAX_CONNS 7
struct pollfd ufds[MAX_CONNS];

#define MAX_NODES 6
#define MAX_SENSORS 6
#define MAX_SENSOR_NAME_LEN 30;

//#define DEBUG

#include "main.h"

typedef unsigned char uchar;
const char str_port_name[] = "/dev/tty.wchusbserialfa130";
const char str_port_name_raspi[] = "/dev/ttyUSB0";
//const char port_name[] = "/dev/tty.wchusbserialfd120";
//const char port_name[] = "/dev/ttyAMA0";

const char *druhy_senzoru[] = { "Vystup ZAP/VYP", "Teplota", "Dverni spinac ON/OFF", "Teplota procesoru",
    "Teplota DS1820", "Napeti baterie 1 clanek LION", "Napeti baterie 2 clanky LION" };

//const uchar sensorReturnLen[] = {1, 2, 1, 1, 2, 2, 2};
const uchar writableSensorTypes[] = {1, 0, 0, 0, 0, 0, 0};

volatile NODE_VALUES_T * volatile nodeValues[MAX_NODES];

volatile uchar *nodeList;

const double TempCorrections[] = {99, 99, 0, 4.5};

typedef struct {
    uchar txAddr;
    uchar rxAddr;
    uchar  type;
    uchar  counter;
    uchar  payload[7];
} mirfPacket;

typedef struct {
    uchar node;
    uchar num_sensors;
    uchar sensors[MAX_SENSORS];
} NodeSpecsT;

volatile uchar numNodes = 0;

#define CMD_NONE 0
#define CMD_REVEAL 1
#define CMD_WRITE 2
#define CMD_READ 3

//array of pointers to functions
//these functions process raw sensor data and stores them to nodeValues
//this is needed to handle different data types in every sensor
//parameters of all functions (NODE_VALUES_T *nodeP, uchar sensorNum, uchar *rawData, int rawLen)
void (*countAndStoreSensorValue[]) STORE_VALUE_PARAMS = \
{decideOnOffValue, NULL, decideOnOffValue, countInternalProcTemp, countDS1820Temp, countBatteryVoltOneCell, countBatteryVoltTwoCell};

//array of pointers to functions
//these functions get sensor value from nodeValues and convert it to string in brackets, example [value_in_string_form] [1] [24.4]
//according to sensor type
//string is allocated inside function, so there is need to free it after use !!!
void (*getSensorValStr[]) GET_VALUE_PARAMS = \
{getIntValStr, NULL, getIntValStr, getFloatValStr, getFloatValStr, getFloatValStr, getFloatValStr};

//array for storing information, how often read which sensor value over UART
//both these vectors must always be the same size!!
//and indexes from one must point to same sensor's data in the other vector
typedef struct {
    int interval;
    int countDown;
} SENSOR_INTERVAL_REC;

volatile SENSOR_INTERVAL_REC * volatile sensorIntervals[MAX_NODES];

typedef struct {
    uchar nodeNum;
    uchar sensorNum;
} SENSOR_INTERVAL_VECT_REC;

std::vector<SENSOR_INTERVAL_VECT_REC> intervalVect;

//this is for queue of UART data transfer separate thread
typedef struct {
    uchar cmd;
    uchar nodeNum;
    uchar sensorNum;
    int sensorVal;
    int intervalOk;
    int intervalErr;
} THREAD_QUEUE_REC;

std::queue<THREAD_QUEUE_REC> threadQueue;
std::mutex mutexQueue;
std::mutex mutexValues;
std::mutex mutexIntervals;

volatile bool threadRevealFinished = true; //should be true most of the time, false only during reveal
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------

void freeArrayOfPointers(void ***pole, int poleLen)
{
    if (*pole != NULL)
    {
        for (int xxx = 0; xxx < poleLen; xxx++) {
            if ((*pole)[xxx] != NULL) free((*pole)[xxx]);
        }
        free(*pole);
    }
}

void freeNodeValStruct(volatile NODE_VALUES_T* volatile *p)
{
    if (*p != NULL)
    {
        freeArrayOfPointers((void***)&(*p)->sensors, (*p)->num_sensors);
        freeArrayOfPointers((void***)&(*p)->sensor_names, (*p)->num_sensors);
        free((void*)*p);
    }
}

void cleanup(void)
{   
    close(fd);
    fclose(out_fd);
    if (socketfd > 0) close(socketfd); //close listening sockets
    for (int i = 0; i < incoming_conns; i++) //close all incoming sockets
    {
        close(ufds[i].fd);
    }
    incoming_conns = 0;
    
    //free stored node values
    for (int i = 0; i < MAX_NODES; i++)
    {
        freeNodeValStruct(&nodeValues[i]);
    }

    //free stored sensor interval values
    freeArrayOfPointers((void***)&sensorIntervals, MAX_NODES);
}

//=================================================================================
// ========= Sensor Data/value specific type dependent manipulation functions =====
//=================================================================================
void countDS1820Temp STORE_VALUE_PARAMS
{
    int TReading = rawData[0] + (rawData[1] << 8);
    
    uchar SignBit = ((TReading & 0x8000) >> 8);  // test most sig bit
    
    if (SignBit > 0) { // negative
        TReading = (TReading ^ 0xffff) + 1; // 2's comp
    }
    
    double result;
    result = (6.0 * TReading) + TReading / 4.0; // multiply by (100 * 0.0625) or 6.25
    result = result / 100.0;
    
    if (SignBit) { // If its negative
        result = result * (-1.0);
    }
    
    result -= TempCorrections[ nodeP->node ];
    
    nodeP->sensors[sensorNum]->float_val = result;
}

void countInternalProcTemp STORE_VALUE_PARAMS
{
    int TReading = rawData[0];
    nodeP->sensors[sensorNum]->float_val = (TReading - 128.0) / 1.07;
}

void decideOnOffValue STORE_VALUE_PARAMS
{
    int TReading = rawData[0];
    nodeP->sensors[sensorNum]->int_val = (TReading > 0) ? 1 : 0;
}

void countBatteryVoltOneCell STORE_VALUE_PARAMS
{
    int val = rawData[0] + (rawData[1] << 8);
    double volts = (5.0 / 1023.0) * val; //with 5V reference
    nodeP->sensors[sensorNum]->float_val = volts;
}

void countBatteryVoltTwoCell STORE_VALUE_PARAMS
{
    int val = rawData[0] + (rawData[1] << 8);
    double volts = (5.0 / 1023.0) * val * 2; //cell voltage divisor on circuit is 2
    nodeP->sensors[sensorNum]->float_val = volts;
}

void getFloatValStr GET_VALUE_PARAMS
{
    *strLen = asprintf( (char**)strBuf, "%.1f", nodeP->sensors[sensorNum]->float_val );
}

void getIntValStr GET_VALUE_PARAMS
{
    *strLen = asprintf( (char**)strBuf, "%i", nodeP->sensors[sensorNum]->int_val );
}

void getUintValStr GET_VALUE_PARAMS
{
    *strLen = asprintf( (char**)strBuf, "%d", nodeP->sensors[sensorNum]->uint_val );
}

void getUcharValStr GET_VALUE_PARAMS
{
    *strLen = asprintf( (char**)strBuf, "%d", nodeP->sensors[sensorNum]->uchar_val );
}

//=====================================================================================
//==== UART related functions =========================================================
//=====================================================================================
int setup_uart(const char *port_name)
{
    fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
	    char bu[100];
	    sprintf(bu, "open_port: Unable to open %s - ", port_name);
	    perror(bu);
	    return(-1);
    }
    
    struct termios options;
    tcgetattr(fd, &options);
    //cfmakeraw(&options);
    
    options.c_iflag = IGNBRK | IGNPAR | IXANY;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cflag = CLOCAL | CREAD | CS8;
    
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    cfsetispeed(&options, B57600);
    cfsetospeed(&options, B57600);
    
    tcsetattr(fd, TCSANOW, &options);
    
    // Turn off blocking for reads, use (fd, F_SETFL, FNDELAY) if you want that
    //fcntl(fd, F_SETFL, FNDELAY);
    
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
    //tcflush(fd, TCIOFLUSH);
    do
    {
        n = write(fd, (void*)paket, 12);
        if (n < 0) {
            return -1;
        }
        
        respReceived = false;
        do
        {
            i = 0;
            memset(&buf, 0, sizeof(buf) );
            //tcflush(fd, TCIFLUSH);
            while(i < 2) {
                n = read(fd, (void*)&c, 1);
                if (n>0)
                {
                    buf[i] = c;
                    i++;
                }
                else
                {
                    usleep(10000);
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
                        n = read(fd, (void*)&c, 1);
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
    
    counted_timeout = timeout / 5;
    
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
                    n = read(fd, (void*)&c, 1);
                    if (n>0)
                    {
                        buf[i] = c;
                        i++;
                    }
                    else
                    {
                        usleep(5000);
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
        usleep(50000);
    } while( !((was_timeout==true) || (repeats_get > (repeats-1))) );
    return -1;
}

//----------------------------------------------------------------------------------------------------
// --- higher level UART functions called by program
//----------------------------------------------------------------------------------------------------
int getNodePresentation(uchar nodeNnum, NodeSpecsT *paket)
{
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
    for (int i=2; i<MAX_NODES; i++) //nodes starting from 2, because 0 is forbidden and 1 is master node
    {
        printf("Trying get info about node %i\n", i);
        NodeSpecsT node_specs;
        if (getNodePresentation(i, &node_specs) > 0 )
        {
            mutexValues.lock();
            //prepare node values array for this node
            if (nodeValues[i] == NULL)
            {
                //freeNodeValStruct(&nodeValues[i]);
                nodeValues[i] = (NODE_VALUES_T*)malloc(sizeof(NODE_VALUES_T) );
                memset((void*)nodeValues[i], 0, sizeof(NODE_VALUES_T) );
                nodeValues[i]->node = i;
                nodeValues[i]->num_sensors = node_specs.num_sensors;
                memcpy((void*)nodeValues[i]->sensor_types, node_specs.sensors, MAX_SENSORS);
                nodeValues[i]->sensor_names = (volatile uchar**) malloc( sizeof(uchar*) * MAX_SENSORS); //alloc array of pointers to user names of sensors
                //int L;
                for (int x = 0; x < node_specs.num_sensors; x++ ) asprintf((char**)&(nodeValues[i]->sensor_names[x]), "unNamed_%d_%i", i, x); //fill all names with default values
                //alloc num_sensors of value unions
                nodeValues[i]->sensors = (volatile SENSOR_VAL_T**) malloc( sizeof(SENSOR_VAL_T*) * node_specs.num_sensors);  //alloc array of pointers to sensors values
                for (int x = 0; x < node_specs.num_sensors; x++ ) //alloc sensor val structures
                {
                    nodeValues[i]->sensors[x] = (SENSOR_VAL_T*) malloc( sizeof(SENSOR_VAL_T) );
                    memset((void*)nodeValues[i]->sensors[x], 255, sizeof(SENSOR_VAL_T) );
                }
            }
            mutexValues.unlock();
        }
    }
    //create simple list (just node nums) of all nodes
    //used also for recount of complete number of sensors
    uchar listik[MAX_NODES];
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
    mutexValues.lock();
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
  uchar paket[12] = {254, 1, 99, 3, 255, 0, 0, 99, 0, 0, 0, 0};
    paket[2] = nodeNum;
    paket[5] = 1; //write cmd
    paket[6] = 1; //length of data written
    paket[7] = sensorNum;
    paket[8] = uchar(sensorData);
  
  int result = transceiveData(paket);
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
    
    mutexValues.lock();
    
    //test for FFFF value as a sign of error reading from node, so store error value to array
    if (result <= 0) //( (int32_t)*rawData == 0xFFFFFFFF)
    {
        nodeValues[nodeNum]->sensors[sensorNum]->uint_val = UINT_MAX;
    }
    else
    {
        //choose proper sensor value handle function (according to sensor type) from array if pointers to functions
        (*countAndStoreSensorValue[nodeValues[nodeNum]->sensor_types[sensorNum] ])(nodeValues[nodeNum], sensorNum, (uchar*)&rawData, rawLen);
    }
    mutexValues.unlock();
    
    if ( result > 0)
    {
        int valStrLen;
        uchar *valStr;
        (*getSensorValStr[nodeValues[nodeNum]->sensor_types[sensorNum] ])(nodeValues[nodeNum], sensorNum, &valStr, &valStrLen);
        printf("## Node %d, sensor %d, value: %s\n", nodeNum,  sensorNum, valStr );
        free(valStr);
        return 1;
    }
    else
    {
        printf("##!! ERR getting Node %d, sensor %d value\n", nodeNum,  sensorNum);
        return -1;
    }
}

//========================================================================        
//====== SOCKET related functions ========================================        
//========================================================================        
void my_ctrl_handler(int s){
    printf("\n## Caught signal %d\n",s);
    tcflush(fd, TCIOFLUSH);
    printf("## Saving config files...\n");
    saveConfigFiles();
    cleanup();
    exit(1);
}

int vytvoritSocket()
{
    int status;
    
    /*struct addrinfo host_info;
     struct addrinfo *host_info_list;
     memset(&host_info, 0, sizeof host_info);
     host_info.ai_family = AF_INET;     // IP version not specified. Can be both.
     host_info.ai_socktype = SOCK_STREAM; // Use SOCK_STREAM for TCP or SOCK_DGRAM for UDP.
     host_info.ai_flags = AI_PASSIVE;     // IP Wildcard
     
     status = getaddrinfo(NULL, "29443", &host_info, &host_info_list);
     if (status != 0)
     {
     printf("getaddrinfo error %s\n", gai_strerror(status) );
     cleanup();
     exit(9);
     }
     */
    
    printf("Creating a socket...\n");
    //socketfd = socket(host_info_list->ai_family, host_info_list->ai_socktype, host_info_list->ai_protocol);
    socketfd = socket(AF_INET, SOCK_STREAM, 0);
    if (socketfd == -1)  { printf("socket error\n"); cleanup(); exit(9); }
    int yes = 1;
    status = setsockopt(socketfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));
    
    sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(29443);
    status = bind(socketfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
    //status = bind(socketfd, host_info_list->ai_addr, host_info_list->ai_addrlen);
    //freeaddrinfo(host_info_list);
    if (status == -1)  {printf("bind error\n"); cleanup(); exit(9); }
    
    //socket callback
    fcntl(socketfd, F_SETOWN, getpid());
    int flags = fcntl(socketfd, F_GETFL, 0);
    fcntl(socketfd, F_SETFL, flags | O_NONBLOCK); //| O_ASYNC);
    ioctl(socketfd, FIONBIO, (char *)&yes);
    /*signalInt = SIGRTMIN + 1;
     printf("socket SignalInt: %d\n", signalInt);
     fcntl(socketfd, F_SETSIG, signalInt);
     sigIntHandler.sa_sigaction = socketConnHandleFunc;
     sigemptyset(&sigIntHandler.sa_mask);
     sigIntHandler.sa_flags = SA_SIGINFO;
     sigaction(signalInt, &sigIntHandler, NULL);
     */
    
    printf("Listening for connections...\n");
    status =  listen(socketfd, MAX_CONNS);
    if (status == -1)  {printf("listen error\n"); cleanup(); exit(9); }
    
    memset(ufds, 0 , sizeof(ufds));
    return 1;
}

int removeUfd(uchar ind)
{
    if (ind == (incoming_conns-1) ) //case when it is last record (even if there is only 1 record)
    {
        //no action needed - we just will decrease incoming_conns
    }
    else
    {
        for (uchar pos = ind; pos < (incoming_conns-2); pos++)
        {
            ufds[pos] = ufds[pos+1];
        }
    }
    
    incoming_conns--;
    return 1;
}

int explode(char* str, const char* delim, char ***r) {
    char **res = (char**) malloc(sizeof(char*) * strlen(str));
    char *p;
    int i = 0;
    while ((p = strtok(str, delim))) {
        res[i] = (char*)malloc(strlen(p) + 1);
        strcpy(res[i], p);
        ++i;
        str = NULL;
    }
    res = (char**)realloc(res, sizeof(char*) * i);
    *r = res;
    return i;
}

int appendToBuffer(uchar **buf, int *len, int *pos, uchar *msg, int msgLen)
{
    int bufRemain = *len - ((*pos)+1); //remainig size of buffer
    
    if (bufRemain < msgLen) //buffer is not big enough, realloc it
    {
        *len = *len + (msgLen-bufRemain);
        *buf = (uchar *)realloc(*buf, *len);
    }
    
    memcpy(*buf + *pos + 1, msg, msgLen);
    *pos = *pos + msgLen;
    
    return 1;
}

int processSockCmd(uchar *inBuff, ssize_t bufLen, uchar **outBuf, int *outBufPos, int *outLen)
{
    uchar *buff;
    int pos = 0;
    ssize_t gpos = 0;
    buff = inBuff; //just store the pointer
    
    //wrong in data handling
    bool chyba = false;
    if (bufLen < 3) { return -1; } //dont add err because this mainly handles linefeed at end
    if (buff[0] != '[' )
    {
        while (buff[pos] != '[') //lets find [ but with some limits
        {
            pos++;
            gpos++;
            //if ( (pos > 30) || (gpos == bufLen) ) {chyba = true; break; }
            if ( gpos == bufLen) {chyba = true; break; }
        }
        
        if (chyba == true)
        {
            appendToBuffer(outBuf, outLen, outBufPos, (uchar*)"<[ERR-PARSE]", 12);
            
            return -1;
        }
    }
    
    buff += pos; //move over previous gap
    
    //data OK, lets process it
    uchar cmd[30];
    pos = 0;
    memset(cmd, 0, sizeof(cmd) );
    while (buff[pos] != 93)
    {
        cmd[pos] = buff[pos];
        pos++;
        gpos++;
        if ( (pos > 30) || (gpos == bufLen) ) {chyba = true; break; }
    }
    if (chyba == true)
    {
        appendToBuffer(outBuf, outLen, outBufPos, (uchar*)"<[ERR-PARSE]", 12);
        appendToBuffer(outBuf, outLen, outBufPos, cmd, pos);
        appendToBuffer(outBuf, outLen, outBufPos, (uchar*)">", 1);
        return -1;
    }
    cmd[pos] = 93; //add trailing ]
    uchar cmdLen = pos+1;
    
    //no need to check here whether CHYBA is false or true, because at this point it is always FALSE
    //previous controls cannot let it go throught if there is some error and must immediately return
    //because those errors are serious and mean, that no command could be extracted from buffer
    
    //process command
    //=== getVal command [getVal:nodeNum:sensorId]
    if (strncmp((const char*)cmd, "[getVal:", 8) == 0)
    {
        //count colons (:) - must be 2
        int colons = 0;
        for (int xxx=0; xxx < cmdLen; xxx++) { if (cmd[xxx] == ':') colons++; }
        if (colons != 2) { chyba = true; }
        
        if (chyba == false)
        {
            //strip command from [ and ], but preserve \0 on end
            uchar cmdStripped[cmdLen-1];
            strncpy((char *)cmdStripped, (const char *)cmd+1, (size_t)cmdLen-2);
            
            //explode cmd to values
            char** pole;
            int poleLen = explode((char *)cmdStripped, ":", &pole);
            uchar nodeNum = atoi(pole[1]);
            uchar nodeSens = atoi(pole[2]);
            freeArrayOfPointers((void***)&pole, poleLen);
            
            //is the request for existing Node and sensor?
            if ( (nodeValues[nodeNum] == NULL) || (nodeValues[nodeNum]->num_sensors-1 < nodeSens) ) chyba = true;
            
            uchar *outMsg;
            int L;
            if (chyba == false)
            {
                uchar *sensorValStr;
                int sensorValStrLen;

                //decide if the value vas ever read from real sensor
                //bcause on initialisation the union of value is filled by 0xFF
                if (nodeValues[nodeNum]->sensors[nodeSens]->uint_val != UINT_MAX)
                {
                    //use right type of sensor value (float or int..)!!
                    mutexValues.lock();
                    (*getSensorValStr[nodeValues[nodeNum]->sensor_types[nodeSens] ])(nodeValues[nodeNum], nodeSens, &sensorValStr, &sensorValStrLen);
                    mutexValues.unlock();
                }
                else //if the value vas never read from sensor, return X instead of value
                {
                    sensorValStrLen = asprintf((char**)&sensorValStr, "x");
                }
                
                L = asprintf((char**)&outMsg, "<[OK]%s[%s]>", cmd, sensorValStr);
                free(sensorValStr);
                printf(">OK: %s\n", cmd);
            }
            else
            {
                L = asprintf((char**)&outMsg, "<[ERR]%s>", cmd);
                printf(">ERR: %s\n", cmd);
            }
            
            appendToBuffer(outBuf, outLen, outBufPos, outMsg, L);
            free(outMsg);
        }
        else //there was error processing this particular command, add error response to output buffer
        {
            appendToBuffer(outBuf, outLen, outBufPos, (uchar*)"<[ERR]", 6);
            appendToBuffer(outBuf, outLen, outBufPos, cmd, cmdLen);
            appendToBuffer(outBuf, outLen, outBufPos, (uchar*)">", 1);
        }
    }
    //=== getNodeProp - get node properties/presentation [getNodeProp:nodeNum]
    else if (strncmp((const char*)cmd, "[getNodeProp:", 13) == 0)
    {
        //count colons (:) - must be 1
        int colons = 0;
        for (int xxx=0; xxx < cmdLen; xxx++) { if (cmd[xxx] == ':') colons++; }
        if (colons != 1) { chyba = true; } 
        
        //check if nodeNum is present (there is not :] together on the end)
        if (cmd[cmdLen-2] == ':') { chyba = true; }

        //strip command from [ and ], but preserve \0 on end
        uchar cmdStripped[cmdLen-1];
        strncpy((char *)cmdStripped, (const char *)cmd+1, (size_t)cmdLen-2);
        
        uchar nodeNum = '\0';
        
        if (chyba == false)
        {
            //explode cmd to values
            char** pole;
            int poleLen = explode((char *)cmdStripped, ":", &pole);
            nodeNum = atoi(pole[1]);
            freeArrayOfPointers((void***)&pole, poleLen);
            
            //is the request for existing Node?
            if (nodeValues[nodeNum] == NULL) chyba = true;
        }
        
        int L;
        uchar *outMsg;
        if (chyba == false)
        {
            uchar sensorsList[30];
            memset(sensorsList, 0, 30);
            for (int xx=0; xx < nodeValues[nodeNum]->num_sensors; xx++)
            {
                sprintf((char *)sensorsList, "%s%d,", sensorsList, nodeValues[nodeNum]->sensor_types[xx]);
                
            }
            sensorsList[strlen((const char *)sensorsList) - 1] = 0; //end of string (that -1 is because it owerwrites over last "," (carka)
            L = asprintf((char**)&outMsg, "<[OK]%s[%d:%s]>", cmd, nodeValues[nodeNum]->num_sensors, sensorsList);
            appendToBuffer(outBuf, outLen, outBufPos, outMsg, L);
            printf(">OK: %s\n", cmd);
            free(outMsg);
        }
        else //there was some error during parsing
        {
            appendToBuffer(outBuf, outLen, outBufPos, (uchar*)"<[ERR]", 6);
            appendToBuffer(outBuf, outLen, outBufPos, cmd, cmdLen);
            appendToBuffer(outBuf, outLen, outBufPos, (uchar*)">", 1);
        }
    }
    //=== setSensVal - set value of sensor which is writeable [setSensVal:nodeNum:sensorNum:value]
    //TODO: so far there is no sensor value type handling - because the only sensor supporting
    //writes is ON/OFF switch and there is used unsigned int(char) type
    else if (strncmp((const char*)cmd, "[setSensVal:", 12) == 0)
    {
        //count colons (:) - must be 3
        int colons = 0;
        for (int xxx=0; xxx < cmdLen; xxx++) { if (cmd[xxx] == ':') colons++; }
        if (colons != 3) { chyba = true; }
        
        //check if nodeNum is present (there is not :] together on the end)
        if (cmd[cmdLen-2] == ':') chyba = true;
        
        //strip command from [ and ], but preserve \0 on end
        uchar cmdStripped[cmdLen-1];
        strncpy((char *)cmdStripped, (const char *)cmd+1, (size_t)cmdLen-2);
        
        uchar nodeNum = '\0';
        uchar sensNum = '\0';
        uchar sensVal = '\0';
        
        if (chyba == false)
        {
            //explode cmd to values
            char** pole;
            int poleLen = explode((char *)cmdStripped, ":", &pole);
            nodeNum = atoi(pole[1]);
            sensNum = atoi(pole[2]);
            sensVal = atoi(pole[3]);
            freeArrayOfPointers((void***)&pole, poleLen);
            
            //is the request for existing Node?
            if (nodeValues[nodeNum] == NULL) chyba = true;
            //sensor on node exists?
            else if (nodeValues[nodeNum]->num_sensors < sensNum+1) chyba = true;
            //is sensor writable?
            else if (writableSensorTypes[ nodeValues[nodeNum]->sensor_types[sensNum] ] != 1) chyba = true;
        }
        
        int L;
        uchar *outMsg;
        if (chyba == false)
        {
            //do not save value to internal values before it is sucesfully written
            //value in internal array will be changed from thread which handles writing/reading when succesfull
            //nodeValues[nodeNum]->sensors[sensNum]->int_val = sensVal; 
            
            //create and push message/command to thread queue
            THREAD_QUEUE_REC rec;
            rec.cmd = CMD_WRITE;
            rec.nodeNum = nodeNum;
            rec.sensorNum = sensNum;
            rec.sensorVal = sensVal;
            rec.intervalErr = 30;
            
            mutexQueue.lock();
            threadQueue.push(rec);
            mutexQueue.unlock();
            
            L = asprintf((char**)&outMsg, "<[OK]%s>", cmd);
            appendToBuffer(outBuf, outLen, outBufPos, outMsg, L);
            printf(">OK: %s\n", cmd);
            free(outMsg);
        }
        else //there was some error during parsing
        {
            appendToBuffer(outBuf, outLen, outBufPos, (uchar*)"<[ERR]", 6);
            appendToBuffer(outBuf, outLen, outBufPos, cmd, cmdLen);
            appendToBuffer(outBuf, outLen, outBufPos, (uchar*)">", 1);
        }
    }
    //=== setInterval - set interval of sensor reading [setInterval:nodeNum:sensorNum:value]
    else if (strncmp((const char*)cmd, "[setInterval:", 13) == 0)
    {
        //count colons (:) - must be 3
        int colons = 0;
        for (int xxx=0; xxx < cmdLen; xxx++) { if (cmd[xxx] == ':') colons++; }
        if (colons != 3) { chyba = true; }
        
        //check if nodeNum is present (there is not :] together on the end)
        if (cmd[cmdLen-2] == ':') chyba = true;
        
        //strip command from [ and ], but preserve \0 on end
        uchar cmdStripped[cmdLen-1];
        strncpy((char *)cmdStripped, (const char *)cmd+1, (size_t)cmdLen-2);

        uchar nodeNum = '\0';
        uchar sensNum = '\0';
        int sensVal = 0;
            
        if (chyba == false)
        {        
            //explode cmd to values
            char** pole;
            int poleLen = explode((char *)cmdStripped, ":", &pole);
            nodeNum = atoi(pole[1]);
            sensNum = atoi(pole[2]);
            sensVal = atoi(pole[3]);
            freeArrayOfPointers((void***)&pole, poleLen);
            
            //is the request for existing Node?
            if (nodeValues[nodeNum] == NULL) chyba = true;
            //sensor on node exists?
            else if (nodeValues[nodeNum]->num_sensors < sensNum+1) chyba = true;
            //is interval in bounds?
            else if ( (sensVal < 0) || (sensVal > 3600) ) chyba = true; //cannot be 0, because other clients could want to read the value
        }
        
        int L;
        uchar *outMsg;
        if (chyba == false)
        {
            //set the interval
            //if there is no record for this sensor - create it
            
            //TODO: somehow figure out client concurrency - if one want it shorter than the second,
            //but then the shorter wants to prolong it (or completely turn off - set 0) - what to do?
            
            //find record
            //we can presume, that even if the value exists and is 0, then it is in fact not existent
            //because it does not exist in intervalVector
            bool found = false;
            if (sensorIntervals[nodeNum] != NULL) {
                if (sensorIntervals[nodeNum][sensNum].interval > 0) found = true;
            }
            
            if (found == true)
            {
                mutexIntervals.lock();
                sensorIntervals[nodeNum][sensNum].interval = sensVal;
                //if countdown is bigger than new interval, set it to new value
                if (sensorIntervals[nodeNum][sensNum].countDown > sensVal) sensorIntervals[nodeNum][sensNum].countDown = sensVal;
                mutexIntervals.unlock();
                
                //if the interval is 0, then remove record from queue
                if (sensVal == 0)
                {
                    //find the record in interval vector
                    std::vector<SENSOR_INTERVAL_VECT_REC>::iterator it;
                    for (it = intervalVect.begin(); it < intervalVect.end(); it++)
                    {
                        if ( (it->nodeNum == nodeNum) && (it->sensorNum == sensNum) )
                        {
                            mutexIntervals.lock();
                            intervalVect.erase(it);
                            mutexIntervals.unlock();
                            break;
                        }
                    }
                }
            }
            else  //record not found, create new if it is not 0
            {
                if(sensVal > 0)
                {
                    //prepare new record to be pushed
                    SENSOR_INTERVAL_VECT_REC rec;
                    rec.nodeNum = nodeNum;
                    rec.sensorNum = sensNum;

                    //just if this is the case when even all node array is not initialised
                    mutexIntervals.lock();
                    if (sensorIntervals[nodeNum] == NULL)
                    {
                        sensorIntervals[nodeNum] = (SENSOR_INTERVAL_REC *) malloc(sizeof(SENSOR_INTERVAL_REC) * MAX_SENSORS);
                        memset((void*)sensorIntervals[nodeNum], 0, sizeof(SENSOR_INTERVAL_REC) * MAX_SENSORS);
                    }
                    
                    sensorIntervals[nodeNum][sensNum].interval = sensVal;
                    sensorIntervals[nodeNum][sensNum].countDown = 1;
                    //this 1 in countdown will invoke real read of value, because it was read never before
                    //and we dont wanna wait whole interval before the value will be available
                    
                    //add record also to interval vector
                    intervalVect.push_back(rec);
                    mutexIntervals.unlock();
                }
            }
            
            L = asprintf((char**)&outMsg, "<[OK]%s>", cmd);
            appendToBuffer(outBuf, outLen, outBufPos, outMsg, L);
            printf(">OK: %s\n", cmd);
            free(outMsg);
        }
        else //there was some error during parsing
        {
            appendToBuffer(outBuf, outLen, outBufPos, (uchar*)"<[ERR]", 6);
            appendToBuffer(outBuf, outLen, outBufPos, cmd, cmdLen);
            appendToBuffer(outBuf, outLen, outBufPos, (uchar*)">", 1);            
        }
    }
    //=== getInterval - gets interval of sensor reading [getInterval:nodeNum:sensorNum]
    else if (strncmp((const char*)cmd, "[getInterval:", 13) == 0)
    {
        //count colons (:) - must be 2
        int colons = 0;
        for (int xxx=0; xxx < cmdLen; xxx++) { if (cmd[xxx] == ':') colons++; }
        if (colons != 2) { chyba = true; }
        
        //check if nodeNum is present (there is not :] together on the end)
        if (cmd[cmdLen-2] == ':') chyba = true;
        
        //strip command from [ and ], but preserve \0 on end
        uchar cmdStripped[cmdLen-1];
        strncpy((char *)cmdStripped, (const char *)cmd+1, (size_t)cmdLen-2);
        
        uchar nodeNum = '\0';
        uchar sensNum = '\0';
        
        if (chyba == false)
        {                
            //explode cmd to values
            char** pole;
            int poleLen = explode((char *)cmdStripped, ":", &pole);
            nodeNum = atoi(pole[1]);
            sensNum = atoi(pole[2]);
            freeArrayOfPointers((void***)&pole, poleLen);
            
            //is the request for existing Node?
            if (nodeValues[nodeNum] == NULL) chyba = true;
            //sensor on node exists?
            else if (nodeValues[nodeNum]->num_sensors < sensNum+1) chyba = true;
        }
        
        int L;
        uchar *outMsg;
        if (chyba == false)
        {
            //find record
            bool found = false;
            if (sensorIntervals[nodeNum] != NULL) found = true;
            int interval = (found == true) ? sensorIntervals[nodeNum][sensNum].interval : 0;
            
            L = asprintf((char**)&outMsg, "<[OK]%s[%i]>", cmd, interval);
            appendToBuffer(outBuf, outLen, outBufPos, outMsg, L);
            printf(">OK: %s\n", cmd);
            free(outMsg);
        }
        else //there was some error during parsing
        {
            appendToBuffer(outBuf, outLen, outBufPos, (uchar*)"<[ERR]", 6);
            appendToBuffer(outBuf, outLen, outBufPos, cmd, cmdLen);
            appendToBuffer(outBuf, outLen, outBufPos, (uchar*)">", 1);
        }
    }
    //=== invokeSearch - starts node reveal process, non blocking mode, will send OK and when finished, will send [FIN][invokeSearch]
    else if (strncmp((const char*)cmd, "[invokeSearch]", 14) == 0)
    {
        //create and push message/command to thread queue
        THREAD_QUEUE_REC rec;
        rec.cmd = CMD_REVEAL;
        //no other vaues in record are necessary, because cmd_reveal has no input parameters
        
        threadRevealFinished = false;
        
        mutexQueue.lock();
        threadQueue.push(rec);
        mutexQueue.unlock();
                    
        uchar *outMsg;
        int L = asprintf((char**)&outMsg, "<[OK]%s>", cmd);
        appendToBuffer(outBuf, outLen, outBufPos, outMsg, L);
        printf(">OK: %s\n", cmd);
        free(outMsg);
    }
    //=== isSearching - returns 1 if node reveal is in progress, else 0
    else if (strncmp((const char*)cmd, "[isSearching]", 13) == 0)
    {
        uchar *outMsg;
        uchar cmdResult = (threadRevealFinished == false) ? 1 : 0;
        int L = asprintf((char**)&outMsg, "<[OK]%s[%d]>", cmd, cmdResult);
        appendToBuffer(outBuf, outLen, outBufPos, outMsg, L);
        printf(">OK: %s\n", cmd);
        free(outMsg);
    }
    //=== getNodeList - [getNodeList] - sends node list
    else if (strncmp((const char*)cmd, "[getNodeList]", 13) == 0)
    {
        uchar *outMsg;
        uchar *copyNodeList;
        int copyNumNodes;
        
        if (numNodes == 0)
        {
            int L = asprintf((char**)&outMsg, "<[OK]%s[x]>", cmd);
            appendToBuffer(outBuf, outLen, outBufPos, outMsg, L);
            free(outMsg);
        }
        else
        {
            //create copy of node list to minimize mutex lock time
            copyNodeList = (uchar*)malloc(sizeof(uchar)*numNodes);
            mutexValues.lock();
            copyNumNodes = numNodes;
            memcpy(copyNodeList, (const void*)nodeList, sizeof(uchar)*numNodes);
            mutexValues.unlock();
            
            
            int L = asprintf((char**)&outMsg, "<[OK]%s", cmd);
            appendToBuffer(outBuf, outLen, outBufPos, outMsg, L);
            appendToBuffer(outBuf, outLen, outBufPos, (uchar*)"[", 1);
            free(outMsg);
            
            outMsg = (uchar*) malloc(10);
            for (int x=0; x < copyNumNodes-1; x++)
            {
                L = sprintf((char*)outMsg, "%d,", copyNodeList[x]);
                appendToBuffer(outBuf, outLen, outBufPos, outMsg, L);
            }
            
            L = sprintf((char*)outMsg, "%d", copyNodeList[copyNumNodes-1]);
            appendToBuffer(outBuf, outLen, outBufPos, outMsg, L);
            free(outMsg);
                        
            appendToBuffer(outBuf, outLen, outBufPos, (uchar*)"]>", 2);
        }
        
                        
        printf(">OK: %s\n", cmd);
    }
    //=== any other unknown command is error
    else
    {
        uchar outMsg[cmdLen+13+1];
        int L = sprintf((char*)&outMsg, "<[ERR-UNKNOWN]%s>", cmd);
        
        appendToBuffer(outBuf, outLen, outBufPos, outMsg, L);
    }
    
    //process iteratively if there is something else unprocessed in input buffer
    //...
    if (gpos+1 < bufLen) processSockCmd(inBuff+gpos+1, bufLen-(gpos+1), outBuf, outBufPos, outLen);
    
    return 1;
}

//=============================================================================
//====== UART thread (reads and saves data in backgroud) =====================
//=============================================================================
unsigned int randr(unsigned int min, unsigned int max)
{
    double scaled = (double)rand()/RAND_MAX;
    return (max - min +1)*scaled + min;
}

void threadProcessQueue(void)
{
    printf("## UART thread started..\n");

    bool work;
    THREAD_QUEUE_REC rec;
    
    while (1)
    {
        work = false;
        mutexQueue.lock();
        if (!threadQueue.empty())
        {
            rec = threadQueue.front();
            threadQueue.pop();
            work = true;
        }
        mutexQueue.unlock();
        
        if (work == true)
        {
            //handle command type
            if (rec.cmd == CMD_READ)
            {
                //we must check if the value from UART sensor was read with success, else
                //if it wasnt and its interval is bigger than 60s, we choose
                //random interval 4-31sec to try it again
                if ( (performUartValueReadAndSave(rec.nodeNum, rec.sensorNum) < 0) ) //&& (rec.intervalOk > 60)
                {
                    mutexIntervals.lock();
                    if (sensorIntervals[rec.nodeNum] != NULL)
                    {
                        if (sensorIntervals[rec.nodeNum][rec.sensorNum].interval > 0)
                        {
                            sensorIntervals[rec.nodeNum][rec.sensorNum].countDown = randr(4, 31);
                            printf("Retry in %ds\n", sensorIntervals[rec.nodeNum][rec.sensorNum].countDown );
                        }
                    }
                    mutexIntervals.unlock();
                }
            }
            else if (rec.cmd == CMD_WRITE)
            {
                //we must check if the value to UART sensor was written with success
                //else we use rec.intervalErr as a MAX REPEAT number (must be set on command create)
                if ( writeUartSensorData(rec.nodeNum, rec.sensorNum, rec.sensorVal) < 0 ) //error writing, lets try it again
                {
                  if (rec.intervalErr > 0) //we have some more tryouts available
                  {
                      rec.intervalErr -= 1;
                      
                      THREAD_QUEUE_REC newRec;
                      newRec.nodeNum = rec.nodeNum;
                      newRec.sensorNum = rec.sensorNum;
                      newRec.sensorVal = rec.sensorVal;
                      newRec.intervalErr = rec.intervalErr;
                      
                      //add again to self-queue with decreased max-try-counter
                      mutexQueue.lock();
                      threadQueue.push(newRec);
                      mutexQueue.unlock();
                  }
                }
                else //success writing
                {
                    //we have to save/mirror new value to internal array
                    mutexValues.lock();
                    nodeValues[rec.nodeNum]->sensors[rec.sensorNum]->int_val = rec.sensorVal;
                    mutexValues.unlock();
                } 
            }
            else if (rec.cmd == CMD_REVEAL)
            {
                revealNodes();
                threadRevealFinished = true;
            }
        }
        else //queue empty
        {
            usleep(50000);
        }
    }
}

//=============================================================================
//==== Load and Save of: interval config, found nodes parameters ==============
//=============================================================================
int saveConfigFiles(void)
{
    FILE *fdIntervals, *fdParams = nullptr;
    bool openSucc = false;

    //try to open files for write
    fdIntervals = fopen("./intervals.cfg", "w");
    if (fdIntervals != NULL)
    {
        fdParams = fopen("./nodeparams.cfg", "w");
        openSucc = (fdParams != NULL) ? true : false;
        if (openSucc == false) fclose(fdIntervals);
    }
    
    if (openSucc == false)
    {
        printf("Cannot open data files for writing!\n");
        return -1;
    } 
    
    //write the nodes params file
    for (int i = 0; i < MAX_NODES; i++)
    {
        if (nodeValues[i] != NULL)
        {
            volatile NODE_VALUES_T* p = nodeValues[i];

            fprintf(fdParams, "%d:%d:%d:%d:%d:%d:%d:%d\r\n", p->node, p->num_sensors, p->sensor_types[0], p->sensor_types[1], p->sensor_types[2], p->sensor_types[3], p->sensor_types[4], p->sensor_types[5]);
            
            //save user defined names of sensors
            for (int x=0; x < p->num_sensors; x++)
            {
                fprintf(fdParams, "%s\r\n", p->sensor_names[x] );
            }
        }
    }
    
    //write intervals file
    for (int i = 0; i < MAX_NODES; i++)
    {
        if (sensorIntervals[i] != NULL)
        {
            volatile SENSOR_INTERVAL_REC *p = sensorIntervals[i];
            
            fprintf(fdIntervals, "%d", i); //start line with node num
            
            //if the intervals array pointer relating to node is not null
            //then it means, thah all sensors structures in this node has been allocated
            for (int x=0; x < MAX_SENSORS; x++)
            {
                fprintf(fdIntervals, ":%i,%i", (p[x]).interval, (p[x]).countDown);
            }
            
            fprintf(fdIntervals, "\r\n");
        }
    }
        
    //close both files
    fclose(fdIntervals);
    fclose(fdParams); 

    return 1;  
}

int readConfigFiles(void)
{
    FILE *fd = nullptr;
    const char *fname;
    
    //first read sensor params file
    //try if file exists and try to open for reading
    fname = "nodeparams.cfg";
    if( access( fname, F_OK ) != -1 )
    {
        fd = fopen(fname, "r");
        if (fd != NULL)
        {
            numNodes = 0;
            uchar listik[MAX_NODES];

            char *line = NULL;
            size_t L = 0;
            ssize_t read;
            
            while ((read = getline(&line, &L, fd)) != -1) {
                //count colons, if there is MAX_SENSORS+1 of them, then it is node line,
                //else it is error
                int colons = 0;
                for (int xxx=0; xxx < read; xxx++) { if (line[xxx] == ':') colons++; }
                if (colons == MAX_SENSORS+1)
                {
                    char **pole;
                    int poleLen;
                    poleLen = explode(line, ":", &pole);
                    int node = atoi(pole[0]);
                    int numS = atoi(pole[1]);
                    
                    nodeValues[node] = (NODE_VALUES_T*)malloc(sizeof(NODE_VALUES_T) );
                    memset((void*)nodeValues[node], 0, sizeof(NODE_VALUES_T) );
                    nodeValues[node]->node = node;
                    nodeValues[node]->num_sensors = numS;
                    
                    for (int xxx=0; xxx < numS; xxx++)
                    {
                        nodeValues[node]->sensor_types[xxx] = atoi(pole[xxx+2]);
                    }
                    
                    freeArrayOfPointers((void***)&pole, poleLen);
                    
                    nodeValues[node]->sensors = (volatile SENSOR_VAL_T**) malloc( sizeof(SENSOR_VAL_T*) * numS);  //alloc array of pointers to sensors values
                    for (int x = 0; x < numS; x++ ) //alloc sensor val structures
                    {
                        nodeValues[node]->sensors[x] = (SENSOR_VAL_T*) malloc( sizeof(SENSOR_VAL_T) );
                        memset((void*)nodeValues[node]->sensors[x], 255, sizeof(SENSOR_VAL_T) );
                    }
                    
                    nodeValues[node]->sensor_names = (volatile uchar**) malloc( sizeof(uchar*) * MAX_SENSORS); //alloc array of
                    memset((void*)nodeValues[node]->sensor_names, 0, sizeof(char*) * MAX_SENSORS);
                    for (int xxx=0; xxx < numS; xxx++)
                    {
                        size_t LL = 0;
                        ssize_t r = 0;
                        volatile uchar* volatile lin = NULL;
                        
                        r = getline((char**)&lin, &LL, fd);
                        //strip \r\n from end of line
                        lin[r-1] = 0;
                        lin[r-2] = 0;
                        
                        nodeValues[node]->sensor_names[xxx] = lin;
                        //do not free *lin because we want to *lin remain allocated in sensor names
                    }

                    listik[numNodes] = (uchar)node;
                    numNodes++;

                }
                
                free(line);
                line = NULL;
                L = 0;
            }
            fclose(fd);
            
            //copy only valid values from temporary list to normal list (clearing the original list before)
            if (numNodes > 0) //only if there are nodes
            {
                nodeList = (uchar *) malloc(numNodes);
                memcpy((void*)nodeList, listik, numNodes);
            }
        }
        else
        {
            printf("Cannot open %s for reading\n", fname);
        }
    }
    else
    {
        printf("File %s does not exist\n", fname);
    }
    
    //second read sensor intervals file
    //try if file exists and try to open for reading
    fname = "intervals.cfg";
    if( access( fname, F_OK ) != -1 )
    {
        fd = fopen(fname, "r");
        if (fd != NULL)
        {
            char *line = NULL;
            size_t L = 0;
            ssize_t read;
            
            while ((read = getline(&line, &L, fd)) != -1) {
                //count commas ",", if there is MAX_SENSORS of them, then it is node line,
                //else it is error
                int commas = 0;
                for (int xxx=0; xxx < read; xxx++) { if (line[xxx] == ',') commas++; }
                if (commas == MAX_SENSORS)
                {
                    char **pole;
                    int poleLen;
                    poleLen = explode(line, ":", &pole);
                    int node = atoi(pole[0]);

                    //node array is not initialised
                    sensorIntervals[node] = (SENSOR_INTERVAL_REC *) malloc(sizeof(SENSOR_INTERVAL_REC) * MAX_SENSORS);
                    memset((void*)sensorIntervals[node], 0, sizeof(SENSOR_INTERVAL_REC) * MAX_SENSORS);
                    
                    char **pole1;
                    int poleLen1;
                    
                    for (int x = 0; x < MAX_SENSORS; x++)
                    {
                        poleLen1 = explode(pole[x+1], ",", &pole1);
                        int interval = atoi(pole1[0]);
                        
                        //two possibilities - interval is 0 - reading disabled (dont push to vector)
                        //or bigger than 0 - interval enabled (push to vector)
                        if (interval > 0)
                        {
                            sensorIntervals[node][x].interval = interval;
                            sensorIntervals[node][x].countDown = 1;
                            //this 1 in countdown will invoke real read of value, because it was read never before
                            //and we dont wanna wait whole interval before the value will be available

                            //prepare new record to be pushed
                            SENSOR_INTERVAL_VECT_REC rec;
                            rec.nodeNum = node;
                            rec.sensorNum = x;
                            
                            //add record also to interval vector
                            intervalVect.push_back(rec);
                        }
                        else
                        {
                            sensorIntervals[node][x].interval = 0;
                            sensorIntervals[node][x].countDown = 0;
                        }
                        
                        freeArrayOfPointers((void***)&pole1, poleLen1);
                    }
                    
                    freeArrayOfPointers((void***)&pole, poleLen);
                }
            }
        }
        else
        {
            printf("Cannot open %s for reading\n", fname);
        }
    }
    else
    {
        printf("File %s does not exist\n", fname);
    }

    return 1;
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
            printf("Cannot open neither MAC nor Raspi UART port!\n");
            return -1;
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
    tcflush(fd, TCIOFLUSH);
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
            for (uchar x=0; x < node->num_sensors; x++) printf(" | %s", druhy_senzoru[ node->sensor_types[x] ]);
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
    
    printf("\n");
    //printf("Temp values of nodes:\n");
    int x = 0;
    while (x < 100) //infinite program loop
    {
        //x++;
        //numNodes = 2;
        //time_t cas = time(NULL) + 3600;
        //struct tm *tmi = localtime ( &cas );
        
//        char outbuf[250];
//        sprintf(outbuf, "%d;%d:%d:%d;", (unsigned int)cas, tmi->tm_hour, tmi->tm_min, tmi->tm_sec);
//        
//        printf("Reading %d / time:%d / %d:%d:%d [\n", x, (unsigned int)cas, tmi->tm_hour, tmi->tm_min, tmi->tm_sec);
//        for (i=0; i<numNodes; i++)
//        {
//            uchar rawData[4];
//            int rawLen;
//            uchar nodeNum = i+2; //node->node;
//            uchar sensorNum = 2;
//            //2 means sensor 2 (3rd - ds1820)
//            if (getSensorRawData(nodeNum, sensorNum, (uchar *)&rawData, &rawLen) > 0)
//            {
//                if ( (nodeValues[nodeNum] != NULL) && (nodeValues[nodeNum]->num_sensors >= sensorNum+1) )
//                {
//                    //choose proper sensor value handle function (according to sensor type) from array if pointers to functions
//                    (*countAndStoreSensorValue[nodeValues[nodeNum]->sensor_types[sensorNum] ])(nodeValues[nodeNum], sensorNum, (uchar*)&rawData, rawLen);
//                    
//                    double temp = nodeValues[nodeNum]->sensors[sensorNum]->float_val;
//                    printf("Node %d temp = %.1f\n", nodeNum,  temp );
//                    sprintf(outbuf, "%s%.1f;", outbuf, temp);
//                }
//            }
//            else
//            {
//                sprintf(outbuf, "%s;", outbuf);
//            }
//        }
//        
//        printf("]\n");
//        
//        //write measured temps to file
//        sprintf(outbuf, "%s\n", outbuf);
//        printf("%s", outbuf);
//        fprintf(out_fd, "%s", outbuf);
//        fflush(out_fd);
        
        for (int aa=0; aa < 30; aa++) //together with inner 1s cycle this should give in result 30s wait time
        {
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
            
            //repeatedly check input Sockets if there is some command
            for (int aaa=0; aaa <10; aaa++) //this cycle should take 1 sec
            {
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
                                uchar inBuff[255];
                                memset(inBuff, 0, sizeof(inBuff) );
                                ssize_t len = recv(ufds[ind].fd, inBuff, sizeof(inBuff), 0);
                                //TODO : if incoming message is bigger than inBuff size, read it repeatedly with realloc to read all data
                                if (len < 0)
                                {
                                    printf("Socket error %d\n", errno);
                                    
                                }
                                else if (len ==0) //socket closed
                                {
                                    printf("Socket Closed\n");
                                    close(ufds[ind].fd);
                                    removeUfd(ind); //remove this descriptor from queue
                                    if (ind < incoming_conns) continue; //without increasing index
                                }
                                else
                                {
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
                
                if (incoming_conns < MAX_CONNS)
                {
                    int acceptResult = accept(socketfd, NULL, NULL);
                    if (acceptResult > 0)
                    {
                        incoming_sd = acceptResult;
                        printf("Connection accepted. Using new socketfd : %d\n", incoming_sd);
                        fcntl(incoming_sd, F_SETOWN, getpid());
                        int flags = fcntl(incoming_sd, F_GETFL, 0);
                        fcntl(incoming_sd, F_SETFL, flags | O_NONBLOCK); //| O_ASYNC);
                        ufds[incoming_conns].fd = incoming_sd;
                        ufds[incoming_conns].events = POLLIN | POLLHUP;
                        send(incoming_sd, "[hello]\n", 8, 0);
                        incoming_conns++;
                    }
                }
                usleep(80000);
            } //end of 1s cycle (10 x 100ms)
        } //end of 30s cycle (30 x 1sec)
        
        //end of infinite program loop
        //tcflush(fd, TCIOFLUSH); //we can only flush in separate thread, because only it can access uart
    }
    
    //cleanup file descriptors etc.
    cleanup();
    return 0;
}
