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

const char str_port_name[] = "/dev/tty.wchusbserialfa130";
const char str_port_name_raspi[] = "/dev/ttyUSB0";
//const char port_name[] = "/dev/tty.wchusbserialfd120";
//const char port_name[] = "/dev/ttyAMA0";

const char *druhy_senzoru[] = { "Vystup ZAP/VYP", "Teplota", "Dverni spinac ON/OFF", "Teplota procesoru",
    "Teplota DS1820", "Napeti baterie 1 clanek LION", "Napeti baterie 2 clanky LION" };
const char low_power_str[] = "low power";

//const uchar sensorReturnLen[] = {1, 2, 1, 1, 2, 2, 2};
const uchar writableSensorTypes[] = {1, 0, 0, 0, 0, 0, 0};

volatile NODE_VALUES_T * volatile nodeValues[MAX_NODES];

volatile uchar *nodeList;

extern const double TempCorrections[] = {99, 99, 0, 0};  //4.5

volatile uchar numNodes = 0;

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
volatile SENSOR_INTERVAL_REC * volatile sensorIntervals[MAX_NODES];
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
volatile bool threadProcessingPriotityCommand = false; //this is used, when we need urgently send something over uart
volatile int  threadProcessingPriotityCommand_result;
volatile uchar urgentReadResponsePacket[11];
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------


//========================================================================        
//====== SOCKET related functions ========================================        
//========================================================================        
bool parseCommandArguments(uchar *cmd, uchar cmdLen, uchar numParams, int *param1 = NULL, int *param2 = NULL, int *param3 = NULL)
{
	bool chyba = false;

    //count colons (:) - must be numParams
    uchar colons = 0;
    for (int xxx=0; xxx < cmdLen; xxx++) { if (cmd[xxx] == ':') colons++; }
    if (colons != numParams) { chyba = true; }

    //check if all params are present (there is not :] together on the end)
    if (cmd[cmdLen-2] == ':') chyba = true;

    //strip command from [ and ], but preserve \0 on end (for strlen)
    uchar cmdStripped[cmdLen-1];
    cmdStripped[cmdLen-2] = 0;
    strncpy((char *)cmdStripped, (const char *)cmd+1, (size_t)cmdLen-2);

    if (chyba == false)
    {
        //explode cmd to values
        char** pole;
        int poleLen = explode((char *)cmdStripped, ":", &pole);

        //copy values into variables given by references
        if (numParams > 0) {
        	if (param1 != NULL) *param1 = atoi(pole[1]);
        }

        if (numParams > 1) {
        	if (param2 != NULL) *param2 = atoi(pole[2]);
        }

        if (numParams > 2) {
        	if (param3 != NULL) *param3 = atoi(pole[3]);
        }

        //free allocated memory
        freeArrayOfPointers((void***)&pole, poleLen);
    }

    return chyba;
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
            if ( gpos == bufLen) {
            	chyba = true;
            	break;
            }
        }
        
        if (chyba == true)
        {
            appendToBuffer(outBuf, outLen, outBufPos, (uchar*)"<[ERR-PARSE]>", 13);
            
            return -1;
        }
    }
    
    buff += pos; //move over previous gap
    
    //data OK, lets process it
    uchar cmd[30];
    pos = 0;
    memset(cmd, 0, sizeof(cmd) );
    while (buff[pos] != ']')
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
    	uchar nodeNum;
    	uchar nodeSens;

    	chyba = parseCommandArguments(cmd, cmdLen, 2, (int*)&nodeNum, (int*)&nodeSens, NULL);
        
    	if (chyba == false) {
            //is the request for existing Node and sensor?
            if ( (nodeValues[nodeNum] == NULL) || (nodeValues[nodeNum]->num_sensors-1 < nodeSens) ) chyba = true;
    	}

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
    			uchar sensType = nodeValues[nodeNum]->sensor_types[nodeSens];
    			if (sensType >= LOW_POWER_NODE_SIGN) sensType -= LOW_POWER_NODE_SIGN;				
				(*getSensorValStr[sensType ])(nodeValues[nodeNum], nodeSens, &sensorValStr, &sensorValStrLen);
				mutexValues.unlock();
			}
			else //if the value vas never read from sensor, return X instead of value
			{
				sensorValStrLen = asprintf((char**)&sensorValStr, "x");
			}

			L = asprintf((char**)&outMsg, "<[OK]%s[%s]>", cmd, sensorValStr);
			free(sensorValStr);
			appendToBuffer(outBuf, outLen, outBufPos, outMsg, L);
			free(outMsg);
			printf(">OK: %s\n", cmd);
		}
        //if chyba was true, print [ERR] tag at the end - common for all commands

    }
    //=== getNodeProp - get node properties/presentation [getNodeProp:nodeNum]
    else if (strncmp((const char*)cmd, "[getNodeProp:", 13) == 0)
    {
        uchar nodeNum;
        
        chyba = parseCommandArguments(cmd, cmdLen, 1, (int*)&nodeNum, NULL, NULL);
        
        if (chyba == false)
        {
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
        //if chyba was true, print [ERR] tag at the end - common for all commands

    }
    //=== setVal - set value of sensor which is writeable [setSensVal:nodeNum:sensorNum:value]
    //TODO: so far there is no sensor value type handling - because the only sensor supporting
    //writes is ON/OFF switch and there is used unsigned int(char) type
    else if (strncmp((const char*)cmd, "[setVal:", 8) == 0)
    {
        uchar nodeNum;
        uchar sensNum;
        uchar sensVal;

        chyba = parseCommandArguments(cmd, cmdLen, 3, (int*)&nodeNum, (int*)&sensNum, (int*)&sensVal);
        
        if (chyba == false)
        {
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
        //if chyba was true, print [ERR] tag at the end - common for all commands

    }
    //=== setInterval - set interval of sensor reading [setInterval:nodeNum:sensorNum:value]
    else if (strncmp((const char*)cmd, "[setInterval:", 13) == 0)
    {
        uchar nodeNum;
        uchar sensNum;
        int sensVal;
        
        chyba = parseCommandArguments(cmd, cmdLen, 3, (int*)&nodeNum, (int*)&sensNum, &sensVal);
        
        if (chyba == false)
        {        
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
                //set new interval value
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
        //if chyba was true, print [ERR] tag at the end - common for all commands

    }
    //=== getInterval - gets interval of sensor reading [getInterval:nodeNum:sensorNum]
    else if (strncmp((const char*)cmd, "[getInterval:", 13) == 0)
    {
        uchar nodeNum;
        uchar sensNum;

        chyba = parseCommandArguments(cmd, cmdLen, 2, (int*)&nodeNum, (int*)&sensNum, NULL);
        
        if (chyba == false)
        {                
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
        //if chyba was true, print [ERR] tag at the end - common for all commands

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
    //===== [setCalib:node:sensor:calib]  - will write calibration value of internal temp sensor
    else if (strncmp((const char*)cmd, "[setCalib:", 10) == 0)
    {

        volatile uchar nodeNum;
        volatile uchar sensNum;
        volatile uchar newCalib;
        
        chyba = parseCommandArguments(cmd, cmdLen, 3, (int*)&nodeNum, (int*)&sensNum, (int*)&newCalib);

        if (chyba == false)
        {
            //is the request for existing Node?
            if (nodeValues[nodeNum] == NULL) chyba = true;
            //sensor on node exists?
            else if (nodeValues[nodeNum]->num_sensors < sensNum+1) chyba = true;
            //is sensor type = internal temp sensor?
            else if (nodeValues[nodeNum]->sensor_types[sensNum] != TEPLOTA_PROCESORU) chyba = true;
        }
        
        int L;
        uchar *outMsg;
        if (chyba == false)
        {
            //create and push message/command to thread queue
            THREAD_QUEUE_REC rec;
            rec.cmd = CMD_WRITE_CALIB;
            rec.nodeNum = nodeNum;
            rec.sensorNum = sensNum;
            rec.sensorVal = newCalib;  //calibration value is stored to SensorVal field of thread record
            rec.intervalErr = 5;
            
            mutexQueue.lock();
            threadQueue.push(rec);
            mutexQueue.unlock();
            
            L = asprintf((char**)&outMsg, "<[OK]%s>", cmd);
            appendToBuffer(outBuf, outLen, outBufPos, outMsg, L);
            printf(">OK: %s, %d\n", cmd, newCalib);
            free(outMsg);
        }
        //if chyba was true, print [ERR] tag at the end - common for all commands
    }
    //===== [getCalib:node:sensor]  - will read calibration value of internal temp sensor
    else if (strncmp((const char*)cmd, "[getCalib:", 10) == 0)
    {
        uchar nodeNum;
        uchar sensNum ;

        chyba = parseCommandArguments(cmd, cmdLen, 2, (int*)&nodeNum, (int*)&sensNum, NULL);
        
        if (chyba == false)
        {
            //is the request for existing Node?
            if (nodeValues[nodeNum] == NULL) chyba = true;
            //sensor on node exists?
            else if (nodeValues[nodeNum]->num_sensors < sensNum+1) chyba = true;
            //is sensor type = internal temp sensor?
            else if (nodeValues[nodeNum]->sensor_types[sensNum] != TEPLOTA_PROCESORU) chyba = true;
        }
        
        int L;
        uchar *outMsg;
        if (chyba == false)
        {
            //create and push message/command to thread queue
            THREAD_QUEUE_REC rec;
            rec.cmd = CMD_URGENT_READ;
            rec.nodeNum = nodeNum;
            rec.sensorNum = sensNum;
            rec.sensorVal = CALIBRATION_READ;  //command for CALIBRATION_READ (its value) is stored to SensorVal field of thread record
            rec.intervalErr = 4;
            
            mutexQueue.lock();
            threadQueue.push(rec);
            threadProcessingPriotityCommand = true;
            mutexQueue.unlock();
            
            while (threadProcessingPriotityCommand == true) {}
            if (threadProcessingPriotityCommand_result == 1)
            {
                
                L = asprintf((char**)&outMsg, "<[OK]%s[%d]>", cmd, urgentReadResponsePacket[7]);
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
        //if chyba was true, print [ERR] tag at the end - common for all commands

    }        
    //=== any other unknown command is error
    else
    {
        uchar outMsg[cmdLen+13+1];
        int L = sprintf((char*)&outMsg, "<[ERR-UNKNOWN]%s>", cmd);
        
        appendToBuffer(outBuf, outLen, outBufPos, outMsg, L);
    }

    //if chyba was true, print [ERR] tag at the end - common for all commands
    if (chyba == true) {
		appendToBuffer(outBuf, outLen, outBufPos, (uchar*)"<[ERR]", 6);
		appendToBuffer(outBuf, outLen, outBufPos, cmd, cmdLen);
		appendToBuffer(outBuf, outLen, outBufPos, (uchar*)">", 1);
		printf(">ERR: %s\n", cmd);
    }

    //process iteratively if there is something else unprocessed in input buffer
    //...
    if (gpos+1 < bufLen) processSockCmd(inBuff+gpos+1, bufLen-(gpos+1), outBuf, outBufPos, outLen);
    
    return 1;
}



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
                if ( (performUartValueReadAndSave(rec.nodeNum, rec.sensorNum) < 0) ) //&& (rec.intervalOk > 60)
                {
                    //decide if normal node or low power node
                    //for normal node choose random new interval
                    //for low power node just decrease its alive interval by value in REC.INTERVAL (!! should be always 1sec for this type of node)
                    if (nodeValues[rec.nodeNum]->is_low_power == 0)
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
                    else
                    {
                        //low power device and sensor - decrease alive interval
                        if ( nodeValues[rec.nodeNum]->sensor_types[rec.sensorNum] >= LOW_POWER_NODE_SIGN)
                        {
							int temp_alive = nodeValues[rec.nodeNum]->low_power_alive;
							int temp_interval = sensorIntervals[rec.nodeNum][rec.sensorNum].interval;
						
							if (temp_interval > temp_alive)
							{
								temp_alive = 0;
							}
							else
							{
								temp_alive -= temp_interval;
							}
							nodeValues[rec.nodeNum]->low_power_alive = temp_alive;
                        }
                    }
                }
                else
                {
                    //after success read - refresh the low_power alive interval (if it is low power node)
                    if (nodeValues[rec.nodeNum]->is_low_power == 1) nodeValues[rec.nodeNum]->low_power_alive = LOW_POWER_ALIVE_TIMEOUT;
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
            else if (rec.cmd == CMD_WRITE_CALIB)
            {
                //we must check if the value to UART sensor was written with success
                //else we use rec.intervalErr as a MAX REPEAT number (must be set on command create)
                if ( writeUartSensorCalib(rec.nodeNum, rec.sensorNum, rec.sensorVal) < 0 ) //error writing, lets try it again
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
                //no need to do anything on success writing
            }
             else if (rec.cmd == CMD_URGENT_READ)
            {
                uchar sensor_read_packet[12] = {254, 1, 99, 3, 255, 0, 1, 99, 0, 0, 0, 0};

                sensor_read_packet[2] = rec.nodeNum;
                sensor_read_packet[5] = rec.sensorVal; //command for request
                sensor_read_packet[7] = rec.sensorNum;

                if (sendAndGetResponse(sensor_read_packet, (uchar*)urgentReadResponsePacket) >= 0)
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
                    printf(" | %s (%s)", druhy_senzoru[ node->sensor_types[x]-LOW_POWER_NODE_SIGN ], low_power_str);
                }
                else
                {
                    printf(" | %s", druhy_senzoru[ node->sensor_types[x] ]);
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
    
    printf("Starting background THREAD for decreasing countdowns in queue\n");
    std::thread threadCOUNTDOWN(threadDecreaseIntervals);
	usleep(50000);
    printf("Starting background THREAD for handling UART comm\n");
    std::thread threadUART(threadProcessQueue);
        
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
        
        //accepting new socket connections (if any)
        if (incoming_conns < MAX_CONNS)
        {
            int acceptResult = accept(socketfd, NULL, NULL);
            if (acceptResult > 0)
            {
                int incoming_sd;
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
        
        usleep(50000);
        
        //end of infinite program loop
        //tcflush(uart_fd, TCIOFLUSH); //we can only flush in separate thread, because only it can access uart
    }
    
    //cleanup file descriptors etc.
    cleanup();
    return 0;
}
