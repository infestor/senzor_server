#include <iostream>
#include <vector>
#include <queue>
#include <mutex>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <limits.h>

#include "main.h"

extern volatile NODE_VALUES_T * volatile nodeValues[MAX_NODES];
extern volatile SENSOR_INTERVAL_REC * volatile sensorIntervals[MAX_NODES];
extern std::vector<SENSOR_INTERVAL_VECT_REC> intervalVect;
extern volatile uchar *nodeList;
extern volatile uchar numNodes;
extern volatile bool threadRevealFinished;
extern volatile bool threadProcessingPriotityCommand;
extern volatile int  threadProcessingPriotityCommand_result;
extern volatile uchar urgentReadResponsePacket[11];

extern std::mutex mutexValues;
extern std::mutex mutexIntervals;
extern std::mutex mutexQueue;
extern std::queue<THREAD_QUEUE_REC> threadQueue;

const uchar writableSensorTypes[] = {1, 0, 0, 0, 0, 0, 0};


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


//************************************************************************************************
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
    //=== getVcc command [getVal:nodeNum] (must be low power node)
    else if (strncmp((const char*)cmd, "[getVcc:", 8) == 0)
    {
    	uchar nodeNum;

    	chyba = parseCommandArguments(cmd, cmdLen, 1, (int*)&nodeNum, NULL, NULL);

    	if (chyba == false) {
            //is the request for existing Node?
            if ( (nodeValues[nodeNum] == NULL) || (nodeValues[nodeNum]->is_low_power == 0) ) chyba = true;
    	}

		uchar *outMsg;
		int L;
		if (chyba == false)
		{
			uchar *sensorValStr;
			int sensorValStrLen;

			//decide if the value vas ever read from real sensor
			//bcause on initialisation the union of value is filled by 0xFF
			if (nodeValues[nodeNum]->low_power_voltage != UINT_MAX)
			{
				//use right type of sensor value (float or int..)!!
				mutexValues.lock();
				sensorValStrLen = asprintf( (char**)&sensorValStr, "%d", nodeValues[nodeNum]->low_power_voltage );
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
