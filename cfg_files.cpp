#include <iostream>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "main.h"

extern volatile NODE_VALUES_T * volatile nodeValues[MAX_NODES];
extern volatile SENSOR_INTERVAL_REC * volatile sensorIntervals[MAX_NODES];
extern volatile uchar *nodeList;
extern volatile uchar numNodes;
extern std::vector<SENSOR_INTERVAL_VECT_REC> intervalVect;

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