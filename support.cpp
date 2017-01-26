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

#include "main.h"

extern int uart_fd;
extern FILE *out_fd; //just for saving temperatures (now unused)
extern int socketfd;
extern int signalInt;
extern int incoming_conns;
extern pollfd ufds[MAX_CONNS];
extern volatile NODE_VALUES_T * volatile nodeValues[MAX_NODES];
extern volatile SENSOR_INTERVAL_REC * volatile sensorIntervals[MAX_NODES];

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

void cleanup(void)
{   
    close(uart_fd);
    fclose(out_fd);
    if (socketfd > 0) close(socketfd); //close listening socket
    for (int i = 0; i < incoming_conns; i++) //close all incoming sockets
    {
        close(ufds[i].fd);
    }
    incoming_conns = 0;

    //free stored sensor interval values (array is static, so just cycle through and set all values to NULL
    //previously there was used function freeArrayOfPointers but it cannot be used
    //since the array is static and it in fact does violation of memory access when it tries to free the array itself
    //freeArrayOfPointers((void***)&sensorIntervals, MAX_NODES);
    //and we will use for that the same cycle as for freeing node values because it also uses MAX_NODES as end
    
    //free stored node values
    for (int i = 0; i < MAX_NODES; i++)
    {
        freeNodeValStruct(&nodeValues[i]);
        free((void *)sensorIntervals[i]);
    }
}

void my_ctrl_handler(int s){
    printf("\n## Caught signal %d\n",s);
    tcflush(uart_fd, TCIOFLUSH);
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
        for (uchar pos = ind; pos <= (incoming_conns-2); pos++)
        {
            ufds[pos] = ufds[pos+1];
        }
    }
    
    incoming_conns--;
    return 1;
}

unsigned int randr(unsigned int min, unsigned int max)
{
    double scaled = (double)rand()/RAND_MAX;
    return (max - min +1)*scaled + min;
}
