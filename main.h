#ifndef __MAIN_H__
#define __MAIN_H__ 1

#include <sys/types.h>

#define MAX_NODES 10 //can be changed up to 255
#define MAX_SENSORS 6 //cannot be changed - hard value
#define MAX_SENSOR_NAME_LEN 30;
#define MAX_CONNS 15 //can be changed to higher number

//commands for thread
#define CMD_NONE 0
#define CMD_REVEAL 1
#define CMD_WRITE 2
#define CMD_READ 3
#define CMD_WRITE_CALIB 4
#define CMD_URGENT_READ 5

//types of sensoors
#define LOW_POWER_NODE_SIGN 128
#define VYSTUP_ZAP_VYP 0
#define TEPLOTA 1
#define DVERNI_SPINAC_ON_OFF 2
#define TEPLOTA_PROCESORU 3
#define TEPLOTA_DS1820 4
#define NAPETI_BATERIE_1_CLANEK 5
#define NAPETI_BATERIE_2_CLANKY 6
#define PWM_1_CHANNEL 7
#define PWM_3_CHANNEL 8

#define LOW_POWER_ALIVE_TIMEOUT 255

typedef unsigned char uchar;

typedef enum {
  RESERVED = 0,
  ACK = 1,
  ACK_RESP = 2,
  REQUEST = 3,
  RESPONSE = 4,
  PRESENTATION_REQUEST = 5,
  AUTOSEND = 6,
  RETRANSLATION = 7,
  CONFIGURATION = 8,
  PING = 9,
  PRESENTATION_RESPONSE = 10
} PACKET_TYPE;

//nrf message commands
typedef enum {
  READ = 0,
  WRITE = 1,
  WRITE_FIRST = 2,
  WRITE_MIDDLE = 3,
  WRITE_LAST = 4,
  CALIBRATION_WRITE = 5,
  CALIBRATION_READ = 6
} CMD_TYPE;

//for type REQUEST
typedef struct {
  uchar cmd; //command to be executed by receiving node
  uchar len;
  uchar for_sensor;
  uchar payload[4];
} payloadRequestStruct;


//for type RESPONSE (kupr odpoved na command READ)
typedef struct {
  uchar cmd; //executed command by receiving node (same as in request packet)
  uchar len;
  uchar from_sensor;
  uchar payload[4];
} payloadResponseStruct;

typedef union {
  payloadRequestStruct request_struct;
  payloadResponseStruct response_struct;
  uchar bytes[sizeof(payloadRequestStruct)];
} payloadStructUnion;

typedef struct {
  uchar txAddr;
  uchar rxAddr;
  uchar type;
  uchar counter;
  payloadStructUnion payload;
} mirfPacket;

typedef struct {
  uchar node;
  uchar num_sensors;
  uchar sensors[MAX_SENSORS];
} NodeSpecsT;

typedef struct {
  uchar value[4];
  uchar num_channels;
} PwmOneToFourBytesT;

typedef union {
  double float_val;
  uchar uchar_val;
  int int_val;
  unsigned int uint_val;
  uchar stream[sizeof(double)];
  PwmOneToFourBytesT pwm_vals;
} SENSOR_VAL_T;

typedef struct {
  uchar node;
  uchar num_sensors;
  uchar is_low_power;
  uchar low_power_alive;
  unsigned int low_power_voltage; //voltage sent along with low power DS18B20 value (it is stored as mV)
  uchar sensor_types[MAX_SENSORS]; //maximum number of sensors on one node
  volatile SENSOR_VAL_T* volatile *sensors; //pointer to array with values(union) size is num_sensors
  volatile SENSOR_VAL_T* volatile *last_valid_values; //pointer to array with pointers to last valid read values(union)
  volatile time_t sensor_read_times[MAX_SENSORS]; //timestamp of last valid reading from sensor
  volatile uchar* volatile *sensor_names; //user defined names of particular sensors
} NODE_VALUES_T;

typedef struct {
  uchar nodeNum;
  uchar sensorNum;
} SENSOR_INTERVAL_VECT_REC;

typedef struct {
  int interval;
  int countDown;
} SENSOR_INTERVAL_REC;

//this is for queue of UART data transfer separate thread
typedef struct {
  uchar cmd;
  uchar nodeNum;
  uchar sensorNum;
  int sensorVal;
  int intervalOk;
  int intervalErr;
} THREAD_QUEUE_REC;


//----------------- FUNCTION PROTOTYPES -------------------------------

#define STORE_VALUE_PARAMS (volatile NODE_VALUES_T *nodeP, uchar sensorNum, uchar *rawData, int rawLen)
#define GET_VALUE_PARAMS (volatile NODE_VALUES_T *nodeP, uchar sensorNum, uchar **strBuf, int *strLen)

//node_values.cpp - functions for handling sensor values
void countDS1820Temp STORE_VALUE_PARAMS;
void countInternalProcTemp STORE_VALUE_PARAMS;
void decideOnOffValue STORE_VALUE_PARAMS;
void countBatteryVoltOneCell STORE_VALUE_PARAMS;
void countBatteryVoltTwoCell STORE_VALUE_PARAMS;
void countLowPowerVcc STORE_VALUE_PARAMS;
void storePwmValsOneToFourBytes STORE_VALUE_PARAMS;

void getFloatValStr GET_VALUE_PARAMS;
void getIntValStr GET_VALUE_PARAMS;
void getUintValStr GET_VALUE_PARAMS;
void getUcharValStr GET_VALUE_PARAMS;
void getPwmValStr GET_VALUE_PARAMS;
void copySensorValueToLastValid(volatile NODE_VALUES_T* node, uchar sensorNum);

//defined and initialised in main.cpp
extern void (*countAndStoreSensorValue[]) STORE_VALUE_PARAMS;
extern void (*getSensorValStr[]) GET_VALUE_PARAMS;
int processSockCmd(uchar *inBuff, ssize_t bufLen, uchar **outBuf, int *outBufPos, int *outLen);

//support.cpp (suppport functions)
void freeArrayOfPointers(void ***pole, int poleLen);
void freeNodeValStruct(volatile NODE_VALUES_T* volatile *p);
int explode(char* str, const char* delim, char ***r);
int appendToBuffer(uchar **buf, int *len, int *pos, uchar *msg, int msgLen);
void cleanup(void);
void my_ctrl_handler(int s);
int removeUfd(uchar ind);
int vytvoritSocket(void);
unsigned int randr(unsigned int min, unsigned int max);

//uart.cpp
int setup_uart(const char *port_name);
//int transceiveData(uchar *paket);
int transmitData(uchar *paket);
int sendAndGetResponse(uchar *paket, uchar *response_buffer, unsigned int repeats = 1);
int getSensorRawData(uchar nodeNum, uchar sensorNum, uchar *data, int *dataLen);
int performUartValueReadAndSave(uchar nodeNum, uchar sensorNum);
int writeUartSensorData(uchar nodeNum, uchar sensorNum, int sensorData);
int writeUartSensorCalib(uchar nodeNum, uchar sensorNum, int calibData);
void revealNodes(void);

//cfg_files.cpp
int saveConfigFiles(void);
int readConfigFiles(void);

//process_sock_cmd.cpp
//main function to parse and respond to socket message
//unfortunately I did it as one veery big function
//so I put that in separate file not to mess main.cpp file
int processSockCmd(uchar *inBuff, ssize_t bufLen, uchar **outBuf, int *outBufPos, int *outLen);
bool parseCommandArguments(uchar *cmd, uchar cmdLen, uchar numParams, int *param1, int *param2, int *param3);

#endif //__MAIN_H__

