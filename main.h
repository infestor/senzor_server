typedef unsigned char uchar;

typedef union {
  double float_val;
  uchar uchar_val;
  int int_val;
  unsigned int uint_val;
  uchar stream[sizeof(double)];  
} SENSOR_VAL_T;

typedef struct {
  uchar node;
  uchar num_sensors;
  uchar sensor_types[MAX_SENSORS]; //maximum number of sensors on one node
  volatile SENSOR_VAL_T* volatile *sensors; //pointer to array with values(union) size is num_sensors
  volatile uchar* volatile *sensor_names; //user defined names of particular sensors
} NODE_VALUES_T;

#define STORE_VALUE_PARAMS (volatile NODE_VALUES_T *nodeP, uchar sensorNum, uchar *rawData, int rawLen)
#define GET_VALUE_PARAMS (volatile NODE_VALUES_T *nodeP, uchar sensorNum, uchar **strBuf, int *strLen)

void freeArrayOfPointers(void ***pole, int poleLen);
void freeNodeValStruct(volatile NODE_VALUES_T* volatile *p);
void cleanup(void);

void countDS1820Temp STORE_VALUE_PARAMS;
void countInternalProcTemp STORE_VALUE_PARAMS;
void decideOnOffValue STORE_VALUE_PARAMS;
void countBatteryVoltOneCell STORE_VALUE_PARAMS;
void countBatteryVoltTwoCell STORE_VALUE_PARAMS;

void getFloatValStr GET_VALUE_PARAMS;
void getIntValStr GET_VALUE_PARAMS;
void getUintValStr GET_VALUE_PARAMS;
void getUcharValStr GET_VALUE_PARAMS;

int setup_uart(const char *port_name);
int transceiveData(uchar *paket);
int sendAndGetResponse(uchar *paket, uchar *response_buffer, unsigned int timeout = 1000, unsigned int repeats = 1);
int getSensorRawData(uchar nodeNum, uchar sensorNum, uchar *data, int *dataLen);
void my_ctrl_handler(int s);
int vytvoritSocket(void);
int removeUfd(uchar ind);
int explode(char *str, const char *delim, char ***r);
int appendToBuffer(uchar **buf, int *len, int *pos, uchar *msg, int msgLen);
int processSockCmd(uchar *inBuff, ssize_t bufLen, uchar **outBuf, int *outBufPos, int *outLen);
void revealNodes(void);

int saveConfigFiles(void);
int readConfigFiles(void);