#include <Adafruit_MCP3008.h>
#include <FlexCAN_T4.h>
#include <Wire.h>
#include <elapsedMillis.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

Adafruit_MCP3008 ADCs[8];

#define Reference 2.5
#define ERROR_TIME 700

#define N_ADCs 8
#define N_ADC_CHANNELS 8

// #define BMS_CAN_PERIOD 250
// #define BROADCAST_PERIOD 250
#define BROADCAST_ID 0x301

CAN_message_t BMSInfoMsg;
CAN_message_t tempBroadcast;
CAN_message_t msg_1;
CAN_message_t msg_error;

bool TempErr = 0;
bool BMSErr = 0;

// elapsedMillis timeSinceLastBroadcast = 0;
// elapsedMillis timeSinceLastBMSMessage = 0;

int broadcastIndex = 0;
int broadcastEnabled = 0;
int count = 0;

float read = 0;
double voltage = 0;
double voltage1 = 0;
double voltage2 = 0;
double voltage3 = 0;
double voltage4 = 0;
double temperature = 0.0;
float maxRaw = 0.0;
float minRaw = 60.0;
float rawSum = 0.0;
float AvgRaw = 0.0;

float maxTemp = 0.0;
float minTemp = 60.0;
float sumTemp = 0.0;
float avgTemp = 0.0;


int ADCRaw[8][8] = {
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0}};

float Temps[8][8] = {
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

int TimeTemp[8][8];

elapsedMillis timer_delay;

double ADCconversion(int raw) {
    voltage = (raw * Reference) / 1024.0;
    voltage2 = voltage * voltage; 
    voltage3 = voltage2 * voltage;
    voltage4 = voltage3 * voltage;

    //reg Lucca
    //temperature = 2875.88397 - 5512.867802 * voltage + 4082.002758 * voltage2 - 1358.200746 * voltage3 + 168.841073 * voltage4;

    //New REG
    temperature = 2520.5 - 4738.1 * voltage + 3452.1 * voltage2 - 1131.7 * voltage3 + 138.46 * voltage4;
     
    return temperature;
}

// write a function to read all the ADC values
void readRawADCData() {
    for (int adc = 0; adc < N_ADCs; adc++) {
        for (int channel = 0; channel < N_ADC_CHANNELS; channel++) {
            if (adc == 7 && channel > 3)
                continue;
            if (adc == 3 && channel == 5) //this termistor is disconnected
                continue;
            if (adc == 2 && channel == 2) //this termistor is disconnected
                continue;
            if (adc == 4 && channel == 1) //this termistor is disconnected
                continue;
            ADCRaw[adc][channel] = ADCs[adc].readADC(channel);
            Temps[adc][channel] = ADCconversion(ADCRaw[adc][channel]);
            if(Temps[adc][channel]>70 || Temps[adc][channel]<0)
                continue;
            else
                TimeTemp[adc][channel]=millis();
            minRaw = min(minRaw, ADCRaw[adc][channel]);
            maxRaw = max(maxRaw, ADCRaw[adc][channel]);
            rawSum += ADCRaw[adc][channel];
        }
    }
    AvgRaw = rawSum / (N_ADCs * N_ADC_CHANNELS - 4);  // Only 60 ADC Channels are usable
}

void broadcastRawData() {
    tempBroadcast.id = BROADCAST_ID + broadcastIndex;  // para decidir
    tempBroadcast.len = N_ADC_CHANNELS + 1;
    for (int i = 0; i < N_ADCs; i++)
        tempBroadcast.buf[i] = (uint8_t)ADCRaw[broadcastIndex][i];

    can1.write(tempBroadcast);
    broadcastIndex = (broadcastIndex + 1) % N_ADCs;
}

void CAN_msg() {
    msg_1.id = 0x301;  // para decidir
    msg_1.len = 8;

    msg_1.buf[0] = 0;
    msg_1.buf[1] = (uint8_t)ADCconversion(ADCRaw[0][0]);
    msg_1.buf[2] = (uint8_t)ADCconversion(ADCRaw[0][1]);
    msg_1.buf[3] = (uint8_t)ADCconversion(ADCRaw[0][2]);
    msg_1.buf[4] = (uint8_t)ADCconversion(ADCRaw[0][3]);
    msg_1.buf[5] = (uint8_t)ADCconversion(ADCRaw[0][4]);
    msg_1.buf[6] = (uint8_t)ADCconversion(ADCRaw[0][5]);
    msg_1.buf[7] = (uint8_t)ADCconversion(ADCRaw[0][6]);
    can1.write(msg_1);

    msg_1.buf[0] = 1;
    msg_1.buf[1] = (uint8_t)ADCconversion(ADCRaw[0][7]);
    msg_1.buf[2] = (uint8_t)ADCconversion(ADCRaw[1][0]);
    msg_1.buf[3] = (uint8_t)ADCconversion(ADCRaw[1][1]);
    msg_1.buf[4] = (uint8_t)ADCconversion(ADCRaw[1][2]);
    msg_1.buf[5] = (uint8_t)ADCconversion(ADCRaw[1][3]);
    msg_1.buf[6] = (uint8_t)ADCconversion(ADCRaw[1][4]);
    msg_1.buf[7] = (uint8_t)ADCconversion(ADCRaw[1][5]);
    can1.write(msg_1);

    msg_1.buf[0] = 2;
    msg_1.buf[1] = (uint8_t)ADCconversion(ADCRaw[1][6]);
    msg_1.buf[2] = (uint8_t)ADCconversion(ADCRaw[1][7]);
    msg_1.buf[3] = (uint8_t)ADCconversion(ADCRaw[2][0]);
    msg_1.buf[4] = (uint8_t)ADCconversion(ADCRaw[2][1]);
    msg_1.buf[5] = (uint8_t)ADCconversion(ADCRaw[2][1]);
    msg_1.buf[6] = (uint8_t)ADCconversion(ADCRaw[2][3]);
    msg_1.buf[7] = (uint8_t)ADCconversion(ADCRaw[2][4]);
    can1.write(msg_1);

    msg_1.buf[0] = 3;
    msg_1.buf[1] = (uint8_t)ADCconversion(ADCRaw[2][5]);
    msg_1.buf[2] = (uint8_t)ADCconversion(ADCRaw[2][6]);
    msg_1.buf[3] = (uint8_t)ADCconversion(ADCRaw[2][7]);
    msg_1.buf[4] = (uint8_t)ADCconversion(ADCRaw[3][0]);
    msg_1.buf[5] = (uint8_t)ADCconversion(ADCRaw[3][1]);
    msg_1.buf[6] = (uint8_t)ADCconversion(ADCRaw[3][2]);
    msg_1.buf[7] = (uint8_t)ADCconversion(ADCRaw[3][3]);
    can1.write(msg_1);

    msg_1.buf[0] = 4;
    msg_1.buf[1] = (uint8_t)ADCconversion(ADCRaw[3][4]);
    msg_1.buf[2] = (uint8_t)ADCconversion(ADCRaw[3][4]);
    msg_1.buf[3] = (uint8_t)ADCconversion(ADCRaw[3][6]);
    msg_1.buf[4] = (uint8_t)ADCconversion(ADCRaw[3][7]);
    msg_1.buf[5] = (uint8_t)ADCconversion(ADCRaw[4][0]);
    msg_1.buf[6] = (uint8_t)ADCconversion(ADCRaw[4][2]);
    msg_1.buf[7] = (uint8_t)ADCconversion(ADCRaw[4][2]);
    can1.write(msg_1);

    msg_1.buf[0] = 5;
    msg_1.buf[1] = (uint8_t)ADCconversion(ADCRaw[4][3]);
    msg_1.buf[2] = (uint8_t)ADCconversion(ADCRaw[4][4]);
    msg_1.buf[3] = (uint8_t)ADCconversion(ADCRaw[4][5]);
    msg_1.buf[4] = (uint8_t)ADCconversion(ADCRaw[4][6]);
    msg_1.buf[5] = (uint8_t)ADCconversion(ADCRaw[4][7]);
    msg_1.buf[6] = (uint8_t)ADCconversion(ADCRaw[5][0]);
    msg_1.buf[7] = (uint8_t)ADCconversion(ADCRaw[5][1]);
    can1.write(msg_1);

    msg_1.buf[0] = 6;
    msg_1.buf[1] = (uint8_t)ADCconversion(ADCRaw[5][2]);
    msg_1.buf[2] = (uint8_t)ADCconversion(ADCRaw[5][3]);
    msg_1.buf[3] = (uint8_t)ADCconversion(ADCRaw[5][4]);
    msg_1.buf[4] = (uint8_t)ADCconversion(ADCRaw[5][5]);
    msg_1.buf[5] = (uint8_t)ADCconversion(ADCRaw[5][6]);
    msg_1.buf[6] = (uint8_t)ADCconversion(ADCRaw[5][7]);
    msg_1.buf[7] = (uint8_t)ADCconversion(ADCRaw[6][0]);
    can1.write(msg_1);

    msg_1.buf[0] = 7;
    msg_1.buf[1] = (uint8_t)ADCconversion(ADCRaw[6][1]);
    msg_1.buf[2] = (uint8_t)ADCconversion(ADCRaw[6][2]);
    msg_1.buf[3] = (uint8_t)ADCconversion(ADCRaw[6][3]);
    msg_1.buf[4] = (uint8_t)ADCconversion(ADCRaw[6][4]);
    msg_1.buf[5] = (uint8_t)ADCconversion(ADCRaw[6][5]);
    msg_1.buf[6] = (uint8_t)ADCconversion(ADCRaw[6][6]);
    msg_1.buf[7] = (uint8_t)ADCconversion(ADCRaw[6][7]);
    can1.write(msg_1);

    msg_1.buf[0] = 8;
    msg_1.buf[1] = (uint8_t)ADCconversion(ADCRaw[7][0]);
    msg_1.buf[2] = (uint8_t)ADCconversion(ADCRaw[7][1]);
    msg_1.buf[3] = (uint8_t)ADCconversion(ADCRaw[7][2]);
    msg_1.buf[4] = (uint8_t)ADCconversion(ADCRaw[7][3]);
    can1.write(msg_1);
}

void canbusSniffer(const CAN_message_t& msg) {
    if (msg.id == 0x300)
        broadcastEnabled = 1;

    if(msg.id == 0x270){
        BMSErr = msg.buf[0]; //atualiza falg erro BMS
    }
}

void printdebug(){
    /*
    Serial.printf("--Segmento 1--\n");
    Serial.printf("Cell 1:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[6][2],((ADCRaw[6][2] * Reference) / 1024.0),ADCconversion(ADCRaw[6][2]));    
    Serial.printf("Cell 2:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[6][3],((ADCRaw[6][3] * Reference) / 1024.0),ADCconversion(ADCRaw[6][3]));
    Serial.printf("Cell 3:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[6][4],((ADCRaw[6][4] * Reference) / 1024.0),ADCconversion(ADCRaw[6][4]));
    Serial.printf("Cell 4:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[6][5],((ADCRaw[6][5] * Reference) / 1024.0),ADCconversion(ADCRaw[6][5]));
    Serial.printf("Cell 5:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[6][6],((ADCRaw[6][6] * Reference) / 1024.0),ADCconversion(ADCRaw[6][6]));
    Serial.printf("Cell 6:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[6][7],((ADCRaw[6][7] * Reference) / 1024.0),ADCconversion(ADCRaw[6][7]));
    Serial.printf("Cell 7:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[7][0],((ADCRaw[7][0] * Reference) / 1024.0),ADCconversion(ADCRaw[7][0]));
    Serial.printf("Cell 8:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[7][1],((ADCRaw[7][1] * Reference) / 1024.0),ADCconversion(ADCRaw[7][1]));
    Serial.printf("Cell 9:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[7][2],((ADCRaw[7][2] * Reference) / 1024.0),ADCconversion(ADCRaw[7][2]));
    Serial.printf("Cell 10: Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[7][3],((ADCRaw[7][3] * Reference) / 1024.0),ADCconversion(ADCRaw[7][3]));
    */

    /*
    Serial.printf("--Segmento 2--\n");
    Serial.printf("Cell 1: Bit= %d  | Voltage= %.2f | Temp=%.2f \n",ADCRaw[5][0],((ADCRaw[5][0] * Reference) / 1024.0),ADCconversion(ADCRaw[5][0]));    
    Serial.printf("Cell 2:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[5][1],((ADCRaw[5][1] * Reference) / 1024.0),ADCconversion(ADCRaw[5][1]));
    Serial.printf("Cell 3:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[5][2],((ADCRaw[5][2] * Reference) / 1024.0),ADCconversion(ADCRaw[5][2]));
    Serial.printf("Cell 4:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[5][3],((ADCRaw[5][3] * Reference) / 1024.0),ADCconversion(ADCRaw[5][3]));
    Serial.printf("Cell 5:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[5][4],((ADCRaw[5][4] * Reference) / 1024.0),ADCconversion(ADCRaw[5][4]));
    Serial.printf("Cell 6:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[5][5],((ADCRaw[5][5] * Reference) / 1024.0),ADCconversion(ADCRaw[5][5]));
    Serial.printf("Cell 7:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[5][6],((ADCRaw[5][6] * Reference) / 1024.0),ADCconversion(ADCRaw[5][6]));
    Serial.printf("Cell 8:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[5][7],((ADCRaw[5][7] * Reference) / 1024.0),ADCconversion(ADCRaw[5][7]));
    Serial.printf("Cell 9:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[6][0],((ADCRaw[6][0] * Reference) / 1024.0),ADCconversion(ADCRaw[6][0]));
    Serial.printf("Cell 10: Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[6][1],((ADCRaw[6][1] * Reference) / 1024.0),ADCconversion(ADCRaw[6][1]));
    */

    /*
    Serial.printf("--Segmento 3--\n");
    Serial.printf("Cell 1:  Bit= %d  | Voltage= %.2f | Temp=%.2f \n",ADCRaw[3][6],((ADCRaw[3][6] * Reference) / 1024.0),ADCconversion(ADCRaw[3][6])); 
    Serial.printf("Cell 2:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[3][7],((ADCRaw[3][7] * Reference) / 1024.0),ADCconversion(ADCRaw[3][7]));
    Serial.printf("Cell 3:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[4][0],((ADCRaw[4][0] * Reference) / 1024.0),ADCconversion(ADCRaw[4][0]));
    Serial.printf("Cell 4:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[4][1],((ADCRaw[4][1] * Reference) / 1024.0),ADCconversion(ADCRaw[4][1]));
    Serial.printf("Cell 5:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[4][2],((ADCRaw[4][2] * Reference) / 1024.0),ADCconversion(ADCRaw[4][2]));
    Serial.printf("Cell 6:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[4][3],((ADCRaw[4][3] * Reference) / 1024.0),ADCconversion(ADCRaw[4][3]));
    Serial.printf("Cell 7:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[4][4],((ADCRaw[4][4] * Reference) / 1024.0),ADCconversion(ADCRaw[4][4]));
    Serial.printf("Cell 8:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[4][5],((ADCRaw[4][5] * Reference) / 1024.0),ADCconversion(ADCRaw[4][5]));
    Serial.printf("Cell 9:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[4][6],((ADCRaw[4][6] * Reference) / 1024.0),ADCconversion(ADCRaw[4][6]));
    Serial.printf("Cell 10: Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[4][7],((ADCRaw[4][7] * Reference) / 1024.0),ADCconversion(ADCRaw[4][7]));
    */

    /*
    Serial.printf("--Segmento 4--\n");
    Serial.printf("Cell 1:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[2][4],((ADCRaw[2][4] * Reference) / 1024.0),ADCconversion(ADCRaw[2][4])); 
    Serial.printf("Cell 2:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[2][5],((ADCRaw[2][5] * Reference) / 1024.0),ADCconversion(ADCRaw[2][5]));
    Serial.printf("Cell 3:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[2][6],((ADCRaw[2][6] * Reference) / 1024.0),ADCconversion(ADCRaw[2][6]));
    Serial.printf("Cell 4:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[2][7],((ADCRaw[2][7] * Reference) / 1024.0),ADCconversion(ADCRaw[2][7]));
    Serial.printf("Cell 5:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[3][0],((ADCRaw[3][0] * Reference) / 1024.0),ADCconversion(ADCRaw[3][0]));
    Serial.printf("Cell 6:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[3][1],((ADCRaw[3][1] * Reference) / 1024.0),ADCconversion(ADCRaw[3][1]));
    Serial.printf("Cell 7:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[3][2],((ADCRaw[3][2] * Reference) / 1024.0),ADCconversion(ADCRaw[3][2]));
    Serial.printf("Cell 8:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[3][3],((ADCRaw[3][3] * Reference) / 1024.0),ADCconversion(ADCRaw[3][3]));
    Serial.printf("Cell 9:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[3][4],((ADCRaw[3][4] * Reference) / 1024.0),ADCconversion(ADCRaw[3][4]));
    //Serial.printf("Cell 10: Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[3][5],((ADCRaw[3][5] * Reference) / 1024.0),ADCconversion(ADCRaw[3][5]));
    Serial.printf("Cell 10: thermistor turned off\n");
    */
    

    /*
    Serial.printf("--Segmento 5--\n");
    Serial.printf("Cell 1: Bit= %d  | Voltage= %.2f | Temp=%.2f \n",ADCRaw[1][2],((ADCRaw[1][2] * Reference) / 1024.0),ADCconversion(ADCRaw[1][2])); 
    Serial.printf("Cell 2:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[1][3],((ADCRaw[1][3] * Reference) / 1024.0),ADCconversion(ADCRaw[1][3]));
    Serial.printf("Cell 3:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[1][4],((ADCRaw[1][4] * Reference) / 1024.0),ADCconversion(ADCRaw[1][4]));
    Serial.printf("Cell 4:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[1][5],((ADCRaw[1][5] * Reference) / 1024.0),ADCconversion(ADCRaw[1][5]));
    Serial.printf("Cell 5:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[1][6],((ADCRaw[1][6] * Reference) / 1024.0),ADCconversion(ADCRaw[1][6]));
    Serial.printf("Cell 6:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[1][7],((ADCRaw[1][7] * Reference) / 1024.0),ADCconversion(ADCRaw[1][7]));
    Serial.printf("Cell 7:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[2][0],((ADCRaw[2][0] * Reference) / 1024.0),ADCconversion(ADCRaw[2][0]));
    Serial.printf("Cell 8:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[2][1],((ADCRaw[2][1] * Reference) / 1024.0),ADCconversion(ADCRaw[2][1]));
    Serial.printf("Cell 9:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[2][2],((ADCRaw[2][2] * Reference) / 1024.0),ADCconversion(ADCRaw[2][2]));
    Serial.printf("Cell 10: Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[2][3],((ADCRaw[2][3] * Reference) / 1024.0),ADCconversion(ADCRaw[2][3]));
    */
    /*
    Serial.printf("--Segmento 6--\n");
    Serial.printf("Cell 1:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[0][0],((ADCRaw[0][0] * Reference) / 1024.0),ADCconversion(ADCRaw[0][0])); 
    Serial.printf("Cell 2:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[0][1],((ADCRaw[0][1] * Reference) / 1024.0),ADCconversion(ADCRaw[0][1]));
    Serial.printf("Cell 3:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[0][2],((ADCRaw[0][2] * Reference) / 1024.0),ADCconversion(ADCRaw[0][2]));
    Serial.printf("Cell 4:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[0][3],((ADCRaw[0][3] * Reference) / 1024.0),ADCconversion(ADCRaw[0][3]));
    Serial.printf("Cell 5:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[0][4],((ADCRaw[0][4] * Reference) / 1024.0),ADCconversion(ADCRaw[0][4]));
    Serial.printf("Cell 6:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[0][5],((ADCRaw[0][5] * Reference) / 1024.0),ADCconversion(ADCRaw[0][5]));
    Serial.printf("Cell 7:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[0][6],((ADCRaw[0][6] * Reference) / 1024.0),ADCconversion(ADCRaw[0][6]));
    Serial.printf("Cell 8:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[0][7],((ADCRaw[0][7] * Reference) / 1024.0),ADCconversion(ADCRaw[0][7]));
    Serial.printf("Cell 9:  Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[1][0],((ADCRaw[1][0] * Reference) / 1024.0),ADCconversion(ADCRaw[1][0]));
    Serial.printf("Cell 10: Bit= %d | Voltage= %.2f | Temp=%.2f \n",ADCRaw[1][1],((ADCRaw[1][1] * Reference) / 1024.0),ADCconversion(ADCRaw[1][1]));
    */
}

void printshow(){
    Serial.printf("\t\t\t\t\t\t\t  Stats\n");
    Serial.printf("\t\t\t  Max temperature : %.2fºC | ",ADCconversion(minRaw));    
    Serial.printf("Min temperature : %.2fºC | ",ADCconversion(maxRaw));
    Serial.printf("Avg temperature : %.2fºC\n",ADCconversion(AvgRaw));
    Serial.printf("______________________________________________");
    Serial.printf("______________________________________________");
    Serial.printf("_____________________________________________\n");


    Serial.printf("\t\t\t\t\t\t\t Segmento 1\n");
    Serial.printf("1 : %.2fºC | ",ADCconversion(ADCRaw[6][2]));    
    Serial.printf("2 : %.2fºC | ",ADCconversion(ADCRaw[6][3]));
    Serial.printf("3 : %.2fºC | ",ADCconversion(ADCRaw[6][4]));
    Serial.printf("4 : %.2fºC | ",ADCconversion(ADCRaw[6][5]));
    Serial.printf("5 : %.2fºC | ",ADCconversion(ADCRaw[6][6]));
    Serial.printf("6 : %.2fºC | ",ADCconversion(ADCRaw[6][7]));
    Serial.printf("7 : %.2fºC | ",ADCconversion(ADCRaw[7][0]));
    Serial.printf("8 : %.2fºC | ",ADCconversion(ADCRaw[7][1]));
    Serial.printf("9 : %.2fºC | ",ADCconversion(ADCRaw[7][2]));
    Serial.printf("10: %.2fºC\n",ADCconversion(ADCRaw[7][3]));
    Serial.printf("______________________________________________");
    Serial.printf("______________________________________________");
    Serial.printf("_____________________________________________\n");
    
    
    Serial.printf("\t\t\t\t\t\t\t Segmento 2\n");
    Serial.printf("1 : %.2fºC | ",ADCconversion(ADCRaw[5][0]));    
    Serial.printf("2 : %.2fºC | ",ADCconversion(ADCRaw[5][1]));
    Serial.printf("3 : %.2fºC | ",ADCconversion(ADCRaw[5][2]));
    Serial.printf("4 : %.2fºC | ",ADCconversion(ADCRaw[5][3]));
    Serial.printf("5 : %.2fºC | ",ADCconversion(ADCRaw[5][4]));
    Serial.printf("6 : %.2fºC | ",ADCconversion(ADCRaw[5][5]));
    Serial.printf("7 : %.2fºC | ",ADCconversion(ADCRaw[5][6]));
    Serial.printf("8 : %.2fºC | ",ADCconversion(ADCRaw[5][7]));
    Serial.printf("9 : %.2fºC | ",ADCconversion(ADCRaw[6][0]));
    Serial.printf("10: %.2fºC\n",ADCconversion(ADCRaw[6][1]));
    Serial.printf("______________________________________________");
    Serial.printf("______________________________________________");
    Serial.printf("_____________________________________________\n");
    

    
    Serial.printf("\t\t\t\t\t\t\t Segmento 3\n");
    Serial.printf("1 : %.2fºC | ",ADCconversion(ADCRaw[3][6])); 
    Serial.printf("2 : %.2fºC | ",ADCconversion(ADCRaw[3][7]));
    Serial.printf("3 : %.2fºC | ",ADCconversion(ADCRaw[4][0]));
    Serial.printf("4 : %.2fºC | ",ADCconversion(ADCRaw[4][1]));
    Serial.printf("5 : %.2fºC | ",ADCconversion(ADCRaw[4][2]));
    Serial.printf("6 : %.2fºC | ",ADCconversion(ADCRaw[4][3]));
    Serial.printf("7 : %.2fºC | ",ADCconversion(ADCRaw[4][4]));
    Serial.printf("8 : %.2fºC | ",ADCconversion(ADCRaw[4][5]));
    Serial.printf("9 : %.2fºC | ",ADCconversion(ADCRaw[4][6]));
    Serial.printf("10: %.2fºC\n", ADCconversion(ADCRaw[4][7]));
    Serial.printf("______________________________________________");
    Serial.printf("______________________________________________");
    Serial.printf("_____________________________________________\n");

    
    Serial.printf("\t\t\t\t\t\t\t Segmento 4\n");
    Serial.printf("1 : %.2fºC | ",ADCconversion(ADCRaw[2][4])); 
    Serial.printf("2 : %.2fºC | ",ADCconversion(ADCRaw[2][5]));
    Serial.printf("3 : %.2fºC | ",ADCconversion(ADCRaw[2][6]));
    Serial.printf("4 : %.2fºC | ",ADCconversion(ADCRaw[2][7]));
    Serial.printf("5 : %.2fºC | ",ADCconversion(ADCRaw[3][0]));
    Serial.printf("6 : %.2fºC | ",ADCconversion(ADCRaw[3][1]));
    Serial.printf("7 : %.2fºC | ",ADCconversion(ADCRaw[3][2]));
    Serial.printf("8 : %.2fºC | ",ADCconversion(ADCRaw[3][3]));
    Serial.printf("9 : %.2fºC | ",ADCconversion(ADCRaw[3][4]));
    //Serial.printf("10: %.2fºC\n",ADCconversion(ADCRaw[3][5]));
    Serial.printf("10 : Thermistor turned off\n");
    Serial.printf("______________________________________________");
    Serial.printf("______________________________________________");
    Serial.printf("_____________________________________________\n");
    

    
    Serial.printf("\t\t\t\t\t\t\t Segmento 5\n");
    Serial.printf("1 : %.2fºC | ",ADCconversion(ADCRaw[1][2])); 
    Serial.printf("2 : %.2fºC | ",ADCconversion(ADCRaw[1][3]));
    Serial.printf("3 : %.2fºC | ",ADCconversion(ADCRaw[1][4]));
    Serial.printf("4 : %.2fºC | ",ADCconversion(ADCRaw[1][5]));
    Serial.printf("5 : %.2fºC | ",ADCconversion(ADCRaw[1][6]));
    Serial.printf("6 : %.2fºC | ",ADCconversion(ADCRaw[1][7]));
    Serial.printf("7 : %.2fºC | ",ADCconversion(ADCRaw[2][0]));
    Serial.printf("8 : %.2fºC | ",ADCconversion(ADCRaw[2][1]));
    //Serial.printf("9 : %.2fºC | ",ADCconversion(ADCRaw[2][2]));
    Serial.printf("10: %.2fºC |",ADCconversion(ADCRaw[2][3]));
    Serial.printf("9 : Thermistor turned off\n");
    Serial.printf("______________________________________________");
    Serial.printf("______________________________________________");
    Serial.printf("_____________________________________________\n");
    

    Serial.printf("\t\t\t\t\t\t\t Segmento 6\n");
    Serial.printf("1 : %.2fºC | ",ADCconversion(ADCRaw[0][0])); 
    Serial.printf("2 : %.2fºC | ",ADCconversion(ADCRaw[0][1]));
    Serial.printf("3 : %.2fºC | ",ADCconversion(ADCRaw[0][2]));
    Serial.printf("4 : %.2fºC | ",ADCconversion(ADCRaw[0][3]));
    Serial.printf("5 : %.2fºC | ",ADCconversion(ADCRaw[0][4]));
    Serial.printf("6 : %.2fºC | ",ADCconversion(ADCRaw[0][5]));
    Serial.printf("7 : %.2fºC | ",ADCconversion(ADCRaw[0][6]));
    Serial.printf("8 : %.2fºC | ",ADCconversion(ADCRaw[0][7]));
    Serial.printf("9 : %.2fºC | ",ADCconversion(ADCRaw[1][0]));
    Serial.printf("10: %.2fºC\n",ADCconversion(ADCRaw[1][1]));
    Serial.printf("______________________________________________");
    Serial.printf("______________________________________________");
    Serial.printf("_____________________________________________\n");
    
}

void temp2bms(){
    //calcular max min e avg
    //verificar se alguma celula está disconectada por mais de 700ms
    //envia mensagem de erro

    TempErr = 0;

    for (int adc = 0; adc < N_ADCs; adc++) {
        for (int channel = 0; channel < N_ADC_CHANNELS; channel++) {
            if (adc == 7 && channel > 3)
                continue;
            if (adc == 3 && channel == 5)
                continue; //this termistor is not working
            if (adc == 2 && channel == 2)
                continue;
            if (adc == 4 && channel == 1)
                continue;
            if((millis()-TimeTemp[adc][channel])>ERROR_TIME){
                TempErr = 1;
                maxTemp=70.0; //se a bms receber esta temp vai dar erro
            }
            minTemp = min(minTemp, Temps[adc][channel]); 
            maxTemp = max(maxTemp, Temps[adc][channel]);
            sumTemp += Temps[adc][channel];
            //Serial.printf("new Temp= %f | Sum = %f\n",Temps[adc][channel],sumTemp);
        }
    }

    avgTemp = sumTemp/58;

    //error msg
    BMSInfoMsg.id = 0x306;
    BMSInfoMsg.flags.extended = 1;
    BMSInfoMsg.len = 1;
    BMSInfoMsg.buf[0] = (BMSErr || TempErr);
    can1.write(BMSInfoMsg);

    //Env msg temps value para a BMS
    BMSInfoMsg.id = 0x1839F380;
    BMSInfoMsg.flags.extended = 1;
    BMSInfoMsg.len = 8;
    BMSInfoMsg.buf[0] = 0x00;
    BMSInfoMsg.buf[1] = minTemp < 0 ? 70 : minTemp;  // 60 is maximum allowed temperature before
    //BMSInfoMsg.buf[2] = minTemp < 0 ? 70 : maxTemp;  // triggering an error on the BMS
    BMSInfoMsg.buf[2] = maxTemp > 58 ? 60 : maxTemp;  // triggering an error on the BMS
    BMSInfoMsg.buf[3] = avgTemp > 58 ? 60 : avgTemp;
    BMSInfoMsg.buf[4] = 0x01;
    BMSInfoMsg.buf[5] = 0x01;
    BMSInfoMsg.buf[6] = 0x00;
    BMSInfoMsg.buf[7] = BMSInfoMsg.buf[1] + BMSInfoMsg.buf[2] + BMSInfoMsg.buf[3] + BMSInfoMsg.buf[4] + BMSInfoMsg.buf[5] + BMSInfoMsg.buf[6] + 0x39 + 0x08;
    can1.write(BMSInfoMsg);
}


void setup() {
    // try to connect to the serial monitor
    Serial.begin(9600);
    if (Serial)
        Serial.println("Serial monitor connected");

    can1.begin();
    can1.setBaudRate(125000);

    can1.enableFIFO();
    can1.enableFIFOInterrupt();
    can1.onReceive(canbusSniffer);
    can1.setFIFOFilter(REJECT_ALL);
    (void)can1.setFIFOFilter(0, 0x111, STD);
    (void)can1.setFIFOFilter(1, 0x270, STD);//BMS MPO3 status msg id

    (void)ADCs[0].begin(13, 11, 12, 18);
    (void)ADCs[1].begin(13, 11, 12, 19);
    (void)ADCs[2].begin(13, 11, 12, 20);
    (void)ADCs[3].begin(13, 11, 12, 21);
    (void)ADCs[4].begin(13, 11, 12, 4);
    (void)ADCs[5].begin(13, 11, 12, 5);
    (void)ADCs[6].begin(13, 11, 12, 6);
    (void)ADCs[7].begin(13, 11, 12, 7);

    BMSInfoMsg.id = 0x306;
    BMSInfoMsg.flags.extended = 1;
    BMSInfoMsg.len = 1;
    BMSInfoMsg.buf[0] = 0;
    can1.write(BMSInfoMsg);
}


void loop() {
    // reset measurements
    rawSum = 0;
    maxRaw = 0;
    minRaw = 999;

    maxTemp = 0.0;
    minTemp = 60.0;
    sumTemp = 0.0;
    avgTemp = 0.0;

    readRawADCData();
    broadcastRawData();
    CAN_msg();
    
    temp2bms();
    if (Serial)
        //printdebug();
        printshow();

    //Serial.printf("Temp ERR: %d    BMS ERR: %d\n", TempErr, BMSErr);

    delay(50);

    // delay(100);

}
