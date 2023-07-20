#include <Adafruit_MCP3008.h>
#include <FlexCAN_T4.h>
#include <Wire.h>
#include <elapsedMillis.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

Adafruit_MCP3008 ADCs[8];

#define Reference 2.5
#define ERROR_TIME 900

#define N_ADCs 8
#define N_ADC_CHANNELS 8

#define BMS_CAN_PERIOD 250
#define BROADCAST_PERIOD 2504
#define BROADCAST_ID 0x301

CAN_message_t BMSInfoMsg;
CAN_message_t tempBroadcast;

int broadcastIndex = 0;

elapsedMillis timeSinceLastBroadcast = 0;
elapsedMillis timeSinceLastBMSMessage = 0;

int flag = 0;
int count = 0;

float read = 0;
float voltage = 0;
float voltage1 = 0;
float voltage2 = 0;
float voltage3 = 0;
float voltage4 = 0;
float temperature = 0.0;
float maxTemp = 0.0;
float minTemp = 60.0;
float TempSum = 0.0;
float AvgTemp = 0.0;

float ADCRaw[8][8] = {
    {0, 0, 0, 0, 0, 0, 0, 0},  // seg8
    {0, 0, 0, 0, 0, 0, 0, 0},  // seg7
    {0, 0, 0, 0, 0, 0, 0, 0},  // seg6
    {0, 0, 0, 0, 0, 0, 0, 0},  // seg5
    {0, 0, 0, 0, 0, 0, 0, 0},  // seg4
    {0, 0, 0, 0, 0, 0, 0, 0},  // seg3
    {0, 0, 0, 0, 0, 0, 0, 0},  // seg2
    {0, 0, 0, 0, 0, 0, 0, 0}   // seg1
};

float ADCconversion(float read) {
    voltage = (read * Reference / 1024.0);

    voltage2 = voltage * voltage;

    voltage3 = voltage2 * voltage;

    voltage4 = voltage3 * voltage;

    temperature = 2875.88397 - 5512.867802 * voltage + 4082.002758 * voltage2 - 1358.200746 * voltage3 + 168.841073 * voltage4;

    return temperature;
}

// write a function to read all the ADC values
void readRawADCData() {
    for (int adc = 0; adc < N_ADCs; adc++) {
        for (int channel = 0; channel < N_ADC_CHANNELS; channel++) {
            if (adc == 7 && channel > 3)
                continue;
            ADCRaw[adc][channel] = ADCs[adc].readADC(channel);
            minTemp = min(minTemp, ADCRaw[adc][channel]);
            maxTemp = max(maxTemp, ADCRaw[adc][channel]);
            TempSum += ADCRaw[adc][channel];
        }
    }
}

void CAN_msg() {
    if (timeSinceLastBroadcast < BROADCAST_PERIOD)
        return;

    tempBroadcast.id = BROADCAST_ID;  // para decidir
    tempBroadcast.len = N_ADC_CHANNELS + 1;
    tempBroadcast.buf[0] = broadcastIndex;
    for (int i = 0; i < N_ADCs; i++) {
        tempBroadcast.buf[i + 1] = (uint8_t)ADCRaw[broadcastIndex][i];
    }

    can1.write(tempBroadcast);
    broadcastIndex = (broadcastIndex + 1) % N_ADCs;
}

void BMS_msg() {
    if (timeSinceLastBMSMessage < BMS_CAN_PERIOD)
        return;

    AvgTemp = TempSum / 60;
    BMSInfoMsg.id = 0x1839F380;
    BMSInfoMsg.flags.extended = 1;
    BMSInfoMsg.len = 8;
    BMSInfoMsg.buf[0] = 0x00;
    BMSInfoMsg.buf[1] = (uint8_t)minTemp;
    BMSInfoMsg.buf[2] = (uint8_t)maxTemp;
    BMSInfoMsg.buf[3] = (uint8_t)AvgTemp;
    BMSInfoMsg.buf[4] = 0x01;
    BMSInfoMsg.buf[5] = 0x01;
    BMSInfoMsg.buf[6] = 0x00;
    BMSInfoMsg.buf[7] = BMSInfoMsg.buf[1] + BMSInfoMsg.buf[2] + BMSInfoMsg.buf[3] + BMSInfoMsg.buf[4] + BMSInfoMsg.buf[5] + BMSInfoMsg.buf[6] + 0x39 + 0x08;
    can1.write(BMSInfoMsg);
}

void canbusSniffer(const CAN_message_t& msg) {
    if (Serial) {
        Serial.println("CAN message received");
        Serial.print("Message ID: ");
        Serial.println(msg.id, HEX);
    }
    if (msg.id == 0x300)
        flag = 1;
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

    (void)ADCs[0].begin(13, 11, 12, 18);
    (void)ADCs[1].begin(13, 11, 12, 19);
    (void)ADCs[2].begin(13, 11, 12, 20);
    (void)ADCs[3].begin(13, 11, 12, 21);
    (void)ADCs[4].begin(13, 11, 12, 4);
    (void)ADCs[5].begin(13, 11, 12, 5);
    (void)ADCs[6].begin(13, 11, 12, 6);
    (void)ADCs[7].begin(13, 11, 12, 7);
}

void loop() {
    // reset measurements
    TempSum = 0;
    maxTemp = 0;
    minTemp = 999;

    BMS_msg();
}
