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

// #define BMS_CAN_PERIOD 250
// #define BROADCAST_PERIOD 250
#define BROADCAST_ID 0x301

CAN_message_t BMSInfoMsg;
CAN_message_t tempBroadcast;


// elapsedMillis timeSinceLastBroadcast = 0;
// elapsedMillis timeSinceLastBMSMessage = 0;

int broadcastIndex = 0;
int broadcastEnabled = 0;
int count = 0;

float read = 0;
float voltage = 0;
float voltage1 = 0;
float voltage2 = 0;
float voltage3 = 0;
float voltage4 = 0;
float temperature = 0.0;
float maxRaw = 0.0;
float minRaw = 60.0;
float rawSum = 0.0;
float AvgRaw = 0.0;

float ADCRaw[8][8] = {
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0}};

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
            minRaw = min(minRaw, ADCRaw[adc][channel]);
            maxRaw = max(maxRaw, ADCRaw[adc][channel]);
            rawSum += ADCRaw[adc][channel];
        }
    }
    AvgRaw = rawSum / (N_ADCs * N_ADC_CHANNELS - 4);  // Only 60 ADC Channels are usable
}

void broadcastRawData() {
    // if (timeSinceLastBroadcast < BROADCAST_PERIOD)
    //     return;

    tempBroadcast.id = BROADCAST_ID + broadcastIndex;  // para decidir
    tempBroadcast.len = N_ADC_CHANNELS + 1;
    for (int i = 0; i < N_ADCs; i++)
        tempBroadcast.buf[i] = (uint8_t)ADCRaw[broadcastIndex][i];

    can1.write(tempBroadcast);
    broadcastIndex = (broadcastIndex + 1) % N_ADCs;
}

void sendTempsToBMS() {
    uint8_t minTemp = (uint8_t)ADCconversion(minRaw);
    uint8_t maxTemp = (uint8_t)ADCconversion(maxRaw);
    uint8_t avgTemp = (uint8_t)ADCconversion(AvgRaw);

    BMSInfoMsg.id = 0x1839F380;
    BMSInfoMsg.flags.extended = 1;
    BMSInfoMsg.len = 8;
    BMSInfoMsg.buf[0] = 0x00;
    BMSInfoMsg.buf[1] = minTemp > 58 ? 60 : minTemp;  // 60 is maximum allowed temperature before
    BMSInfoMsg.buf[2] = maxTemp > 58 ? 60 : maxTemp;  // triggering an error on the BMS
    BMSInfoMsg.buf[3] = avgTemp > 58 ? 60 : avgTemp;
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
        broadcastEnabled = 1;
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
    rawSum = 0;
    maxRaw = 0;
    minRaw = 999;

    readRawADCData();
    broadcastRawData();
    sendTempsToBMS();

    delay(100);
}
