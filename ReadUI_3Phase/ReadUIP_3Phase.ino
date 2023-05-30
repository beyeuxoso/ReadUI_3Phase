/*
 Name:		ReadUIP_3Phase.ino
 Created:	5/23/2023 10:50:30 PM
 Author:	Công Minh
*/
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <IWatchdog.h>
#include "ThreadController.h"
#include "Thread.h"
#include "StaticThreadController.h"
#include "ModbusRtu.h"
#include "STM32TimerInterrupt.h"

#define TXEN (PA1)
#define SlaveID (1U)

#define RX_PZEM1_1     PA0
#define TX_PZEM1_1     PA4
#define RX_PZEM1_2     PA5
#define TX_PZEM1_2     PA6
#define RX_PZEM1_3     PA7
#define TX_PZEM1_3     PA8
#define RX_PZEM2_1     PA11
#define TX_PZEM2_1     PA12
#define RX_PZEM2_2     PB0
#define TX_PZEM2_2     PB1
#define RX_PZEM2_3     PB3
#define TX_PZEM2_3     PB4

#define PZEM_NUMBER_MAX         (6U)
#define bytes_request           (25U)
#define SCALE_V               (0.1)
#define SCALE_A               (0.001)
#define SCALE_P               (0.1)
#define SCALE_E               (1)
#define SCALE_H               (0.1)
#define SCALE_PF              (0.01)
#define PZEM_CONVERT(low,high,scale)        (((high<<8) + low) * scale)
#define PZEM_GET_VALUE(unit, scale)       (float)(PZEM_CONVERT(myBuf[_##unit##_L__], myBuf[_##unit##_H__],scale))
#define PZEM_GET_VALUE2(unit, scale)      (float)(((PZEM_CONVERT(myBuf[_##unit##_1L__], myBuf[_##unit##_1H__],1) << 24)|\
                                                  PZEM_CONVERT(myBuf[_##unit##_L__], myBuf[_##unit##_H__],1)) * scale)
byte getValue[8] = { 0xf8, 0x04, 0x00, 0x00, 0x00, 0x0a, 0x64, 0x64 };
byte resetEnergy[4] = { 0xf8, 0x42, 0xc2, 0x41 };

HardwareSerial Serial_dbg(PA10, PA9);//RX-TX

const int16_t I2C_SLAVE = 0x13;


enum {
    _address__ = 0,
    _byteSuccess__,
    _numberOfByte__,
    _voltage_H__,
    _voltage_L__,
    _ampe_H__,
    _ampe_L__,
    _ampe_1H__,
    _ampe_1L__,
    _power_H__,
    _power_L__,
    _power_1H__,
    _power_1L__,
    _energy_H__,
    _energy_L__,
    _energy_1H__,
    _energy_1L__,
    _freq_H__,
    _freq_L__,
    _powerFactor_H__,
    _powerFactor_L__,
    _nouse4H__,
    _nouse5L__,
    _crc_H__,
    _crc_L__,
    _RESPONSE_SIZE__
};

typedef struct __attribute__((packed))
{
    //uint8_t PzemID;
    uint8_t Status;
    uint16_t Voltage;
    uint16_t Current;
    uint16_t Frequency;
    uint16_t PF;
    uint16_t Power;
    uint32_t Energy;
} PzemData_t;

PzemData_t Pzem_data_table[PZEM_NUMBER_MAX];


uint8_t Reset_Pzem = 0;
int i = 0;
uint8_t CheckPzem[PZEM_NUMBER_MAX];

SoftwareSerial* yy1 = new SoftwareSerial(TX_PZEM1_1, RX_PZEM1_1);
Stream* port1 = yy1;
SoftwareSerial* yy2 = new SoftwareSerial(TX_PZEM1_2, RX_PZEM1_2);
Stream* port2 = yy2;
SoftwareSerial* yy3 = new SoftwareSerial(TX_PZEM1_3, RX_PZEM1_3);
Stream* port3 = yy3;
SoftwareSerial* yy4 = new SoftwareSerial(TX_PZEM2_1, RX_PZEM2_1);
Stream* port4 = yy4;
SoftwareSerial* yy5 = new SoftwareSerial(TX_PZEM2_2, RX_PZEM2_2);
Stream* port5 = yy5;
SoftwareSerial* yy6 = new SoftwareSerial(TX_PZEM2_3, RX_PZEM2_3);
Stream* port6 = yy6;

unsigned long time = 0;

void requestEvent()
{
    static uint8_t PzemId = 1;
    Wire.write(&PzemId, 1);
    Wire.write((uint8_t*)&Pzem_data_table[PzemId-1].Status, 15);
    Wire.endTransmission();
    PzemId++;
    if (PzemId > PZEM_NUMBER_MAX) PzemId = 1;
}

void receiveEvent(int bytes) {
    uint8_t data_i2c[bytes];
    uint8_t idx_i2c=0;
    while (Wire.available())
    {
        data_i2c[idx_i2c] = Wire.read();
        idx_i2c++;
    }
    if (data_i2c[0] == 4) {
        Reset_Pzem = data_i2c[1];
    }
}

void Init_I2C()
{
    Wire.setSCL(PB6);
    Wire.setSDA(PB7);
    Wire.begin(I2C_SLAVE);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);
}

void ReadPzemProcess(Stream* port)
{
    static uint8_t PzemIdx = 1;
    SoftwareSerial* ptr;
    ptr = (SoftwareSerial*)port;
    ptr->begin(9600);
    port->flush();
    if(Reset_Pzem==0)
    port->write(getValue, sizeof(getValue));
    if (PzemIdx == Reset_Pzem) {
        port->write(resetEnergy, sizeof(resetEnergy));
        Reset_Pzem = 0;
    }
    unsigned long temTime = millis();
    bool b_complete = false;
    uint8_t myBuf[bytes_request];
    while ((millis() - temTime) < 100) {
        if (port->available()) {
            port->readBytes(myBuf, bytes_request);
            b_complete = true;
            yield();
            break;
        }
    }
    if (b_complete) {
        PzemData_t PzemData = {};
        PzemData.Voltage = (uint16_t)(PZEM_GET_VALUE(voltage, SCALE_V) * 100);
        PzemData.Current = (uint16_t)(PZEM_GET_VALUE2(ampe, SCALE_A) * 100);
        PzemData.Power = (uint16_t)(PZEM_GET_VALUE2(power, SCALE_P) * 100);
        PzemData.Energy = (uint32_t)(PZEM_GET_VALUE2(energy, SCALE_E) * 1000);
        PzemData.Frequency = (uint16_t)(PZEM_GET_VALUE(freq, SCALE_H) * 100);
        PzemData.PF = (uint16_t)(PZEM_GET_VALUE(powerFactor, SCALE_PF) * 100);
        PzemData.Status = 1;
        noInterrupts();
        memcpy(&Pzem_data_table[PzemIdx-1].Status, &PzemData.Status, 15);
        interrupts();
    }
    else {
        if (CheckPzem[PzemIdx - 1] > 3) {
            CheckPzem[PzemIdx - 1] = 0;
            noInterrupts();
            memcpy(&Pzem_data_table[PzemIdx - 1].Status, 0, 15);
            interrupts();
        }
        CheckPzem[PzemIdx - 1]++;
    }
    ptr->end();
    PzemIdx++;
    if (PzemIdx > 6) PzemIdx = 1;
}
void PzemReadDataMain()
{
    static uint8_t count = 1;
    switch (count)
    {
    case 1:
        ReadPzemProcess(port1);
        break;
    case 2:
        ReadPzemProcess(port2);
        break;
    case 3:
        ReadPzemProcess(port3);
        break;
    case 4:
        ReadPzemProcess(port4);
        break;
    case 5:
        ReadPzemProcess(port5);
        break;
    case 6:
        ReadPzemProcess(port6);
        break;
    default:
        break;
    }
    count++;
    if (count > 6) count = 1;
}
void ReadUIP_3Phase_I2C(int pzem_id) {
    Serial_dbg.println("\\\\\\\\");
    Serial_dbg.print("PzemID: "); Serial_dbg.println(pzem_id); \
        Serial_dbg.print("Voltage: "); Serial_dbg.println((float)Pzem_data_table[pzem_id].Voltage / 100.0, 2); \
        Serial_dbg.print("Current: "); Serial_dbg.println((float)Pzem_data_table[pzem_id].Current / 100.0, 2); \
        Serial_dbg.print("Power: "); Serial_dbg.println((float)Pzem_data_table[pzem_id].Power / 100.0, 2); \
        Serial_dbg.print("Energy: "); Serial_dbg.println((float)Pzem_data_table[pzem_id].Energy / 100.0, 3); \
        Serial_dbg.print("Frequency: "); Serial_dbg.println((float)Pzem_data_table[pzem_id].Frequency / 100.0, 2); \
        Serial_dbg.print("PF: "); Serial_dbg.println((float)Pzem_data_table[pzem_id].PF / 100.0, 2);
}

// the setup function runs once when you press reset or power the board
void setup() {

    Serial_dbg.begin(9600);
    Init_I2C();
    IWatchdog.begin(4000000);
    //Slave.start();
    Serial_dbg.println("Start:");

}

// the loop function runs over and over again until power down or reset
void loop() {
#if 0
    if (millis() - time > 1000) {
        if (i == 6) i = 0;
        ReadUIP_3Phase_I2C(i);
        i++;
        time = millis();
    }
#endif

    PzemReadDataMain();    

    IWatchdog.reload();
}