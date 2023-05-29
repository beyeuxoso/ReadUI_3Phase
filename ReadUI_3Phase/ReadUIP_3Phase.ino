/*
 Name:		ReadUIP_3Phase.ino
 Created:	5/23/2023 10:50:30 PM
 Author:	Công Minh
*/
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include <Wire.h>
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

#define PZEM_COUNT (6U)
#define bytes_request (25U)
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

typedef struct{
    uint8_t PzemID;
    uint16_t Voltage;
    uint16_t Current;
    uint16_t Power;
    uint32_t Energy;
    uint16_t Frequency;
    uint16_t PF;
}Pzem_data;

Pzem_data Pzem_data_table[PZEM_COUNT];
uint8_t Pzem_idx = 0;
uint8_t Pzem_ID = 0;
uint8_t Reset_Pzem = 0;
int i = 0;

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

Thread ReadVoltCur1_1 = Thread();
Thread ReadVoltCur1_2 = Thread();
Thread ReadVoltCur1_3 = Thread();
Thread ReadVoltCur2_1 = Thread();
Thread ReadVoltCur2_2 = Thread();
Thread ReadVoltCur2_3 = Thread();
ThreadController controller = ThreadController();

//Modbus Slave(SlaveID, Serial, TXEN);
uint16_t data[36] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36 };

unsigned long time = 0;

void requestEvent()
{
    if (Pzem_ID == 6) Pzem_ID = 0;
        Wire.write(Pzem_data_table[Pzem_ID].PzemID);
        Wire.write((uint8_t)(Pzem_data_table[Pzem_ID].Voltage) & 0xFF);
        Wire.write((uint8_t)((Pzem_data_table[Pzem_ID].Voltage)>>8) & 0xFF);
        Wire.write((uint8_t)(Pzem_data_table[Pzem_ID].Current) & 0xFF);
        Wire.write((uint8_t)((Pzem_data_table[Pzem_ID].Current) >> 8) & 0xFF);
        Wire.write((uint8_t)(Pzem_data_table[Pzem_ID].Frequency) & 0xFF);
        Wire.write((uint8_t)((Pzem_data_table[Pzem_ID].Frequency) >> 8) & 0xFF);
        Wire.write((uint8_t)(Pzem_data_table[Pzem_ID].PF) & 0xFF);
        Wire.write((uint8_t)((Pzem_data_table[Pzem_ID].PF) >> 8) & 0xFF);
        Wire.write((uint8_t)(Pzem_data_table[Pzem_ID].Power) & 0xFF);
        Wire.write((uint8_t)((Pzem_data_table[Pzem_ID].Power) >> 8) & 0xFF);
        Wire.write((uint8_t)(Pzem_data_table[Pzem_ID].Energy) & 0xFF);
        Wire.write((uint8_t)((Pzem_data_table[Pzem_ID].Energy) >> 8) & 0xFF);
        Wire.write((uint8_t)((Pzem_data_table[Pzem_ID].Energy) >> 16) & 0xFF);
        Wire.write((uint8_t)((Pzem_data_table[Pzem_ID].Energy) >> 24) & 0xFF);
        Pzem_ID++;
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

void Read_PZEM1_1()
{
    SoftwareSerial* tt1;
    tt1 = (SoftwareSerial*)port1;
    tt1->begin(9600);
    while (port1->available()) {
        port1->read();
    }
    if (Reset_Pzem == 0) {
        port1->write(getValue, sizeof(getValue));
    }
    if (Reset_Pzem == 1) {
        port1->write(resetEnergy, sizeof(resetEnergy));
        Reset_Pzem = 0;
    }
    unsigned long temTime = millis();
    bool b_complete = false;
    uint8_t myBuf[bytes_request];
    while ((millis() - temTime) < 100) {
        if (port1->available()) {
            port1->readBytes(myBuf, bytes_request);
            b_complete = true;
            yield();
            break;
        }
    }
    if (b_complete) {
        Pzem_data_table[Pzem_idx].PzemID=Pzem_idx+1;
        Pzem_data_table[Pzem_idx].Voltage = (uint16_t)(PZEM_GET_VALUE(voltage, SCALE_V) * 100);
        Pzem_data_table[Pzem_idx].Current = (uint16_t)(PZEM_GET_VALUE2(ampe, SCALE_A) * 100);
        Pzem_data_table[Pzem_idx].Power = (uint16_t)(PZEM_GET_VALUE2(power, SCALE_P) * 100);
        Pzem_data_table[Pzem_idx].Energy = (uint32_t)(PZEM_GET_VALUE2(energy, SCALE_E) * 1000);
        Pzem_data_table[Pzem_idx].Frequency = (uint16_t)(PZEM_GET_VALUE(freq, SCALE_H) * 100);
        Pzem_data_table[Pzem_idx].PF = (uint16_t)(PZEM_GET_VALUE(powerFactor, SCALE_PF) * 100);
    }
    else {
        Pzem_data_table[Pzem_idx].PzemID = Pzem_idx+1;
        Pzem_data_table[Pzem_idx].Voltage = 0;
        Pzem_data_table[Pzem_idx].Current = 0;
        Pzem_data_table[Pzem_idx].Power = 0;
        Pzem_data_table[Pzem_idx].Energy = 0;
        Pzem_data_table[Pzem_idx].Frequency = 0;
        Pzem_data_table[Pzem_idx].PF = 0;
    }
    Pzem_idx++;
    yy1->end();
    tt1->end();
}

void Read_PZEM1_2()
{
    SoftwareSerial* tt2;
    tt2 = (SoftwareSerial*)port2;
    tt2->begin(9600);
    while (port2->available()) {
        port2->read();
    }
    if (Reset_Pzem == 0) {
        port2->write(getValue, sizeof(getValue));
    }
    if (Reset_Pzem == 2) {
        port2->write(resetEnergy, sizeof(resetEnergy));
        Reset_Pzem = 0;
    }
unsigned long temTime = millis();
bool b_complete = false;
uint8_t myBuf[bytes_request];

while ((millis() - temTime) < 100) {
    if (port2->available()) {
        port2->readBytes(myBuf, bytes_request);
        b_complete = true;
        yield();
        break;
    }
}
if (b_complete) {
    Pzem_data_table[Pzem_idx].PzemID = Pzem_idx+1;
    Pzem_data_table[Pzem_idx].Voltage = (uint16_t)(PZEM_GET_VALUE(voltage, SCALE_V) * 100);
    Pzem_data_table[Pzem_idx].Current = (uint16_t)(PZEM_GET_VALUE2(ampe, SCALE_A) * 100);
    Pzem_data_table[Pzem_idx].Power = (uint16_t)(PZEM_GET_VALUE2(power, SCALE_P) * 100);
    Pzem_data_table[Pzem_idx].Energy = (uint32_t)(PZEM_GET_VALUE2(energy, SCALE_E) * 1000);
    Pzem_data_table[Pzem_idx].Frequency = (uint16_t)(PZEM_GET_VALUE(freq, SCALE_H) * 100);
    Pzem_data_table[Pzem_idx].PF = (uint16_t)(PZEM_GET_VALUE(powerFactor, SCALE_PF) * 100);

}
else {
    Pzem_data_table[Pzem_idx].PzemID = Pzem_idx+1;
    Pzem_data_table[Pzem_idx].Voltage = 0;
    Pzem_data_table[Pzem_idx].Current = 0;
    Pzem_data_table[Pzem_idx].Power = 0;
    Pzem_data_table[Pzem_idx].Energy = 0;
    Pzem_data_table[Pzem_idx].Frequency = 0;
    Pzem_data_table[Pzem_idx].PF = 0;
}
Pzem_idx++;
yy2->end();
tt2->end();
}

void Read_PZEM1_3()
{
    SoftwareSerial* tt3;
    tt3 = (SoftwareSerial*)port3;
    tt3->begin(9600);
    while (port3->available()) {
        port3->read();
    }
    if (Reset_Pzem == 0) {
        port3->write(getValue, sizeof(getValue));
    }
    if (Reset_Pzem == 3) {
        port3->write(resetEnergy, sizeof(resetEnergy));
        Reset_Pzem = 0;
    }
    unsigned long temTime = millis();
    bool b_complete = false;
    uint8_t myBuf[bytes_request];
    while ((millis() - temTime) < 100) {
        if (port3->available()) {
            port3->readBytes(myBuf, bytes_request);
            b_complete = true;
            yield();
            break;
        }
    }
    if (b_complete) {
        Pzem_data_table[Pzem_idx].PzemID = Pzem_idx+1;
        Pzem_data_table[Pzem_idx].Voltage = (uint16_t)(PZEM_GET_VALUE(voltage, SCALE_V) * 100);
        Pzem_data_table[Pzem_idx].Current = (uint16_t)(PZEM_GET_VALUE2(ampe, SCALE_A) * 100);
        Pzem_data_table[Pzem_idx].Power = (uint16_t)(PZEM_GET_VALUE2(power, SCALE_P) * 100);
        Pzem_data_table[Pzem_idx].Energy = (uint32_t)(PZEM_GET_VALUE2(energy, SCALE_E) * 1000);
        Pzem_data_table[Pzem_idx].Frequency = (uint16_t)(PZEM_GET_VALUE(freq, SCALE_H) * 100);
        Pzem_data_table[Pzem_idx].PF = (uint16_t)(PZEM_GET_VALUE(powerFactor, SCALE_PF) * 100);
    }
    else {
        Pzem_data_table[Pzem_idx].PzemID = Pzem_idx+1;
        Pzem_data_table[Pzem_idx].Voltage = 0;
        Pzem_data_table[Pzem_idx].Current = 0;
        Pzem_data_table[Pzem_idx].Power = 0;
        Pzem_data_table[Pzem_idx].Energy = 0;
        Pzem_data_table[Pzem_idx].Frequency = 0;
        Pzem_data_table[Pzem_idx].PF = 0;
    }
    Pzem_idx++;
    yy3->end();
    tt3->end();
}

void Read_PZEM2_1()
{
    SoftwareSerial* tt4;
    tt4 = (SoftwareSerial*)port4;
    tt4->begin(9600);
    while (port4->available()) {
        port4->read();
    }
    if (Reset_Pzem == 0) {
        port4->write(getValue, sizeof(getValue));
    }
    if (Reset_Pzem == 4) {
        port4->write(resetEnergy, sizeof(resetEnergy));
        Reset_Pzem = 0;
    }
    unsigned long temTime = millis();
    bool b_complete = false;
    uint8_t myBuf[bytes_request];
    while ((millis() - temTime) < 100) {
        if (port4->available()) {
            port4->readBytes(myBuf, bytes_request);
            b_complete = true;
            yield();
            break;
        }
    }
    if (b_complete) {
        Pzem_data_table[Pzem_idx].PzemID = Pzem_idx+1;
        Pzem_data_table[Pzem_idx].Voltage = (uint16_t)(PZEM_GET_VALUE(voltage, SCALE_V) * 100);
        Pzem_data_table[Pzem_idx].Current = (uint16_t)(PZEM_GET_VALUE2(ampe, SCALE_A) * 100);
        Pzem_data_table[Pzem_idx].Power = (uint16_t)(PZEM_GET_VALUE2(power, SCALE_P) * 100);
        Pzem_data_table[Pzem_idx].Energy = (uint32_t)(PZEM_GET_VALUE2(energy, SCALE_E) * 1000);
        Pzem_data_table[Pzem_idx].Frequency = (uint16_t)(PZEM_GET_VALUE(freq, SCALE_H) * 100);
        Pzem_data_table[Pzem_idx].PF = (uint16_t)(PZEM_GET_VALUE(powerFactor, SCALE_PF) * 100);
    }
    else {
        Pzem_data_table[Pzem_idx].PzemID = Pzem_idx+1;
        Pzem_data_table[Pzem_idx].Voltage = 0;
        Pzem_data_table[Pzem_idx].Current = 0;
        Pzem_data_table[Pzem_idx].Power = 0;
        Pzem_data_table[Pzem_idx].Energy = 0;
        Pzem_data_table[Pzem_idx].Frequency = 0;
        Pzem_data_table[Pzem_idx].PF = 0;
    }
    Pzem_idx++;
    yy4->end();
    tt4->end();
}

void Read_PZEM2_2()
{
    SoftwareSerial* tt5;
    tt5 = (SoftwareSerial*)port5;
    tt5->begin(9600);
    while (port5->available()) {
        port5->read();
    }
    if (Reset_Pzem == 0) {
        port5->write(getValue, sizeof(getValue));
    }
    if (Reset_Pzem == 5) {
        port5->write(resetEnergy, sizeof(resetEnergy));
        Reset_Pzem = 0;
    }
    unsigned long temTime = millis();
    bool b_complete = false;
    uint8_t myBuf[bytes_request];
    while ((millis() - temTime) < 100) {
        if (port5->available()) {
            port5->readBytes(myBuf, bytes_request);
            b_complete = true;
            yield();
            break;
        }
    }
    if (b_complete) {
        Pzem_data_table[Pzem_idx].PzemID = Pzem_idx+1;
        Pzem_data_table[Pzem_idx].Voltage = (uint16_t)(PZEM_GET_VALUE(voltage, SCALE_V) * 100);
        Pzem_data_table[Pzem_idx].Current = (uint16_t)(PZEM_GET_VALUE2(ampe, SCALE_A) * 100);
        Pzem_data_table[Pzem_idx].Power = (uint16_t)(PZEM_GET_VALUE2(power, SCALE_P) * 100);
        Pzem_data_table[Pzem_idx].Energy = (uint32_t)(PZEM_GET_VALUE2(energy, SCALE_E) * 1000);
        Pzem_data_table[Pzem_idx].Frequency = (uint16_t)(PZEM_GET_VALUE(freq, SCALE_H) * 100);
        Pzem_data_table[Pzem_idx].PF = (uint16_t)(PZEM_GET_VALUE(powerFactor, SCALE_PF) * 100);
    }
    else {
        Pzem_data_table[Pzem_idx].PzemID = Pzem_idx+1;
        Pzem_data_table[Pzem_idx].Voltage = 0;
        Pzem_data_table[Pzem_idx].Current = 0;
        Pzem_data_table[Pzem_idx].Power = 0;
        Pzem_data_table[Pzem_idx].Energy = 0;
        Pzem_data_table[Pzem_idx].Frequency = 0;
        Pzem_data_table[Pzem_idx].PF = 0;
    }
    Pzem_idx++;
    yy5->end();
    tt5->end();
}

void Read_PZEM2_3()
{
    SoftwareSerial* tt6;
    tt6 = (SoftwareSerial*)port6;
    tt6->begin(9600);
    while (port6->available()) {
        port6->read();
    }
    if (Reset_Pzem == 0) {
        port6->write(getValue, sizeof(getValue));
    }
    if (Reset_Pzem == 6) {
        port6->write(resetEnergy, sizeof(resetEnergy));
        Reset_Pzem = 0;
    }
    unsigned long temTime = millis();
    bool b_complete = false;
    uint8_t myBuf[bytes_request];
    while ((millis() - temTime) < 100) {
        if (port6->available()) {
            port6->readBytes(myBuf, bytes_request);
            b_complete = true;
            yield();
            break;
        }
    }
    if (b_complete) {
        Pzem_data_table[Pzem_idx].PzemID = Pzem_idx + 1;
        Pzem_data_table[Pzem_idx].Voltage = (uint16_t)(PZEM_GET_VALUE(voltage, SCALE_V) * 100);
        Pzem_data_table[Pzem_idx].Current = (uint16_t)(PZEM_GET_VALUE2(ampe, SCALE_A) * 100);
        Pzem_data_table[Pzem_idx].Power = (uint16_t)(PZEM_GET_VALUE2(power, SCALE_P) * 100);
        Pzem_data_table[Pzem_idx].Energy = (uint32_t)(PZEM_GET_VALUE2(energy, SCALE_E) * 1000);
        Pzem_data_table[Pzem_idx].Frequency = (uint16_t)(PZEM_GET_VALUE(freq, SCALE_H) * 100);
        Pzem_data_table[Pzem_idx].PF = (uint16_t)(PZEM_GET_VALUE(powerFactor, SCALE_PF) * 100);
    }
    else {
        Pzem_data_table[Pzem_idx].PzemID = Pzem_idx + 1;
        Pzem_data_table[Pzem_idx].Voltage = 0;
        Pzem_data_table[Pzem_idx].Current = 0;
        Pzem_data_table[Pzem_idx].Power = 0;
        Pzem_data_table[Pzem_idx].Energy = 0;
        Pzem_data_table[Pzem_idx].Frequency = 0;
        Pzem_data_table[Pzem_idx].PF = 0;
    }
    Pzem_idx = 0;
    yy6->end();
    tt6->end();
}

void Init_Thread()
{
    ReadVoltCur1_1.enabled = true;
    ReadVoltCur1_1.setInterval(1000);
    ReadVoltCur1_1.onRun(Read_PZEM1_1);

    ReadVoltCur1_2.enabled = true;
    ReadVoltCur1_2.setInterval(1000);
    ReadVoltCur1_2.onRun(Read_PZEM1_2);

    ReadVoltCur1_3.enabled = true;
    ReadVoltCur1_3.setInterval(1000);
    ReadVoltCur1_3.onRun(Read_PZEM1_3);

    ReadVoltCur2_1.enabled = true;
    ReadVoltCur2_1.setInterval(1000);
    ReadVoltCur2_1.onRun(Read_PZEM2_1);

    ReadVoltCur2_2.enabled = true;
    ReadVoltCur2_2.setInterval(1000);
    ReadVoltCur2_2.onRun(Read_PZEM2_2);

    ReadVoltCur2_3.enabled = true;
    ReadVoltCur2_3.setInterval(1000);
    ReadVoltCur2_3.onRun(Read_PZEM2_3);

    controller.setInterval(10);
    controller.add(&ReadVoltCur1_1);
    controller.add(&ReadVoltCur1_2);
    controller.add(&ReadVoltCur1_3);
    controller.add(&ReadVoltCur2_1);
    controller.add(&ReadVoltCur2_2);
    controller.add(&ReadVoltCur2_3);
}

//// Init STM32 timer TIM1
//STM32Timer ITimer0(TIM1);
//#define ISR_TIMER_INTERVAL  1
//void TimerHandler0()
//{
//    //Serial_dbg.println("Call timmer 0");
//    Slave.poll(data, 36);
//}

// the setup function runs once when you press reset or power the board
void setup() {
    Init_Thread();
    Serial.begin(9600);
    Serial_dbg.begin(9600);
    Init_I2C();
    //Slave.start();
    Serial_dbg.println("Start:");
    // Interval in microsecs
    /*if (ITimer0.attachInterruptInterval(ISR_TIMER_INTERVAL * 50, TimerHandler0))
    {
        Serial_dbg.print(F("Starting ITimer0 OK, millis() = ")); Serial_dbg.println(millis());
    }
    else
        Serial_dbg.println(F("Can't set ITimer0. Select another freq. or timer"));*/
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
    controller.run();
}


void ReadUIP_3Phase_modbus() {
        Serial_dbg.println("\\\\\\\\");
        if (data[i] != NAN || data[i+6] != NAN || data[i+12] != NAN || data[i+18] != NAN || data[i+24] != NAN || data[i+30] != NAN) {
            Serial_dbg.printf("Pzem%d: ", i + 1);  Serial_dbg.println();
            Serial_dbg.print("Voltage:");    Serial_dbg.println((float)data[i] / 100.0, 2); \
            Serial_dbg.print("Current:");    Serial_dbg.println((float)data[i+1] / 100.0, 2); \
            Serial_dbg.print("Power:");    Serial_dbg.println((float)data[i+2] / 100.0, 2); \
            Serial_dbg.print("Energy:");    Serial_dbg.println((float)data[i+3] / 100.0, 3); \
            Serial_dbg.print("Frequency:");    Serial_dbg.println((float)data[i+4] / 100.0, 2); \
            Serial_dbg.print("PF:");    Serial_dbg.println((float)data[i+5] / 100.0, 2); \
        }
        else {
            Serial_dbg.println("Error reading");
        }
}

void ReadUIP_3Phase_I2C(int pzem_id) {
            Serial_dbg.println("\\\\\\\\");
            Serial_dbg.print("PzemID: "); Serial_dbg.println(Pzem_data_table[pzem_id].PzemID); \
                Serial_dbg.print("Voltage: "); Serial_dbg.println((float)Pzem_data_table[pzem_id].Voltage / 100.0, 2); \
                Serial_dbg.print("Current: "); Serial_dbg.println((float)Pzem_data_table[pzem_id].Current / 100.0, 2); \
                Serial_dbg.print("Power: "); Serial_dbg.println((float)Pzem_data_table[pzem_id].Power / 100.0, 2); \
                Serial_dbg.print("Energy: "); Serial_dbg.println((float)Pzem_data_table[pzem_id].Energy / 100.0, 3); \
                Serial_dbg.print("Frequency: "); Serial_dbg.println((float)Pzem_data_table[pzem_id].Frequency / 100.0, 2); \
                Serial_dbg.print("PF: "); Serial_dbg.println((float)Pzem_data_table[pzem_id].PF / 100.0, 2);
}
