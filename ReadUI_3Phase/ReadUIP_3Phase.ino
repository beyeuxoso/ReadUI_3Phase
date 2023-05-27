/*
 Name:		ReadUIP_3Phase.ino
 Created:	5/23/2023 10:50:30 PM
 Author:	Công Minh
*/
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
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

Modbus Slave(SlaveID, Serial, TXEN);
uint16_t data[36] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36 };

unsigned long time = 0;

void Read_PZEM1_1()
{
    SoftwareSerial* tt1;
    tt1 = (SoftwareSerial*)port1;
    tt1->begin(9600);
    while (port1->available()) {
        port1->read();
    }
    port1->write(getValue, sizeof(getValue));
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
        data[0] = (uint16_t)(PZEM_GET_VALUE(voltage, SCALE_V) * 100);
        data[1] = (uint16_t)(PZEM_GET_VALUE2(ampe, SCALE_A) * 100);
        data[2] = (uint16_t)(PZEM_GET_VALUE2(power, SCALE_P) * 100);
        data[3] = (uint16_t)(PZEM_GET_VALUE2(energy, SCALE_E) * 100);
        data[4] = (uint16_t)(PZEM_GET_VALUE(freq, SCALE_H) * 100);
        data[5] = (uint16_t)(PZEM_GET_VALUE(powerFactor, SCALE_PF) * 100);
    }
    else {
        data[0] = 0;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        data[4] = 0;
        data[5] = 0;
    }
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
port2->write(getValue, sizeof(getValue));

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
    data[6] = (uint16_t)(PZEM_GET_VALUE(voltage, SCALE_V) * 100);
    data[7] = (uint16_t)(PZEM_GET_VALUE2(ampe, SCALE_A) * 100);
    data[8] = (uint16_t)(PZEM_GET_VALUE2(power, SCALE_P) * 100);
    data[9] = (uint16_t)(PZEM_GET_VALUE2(energy, SCALE_E) * 100);
    data[10] = (uint16_t)(PZEM_GET_VALUE(freq, SCALE_H) * 100);
    data[11] = (uint16_t)(PZEM_GET_VALUE(powerFactor, SCALE_PF) * 100);

}
else {
    data[6] = 0;
    data[7] = 0;
    data[8] = 0;
    data[9] = 0;
    data[10] = 0;
    data[11] = 0;
}
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
    port3->write(getValue, sizeof(getValue));
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
        data[12] = (uint16_t)(PZEM_GET_VALUE(voltage, SCALE_V) * 100);
        data[13] = (uint16_t)(PZEM_GET_VALUE2(ampe, SCALE_A) * 100);
        data[14] = (uint16_t)(PZEM_GET_VALUE2(power, SCALE_P) * 100);
        data[15] = (uint16_t)(PZEM_GET_VALUE2(energy, SCALE_E) * 100);
        data[16] = (uint16_t)(PZEM_GET_VALUE(freq, SCALE_H) * 100);
        data[17] = (uint16_t)(PZEM_GET_VALUE(powerFactor, SCALE_PF) * 100);
    }
    else {
        data[12] = 0;
        data[13] = 0;
        data[14] = 0;
        data[15] = 0;
        data[16] = 0;
        data[17] = 0;
    }
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
    port4->write(getValue, sizeof(getValue));
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
        data[18] = (uint16_t)(PZEM_GET_VALUE(voltage, SCALE_V) * 100);
        data[19] = (uint16_t)(PZEM_GET_VALUE2(ampe, SCALE_A) * 100);
        data[20] = (uint16_t)(PZEM_GET_VALUE2(power, SCALE_P) * 100);
        data[21] = (uint16_t)(PZEM_GET_VALUE2(energy, SCALE_E) * 100);
        data[22] = (uint16_t)(PZEM_GET_VALUE(freq, SCALE_H) * 100);
        data[23] = (uint16_t)(PZEM_GET_VALUE(powerFactor, SCALE_PF) * 100);
    }
    else {
        data[18] = 0;
        data[19] = 0;
        data[20] = 0;
        data[21] = 0;
        data[22] = 0;
        data[23] = 0;
    }
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
    port5->write(getValue, sizeof(getValue));
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
        data[24] = (uint16_t)(PZEM_GET_VALUE(voltage, SCALE_V) * 100);
        data[25] = (uint16_t)(PZEM_GET_VALUE2(ampe, SCALE_A) * 100);
        data[26] = (uint16_t)(PZEM_GET_VALUE2(power, SCALE_P) * 100);
        data[27] = (uint16_t)(PZEM_GET_VALUE2(energy, SCALE_E) * 100);
        data[28] = (uint16_t)(PZEM_GET_VALUE(freq, SCALE_H) * 100);
        data[29] = (uint16_t)(PZEM_GET_VALUE(powerFactor, SCALE_PF) * 100);
    }
    else {
        data[24] = 0;
        data[25] = 0;
        data[26] = 0;
        data[27] = 0;
        data[28] = 0;
        data[29] = 0;
    }
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
    port6->write(getValue, sizeof(getValue));
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
        data[30] = (uint16_t)(PZEM_GET_VALUE(voltage, SCALE_V) * 100);
        data[31] = (uint16_t)(PZEM_GET_VALUE2(ampe, SCALE_A) * 100);
        data[32] = (uint16_t)(PZEM_GET_VALUE2(power, SCALE_P) * 100);
        data[33] = (uint16_t)(PZEM_GET_VALUE2(energy, SCALE_E) * 100);
        data[34] = (uint16_t)(PZEM_GET_VALUE(freq, SCALE_H) * 100);
        data[35] = (uint16_t)(PZEM_GET_VALUE(powerFactor, SCALE_PF) * 100);
    }
    else {
        data[30] = 0;
        data[31] = 0;
        data[32] = 0;
        data[33] = 0;
        data[34] = 0;
        data[35] = 0;
    }
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

// Init STM32 timer TIM1
STM32Timer ITimer0(TIM1);
#define ISR_TIMER_INTERVAL  1
void TimerHandler0()
{
    //Serial_dbg.println("Call timmer 0");
    Slave.poll(data, 36);
}

// the setup function runs once when you press reset or power the board
void setup() {
    Init_Thread();
    Serial.begin(9600);
    Serial_dbg.begin(9600);
    Slave.start();
    Serial_dbg.println("Start:");
    // Interval in microsecs
    if (ITimer0.attachInterruptInterval(ISR_TIMER_INTERVAL * 50, TimerHandler0))
    {
        Serial_dbg.print(F("Starting ITimer0 OK, millis() = ")); Serial_dbg.println(millis());
    }
    else
        Serial_dbg.println(F("Can't set ITimer0. Select another freq. or timer"));
}

// the loop function runs over and over again until power down or reset
void loop() {
#if 0
    if (millis() - time > 1000) {
        ReadUIP_3Phase();
        time = millis();
    }
#endif
    controller.run();
    //Slave.poll(data, 36);
}


void ReadUIP_3Phase() {
    Serial_dbg.println("\\\\\\\\");
    if (data[0] != NAN || data[6] != NAN || data[12] != NAN || data[18] != NAN || data[24] != NAN || data[30] != NAN) {
        Serial_dbg.print("Voltage: "); Serial_dbg.print((float)data[0] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.print((float)data[6] / 100.0, 2); Serial_dbg.print("     "); \
            Serial_dbg.print((float)data[12] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.print((float)data[18] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.print((float)data[24] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.println((float)data[30] / 100.0, 2);
    }
    else {
        Serial_dbg.println("Error reading voltage");
    }

    if (data[1] != NAN || data[7] != NAN || data[13] != NAN || data[19] != NAN || data[25] != NAN || data[31] != NAN) {
        Serial_dbg.print("Current: "); Serial_dbg.print((float)data[1] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.print((float)data[7] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.print((float)data[13] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.print((float)data[19] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.print((float)data[25] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.println((float)data[31] / 100.0, 2);
    }
    else {
        Serial_dbg.println("Error reading current");
    }

    if (data[2] != NAN || data[8] != NAN || data[14] != NAN || data[20] != NAN || data[26] != NAN || data[32] != NAN) {
        Serial_dbg.print("Power: "); Serial_dbg.print((float)data[2] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.print((float)data[8] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.print((float)data[14] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.print((float)data[20] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.print((float)data[26] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.println((float)data[32] / 100.0, 2);
    }
    else {
        Serial_dbg.println("Error reading Power");
    }

    if (data[3] != NAN || data[9] != NAN || data[15] != NAN || data[21] != NAN || data[27] != NAN || data[33] != NAN) {
        Serial_dbg.print("Energy: "); Serial_dbg.print((float)data[3] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.print((float)data[9] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.print((float)data[15] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.print((float)data[21] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.print((float)data[27] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.println((float)data[33] / 100.0, 2);
    }
    else {
        Serial_dbg.println("Error reading energy");
    }

    if (data[4] != NAN || data[10] != NAN || data[16] != NAN || data[22] != NAN || data[28] != NAN || data[34] != NAN) {
        Serial_dbg.print("Frequency: "); Serial_dbg.print((float)data[4] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.print((float)data[10] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.print((float)data[16] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.print((float)data[22] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.print((float)data[28] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.println((float)data[34] / 100.0, 2);
    }
    else {
        Serial_dbg.println("Error reading frequency");
    }

    if (data[5] != NAN || data[11] != NAN || data[17] != NAN || data[23] != NAN || data[29] != NAN || data[35] != NAN) {
        Serial_dbg.print("PF: "); Serial_dbg.print((float)data[5] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.print((float)data[11] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.print((float)data[17] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.print((float)data[23] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.print((float)data[29] / 100.0, 2); Serial_dbg.print("   "); \
            Serial_dbg.println((float)data[35] / 100.0, 2);
    }
    else {
        Serial_dbg.println("Error reading power factor");
    }
}
