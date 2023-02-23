#ifndef MANIPULATOR_SERIALCOMMUNICATION_H
#define MANIPULATOR_SERIALCOMMUNICATION_H

#include <cstdio>
#include <cstdint>
#include <cerrno>

#include <fcntl.h>
#include <unistd.h>

#include <termios.h>


void SerialPortInitialization();
void SerialPortDeinitialization();
void SerialCheck();
bool SerialWrite(uint8_t *writeBuffer, uint8_t length);
int SerialReadBuffer();

#endif //MANIPULATOR_SERIALCOMMUNICATION_H

