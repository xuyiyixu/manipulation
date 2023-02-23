#include "../headfiles/SerialCommunication.h"

int fd;

/**
 * 检查串口是否打开
 */
void SerialCheck() {
    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) /* Error Checking */
        printf("\n  Error! in Opening ttyUSB0  ");
    else
        printf("\n  ttyUSB0 Opened Successfully ");
}

/**
 * 设置串口数据
 */
void SerialPortInitialization() {

    SerialCheck();

    struct termios SerialPortSettings;

    tcgetattr(fd, &SerialPortSettings);

    //设置波特率
    cfsetispeed(&SerialPortSettings, B115200);
    cfsetospeed(&SerialPortSettings, B115200);

    //设置没有校验
    SerialPortSettings.c_cflag &= ~PARENB;

    //停止位 = 1
    SerialPortSettings.c_cflag &= ~CSTOPB;
    SerialPortSettings.c_cflag &= ~CSIZE;

    //设置数据位 = 8
    SerialPortSettings.c_cflag |= CS8;

    SerialPortSettings.c_cflag &= ~CRTSCTS;
    SerialPortSettings.c_cflag |= CREAD | CLOCAL;

    //关闭软件流动控制
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);

    //设置操作模式
    SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    SerialPortSettings.c_oflag &= ~OPOST;

    if ((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0)
        printf("\n  ERROR ! in Setting attributes");
    else
        printf("\n  BaudRate = 115200 \n  StopBits = 1 \n  Parity   = none" );
}

/**
 * 写数据
 * @param writeBuffer 需要读进去的数据
 * @return 范围是否发送成功，0不成功，1成功
 */
bool SerialWrite(uint8_t *writeBuffer, uint8_t length) {
    ssize_t bytes_written = write(fd, writeBuffer, length);
    printf("\n  %s written to ttyUSB0", writeBuffer);
    printf("\n  %zd Bytes written to ttyUSB0", bytes_written);
    printf("\n +----------------------------------+\n\n");
    if(bytes_written == -1)
        return SerialWrite(writeBuffer,length);
    else
        return true;
}

uint8_t readBuffer[1024] = {0};

/**
 * 发送数据
 * @return 0发送不成功，1发送成功
 */
int SerialReadBuffer() {
    ssize_t serialReadCnt = read(fd, readBuffer, sizeof(readBuffer));

    usleep(2000);
    if(readBuffer[0]=='\n')
        return true;

    if(serialReadCnt == -1) {
        return false;
    }
    else {
        printf("bytesCnt = %zd ", serialReadCnt);
        printf(" %s read to ttyUSB0 \r\n\r\n", readBuffer);
        return readBuffer[0];
    }
}

void SerialPortDeinitialization() {
    close(fd);
}
