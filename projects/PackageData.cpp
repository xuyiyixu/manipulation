//
// Created by asus on 2023/1/20.
//

#include "../headfiles/PackageData.h"

/**
 * LRC校验函数
 * @param data 需要校验的数据
 * @param data_len 数据长度
 * @return
 */
unsigned char LRC(unsigned char data[], int data_len)
{
    unsigned char lrc = 0;


    for (int i = 0; i < data_len; i++)
    {
        lrc ^=  data[i];
        //printf("%02X\n",lrc);
    }
    return lrc;
}


/**
 * 发送数据函数
 * @param value ,输入的double类型数据
 */
void ADC_ConvertSend(double theta1,double speed,double theta2)
{
    unsigned char temp[12];
    char CheckStart = 0x7A;//数据传输开始
    char CheckEnd='\n';//表示一次传输完成
    unsigned short i=0,j=0;
    double doubleVariable1=theta1;
    auto * fortheta1 = (unsigned char *)&doubleVariable1;
    auto * fortheta2 = (unsigned char *)&theta2;
    auto * forspeed = (unsigned char *)&speed;
    temp[0]=CheckStart;//数据包头
    temp[1]=0x01;
    temp[2]=0x02;
    temp[3]=6;
    for(i=4;i<6;i++)//关节一角度
    {
        temp[i] = *fortheta1++;//double转BYTE
    }
    for( j=6;j<8;j++)
    {
        temp[i] = *forspeed++;
    }
    for( j=8;j<10;j++)
    {
        temp[i] = *fortheta2++;
    }
    temp[10] = LRC(temp,10);
    temp[11] = CheckEnd;//数据包尾
}
