//
// Created by yiyiyixu on 2/14/23.
//

#include "../headfiles/CommunicatinoBetween.h"

void Transfer(std::vector<std::string> information){
    std::string data;
    int a[6],b[6],c[6];
    Target x[6];
    data = std::accumulate(information.begin(),information.end(),data);
    for(int i = 0; i < 6; i++)
    {
        c[x[i].color] = i;
    }
    for(int i = 0;i < 3;i++)
        a[i] = int(data[i]);
    for(int i = 3;i < 6; i++)
        a[i] = int(data[i+1]);
    if(x[5].color != -1){
        for(int i = 3; i < 6; i++){
            SendOnTime(c[a[i]]);
            while(SerialReadBuffer() == 0){
                usleep(1000);
            }
            SendOnTime(6);
        }
    }
    for(int i = 0; i < 3; i++){
        a[i] = int(data[i]);
        SendOnTime(c[a[i]]);
        while(SerialReadBuffer() == 0){
            usleep(1000);
        }
        SendOnTime(6);
    }
}
