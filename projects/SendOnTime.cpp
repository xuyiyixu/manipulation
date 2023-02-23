#include "../headfiles/SendOnTime.h"

/**
 * 获取毫秒的时间
 * @return 毫秒
 */
long getCurrentTime() {
    struct timeval tv{};
    gettimeofday(&tv, nullptr);
    return tv.tv_sec * 1000000 + tv.tv_usec;
}

uint8_t *send_message_dynamic(MessageType msg_type, const uint8_t *msg_content, uint32_t length) {
    if (msg_type >= MESSAGE_TYPE_NUMBER) throw std::out_of_range("message type out of range");
    std::cout << "send message dynamic length::" << length << std::endl;

    std::shared_ptr<uint8_t> len8(new uint8_t );

    len8.get()[0] = length >> 24;
    len8.get()[1] = (length >> 16) & 0xff;
    len8.get()[2] = (length >> 8) & 0xff;
    len8.get()[3] = length & 0xff;

    auto *p_pack(new uint8_t[length + 8]{BEGIN_BYTE, NUC, msg_type, len8.get()[0], len8.get()[1], len8.get()[2], len8.get()[3]});
    //把数据塞进去
    for (int i = 0; i < length; i++) {
        p_pack[i + 7] = msg_content[i];
    }
    //lrc计算
    for (int j = 0; j < length + 7; j++) {
        p_pack[length + 7] += p_pack[j];
    }

    return p_pack;

}

std::shared_ptr<uint8_t > Information(uint16_t ThetaOfOne, uint16_t ThetaOfTwo, uint16_t Z) {
    std::shared_ptr<uint8_t> data(new uint8_t );
    data.get()[0] = ThetaOfOne & 0xFF;
    data.get()[1] = (ThetaOfOne >> 8) & 0xFF;
    data.get()[2] = ThetaOfTwo & 0xFF;
    data.get()[3] = (ThetaOfTwo >> 8) & 0xFF;
    data.get()[4] = Z & 0xFF;
    data.get()[5] = (Z >> 8) & 0xFF;
    return data;
}
static int count[7] = {0,0,0,0,0,0,1};

void SendOnTime(int signal) {
    SerialPortInitialization();
    int n = signal;
    SendMessege FINAL;
    FINAL = {Connection(n)};
    while (!FINAL.final.Theta_2.empty()) {
        unsigned int m;
        long time_stamp ;
        time_stamp = getCurrentTime();//getCurrentTime为上面介绍的函数
        uint8_t  *data1;
        std::shared_ptr<uint8_t> data;
        uint16_t theta_1 = Discretization(FINAL.final.Theta_1.front(),2);
        uint16_t theta_2 = Discretization(FINAL.final.Theta_2.front(),1);
        uint16_t z_aixs = Discretization(FINAL.final.Z.front(),3);
        data = Information(theta_1,theta_2,z_aixs);
        data1 = send_message_dynamic(POSE, data.get(), 8);
        SerialWrite(data1, 14);

        /**
         * 计算睡眠时间（毫秒），保证每隔20ms循环一次
         */
        int dis = int(getCurrentTime() - time_stamp);
        if ((dis >= 10) || (dis <= 0)) {
            m = 0;
        } else {
            m = 10 - dis;
        }
        if(FINAL.final.st.front() == 1){
            if(count[signal]%2 == 0){
                data = Information(0,0,0);
                data1 = send_message_dynamic(ARM, data.get(), 8);
                SerialWrite(data1, 14);//之后改，还得加分类讨论是抓还是放
            }
            else{
                data = Information(1,1,1);
                data1 = send_message_dynamic(ARM, data.get(), 8);
                SerialWrite(data1, 14);//之后改，还得加分类讨论是抓还是放
            }
        }


        FINAL.final.Theta_1.pop();
        FINAL.final.Theta_2.pop();
        FINAL.final.Z.pop();
        FINAL.final.st.pop();
        /**
         * usleep()头文件：unistd.h
         */
        usleep(m);//等待m*1000微秒后继续循环，
    }
    count[signal] +=1;
}

