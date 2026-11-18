#include "Arduino.h"

unsigned char arduinoID;
serial::Serial arduinoPort;
void Arduino::serinit(){
    arduinoPort.setPort("/dev/ttyS3");//选择要开启的串口号
    arduinoPort.setBaudrate(115200);//设置波特率
    try {
        arduinoPort.open();
    } catch (...) {
        // 仿真/无物理串口环境：打开失败则静默跳过，不崩溃
    }
};

void Arduino::send(){
    if (!arduinoPort.isOpen()) return;  // 仿真环境无串口，直接返回
     unsigned char arduinolink[4] = { 0xfe, 0xfd, 0, 0};
    arduinolink[2] = arduinoID;
    arduinolink[3] = arduinoID%2;
    arduinoPort.write(arduinolink,sizeof(arduinolink));
    sleep(1);
};

void Arduino::bomb_0(){
    arduinoID = 1;
    Arduino::send();
  };

void Arduino::bomb_1(){
    arduinoID = 2;
    Arduino::send();
  };

void Arduino::bomb_2(){
    arduinoID = 3;
    Arduino::send();
  };

void Arduino::led_1s(){
    arduinoID = 4;
    Arduino::send();
  };

void Arduino::led_on(){
    arduinoID = 5;
    Arduino::send();
  };

void Arduino::loud(){
    arduinoID = 7;
    Arduino::send();
  };

void Arduino::laser(){
    arduinoID = 8;
    Arduino::send();
  };

void Arduino::oak_up(){
    arduinoID = 9;
    Arduino::send();
  };

void Arduino::oak_down(){
    arduinoID = 10;
    Arduino::send();
  };

void Arduino::bigbomb_1(){
    arduinoID = 11;
    Arduino::send();
  };

void Arduino::bigbomb_2(){
    arduinoID = 12;
    Arduino::send();
  };

void Arduino::bigbomb_3(){
    arduinoID = 14;
    Arduino::send();
  };