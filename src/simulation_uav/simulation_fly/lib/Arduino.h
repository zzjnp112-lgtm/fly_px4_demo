#include <serial/serial.h> 

#include <stdio.h>
#include <stdlib.h>
#include <csignal>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>


class Arduino
{
    public:
        static void serinit();
        static void send();
        static void bomb_0();
        static void bomb_1();
        static void bomb_2();
        static void led_1s();
        static void led_on();
        static void loud();
        static void laser();
        static void oak_up();
        static void oak_down();
        static void bigbomb_1();
        static void bigbomb_2();
        static void bigbomb_3();
  };
