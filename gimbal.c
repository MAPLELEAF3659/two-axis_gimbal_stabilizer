#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <pigpio.h>
#include <math.h>

#define SERVO1_PIN 18 // WiringPi 引腳號（對應 GPIO18）
#define SERVO2_PIN 12 // WiringPi 引腳號（對應 GPIO26）

// 設置角度對應的 PWM 寬度 (單位: 微秒)
int angle_to_pulse(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    return 500 + (angle * 2000) / 180; // 500~2500 微秒對應 0°~180°
}

int main(int argc, char *argv[])
{
    if (gpioInitialise() < 0) {
        printf("pigpio init！\n");
        return 1;
    }

    short resive_data[3];
    float roll, pitch;
    printf("mpu6050 test\n");

    int fd = open("/dev/ml-mpu6050", O_RDWR);
    if(fd < 0)
    {
        printf("open file : %s failed !\n", argv[0]);
        return -1;
    }
    int error = 0;

    while (error == 0) {
        error = read(fd, resive_data, 6);
        if(error < 0)
        {
            printf("read file error! \n");
            close(fd);
            return 0;
        }
        
        printf("AX=%d, AY=%d, AZ=%d, ", (int)resive_data[0], (int)resive_data[1], (int)resive_data[2]);
        //printf("GX=%d, GY=%d, GZ=%d \n \n",(int)resive_data[3],(int)resive_data[4],(int)resive_data[5]);

        roll = atan2((float)resive_data[1], sqrt((float)(resive_data[0] * resive_data[0] + resive_data[2] * resive_data[2]))) * (180.0 / M_PI);
        pitch = atan2(-(float)resive_data[0], (float)resive_data[2]) * (180.0 / M_PI);

        gpioServo(SERVO1_PIN, angle_to_pulse(90 + roll));
        gpioServo(SERVO2_PIN, angle_to_pulse(70 + pitch + 80));

        printf("roll=%f, pitch=%f\n", roll, pitch);

        usleep(200000); // ns
    }

    error = close(fd);
    if(error < 0)
    {
        printf("close file error! \n");
    }

    return 0;
}
