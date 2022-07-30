#ifndef __Shell__
#define __Shell__

#include <Arduino.h>
// #include <Wire.h>

#include "Shell.h"

int scan_device(int argc, char **argv)
{
    shell_println("-----------------------------------------------");

    shell_println("-----------------------------------------------");
    return SHELL_RET_SUCCESS;
}

int help(int argc, char **argv)
{
    shell_println("-----------------------------------------------");
    shell_println(" ====M5Stack Fire I2C MUX Control Center===== ");
    shell_println(" use command below to control your I2C device ");
    shell_println(" device> haptic [CHANNEL] [EFFECT ID]");
    shell_println("         actuate haptic motor with drv2605, which is connected to TCA9548A at [CHANNEL (0-7)] : %s with [EFFECT ID (1-123)] :");
    shell_println("         example 1: haptic 0 1");
    shell_println("         example 2: haptic 7 123");
    shell_println(" device> dac [CHANNEL] [VOLTAGE]");
    shell_println("         generate analog voltage with MCP25, which is connected to TCA9548A at [CHANNEL (0-7)] : %s with [VOLTAGE (0-4000)] :");
    shell_println("         example 1: dac 0 0");
    shell_println("         example 2: dac 7 4000");
    shell_println(" device> help-display this help");
    shell_println("-----------------------------------------------");
    return SHELL_RET_SUCCESS;
}

uint16_t channel;
uint8_t effect_id;
byte loop_times;

int act_haptic(int argc, char **argv)
{
    shell_println("-----------------------------------------------");
    // shell_printf("actuate haptic motor[0-7]: %s, effect ID[1-123]: %s\r\n", argv[1], argv[2]);
    shell_println("-----------------------------------------------");
    channel = atoi(argv[1]);
    effect_id = atoi(argv[2]);
    loop_times = atoi(argv[3]);
    for (byte i = 0; i < loop_times; i++)
    {
        // drv.go();
        shell_printf("- %d\r\n", i);
        delay(100);
    }
    
    // mux.setNoChannel();
    return SHELL_RET_SUCCESS;
}


/**
 * Function to read data from serial port
 * Functions to read from physical media should use this prototype:
 * int my_reader_function(char * data)
 */
int shell_reader(char *data)
{
    // Wrapper for Serial.read() method
    if (Serial.available())
    {
        *data = Serial.read();
        return 1;
    }
    return 0;
}

/**
 * Function to write data to serial port
 * Functions to write to physical media should use this prototype:
 * void my_writer_function(char data)
 */
void shell_writer(char data)
{
    // Wrapper for Serial.write() method
    Serial.write(data);
}


class Shell{
    public:
    Shell(){}
    ~Shell(){};
    void begin(){
        shell_init(shell_reader, shell_writer, 0);
        shell_register(help, PSTR("help"));
        shell_register(act_haptic, PSTR("haptic"));
        // shell_register(act_haptic_all, PSTR("haptic_all"));
        // shell_register(generate_dac, PSTR("dac"));
        shell_register(scan_device, PSTR("scan"));
        // mux.begin();
        // mux.scanChannel();
    };
    void run(){
        shell_task();
    }
    private:
};
#endif // !__Shell__
