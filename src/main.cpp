// --------------------------------------
// i2c_scanner
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    https://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// Version 6, November 27, 2015.
//    Added waiting for the Leonardo serial communication.
//
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//

// +-------------------------------------------------------------
//
// Equipment:
// ESP32-Dev Kit V1, MPU-6050, OLED SSD1306, WS2812B Addressable LEDS x3,
// 4-relay module, 2 linear actuators, RT-11(JCHT35K9) remote/handset,
// and 650W 12V Power supply
//
// File: main.cpp
//
// Description:
//
// Creates effects on addressable LEDs, displays info on OLED Screen
// and provides commands to standup desk
//
// History:     2-Feb-2024     Scarecrow1965   Created
//
// +-------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

// for OLED screen
// #include <SPI.h>
#include <U8g2lib.h>
#include <Adafruit_SSD1306.h>

// for MPU-6050
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Adafruit_MPU6050.h>

#define OLED_CLOCK 22  // SCA pin on Display = pin 17 (I2C_SCL) on ESP32 DEVKIT V1 = GPIO 22
#define OLED_DATA 21   // SDL pin on display = pin 20 (I2C_SDA) on ESP32 DEVKIT V1 = GPIO 21
// This works but according to the function, it shouldn't
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, OLED_CLOCK, OLED_DATA, U8X8_PIN_NONE);
float fps = 0; // to start recording the fps of the display
static bool bLED = LOW;
static uint8_t oled_LineH = 0;

// #define LED_BUILTIN 2   // GPIO2 of ESP32
#define INTERRUPT_PIN 4 // interrupt connected to MPU-6050 // GPIO 4 on the ESP32
MPU6050 mpu1;
int16_t AccX, AccY, AccZ, Temp, GyroX, GyroY, GyroZ;
int16_t previousGyroX = 0, previousGyroY = 0, previousAccX = 0, previousAccY = 0; // to help with providing an interrupt
Adafruit_MPU6050 mpu2;
const int MPU6050_addr = 0x68; // to initialize the address of the MPU-6050
// const int MPU6050_addr = 0x3c;// #define SCREEN_I2C_ADDR 0x3C = normally would be the OLED screen address

void setup()
{
    Wire.begin();

    Serial.begin(115200);
    while (!Serial)
        ; // Leonardo: wait for serial monitor
    Serial.println("\nI2C Scanner");

    pinMode(LED_BUILTIN, OUTPUT); // relying on GPIO2 LED to light up on MB

    u8g2.begin();
    u8g2.clear();
    u8g2.setFont(u8g2_font_profont10_tf);
    oled_LineH = u8g2.getFontAscent() + u8g2.getFontAscent();
    if (!u8g2.begin())
    {
        Serial.println(F("SSD1306 allocation failed!"));
        for (;;)
        {
            // don't proceed, loop forever
        }
    }
    Serial.println("U8G2 is loaded and ready"); // used for testing purposes only

    // display.begin(SSD1306_SWITCHCAPVCC, SCREEN_I2C_ADDR);
    // display.clearDisplay();
    
    // if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_I2C_ADDR))
    // { // Address 0x3D for 128x64
    //     Serial.println(F("SSD1306 allocation failed"));
    //     for (;;)
    //     {
    //         // don't proceed, loop forever
    //     }
    // }
    // Serial.println("Adafruit library up and running"); // used for testing purposes only

} // end setup function

void loop()
{
    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.print(address, HEX);
            Serial.println("  !");

            nDevices++;
        }
        else if (error == 4)
        {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.println(address, HEX);
        }
    }

    if (nDevices == 0)
    {
    Serial.println("No I2C devices found\n");
    }
    else
    {
        Serial.println("done\n");
    }

    delay(5000); // wait 5 seconds for next scan
} // end loop function
