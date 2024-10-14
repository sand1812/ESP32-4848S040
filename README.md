# ESP32-4848S040
ESP32-4848S040 + lvgl platform.io example

This is an example project using lgvl / Arduino GFX for the ESP32-4848S040

This hardware includes :
- ESP32-S3 processor
- 16 MByte Flash in QIO mode
- 8 MByte PSRAM OPI
- 4.3 inch, 480 * 480 px Display based on ST7701: 16 bit color, special ESP32-S3 parallel mode The display is supported by the “GFX Library for Arduino”.
- Touch Sensor: GT911 on I2C, Address 0x5D
- I2C bus using SDA=19, CLK=20
- SD Card slot
- 3 relays


Using lvgl@^9.2.0