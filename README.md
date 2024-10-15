# satcom
Read, parse and sort data from a satellite downlink with a WTGPS300 and a microcontroller.

Optionally outputs to a SSD1306. Santization (in this case string lenghts) for SSD1306 is undergoing
development to ensure each displayed string(s) are no longer than SSD1306 columns max.

Ultimately RF24 with noaa/more sat compatibility will be added to this for another Rad node but I will keep this code here
as a pure and simple WTGPS300 downlink.

A TCA9548A i2C multiplexer is required for satcom multiplexed, which allows more information to be displayed
without using a larger although possibly dimmer oled display over the bright monochrome SSD1306's, which I prefer.

Overview: If we would like to make a clock that utilizes the time element of an NMEA sentenece then we could make
and embed a function that does just that.. But what about next time we need to write for a different system that needs
not only time, but time and or other GPS data? Start again? No thank you. This is a programmable system that intends
to utilize each and every element of an NMEA sentence including INS data, that can be used for an infinite number of projects
weather standalone (with a relay hat responding to eros and ones) or as a hardware module (sending zeros and ones over serial).
