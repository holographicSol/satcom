# satcom
Read, parse and sort data from a satellite downlink with a WTGPS300 and a microcontroller.

Optionally outputs to a SSD1306. Santization (in this case string lenghts) for SSD1306 is undergoing
development to ensure each displayed string(s) are no longer than SSD1306 columns max.

Ultimately RF24 with noaa/more sat compatibility will be added and this for another Rad node but I will keep this code here
as a pure and simple WTGPS300 downlink.
