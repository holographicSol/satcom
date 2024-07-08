---

SATCOM - A powerfull general purpose programmable, satellite controlled matrix switch.

---

![plot](./resources/img_001.JPG)

---

![plot](./resources/img_000.JPG)

---

Pending documentation as SATCOM will require a handbook.

---

                                          SATCOM - Written by Benjamin Jack Cullen.

                                                                                                                       
                                     Receives and Processes Transmissions from Satellites.
                   Possible combinations example: 100 checks ^ 10 functions = 100,000,000,000,000,000,000 combinations.
                                                                              (100 Quintillion)
                                    Calibratable matrix for a wide range of applications.
    A global location Sensor & Time Sensor. For projects requiring location data and or time to be positionally aware and syncronized.
         Tested with WTGPS300 (WITMOTION) Satellite module this project receives data from satellites, sorts that data into
     variables that can be used later, and creates new $ sentences containing new data, calculated from the data received.


                        Converts Latitude and Longitude from $ sentences into Decimal Latitude and Longitude.

 
                                                       Timestamps
      A datetime string is created from GNGGA and GNRMC $ sentence information, forms a part of the new $SATCOM sentence
                                                     

                                             Timestamp Zero Satellite Periods
  A Timestamp is created for keeping track of when the system last heard satellites, forms part of the new $SATCOM sentence
                        

                                                        Ranging
Specified coordinates at specified meter/mile ranges. For location pinning, guidance, tracking, motion detection, autonomy  etc.
                                  these values can be added to the new $SATCOM sentence.

                                                
                                                       Serial Dump
   All extra calculated data is dumped in its own $SATCOM sentence along with the other $ sentences which are passed through
        as they would be seen as if the WTGPS300 is plugged in via USB. Dumping more (sentences/information) and dumping in
                                        a standard way and for generic use.

                                        
                                                       Compatibility
                     To Be Potentially Anything And Function As A Hardware Plugin For Other Projects
                                         Headless / Standalone / Serial / Remote


                              Wiring for Optional Multiplexed OLED Displays (SSD1306 Monochromes)
                                       WTGPS300 TX              --> ESP32 io26 as RXD
                                       WTGPS300 VCC             --> ESP32 3.3/5v
                                       TCA9548A i2C Multiplexer --> ESP32 i2C
                                       x3 SSD1306               --> TCA9548A i2C Multiplexer

                                       
                                  Wiring for Optional MicroSD Card (ESP32 Keyestudio Dev Board)
                                                  HW-125 MicroSDCard Module
                                                        CS    ->  5
                                                        MISO  ->  19
                                                        MOSI  ->  23
                                                        SCK   ->  18


                                                      SENTENCE $SATCOM
                                                                                    
                      START Tag                Last Sat Time                    Converted Longitude        
                         |                   |               |                   |               |                  
                      $SATCOM,000000000000.00,000000000000.00,00.00000000000000,00.00000000000000,*Z
                             |               |               |                 |                              
                               DatetimeStamp                  Converted Latitude                                 


                             Create More Values For $ Ssentences And Add More Satellite Hardware.
          This sketch is a layer between a satellite module and other systems to provide a new sentence and or the above+.

            Ultimately this system is being built to turn on/off many multiplexed relays (40+) and will have a control panel.
             The relays will be switching on/off other MCU's for modularity/performance/efficiency reasons, so that I have
                               an interfaceble, programmable satellite utility system with logging.
                                        Basically a big switch I can plug things into to.

                            Time server to robots and flying machines, and everything in between.
                       Some star and constellation calculations will be added soon that can be used at will
                              along with any other data currently available to calculate with.

---



---
