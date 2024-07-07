---

SATCOM - A powerfull general purpose satellite controlled matrix switch.

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
                                    Calibratable matrix for a wide range of applications.
    A global location Sensor & Time Sensor. For projects requiring location data and or time to be positionally aware and syncronized.
         Tested with WTGPS300 (WITMOTION) Satellite module this project receives data from satellites, sorts that data into
     variables that can be used later, and creates new $ sentences containing new data, calculated from the data received.


                        Converts Latitude and Longitude from $ sentences into Decimal Latitude and Longitude.
  This helps when compaing latitude and longitude with other maps, these values are then stored and placed into a new $ sentence.

 
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


                                                      SENTENCE $SATCOM
                                                                                    
                      START Tag                Last Sat Time                    Converted Longitude        
                         |                   |               |                   |               |                  
                      $SATCOM,000000000000.00,000000000000.00,00.00000000000000,00.00000000000000,*Z
                             |               |               |                 |                              
                               DatetimeStamp                  Converted Latitude                                 


                             Create More Values For $ Ssentences And Add More Satellite Hardware.
                            Create Unique Ranging Sentences For Lists of Variable Specified Ranges.
          This sketch is a layer between a satellite module and other systems to provide a new sentence and or the above+.
    Ranging and other activities can be performed here but it may be preferrable to make satcom a pure layer between systems,
                                              depending on performance.
          If data is intened to be passed through to a more powerfull system then turning off coordinte_convert and using
          another (more powerfull) systems resources to perform the coordinate conversions may be preferrable however
                                      it can all be done here on the same system.
                  Sytem can be made more or less efficient/standalone by turning some bools on or off.

            Ultimately this system is being built to turn on/off many relays and will be interfaceable. The relays
          will be switching on/off other MCU's for performance and efficiency reasons, so that I have an interfaceable
        programmable satellite utility system with logging. plug everything in and make data easily useable for projects.
    For security reasons The peripheral MCU's should be on hard switches to enable/disable features provided by those MCU's.

    Due to limited resources I may not currently be able to make full use of such a module as the WTGPS300, however, this
    system once interfaceable, enables full utilization of the WTGPS300 should the resources become available. Use of the
    inertial navigation system data alone would require vehicular devices, drones, robots, etc. This system intends to
                              unlock the potential without having to modify the system later.
                The end system should consist of displays, control panel, relays at the back and sides and sufficient
                           RXD ports so that peripheral systems can share the WTGPS300 dump.
                  The control panel should provide direct access to the various matrices, allowing for the system
                    to be anything from a time keeping device to a complex autonomous flight control system, turning
                        on/off peripheral systems forming a central part of a larger system as a whole.
