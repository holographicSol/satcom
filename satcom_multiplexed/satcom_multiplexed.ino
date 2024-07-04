/*

                                          SATCOM - Written by Benjamin Jack Cullen.

                                                                                                                       
                                     Receives and Processes Transmissions from Satellites.
                                  Calibratable/feedable matrix for a wide range of applications.
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
                                                                                    
                      START Tag                Last Sat Time                     Converted Longitude        
                         |                   |               |                   |                |                  
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
*/

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    LIBRARIES

#include <stdio.h>
#include <string.h>
#include <Wire.h>
#include <SSD1306Wire.h>   // SSD1306Wire                                https://gitlab.com/alexpr0/ssd1306wire
#include <OLEDDisplayUi.h> // ESP8266 and ESP32 OLED driver for SSD1306  https://github.com/ThingPulse/esp8266-oled-ssd1306
#include <Timezone.h>      // Timezone                                   https://github.com/JChristensen/Timezone

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                      DEFINES

#define TCAADDR 0x70

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                       WIRING

SSD1306Wire   display_7(0x3c, SDA, SCL);  // let SSD1306Wire wire up our SSD1306 on the i2C bus
SSD1306Wire   display_6(0x3c, SDA, SCL); // let SSD1306Wire wire up our SSD1306 on the i2C bus
SSD1306Wire   display_5(0x3c, SDA, SCL); // let SSD1306Wire wire up our SSD1306 on the i2C bus
SSD1306Wire   display_4(0x3c, SDA, SCL); // let SSD1306Wire wire up our SSD1306 on the i2C bus
SSD1306Wire   display_3(0x3c, SDA, SCL); // let SSD1306Wire wire up our SSD1306 on the i2C bus

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   DEBUG DATA

struct sysDebugStruct {
  bool gngga_sentence = true;
  bool gnrmc_sentence = false;
  bool gpatt_sentence = false;
  bool desbi_sentence = false;
};
sysDebugStruct sysDebugData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                  SERIAL DATA

struct SerialStruct {
  unsigned long nbytes;
  unsigned long iter_token;
  char BUFFER[2048];
  char * token = strtok(BUFFER, ",");
};
SerialStruct serialData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                      MULTIPLEXER PORT SELECT

void tcaselect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                          INITIALIZE DISPLAY

void initDisplay7() {
  display_7.init();
  display_7.flipScreenVertically();
  display_7.setContrast(255);
  display_7.setFont(ArialMT_Plain_10);
  display_7.cls();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                          INITIALIZE DISPLAY

void initDisplay6() {
  display_6.init();
  display_6.flipScreenVertically();
  display_6.setContrast(255);
  display_6.setFont(ArialMT_Plain_10);
  display_6.cls();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                          INITIALIZE DISPLAY

void initDisplay5() {
  display_5.init();
  display_5.flipScreenVertically();
  display_5.setContrast(255);
  display_5.setFont(ArialMT_Plain_10);
  display_5.cls();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                          INITIALIZE DISPLAY

void initDisplay4() {
  display_4.init();
  display_4.flipScreenVertically();
  display_4.setContrast(255);
  display_4.setFont(ArialMT_Plain_10);
  display_4.cls();
}


// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                          INITIALIZE DISPLAY

void initDisplay3() {
  display_3.init();
  display_3.flipScreenVertically();
  display_3.setContrast(255);
  display_3.setFont(ArialMT_Plain_10);
  display_3.cls();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   VALIDATION

/*
checks can be ellaborated upon individually
*/

bool val_utc_time(char * data) {
  bool check_pass = false;
  if (strlen(data) == 9) {
    if (data[6] == '.') {
      if ((atoi(data) >= 0.0) && (atoi(data) <= 235959.99)) {check_pass = true;}
    }
  }
  return check_pass;
}

bool val_latitude(char * data) {
  bool check_pass = false;
  if (strlen(data) == 13) {
    if (data[4] == '.') {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_longitude(char * data) {
  bool check_pass = false;
  if (strlen(data) == 14) {
    if (data[5] == '.') {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_latitude_H(char * data) {
  bool check_pass = false;
  if (strlen(data) == 1) {
    if ((data[0] == 'N') || (data[0] == 'S')) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_longitude_H(char * data) {
  bool check_pass = false;
  if (strlen(data) == 1) {
    if ((data[0] == 'E') || (data[0] == 'W')) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_positioning_status(char * data) {
  bool check_pass = false;
  if (strlen(data) == 1) {
    if ((atoi(data) >= 0) && (atoi(data) <= 6)) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_satellite_count(char * data) {
  bool check_pass = false;
  if (atoi(data) >= 0){
      check_pass = true;
    }
  return check_pass;
}

bool val_hdop_precision_factor(char * data) {
  bool check_pass = false;
  if (atoi(data) >= 0){
      check_pass = true;
    }
  return check_pass;
}

bool val_altitude(char * data) {
  bool check_pass = false;
  if (atoi(data) >= -20000000){
      check_pass = true;
    }
  return check_pass;
}

bool val_altitude_units(char * data) {
  bool check_pass = false;
  if (strlen(data) == 1) {
    if (data[0] == 'M') {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_geoidal(char * data) {
  bool check_pass = false;
  if (atoi(data) >= 0){
    check_pass = true;
  }
  return check_pass;
}

bool val_geoidal_units(char * data) {
  bool check_pass = false;
  if (strlen(data) == 1) {
    if (data[0] == 'M') {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_differential_delay(char * data) {
  bool check_pass = false;
  if (atoi(data) >= 0){
    check_pass = true;
  }
  return check_pass;
}

bool val_basestation_id(char * data) {
  bool check_pass = false;
  if (strlen(data) == 4) {
    check_pass = true;
  }
  return check_pass;
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   GNGGA DATA

struct GNGGAStruct {
  char tag[56];                         // <0> Log header
  char utc_time[56];                    // <1> UTC time, the format is hhmmss.sss
  char latitude[56];                    // <2> Latitude, the format is  ddmm.mmmmmmm
  char latitude_hemisphere[56];         // <3> Latitude hemisphere, N or S (north latitude or south latitude)
  char longitude[56];                   // <4> Longitude, the format is dddmm.mmmmmmm
  char longitude_hemisphere[56];        // <5> Longitude hemisphere, E or W (east longitude or west longitude)
  char positioning_status[56];          // <6> GNSS positioning status: 0 not positioned, 1 single point positioning, 2: pseudorange difference, 6: pure INS */
  char satellite_count_gngga[56] = "0"; // <7> Number of satellites used
  char hdop_precision_factor[56];       // <8> HDOP level precision factor
  char altitude[56];                    // <9> Altitude
  char altitude_units[56];              // <10> 
  char geoidal[56];                     // <11> The height of the earth ellipsoid relative to the geoid 
  char geoidal_units[56];               // <12> 
  char differential_delay[56];          // <13>
  char id[56];                          // <14> base station ID
  char check_sum[56];                   // <15> XOR check value of all bytes starting from $ to *
  char temporary_data[56];
  int check_data = 0;                   // should be 16
};
GNGGAStruct gnggaData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        GNGGA

//

void GNGGA() {

  gnggaData.check_data = 0;
  
  memset(gnggaData.tag, 0, 56);
  memset(gnggaData.utc_time, 0, 56);
  memset(gnggaData.latitude, 0, 56);
  memset(gnggaData.latitude_hemisphere, 0, 56);
  memset(gnggaData.longitude, 0, 56);
  memset(gnggaData.longitude_hemisphere, 0, 56);
  memset(gnggaData.positioning_status, 0, 56);
  memset(gnggaData.satellite_count_gngga, 0, 56);
  memset(gnggaData.hdop_precision_factor, 0, 56);
  memset(gnggaData.altitude, 0, 56);
  memset(gnggaData.altitude_units, 0, 56);
  memset(gnggaData.geoidal, 0, 56);
  memset(gnggaData.geoidal_units, 0, 56);
  memset(gnggaData.differential_delay, 0, 56);
  memset(gnggaData.id, 0, 56);
  memset(gnggaData.check_sum, 0, 56);
  memset(gnggaData.temporary_data, 0, 56);
  serialData.iter_token = 0;
  serialData.token = strtok(serialData.BUFFER, ",");
  while( serialData.token != NULL ) {
    if     (serialData.iter_token == 0)                                                            {strcpy(gnggaData.tag, "GNGGA");                            gnggaData.check_data++;}
    else if (serialData.iter_token ==1)  {if (val_utc_time(serialData.token) == true)              {strcpy(gnggaData.utc_time, serialData.token);              gnggaData.check_data++;}}
    else if (serialData.iter_token ==2)  {if (val_latitude(serialData.token) == true)              {strcpy(gnggaData.latitude, serialData.token);              gnggaData.check_data++;}}
    else if (serialData.iter_token ==3)  {if (val_latitude_H(serialData.token) == true)            {strcpy(gnggaData.latitude_hemisphere, serialData.token);   gnggaData.check_data++;}}
    else if (serialData.iter_token ==4)  {if (val_longitude(serialData.token) == true)             {strcpy(gnggaData.longitude, serialData.token);             gnggaData.check_data++;}}
    else if (serialData.iter_token ==5)  {if (val_longitude_H(serialData.token) == true)           {strcpy(gnggaData.longitude_hemisphere, serialData.token);  gnggaData.check_data++;}}
    else if (serialData.iter_token ==6)  {if (val_positioning_status(serialData.token) == true)    {strcpy(gnggaData.positioning_status, serialData.token);    gnggaData.check_data++;}}
    else if (serialData.iter_token ==7)  {if (val_satellite_count(serialData.token) == true)       {strcpy(gnggaData.satellite_count_gngga, serialData.token); gnggaData.check_data++;}}
    else if (serialData.iter_token ==8)  {if (val_hdop_precision_factor(serialData.token) == true) {strcpy(gnggaData.hdop_precision_factor, serialData.token); gnggaData.check_data++;}}
    else if (serialData.iter_token ==9)  {if (val_altitude(serialData.token) == true)              {strcpy(gnggaData.altitude, serialData.token);              gnggaData.check_data++;}}
    else if (serialData.iter_token ==10) {if (val_altitude_units(serialData.token) == true)        {strcpy(gnggaData.altitude_units, serialData.token);        gnggaData.check_data++;}}
    else if (serialData.iter_token ==11) {if (val_geoidal(serialData.token) == true)               {strcpy(gnggaData.geoidal, serialData.token);               gnggaData.check_data++;}}
    else if (serialData.iter_token ==12) {if (val_geoidal_units(serialData.token) == true)         {strcpy(gnggaData.geoidal_units, serialData.token);         gnggaData.check_data++;}}
    else if (serialData.iter_token ==13) {if (val_differential_delay(serialData.token) == true)    {strcpy(gnggaData.differential_delay, serialData.token);    gnggaData.check_data++;}}
    else if (serialData.iter_token ==14) {if (strlen(serialData.token) == 8) {strncpy(gnggaData.temporary_data, serialData.token, 4);
    if (val_basestation_id(gnggaData.temporary_data) == true) {strcpy(gnggaData.id, gnggaData.temporary_data); gnggaData.check_data++;} serialData.token = strtok(serialData.token, "*"); serialData.token = strtok(NULL, "*");}
    if (strlen(serialData.token) == 3) {strcpy(gnggaData.check_sum, serialData.token); gnggaData.check_data++;}}
    serialData.token = strtok(NULL, ",");
    serialData.iter_token++;
  }
  if (sysDebugData.gngga_sentence == true) {
    Serial.println("[gnggaData.tag] "                     + String(gnggaData.tag));
    Serial.println("[gnggaData.utc_time] "                + String(gnggaData.utc_time));
    Serial.println("[gnggaData.latitude] "                + String(gnggaData.latitude));
    Serial.println("[gnggaData.latitude_hemisphere] "     + String(gnggaData.latitude_hemisphere));
    Serial.println("[gnggaData.longitude] "               + String(gnggaData.longitude));
    Serial.println("[gnggaData.longitude_hemisphere] "    + String(gnggaData.longitude_hemisphere));
    Serial.println("[gnggaData.positioning_status] "      + String(gnggaData.positioning_status));
    Serial.println("[gnggaData.satellite_count_gngga] "   + String(gnggaData.satellite_count_gngga));
    Serial.println("[gnggaData.hdop_precision_factor] "   + String(gnggaData.hdop_precision_factor));
    Serial.println("[gnggaData.altitude] "                + String(gnggaData.altitude));
    Serial.println("[gnggaData.altitude_units] "          + String(gnggaData.altitude_units));
    Serial.println("[gnggaData.geoidal] "       + String(gnggaData.geoidal));
    Serial.println("[gnggaData.geoidal_units] " + String(gnggaData.geoidal_units));
    Serial.println("[gnggaData.differential_delay] " + String(gnggaData.differential_delay));
    Serial.println("[gnggaData.id] "                      + String(gnggaData.id));
    Serial.println("[gnggaData.check_sum] "               + String(gnggaData.check_sum));
    Serial.println("[gnggaData.check_data] "              + String(gnggaData.check_data));
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   GNRMC DATA

struct GNRMCStruct {
  char tag[56];                            // <0> Log header
  char utc_time[56];                       // <1> UTC time, the format is hhmmss.sss
  char positioning_status[56];             // <2> Positioning status, A=effective positioning, V=invalid positioning
  char latitude[56];                       // <3> Latitude, the format is  ddmm.mmmmmmm
  char latitude_hemisphere[56];            // <4> Latitude hemisphere, N or S (north latitude or south latitude)
  char longitude[56];                      // <5> Longitude, the format is dddmm.mmmmmmm
  char longitude_hemisphere[56];           // <6> Longitude hemisphere, E or W (east longitude or west longitude)
  char ground_speed[56];                   // <7> Ground speed
  char ground_heading[56];                 // <8> Ground heading (take true north as the reference datum)
  char utc_date[56];                       // <9> UTC date, the format is ddmmyy (day, month, year)
  char magnetic_declination[56];           // <10> Magnetic declination (000.0~180.0 degrees)
  char magnetic_declination_direction[56]; // <11> Magnetic declination direction, E (east) or W (west)
  char mode_indication[56];                // <12> Mode indication (A=autonomous positioning, D=differential E=estimation, N=invalid data) */
  char check_sum[56];                      // <13> XOR check value of all bytes starting from $ to *
  char temporary_data[56];
};
GNRMCStruct gnrmcData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        GNRMC

void GNRMC() {
  
  memset(gnrmcData.tag, 0, 56);
  memset(gnrmcData.utc_time, 0, 56);
  memset(gnrmcData.positioning_status, 0, 56);
  memset(gnrmcData.latitude, 0, 56);
  memset(gnrmcData.latitude_hemisphere, 0, 56);
  memset(gnrmcData.longitude, 0, 56);
  memset(gnrmcData.longitude_hemisphere, 0, 56);
  memset(gnrmcData.ground_speed, 0, 56);
  memset(gnrmcData.ground_heading, 0, 56);
  memset(gnrmcData.utc_date, 0, 56);
  memset(gnrmcData.magnetic_declination, 0, 56);
  memset(gnrmcData.magnetic_declination_direction, 0, 56);
  memset(gnrmcData.mode_indication, 0, 56);
  memset(gnrmcData.check_sum, 0, 56);

  serialData.iter_token = 0;
  serialData.token = strtok(serialData.BUFFER, ",");
  while( serialData.token != NULL ) {
    if     (serialData.iter_token == 0) {strcpy(gnrmcData.tag, "GNGGA");}
    else if (serialData.iter_token ==1) {if (strlen(serialData.token) <= 9) {strcpy(gnggaData.utc_time, serialData.token);}}
    else if (serialData.iter_token ==2) {strcpy(gnrmcData.positioning_status, serialData.token);}
    else if (serialData.iter_token ==3) {if (strlen(serialData.token) <= 17) {strcpy(gnrmcData.latitude, serialData.token);}}
    else if (serialData.iter_token ==4) {if (strlen(serialData.token) <= 1) {strcpy(gnrmcData.latitude_hemisphere, serialData.token);}}
    else if (serialData.iter_token ==5) {if (strlen(serialData.token) <= 17) {strcpy(gnrmcData.longitude, serialData.token);}}
    else if (serialData.iter_token ==6) {if (strlen(serialData.token) <= 1) {strcpy(gnrmcData.longitude_hemisphere, serialData.token);}}
    else if (serialData.iter_token ==7) {strcpy(gnrmcData.ground_speed, serialData.token);}
    else if (serialData.iter_token ==8) {strcpy(gnrmcData.ground_heading, serialData.token);}
    else if (serialData.iter_token ==9) {if (strlen(serialData.token) <= 6) {strcpy(gnrmcData.utc_date, serialData.token);}}
    else if (serialData.iter_token ==10) {strcpy(gnrmcData.magnetic_declination, serialData.token);}
    else if (serialData.iter_token ==11) {strcpy(gnrmcData.magnetic_declination_direction, serialData.token);}
    else if (serialData.iter_token ==12) {
      strcpy(gnrmcData.temporary_data, serialData.token);
      strncpy(gnrmcData.mode_indication, gnrmcData.temporary_data, 1);
      serialData.token = strtok(gnrmcData.temporary_data, "*");
      serialData.token = strtok(NULL, "*");
      strcpy(gnrmcData.check_sum, serialData.token);
      }
    serialData.token = strtok(NULL, ",");
    serialData.iter_token++;
  }
  if (sysDebugData.gnrmc_sentence == true) {
    Serial.println("[gnrmcData.tag] "                            + String(gnrmcData.tag));
    Serial.println("[gnrmcData.utc_time] "                       + String(gnrmcData.utc_time));
    Serial.println("[gnrmcData.positioning_status] "             + String(gnrmcData.positioning_status));
    Serial.println("[gnrmcData.latitude] "                       + String(gnrmcData.latitude));
    Serial.println("[gnrmcData.latitude_hemisphere] "            + String(gnrmcData.latitude_hemisphere));
    Serial.println("[gnrmcData.longitude] "                      + String(gnrmcData.longitude));
    Serial.println("[gnrmcData.longitude_hemisphere] "           + String(gnrmcData.longitude_hemisphere));
    Serial.println("[gnrmcData.ground_speed] "                   + String(gnrmcData.ground_speed));
    Serial.println("[gnrmcData.ground_heading] "                 + String(gnrmcData.ground_heading));
    Serial.println("[gnrmcData.utc_date] "                       + String(gnrmcData.utc_date));
    Serial.println("[gnrmcData.magnetic_declination] "           + String(gnrmcData.magnetic_declination));
    Serial.println("[gnrmcData.magnetic_declination_direction] " + String(gnrmcData.magnetic_declination_direction));
    Serial.println("[gnrmcData.mode_indication] "                + String(gnrmcData.mode_indication));
    Serial.println("[gnrmcData.check_sum] "                      + String(gnrmcData.check_sum));
  }
}



// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   GPATT DATA

struct GPATTStruct {
  char tag[56];              // <0> Log header
  char pitch[56];            // <1> pitch angle
  char angle_channel_0[56];  // <2> P
  char roll[56];             // <3> Roll angle
  char angle_channel_1[56];  // <4> R
  char yaw[56];              // <5> Yaw angle
  char angle_channel_2[56];  // <6> Y
  char software_version[56]; // <7> software verion
  char version_channel[56];  // <8> S
  char product_id[56];       // <9> Product ID: 96 bit unique ID
  char id_channel[56];       // <10> ID 
  char ins[56];              // <11> INS Default open inertial navigation system
  char ins_channel[56];      // <12> whether inertial navigation open
  char hardware_version[56]; // <13> Named after master chip
  char run_state_flag[56];   // <14> Algorithm status flag: 1->3
  char mis_angle_num[56];    // <15> number of Installation
  char custom_flag_0[56];    // <16> 
  char custom_flag_1[56];    // <17> 
  char mtk_version[56];      // <18> M:MTK1.6.0Version 7: MTK1.7.0Version
  char static_flag[56];      // <19> 1:Static 0：dynamic
  char user_code[56];        // <20> 1：Normal user X：Customuser
  char gst_data[56];         // <21> User satellite accuracy
  char line_flag[56];        // <22> 1：straight driving，0：curve driving
  char custom_flag_2[56];    // <23> F:Full Update D:Full Update and Part Update
  char custom_flag_3[56];    // <24> 
  char imu_kind[56];         // <25> Sensor Type: 0->BIM055; 1->BMI160; 2->LSM6DS3TR-C; 3->LSM6DSOW 4->ICM-40607; 5->ICM-40608 6->ICM-42670; 7->LSM6DSR
  char subi_car_kind[56];    // <26> 1: small car, 2: big car
  char mileage[56];          // <27> kilometers: max 9999 kilometers
  char custom_flag_4[56];    // <28> D
  char ang_dget_flag[56];    // <29> 1: The Flash has an installation Angle 0: The Flash has no installation Angle
  char run_inetial_flag[56]; // <30> 1->4
  char custom_flag_5[56];    // <31> B
  char custom_flag_6[56];    // <32> 
  char custom_flag_7[56];    // <33> 
  char custom_flag_8[56];    // <34> 
  char custom_flag_9[56];    // <35> 
  char time_save_num[56];    // <36> Ephemeris stored times
  char fix_angle_flag[56];   // <37> F：Fix
  char ang_lock_flag[56];    // <38> 1：fixed setting，0：Self adaptive installation
  char extensible[56];       // <39> 
  char check_sum[56];        // <40> XOR check value of all bytes starting from $ to *
  char temporary_data[56];
};
GPATTStruct gpattData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        GPATT

void GPATT() {
  memset(gpattData.tag, 0, 56);
  memset(gpattData.pitch, 0, 56);
  memset(gpattData.angle_channel_0, 0, 56);
  memset(gpattData.roll, 0, 56);
  memset(gpattData.angle_channel_1, 0, 56);
  memset(gpattData.yaw, 0, 56);
  memset(gpattData.angle_channel_2, 0, 56);
  memset(gpattData.software_version, 0, 56);
  memset(gpattData.version_channel, 0, 56);
  memset(gpattData.product_id, 0, 56);
  memset(gpattData.id_channel, 0, 56);
  memset(gpattData.ins, 0, 56);
  memset(gpattData.ins_channel, 0, 56);
  memset(gpattData.hardware_version, 0, 56);
  memset(gpattData.run_state_flag, 0, 56);
  memset(gpattData.mis_angle_num, 0, 56);
  memset(gpattData.custom_flag_0, 0, 56);
  memset(gpattData.custom_flag_1, 0, 56);
  memset(gpattData.mtk_version, 0, 56);
  memset(gpattData.static_flag, 0, 56);
  memset(gpattData.user_code, 0, 56);
  memset(gpattData.gst_data, 0, 56);
  memset(gpattData.line_flag, 0, 56);
  memset(gpattData.custom_flag_2, 0, 56);
  memset(gpattData.custom_flag_3, 0, 56);
  memset(gpattData.imu_kind, 0, 56);
  memset(gpattData.subi_car_kind, 0, 56);
  memset(gpattData.mileage, 0, 56);
  memset(gpattData.custom_flag_4, 0, 56);
  memset(gpattData.ang_dget_flag, 0, 56);
  memset(gpattData.run_inetial_flag, 0, 56);
  memset(gpattData.custom_flag_5, 0, 56);
  memset(gpattData.custom_flag_6, 0, 56);
  memset(gpattData.custom_flag_7, 0, 56);
  memset(gpattData.custom_flag_8, 0, 56);
  memset(gpattData.custom_flag_9, 0, 56);
  memset(gpattData.time_save_num, 0, 56);
  memset(gpattData.fix_angle_flag, 0, 56);
  memset(gpattData.ang_lock_flag, 0, 56);
  memset(gpattData.extensible, 0, 56);
  memset(gpattData.check_sum, 0, 56);
  
  serialData.iter_token = 0;
  serialData.token = strtok(serialData.BUFFER, ",");
  while( serialData.token != NULL ) {

    if      (serialData.iter_token == 0) {strcpy(gpattData.tag, "GPATT");}
    else if (serialData.iter_token == 1) {strcpy(gpattData.pitch, serialData.token);}
    else if (serialData.iter_token == 2) {strcpy(gpattData.angle_channel_0, serialData.token);}
    else if (serialData.iter_token == 3) {strcpy(gpattData.roll, serialData.token);}
    else if (serialData.iter_token == 4) {strcpy(gpattData.angle_channel_1, serialData.token);}
    else if (serialData.iter_token == 5) {strcpy(gpattData.yaw, serialData.token);}
    else if (serialData.iter_token == 6) {strcpy(gpattData.angle_channel_2, serialData.token);}
    else if (serialData.iter_token == 7) {strcpy(gpattData.software_version, serialData.token);}
    else if (serialData.iter_token == 8) {strcpy(gpattData.version_channel, serialData.token);}
    else if (serialData.iter_token == 9) {strcpy(gpattData.product_id, serialData.token);}
    else if (serialData.iter_token == 10) {strcpy(gpattData.id_channel, serialData.token);}
    else if (serialData.iter_token == 11) {strcpy(gpattData.ins, serialData.token);}
    else if (serialData.iter_token == 12) {strcpy(gpattData.ins_channel, serialData.token);}
    else if (serialData.iter_token == 13) {strcpy(gpattData.hardware_version, serialData.token);}
    else if (serialData.iter_token == 14) {strcpy(gpattData.run_state_flag, serialData.token);}
    else if (serialData.iter_token == 15) {strcpy(gpattData.mis_angle_num, serialData.token);}
    else if (serialData.iter_token == 16) {strcpy(gpattData.custom_flag_0, serialData.token);}
    else if (serialData.iter_token == 17) {strcpy(gpattData.custom_flag_1, serialData.token);}
    else if (serialData.iter_token == 18) {strcpy(gpattData.mtk_version, serialData.token);}
    else if (serialData.iter_token == 19) {strcpy(gpattData.static_flag, serialData.token);}
    else if (serialData.iter_token == 20) {strcpy(gpattData.user_code, serialData.token);}
    else if (serialData.iter_token == 21) {strcpy(gpattData.gst_data, serialData.token);}
    else if (serialData.iter_token == 22) {strcpy(gpattData.line_flag, serialData.token);}
    else if (serialData.iter_token == 23) {strcpy(gpattData.custom_flag_2, serialData.token);}
    else if (serialData.iter_token == 24) {strcpy(gpattData.custom_flag_3, serialData.token);}
    else if (serialData.iter_token == 25) {strcpy(gpattData.imu_kind, serialData.token);}
    else if (serialData.iter_token == 26) {strcpy(gpattData.subi_car_kind, serialData.token);}
    else if (serialData.iter_token == 27) {strcpy(gpattData.mileage, serialData.token);}
    else if (serialData.iter_token == 28) {strcpy(gpattData.custom_flag_4, serialData.token);}
    else if (serialData.iter_token == 29) {strcpy(gpattData.ang_dget_flag, serialData.token);}
    else if (serialData.iter_token == 20) {strcpy(gpattData.run_inetial_flag, serialData.token);}
    else if (serialData.iter_token == 31) {strcpy(gpattData.custom_flag_5, serialData.token);}
    else if (serialData.iter_token == 32) {strcpy(gpattData.custom_flag_6, serialData.token);}
    else if (serialData.iter_token == 33) {strcpy(gpattData.custom_flag_7, serialData.token);}
    else if (serialData.iter_token == 34) {strcpy(gpattData.custom_flag_8, serialData.token);}
    else if (serialData.iter_token == 35) {strcpy(gpattData.custom_flag_9, serialData.token);}
    else if (serialData.iter_token == 36) {strcpy(gpattData.time_save_num, serialData.token);}
    else if (serialData.iter_token == 37) {strcpy(gpattData.fix_angle_flag, serialData.token);}
    else if (serialData.iter_token == 38) {strcpy(gpattData.ang_lock_flag, serialData.token);}
    else if (serialData.iter_token == 39) {
      strcpy(gpattData.temporary_data, serialData.token);
      serialData.token = strtok(gpattData.temporary_data, "*");
      serialData.token = strtok(NULL, "*");
      strcpy(gpattData.check_sum, serialData.token);
      }

    serialData.token = strtok(NULL, ",");
    serialData.iter_token++;
  }
  if (sysDebugData.gpatt_sentence == true) {
    Serial.println("[gpattData.tag] "              + String(gpattData.tag));
    Serial.println("[gpattData.pitch] "            + String(gpattData.pitch));
    Serial.println("[gpattData.angle_channel_0] "  + String(gpattData.angle_channel_0));
    Serial.println("[gpattData.roll] "             + String(gpattData.roll));
    Serial.println("[gpattData.angle_channel_1] "  + String(gpattData.angle_channel_1));
    Serial.println("[gpattData.yaw] "              + String(gpattData.yaw)); 
    Serial.println("[gpattData.angle_channel_2] "  + String(gpattData.angle_channel_2));
    Serial.println("[gpattData.software_version] " + String(gpattData.software_version));
    Serial.println("[gpattData.version_channel] "  + String(gpattData.version_channel));
    Serial.println("[gpattData.product_id] "       + String(gpattData.product_id));
    Serial.println("[gpattData.id_channel] "       + String(gpattData.id_channel));
    Serial.println("[gpattData.ins] "              + String(gpattData.ins));
    Serial.println("[gpattData.ins_channel] "      + String(gpattData.ins_channel));
    Serial.println("[gpattData.hardware_version] " + String(gpattData.hardware_version));
    Serial.println("[gpattData.run_state_flag] "   + String(gpattData.run_state_flag));
    Serial.println("[gpattData.mis_angle_num] "    + String(gpattData.mis_angle_num));
    Serial.println("[gpattData.custom_flag_0] "    + String(gpattData.custom_flag_0));
    Serial.println("[gpattData.custom_flag_1] "    + String(gpattData.custom_flag_1));
    Serial.println("[gpattData.mtk_version] "      + String(gpattData.mtk_version));
    Serial.println("[gpattData.static_flag] "      + String(gpattData.static_flag));
    Serial.println("[gpattData.user_code] "        + String(gpattData.user_code));
    Serial.println("[gpattData.gst_data] "         + String(gpattData.gst_data));
    Serial.println("[gpattData.line_flag] "        + String(gpattData.line_flag));
    Serial.println("[gpattData.custom_flag_2] "    + String(gpattData.custom_flag_2));
    Serial.println("[gpattData.custom_flag_3] "    + String(gpattData.custom_flag_3));
    Serial.println("[gpattData.imu_kind] "         + String(gpattData.imu_kind));
    Serial.println("[gpattData.subi_car_kind] "    + String(gpattData.subi_car_kind));
    Serial.println("[gpattData.mileage] "          + String(gpattData.mileage));
    Serial.println("[gpattData.custom_flag_4] "    + String(gpattData.custom_flag_4));
    Serial.println("[gpattData.ang_dget_flag] "    + String(gpattData.ang_dget_flag));
    Serial.println("[gpattData.run_inetial_flag] " + String(gpattData.run_inetial_flag));
    Serial.println("[gpattData.custom_flag_5] "    + String(gpattData.custom_flag_5));
    Serial.println("[gpattData.custom_flag_6] "    + String(gpattData.custom_flag_6));
    Serial.println("[gpattData.custom_flag_7] "    + String(gpattData.custom_flag_7));
    Serial.println("[gpattData.custom_flag_8] "    + String(gpattData.custom_flag_8));
    Serial.println("[gpattData.custom_flag_9] "    + String(gpattData.custom_flag_9));
    Serial.println("[gpattData.time_save_num] "    + String(gpattData.time_save_num));
    Serial.println("[gpattData.fix_angle_flag] "   + String(gpattData.fix_angle_flag));
    Serial.println("[gpattData.ang_lock_flag] "    + String(gpattData.ang_lock_flag));
    Serial.println("[gpattData.extensible] "       + String(gpattData.extensible)); // intentionally unpopulated
    Serial.println("[gpattData.check_sum] "        + String(gpattData.check_sum));
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   DESBI DATA
// final sentence data struct still in development: desbi
struct DESBIStruct {
};
DESBIStruct desbiData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   SPEED DATA

/*

status        ubi_state_kind
0             normal driving status
1             normal acceleration state
2             rapid acceleration state
3             normal decceleration state
4             rapid decceleration state
5             emergency road status
6             normal turning state
7             sharp turn state
8             abnormal posture

*/

struct SPEEDStruct {
  char tag[56];                              // <0> log header
  char utc_time[56];                         // <1> utc time
  char speed[56];                            // <2> ground speed: knots
  char status[56];                           // <3> 0=invalid data, 1=converging, 2=valid data
  char acceleration_delimiter[56];           // <4> represents acceleration
  char acc_X[56];                            // <5> x-axis acceleration
  char acc_Y[56];                            // <6> y-axis acceleration
  char acc_Z[56];                            // <7> z-axis acceleration
  char angular_velocity_delimiter[56] = "0"; // <8> represents angular velocity
  char gyro_X[56];                           // <9> x-axis angular velocity
  char gyro_Y[56];                           // <10> y-axis angular velocity
  char gyro_Z[56];                           // <11> z-axis angular velocity
  char status_delimiter[56];                 // <12> represents status
  char ubi_state_flag[56];                   // <13> 0=smooth driving, 1=unsteady driving
  char ubi_state_kind[56];                   // <14> status tyoe: see ubi_state_kind table
  char ubi_state_value[56];                  // <15> status threshold: see ubi_state_kind table
  char check_sum[56];                        // <16> XOR check value of all bytes starting from $ to *
  char temporary_data[56];
};
SPEEDStruct speedData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        SPEED

void SPEED() {
  memset(speedData.tag, 0, 56);
  memset(speedData.utc_time, 0, 56);
  memset(speedData.speed, 0, 56);
  memset(speedData.status, 0, 56);
  memset(speedData.acceleration_delimiter, 0, 56);
  memset(speedData.acc_X, 0, 56);
  memset(speedData.acc_Y, 0, 56);
  memset(speedData.acc_Z, 0, 56);
  memset(speedData.angular_velocity_delimiter, 0, 56);
  memset(speedData.gyro_X, 0, 56);
  memset(speedData.gyro_Y, 0, 56);
  memset(speedData.gyro_Z, 0, 56);
  memset(speedData.status_delimiter, 0, 56);
  memset(speedData.ubi_state_flag, 0, 56);
  memset(speedData.ubi_state_kind, 0, 56);
  memset(speedData.ubi_state_value, 0, 56);
  memset(speedData.check_sum, 0, 56);
  serialData.iter_token = 0;
  serialData.token = strtok(serialData.BUFFER, ",");
  while( serialData.token != NULL ) {
    if      (serialData.iter_token == 0) {strcpy(speedData.tag, "SPEED");}
    else if (serialData.iter_token == 1) {strcpy(speedData.utc_time, serialData.token);}
    else if (serialData.iter_token == 2) {strcpy(speedData.speed, serialData.token);}
    else if (serialData.iter_token == 3) {strcpy(speedData.status, serialData.token);}
    else if (serialData.iter_token == 4) {strcpy(speedData.acceleration_delimiter, serialData.token);}
    else if (serialData.iter_token == 5) {strcpy(speedData.acc_X, serialData.token);}
    else if (serialData.iter_token == 6) {strcpy(speedData.acc_Y, serialData.token);}
    else if (serialData.iter_token == 7) {strcpy(speedData.acc_Z, serialData.token);}
    else if (serialData.iter_token == 8) {strcpy(speedData.angular_velocity_delimiter, serialData.token);}
    else if (serialData.iter_token == 9) {strcpy(speedData.gyro_X, serialData.token);}
    else if (serialData.iter_token == 10) {strcpy(speedData.gyro_Y, serialData.token);}
    else if (serialData.iter_token == 11) {strcpy(speedData.gyro_Z, serialData.token);}
    else if (serialData.iter_token == 12) {strcpy(speedData.status_delimiter, serialData.token);}
    else if (serialData.iter_token == 13) {strcpy(speedData.ubi_state_flag, serialData.token);}
    else if (serialData.iter_token == 14) {strcpy(speedData.ubi_state_kind, serialData.token);}
    else if (serialData.iter_token == 15) {
      strcpy(speedData.temporary_data, serialData.token);
      strncpy(speedData.ubi_state_value, speedData.temporary_data, 1);
      serialData.token = strtok(speedData.temporary_data, "*");
      serialData.token = strtok(NULL, "*");
      strcpy(speedData.check_sum, serialData.token);
      }
    serialData.token = strtok(NULL, ",");
    serialData.iter_token++;
  }
  if (sysDebugData.gpatt_sentence == true) {
    Serial.println("[speedData.tag] "                        + String(speedData.tag));
    Serial.println("[speedData.utc_time] "                   + String(speedData.utc_time));
    Serial.println("[speedData.speed] "                      + String(speedData.speed));
    Serial.println("[speedData.status] "                     + String(speedData.status));
    Serial.println("[speedData.acceleration_delimiter] "     + String(speedData.acceleration_delimiter));
    Serial.println("[speedData.acc_X] "                      + String(speedData.acc_X));
    Serial.println("[speedData.acc_Y] "                      + String(speedData.acc_Y));
    Serial.println("[speedData.acc_Z] "                      + String(speedData.acc_Z));
    Serial.println("[speedData.angular_velocity_delimiter] " + String(speedData.angular_velocity_delimiter));
    Serial.println("[speedData.gyro_X] "                     + String(speedData.gyro_X));
    Serial.println("[speedData.gyro_Y] "                     + String(speedData.gyro_Y));
    Serial.println("[speedData.gyro_Z] "                     + String(speedData.gyro_Z));
    Serial.println("[speedData.status_delimiter] "           + String(speedData.status_delimiter));
    Serial.println("[speedData.ubi_state_flag] "             + String(speedData.ubi_state_flag));
    Serial.println("[speedData.ubi_state_kind] "             + String(speedData.ubi_state_kind));
    Serial.println("[speedData.ubi_state_value] "            + String(speedData.ubi_state_value));
    Serial.println("[speedData.check_sum] "                  + String(speedData.check_sum));
  }
}


// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   ERROR DATA

struct ERRORStruct {
  char tag[56];         // <0> Log header
  char utc[56];         // <1> utc time
  char code_flag[56];   // <2> encryption chip: 1=problem, 0=normal
  char gset_flag[56];   // <3> positioning chip: 1=problem, 0=normal
  char sset_flag[56];   // <4> sensor chip: 1=problem, 0=normal
  char customize_0[56]; // <5> customize 0-20
  char customize_1[56]; // <6> customize float
  char check_sum[56];   // <7> XOR check value of all bytes starting from $ to *
  char temporary_data[56];
};
ERRORStruct errorData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        ERROR

void ERROR() {
  memset(errorData.tag, 0, 56);
  memset(errorData.utc, 0, 56);
  memset(errorData.code_flag, 0, 56);
  memset(errorData.gset_flag, 0, 56);
  memset(errorData.sset_flag, 0, 56);
  memset(errorData.customize_0, 0, 56);
  memset(errorData.customize_1, 0, 56);
  memset(errorData.check_sum, 0, 56);
  serialData.iter_token = 0;
  serialData.token = strtok(serialData.BUFFER, ",");
  while( serialData.token != NULL ) {
    if      (serialData.iter_token == 0) {strcpy(errorData.tag, "ERROR");}
    else if (serialData.iter_token == 1) {strcpy(errorData.utc, serialData.token);}
    else if (serialData.iter_token == 2) {strcpy(errorData.code_flag, serialData.token);}
    else if (serialData.iter_token == 3) {strcpy(errorData.gset_flag, serialData.token);}
    else if (serialData.iter_token == 4) {strcpy(errorData.sset_flag, serialData.token);}
    else if (serialData.iter_token == 5) {strcpy(errorData.customize_0, serialData.token);}
    else if (serialData.iter_token == 6) {
      strcpy(errorData.temporary_data, serialData.token);
      serialData.token = strtok(errorData.temporary_data, "*");
      serialData.token = strtok(NULL, "*");
      strcpy(errorData.check_sum, serialData.token);
      }
    serialData.token = strtok(NULL, ",");
    serialData.iter_token++;
  }
  if (sysDebugData.gpatt_sentence == true) {
    Serial.println("[errorData.tag] "         + String(errorData.tag));
    Serial.println("[errorData.utc] "         + String(errorData.utc));
    Serial.println("[errorData.code_flag] "   + String(errorData.code_flag));
    Serial.println("[errorData.gset_flag] "   + String(errorData.gset_flag));
    Serial.println("[errorData.sset_flag] "   + String(errorData.sset_flag));
    Serial.println("[errorData.customize_0] " + String(errorData.customize_0));
    Serial.println("[errorData.customize_1] " + String(errorData.customize_1)); // intentionally unpopulated
    Serial.println("[errorData.check_sum] "   + String(errorData.check_sum));
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   DEBUG DATA

struct DEBUGStruct {
  char tag[56];            // <0> log header
  char ang_dget_flag[56];  // <1> installation azimuth: 1=with azimuth, 0=without azimuth
  char fix_kind_flag[56];  // <2> type of installed coordinate system
  char ins_run_flag[56];   // <3> forced ins: 1=forced, 0=normal
  char fix_roll_flag[56];  // <4> installation roll angle
  char fix_pitch_flag[56]; // <5> installation pitch angle
  char ubi_on_flag[56];    // <6> 0 to 8
  char ubi_kind_flag[56];  // <7> 0=none, 1=ubi event, 2=ubi alarm
  char ubi_a_set[56];      // <8> ubi a parameter setting value
  char ubi_b_set[56];      // <9> ubi b parameter setting value
  char acc_X_data[56];     // <10> vehicle longitudinal acceleration: 0.1m/s2
  char acc_Y_data[56];     // <11> vehicle lateral acceleration: 0.1m/s2
  char gyro_Z_data[56];    // <12> vehicle z axis angular velocity: degrees
  char pitch_angle[56];    // <13> vehicle pitch angle: degrees
  char roll_angle[56];     // <14> vehicle roll angle: degrees
  char yaw_angle[56];      // <15> vehicle direction change angle: degrees
  char car_speed[56];      // <16> vehicle speed: m/s
  char ins_flag[56];       // <17> intertial navigation convergence flag
  char ubi_num[56];        // <18> serial number
  char ubi_valid[56];      // <19> ubi valid flag: 1=valid, 0=invalid
  char coll_T_data[56];    // <20> collision factor
  char coll_T_heading[56]; // <21> collision direction
  char custom_logo_0[56];  // <22> 
  char custom_logo_1[56];  // <23> 
  char custom_logo_2[56];  // <24> 
  char custom_logo_3[56];  // <25> 
  char custom_logo_4[56];  // <26> 
  char custom_logo_5[56];  // <27> 
  char check_sum[56];      // <28> XOR check value of all bytes starting from $ to *
  char temporary_data[56];
};
DEBUGStruct debugData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        DEBUG

void DEBUG() {
  memset(debugData.tag, 0, 56);
  memset(debugData.ang_dget_flag, 0, 56);
  memset(debugData.fix_kind_flag, 0, 56);
  memset(debugData.ins_run_flag, 0, 56);
  memset(debugData.fix_roll_flag, 0, 56);
  memset(debugData.fix_pitch_flag, 0, 56);
  memset(debugData.ubi_on_flag, 0, 56);
  memset(debugData.ubi_kind_flag, 0, 56);
  memset(debugData.ubi_a_set, 0, 56);
  memset(debugData.ubi_b_set, 0, 56);
  memset(debugData.acc_X_data, 0, 56);
  memset(debugData.acc_Y_data, 0, 56);
  memset(debugData.gyro_Z_data, 0, 56);
  memset(debugData.pitch_angle, 0, 56);
  memset(debugData.roll_angle, 0, 56);
  memset(debugData.yaw_angle, 0, 56);
  memset(debugData.car_speed, 0, 56);
  memset(debugData.ins_flag, 0, 56);
  memset(debugData.ubi_num, 0, 56);
  memset(debugData.ubi_valid, 0, 56);
  memset(debugData.coll_T_data, 0, 56);
  memset(debugData.coll_T_heading, 0, 56);
  memset(debugData.custom_logo_0, 0, 56);
  memset(debugData.custom_logo_1, 0, 56);
  memset(debugData.custom_logo_2, 0, 56);
  memset(debugData.custom_logo_4, 0, 56);
  memset(debugData.custom_logo_5, 0, 56);
  memset(debugData.check_sum, 0, 56);
  serialData.iter_token = 0;
  serialData.token = strtok(serialData.BUFFER, ",");
  while( serialData.token != NULL ) {
    if      (serialData.iter_token == 0) {strcpy(debugData.tag, "DEBUG");}
    else if (serialData.iter_token == 1) {strcpy(debugData.ang_dget_flag, serialData.token);}
    else if (serialData.iter_token == 2) {strcpy(debugData.fix_kind_flag, serialData.token);}
    else if (serialData.iter_token == 3) {strcpy(debugData.ins_run_flag, serialData.token);}
    else if (serialData.iter_token == 4) {strcpy(debugData.fix_roll_flag, serialData.token);}
    else if (serialData.iter_token == 5) {strcpy(debugData.fix_pitch_flag, serialData.token);}
    else if (serialData.iter_token == 6) {strcpy(debugData.ubi_on_flag, serialData.token);}
    else if (serialData.iter_token == 7) {strcpy(debugData.ubi_kind_flag, serialData.token);}
    else if (serialData.iter_token == 8) {strcpy(debugData.ubi_a_set, serialData.token);}
    else if (serialData.iter_token == 9) {strcpy(debugData.ubi_b_set, serialData.token);}
    else if (serialData.iter_token == 10) {strcpy(debugData.acc_X_data, serialData.token);}
    else if (serialData.iter_token == 11) {strcpy(debugData.acc_Y_data, serialData.token);}
    else if (serialData.iter_token == 12) {strcpy(debugData.gyro_Z_data, serialData.token);}
    else if (serialData.iter_token == 13) {strcpy(debugData.pitch_angle, serialData.token);}
    else if (serialData.iter_token == 14) {strcpy(debugData.roll_angle, serialData.token);}
    else if (serialData.iter_token == 15) {strcpy(debugData.yaw_angle, serialData.token);}
    else if (serialData.iter_token == 16) {strcpy(debugData.car_speed, serialData.token);}
    else if (serialData.iter_token == 17) {strcpy(debugData.ins_flag, serialData.token);}
    else if (serialData.iter_token == 18) {strcpy(debugData.ubi_num, serialData.token);}
    else if (serialData.iter_token == 19) {strcpy(debugData.ubi_valid, serialData.token);}
    else if (serialData.iter_token == 20) {strcpy(debugData.coll_T_data, serialData.token);}
    else if (serialData.iter_token == 21) {strcpy(debugData.coll_T_heading, serialData.token);}
    else if (serialData.iter_token == 22) {strcpy(debugData.custom_logo_0, serialData.token);}
    else if (serialData.iter_token == 23) {strcpy(debugData.custom_logo_1, serialData.token);}
    else if (serialData.iter_token == 24) {strcpy(debugData.custom_logo_2, serialData.token);}
    else if (serialData.iter_token == 25) {strcpy(debugData.custom_logo_3, serialData.token);}
    else if (serialData.iter_token == 26) {strcpy(debugData.custom_logo_4, serialData.token);}
    else if (serialData.iter_token == 27) {
      strcpy(debugData.temporary_data, serialData.token);
      serialData.token = strtok(debugData.temporary_data, "*");
      serialData.token = strtok(NULL, "*");
      strcpy(debugData.check_sum, serialData.token);
      }
    serialData.token = strtok(NULL, ",");
    serialData.iter_token++;
  }
  if (sysDebugData.gpatt_sentence == true) {
    Serial.println("[debugData.tag] "            + String(debugData.tag));
    Serial.println("[debugData.ang_dget_flag] "  + String(debugData.ang_dget_flag));
    Serial.println("[debugData.fix_kind_flag] "  + String(debugData.fix_kind_flag));
    Serial.println("[debugData.ins_run_flag] "   + String(debugData.ins_run_flag));
    Serial.println("[debugData.fix_roll_flag] "  + String(debugData.fix_roll_flag));
    Serial.println("[debugData.fix_pitch_flag] " + String(debugData.fix_pitch_flag));
    Serial.println("[debugData.ubi_on_flag] "    + String(debugData.ubi_on_flag));
    Serial.println("[debugData.ubi_kind_flag] "  + String(debugData.ubi_kind_flag));
    Serial.println("[debugData.ubi_a_set] "      + String(debugData.ubi_a_set));
    Serial.println("[debugData.ubi_b_set] "      + String(debugData.ubi_b_set));
    Serial.println("[debugData.acc_X_data] "     + String(debugData.acc_X_data));
    Serial.println("[debugData.acc_Y_data] "     + String(debugData.acc_Y_data));
    Serial.println("[debugData.gyro_Z_data] "    + String(debugData.gyro_Z_data));
    Serial.println("[debugData.pitch_angle] "    + String(debugData.pitch_angle));
    Serial.println("[debugData.roll_angle] "     + String(debugData.roll_angle));
    Serial.println("[debugData.yaw_angle] "      + String(debugData.yaw_angle));
    Serial.println("[debugData.car_speed] "      + String(debugData.car_speed));
    Serial.println("[debugData.ins_flag] "       + String(debugData.ins_flag));
    Serial.println("[debugData.ubi_num] "        + String(debugData.ubi_num));
    Serial.println("[debugData.ubi_valid] "      + String(debugData.ubi_valid));
    Serial.println("[debugData.coll_T_data] "    + String(debugData.coll_T_data));
    Serial.println("[debugData.coll_T_heading] " + String(debugData.coll_T_heading));
    Serial.println("[debugData.custom_logo_0] "  + String(debugData.custom_logo_0));
    Serial.println("[debugData.custom_logo_1] "  + String(debugData.custom_logo_1));
    Serial.println("[debugData.custom_logo_2] "  + String(debugData.custom_logo_2));
    Serial.println("[debugData.custom_logo_3] "  + String(debugData.custom_logo_3));
    Serial.println("[debugData.custom_logo_4] "  + String(debugData.custom_logo_4));
    Serial.println("[debugData.custom_logo_5] "  + String(debugData.custom_logo_5));
    Serial.println("[debugData.check_sum] "      + String(debugData.check_sum));
  }
}


// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                          SATCOM DATA STRUCT

struct SatDatatruct {
  char satcom_sentence[1024];
  char sat_time_stamp_string[56];                                  // datetime timestamp from satellite
  char satDataTag[10]                 = "$SATCOM";                 // satcom sentence tag
  char last_sat_time_stamp_str[56]    = "00.00";                   // record last time satellites were seen
  char coordinate_conversion_mode[10] = "GNGGA";                   // choose a sentence that degrees/decimal coordinates will be created from
  double latitude_meter               = 0.00000901;                // one meter (tune)
  double longitude_meter              = 0.00000899;                // one meter (tune)
  double latitude_mile                = latitude_meter  * 1609.34; // one mile
  double longitude_mile               = longitude_meter * 1609.34; // one mile
  double abs_latitude_gngga_0         = 0.0;                       // absolute latitude from $ sentence
  double abs_longitude_gngga_0        = 0.0;                       // absolute longditude $ sentence
  double abs_latitude_gnrmc_0         = 0.0;                       // absolute latitude $ sentence
  double abs_longitude_gnrmc_0        = 0.0;                       // absolute longditude $ sentence
  double temp_latitude_gngga;                                      // degrees converted from absolute
  double temp_longitude_gngga;                                     // degrees converted from absolute
  double temp_latitude_gnrmc;                                      // degrees converted from absolute
  double temp_longitude_gnrmc;                                     // degrees converted from absolute
  double location_latitude_gngga;                                  // degrees converted from absolute
  double location_longitude_gngga;                                 // degrees converted from absolute
  double location_latitude_gnrmc;                                  // degrees converted from absolute
  double location_longitude_gnrmc;                                 // degrees converted from absolute
  char location_latitude_gngga_str[56];                            // degrees converted from absolute
  char location_longitude_gngga_str[56];                           // degrees converted from absolute
  char location_latitude_gnrmc_str[56];                            // degrees converted from absolute
  char location_longitude_gnrmc_str[56];                           // degrees converted from absolute
  double minutesLat;                                               // used for converting absolute latitude and longitude
  double minutesLong;                                              // used for converting absolute latitude and longitude
  double degreesLat;                                               // used for converting absolute latitude and longitude
  double degreesLong;                                              // used for converting absolute latitude and longitude
  double secondsLat;                                               // used for converting absolute latitude and longitude
  double secondsLong;                                              // used for converting absolute latitude and longitude
  double millisecondsLat;                                          // used for converting absolute latitude and longitude
  double millisecondsLong;                                         // used for converting absolute latitude and longitude
};
SatDatatruct satData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                        CREATE COORDINTE DATA
void calculateLocation(){

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                GNGGA COORDINATE CONVERSION

  if (String(satData.coordinate_conversion_mode) == "GNGGA") {

    // convert GNGGA latitude
    satData.temp_latitude_gngga = satData.abs_latitude_gngga_0;
    satData.degreesLat = atof(String(trunc(satData.temp_latitude_gngga / 100)).c_str());
    satData.minutesLat = atof(String(satData.temp_latitude_gngga - (satData.degreesLat * 100)).c_str());
    satData.secondsLat = atof(String(satData.minutesLat - trunc(satData.minutesLat)).c_str()) * 60;
    satData.millisecondsLat = atof(String(satData.secondsLat - trunc(satData.secondsLat)).c_str()) * 1000;
    satData.minutesLat = atof(String(trunc(satData.minutesLat)).c_str());
    satData.secondsLat = atof(String(trunc(satData.secondsLat)).c_str());
    satData.location_latitude_gngga =
    atof(String(satData.degreesLat + satData.minutesLat / 60 + satData.secondsLat / 3600 + satData.millisecondsLat / 3600000).c_str());
    if (strcmp(gnggaData.latitude_hemisphere, "S") == 0) {
      satData.location_latitude_gngga = atof(String(0 - satData.location_latitude_gngga).c_str());
    }
    scanf("%f17", &satData.location_latitude_gngga);
    sprintf(satData.location_latitude_gngga_str, "%f", satData.location_latitude_gngga);

    // convert GNGGA longitude
    satData.temp_longitude_gngga = satData.abs_longitude_gngga_0;
    satData.degreesLong = atof(String(trunc(satData.temp_longitude_gngga / 100)).c_str());
    satData.minutesLong = atof(String(satData.temp_longitude_gngga - (satData.degreesLong * 100)).c_str());
    satData.secondsLong = atof(String(satData.minutesLong - atof(String(trunc(satData.minutesLong)).c_str())).c_str()) * 60;
    satData.millisecondsLong = 
    atof(String(satData.secondsLong - atof(String(trunc(satData.secondsLong)).c_str())).c_str()) * 1000;
    satData.location_longitude_gngga =
    atof(String(satData.degreesLong + satData.minutesLong / 60 + satData.secondsLong / 3600 + satData.millisecondsLong / 3600000).c_str());
    if (strcmp(gnggaData.longitude_hemisphere, "W") == 0) {
      satData.location_longitude_gngga = 0 - satData.location_longitude_gngga;
    }
    scanf("%f17", &satData.location_longitude_gngga);
    sprintf(satData.location_longitude_gngga_str, "%f", satData.location_longitude_gngga);
  }

  // ------------------------------------------------------------------------------------------------------------------------
  //                                                                                              GNRMC COORDINATE CONVERSION

  else if (String(satData.coordinate_conversion_mode) == "GNRMC") {

    // convert GNRMC latitude
    satData.temp_latitude_gnrmc = satData.abs_latitude_gnrmc_0;
    satData.degreesLat = atof(String(trunc(satData.temp_latitude_gnrmc / 100)).c_str());
    satData.minutesLat = atof(String(satData.temp_latitude_gnrmc - (satData.degreesLat * 100)).c_str());
    satData.secondsLat = atof(String(satData.minutesLat - trunc(satData.minutesLat)).c_str()) * 60;
    satData.millisecondsLat = atof(String(satData.secondsLat - trunc(satData.secondsLat)).c_str()) * 1000;
    satData.minutesLat = atof(String(trunc(satData.minutesLat)).c_str());
    satData.secondsLat = atof(String(trunc(satData.secondsLat)).c_str());
    satData.location_latitude_gnrmc =
    atof(String(satData.degreesLat + satData.minutesLat / 60 + satData.secondsLat / 3600 + satData.millisecondsLat / 3600000).c_str());
    if (strcmp(gnrmcData.latitude_hemisphere, "S") == 0) {
      satData.location_latitude_gnrmc = atof(String(0 - satData.location_latitude_gnrmc).c_str());
    }
    scanf("%f17", &satData.location_latitude_gnrmc);
    sprintf(satData.location_latitude_gnrmc_str, "%f", satData.location_latitude_gnrmc);

    // convert GNRMC longitude
    satData.temp_longitude_gnrmc = satData.abs_longitude_gnrmc_0;
    satData.degreesLong = atof(String(trunc(satData.temp_longitude_gnrmc / 100)).c_str());
    satData.minutesLong = atof(String(satData.temp_longitude_gnrmc - (satData.degreesLong * 100)).c_str());
    satData.secondsLong = atof(String(satData.minutesLong - atof(String(trunc(satData.minutesLong)).c_str())).c_str()) * 60;
    satData.millisecondsLong = atof(String(satData.secondsLong - atof(String(trunc(satData.secondsLong)).c_str())).c_str()) * 1000;
    satData.location_longitude_gnrmc =
    atof(String(satData.degreesLong + satData.minutesLong / 60 + satData.secondsLong / 3600 + satData.millisecondsLong / 3600000).c_str());
    if (strcmp(gnrmcData.longitude_hemisphere, "W") == 0) {
      satData.location_longitude_gnrmc = 0 - satData.location_longitude_gnrmc;
    }
    scanf("%f17", &satData.location_longitude_gnrmc);
    sprintf(satData.location_longitude_gnrmc_str, "%f", satData.location_longitude_gnrmc);
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                EXTRAPOLATE & DUMP EXTRA DATA

void extrapulatedSatData() {

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                     SATCOM SENTENCE: BEGIN

  memset(satData.satcom_sentence, 0, 1024);
  strcat(satData.satcom_sentence, satData.satDataTag);
  strcat(satData.satcom_sentence, ",");

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                   SATCOM SENTENCE: TIMESTAMP FROM SAT TIME

  memset(satData.sat_time_stamp_string, 0, 56);
  strcat(satData.sat_time_stamp_string, gnrmcData.utc_date);
  strcat(satData.sat_time_stamp_string, gnggaData.utc_time);

  strcat(satData.satcom_sentence, satData.sat_time_stamp_string);
  strcat(satData.satcom_sentence, ",");

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                       SATCOM SENTENCE: LAST KNOWN DOWNLINK

  if (atoi(gnggaData.satellite_count_gngga) > 0) {
    memset(satData.last_sat_time_stamp_str, 0, 56);
    strcpy(satData.last_sat_time_stamp_str, satData.sat_time_stamp_string);
  }
  strcat(satData.satcom_sentence, satData.last_sat_time_stamp_str);
  strcat(satData.satcom_sentence, ",");

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                 SATCOM SENTENCE: CONVERT DATA FROM STRINGS

  if (String(satData.coordinate_conversion_mode) == "GNGGA") {
    satData.abs_latitude_gngga_0 = atof(String(gnggaData.latitude).c_str());
    satData.abs_longitude_gngga_0 = atof(String(gnggaData.longitude).c_str());
  }
  else if (String(satData.coordinate_conversion_mode) == "GNRMC") {
    satData.abs_latitude_gnrmc_0 = atof(String(gnrmcData.latitude).c_str());
    satData.abs_longitude_gnrmc_0 = atof(String(gnrmcData.longitude).c_str());
  }
  calculateLocation();
  if (String(satData.coordinate_conversion_mode) == "GNGGA") {

    strcat(satData.satcom_sentence, satData.location_latitude_gngga_str);
    strcat(satData.satcom_sentence, ",");
    strcat(satData.satcom_sentence, satData.location_longitude_gngga_str);
    strcat(satData.satcom_sentence, ",");
  }
  else if (String(satData.coordinate_conversion_mode) == "GNRMC") {

    strcat(satData.satcom_sentence, satData.location_latitude_gnrmc_str);
    strcat(satData.satcom_sentence, ",");
    strcat(satData.satcom_sentence, satData.location_longitude_gnrmc_str);
    strcat(satData.satcom_sentence, ",");
  }

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                       SATCOM SENTENCE: END
  strcat(satData.satcom_sentence, "*Z");
  Serial.println(satData.satcom_sentence);
  }

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    DISPLAY 0

void SSD_Display_6() {
  tcaselect(6);
  display_7.setTextAlignment(TEXT_ALIGN_CENTER);
  display_7.setColor(WHITE);
  display_7.clear();
  display_7.drawString(display_7.getWidth()/2, 0, "GNGGA");
  display_7.drawString(display_7.getWidth()/2, 14, "P " + String(gnggaData.positioning_status) + " S " + String(gnggaData.satellite_count_gngga));
  display_7.drawString(display_7.getWidth()/2, 24, String(gnggaData.utc_time));
  display_7.drawString(display_7.getWidth()/2, 34, String(gnggaData.latitude_hemisphere) + " " + String(gnggaData.latitude));
  display_7.drawString(display_7.getWidth()/2, 44, String(gnggaData.longitude_hemisphere) + " " + String(gnggaData.longitude));
  display_7.drawString(display_7.getWidth()/2, 54, "A " + String(gnggaData.altitude));
  display_7.display();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    DISPLAY 1

void SSD_Display_7() {
  tcaselect(7);
  display_6.setTextAlignment(TEXT_ALIGN_CENTER);
  display_6.setColor(WHITE);
  display_6.clear();
  display_6.drawString(display_6.getWidth()/2, 0, "GNRMC");
  display_6.drawString(display_6.getWidth()/2, 14, "P " + String(gnrmcData.positioning_status) + " M " + String(gnrmcData.mode_indication));
  display_6.drawString(display_6.getWidth()/2, 24, String(gnrmcData.utc_time) + " " + String(gnrmcData.utc_date));
  display_6.drawString(display_6.getWidth()/2, 34, String(gnrmcData.latitude_hemisphere) + " " + String(gnrmcData.latitude));
  display_6.drawString(display_6.getWidth()/2, 44, String(gnrmcData.longitude_hemisphere) + " " + String(gnrmcData.longitude));
  display_6.drawString(display_6.getWidth()/2, 54, "H " + String(gnrmcData.ground_heading) + " S " + String(gnrmcData.ground_speed));
  display_6.display();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    DISPLAY 2

void SSD_Display_5() {
  tcaselect(5);
  display_5.setTextAlignment(TEXT_ALIGN_CENTER);
  display_5.setColor(WHITE);
  display_5.clear();
  display_5.drawString(display_5.getWidth()/2, 0, "SATCOM");
  display_5.drawString(display_5.getWidth()/2, 14, satData.sat_time_stamp_string);
  display_5.drawString(display_5.getWidth()/2, 24, String(satData.last_sat_time_stamp_str));
  display_5.drawString(display_5.getWidth()/2, 44, String(gnggaData.latitude_hemisphere) + " " + satData.location_latitude_gngga_str);
  display_5.drawString(display_5.getWidth()/2, 54, String(gnggaData.longitude_hemisphere) + " " + satData.location_longitude_gngga_str);
  display_5.display();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    DISPLAY 3

void SSD_Display_4() {
  tcaselect(4);
  display_4.setTextAlignment(TEXT_ALIGN_CENTER);
  display_4.setColor(WHITE);
  display_4.clear();
  display_4.drawString(display_4.getWidth()/2, 0, "GPATT");
  display_4.drawString(display_4.getWidth()/2, 14, "P " + String(gpattData.pitch));
  display_4.drawString(display_4.getWidth()/2, 24, "R " + String(gpattData.roll));
  display_4.drawString(display_4.getWidth()/2, 34, "Y " + String(gpattData.yaw));
  display_4.drawString(display_4.getWidth()/2, 44, String("RSF " + String(gpattData.run_state_flag) + " GST " + String(gpattData.gst_data) + " RIF " + String(gpattData.run_inetial_flag)));
  display_4.drawString(display_4.getWidth()/2, 54, String("E " + String(gpattData.time_save_num) + " FA " + String(gpattData.fix_angle_flag)));
  display_4.display();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    DISPLAY 4

void SSD_Display_3_Splash_0() {
  tcaselect(3);
  display_3.setTextAlignment(TEXT_ALIGN_CENTER);
  display_3.setColor(WHITE);
  display_3.clear();
  display_3.drawString(display_3.getWidth()/2, 0, "        _,--',   _._.--.___");
  display_3.drawString(display_3.getWidth()/2, 10, " .--.--';_'-.', ';_      _.,-'");
  display_3.drawString(display_3.getWidth()/2, 20, ".'--'.  _.'    {`'-;_ .-.>.'");
  display_3.drawString(display_3.getWidth()/2, 30, "      '-:_      )  / `' '=.");
  display_3.drawString(display_3.getWidth()/2, 40, "        ) >     {_/,     /~)");
  display_3.drawString(display_3.getWidth()/2, 50, "snd     |/               `^ .'");
  display_3.display();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        SETUP

void setup() {

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                               SETUP SERIAL

  Serial.begin(115200);
  Serial1.begin(115200); // ( io26 on ESP32 )

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                 SETUP WIRE

  Wire.begin();

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                              SETUP DISPLAY

  tcaselect(3);
  initDisplay3();

  tcaselect(4);
  initDisplay4();

  tcaselect(5);
  initDisplay5();

  tcaselect(6);
  initDisplay6();

  tcaselect(7);
  initDisplay7();

  SSD_Display_3_Splash_0();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   READ RXD 1

void readRXD_1() {

  if (Serial1.available() > 0) {
    
    memset(serialData.BUFFER, 0, 2048);
    serialData.nbytes = (Serial1.readBytesUntil('\n', serialData.BUFFER, sizeof(serialData.BUFFER)));
    // Serial.println(serialData.nbytes); // debug

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                                    GNGGA

    if (strncmp(serialData.BUFFER, "$GNGGA", 6) == 0) {
      if ((serialData.nbytes == 94) || (serialData.nbytes == 90) ) {
        Serial.print(""); Serial.println(serialData.BUFFER);
        GNGGA();
      }
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                                    GNRMC

    else if (strncmp(serialData.BUFFER, "$GNRMC", 6) == 0) {
      if ((serialData.nbytes == 78) || (serialData.nbytes == 80)) {
        // Serial.print(""); Serial.println(serialData.BUFFER);
        // GNRMC();
      }
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                                    GPATT

    else if (strncmp(serialData.BUFFER, "$GPATT", 6) == 0) {
      if ((serialData.nbytes == 136) || (serialData.nbytes == 189)) {
        // Serial.print(""); Serial.println(serialData.BUFFER);
        // GPATT();
      }
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                                    DESBI

    else if (strncmp(serialData.BUFFER, "$DESBI", 6) == 0) {
      // Serial.print(""); Serial.println(serialData.BUFFER);
      // awaiting length checks and clarification: wait for clarification, take a ride with the laptop
      // DESBI();
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                                    SPEED

    else if (strncmp(serialData.BUFFER, "$SPEED", 6) == 0) {
      // Serial.print(""); Serial.println(serialData.BUFFER);
      // awaiting length checks: take a ride with the laptop
      // SPEED();
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                                    ERROR

    else if (strncmp(serialData.BUFFER, "$ERROR", 6) == 0) {
      // Serial.print(""); Serial.println(serialData.BUFFER);
      // awaiting length checks: take a ride with the laptop
      // ERROR();
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                                    DEBUG

    else if (strncmp(serialData.BUFFER, "$DEBUG", 6) == 0) {
      // Serial.print(""); Serial.println(serialData.BUFFER);
      // awaiting length checks: take a ride with the laptop
      // DEBUG();
    }
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                  RELAYS DATA

/*
A minimum of N relays would be required to satisfy various flags. This can allow satcom to be as general purpose as intended,
from minimal to maximal operation/utilization of the WTGPS300 as and when required by different projects, even turning on/off other
systems that begin running their own routines, by having them turn on/off with these relays/functions.
each relay should have its own char array which can be checked each loop, after which a function corrrspinding to a relays char
array will be ran if a selected condition is met, then the corresponding relay will be turned on/off when that condition is met.
additional configuration could include running once, running each time etc. for systems/routines to be activated/deactivated.
Note: Each relay/function is activated/deactivated according to compound conditions, meaning we can be more or less strict.
*/

struct RelayStruct {

  int MAX_RELAYS = 10;
  int MAX_RELAY_ELEMENTS = 30;
  
  char relays[10][30][100] = {
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE",
     },};

  /*

  Matrix containing sets of values per relay.
               
      0      1     2     3     4       5            6
      >     <     ==     X     Y     Range     Standard/Inverted On/Off   
  {   0.0,  0.0,  0.0,   0.0,  0.0,   0.0,          1              }

  6: 0 = turn off if condition is true, turn on if condition is false
     1 = turn on if condition is true, turn off if condition is false

  */

  // calibratable matrix data (via local interface devices / RF / serial / baked-in here below if required)
  double relays_data[10][30+1][7] = {
    {
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0},
    },
    {
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0},
    },
    {
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0},
    },
    {
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0},
    },
    {
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0},
    },
    {
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0},
    },
    {
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0},
    },
    {
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0},
    },
    {
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0},
    },
    {
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 1},
      {0},
    },
  };

  // default and specifiable value to indicate a relay should not be activated/deactivated
  char default_relay_function[56] = "$NONE";

  char run_inetial_flag_gpatt_equal[56]    = "run_inetial_flag_gpatt_equal";
  char line_flag_gpatt_equal[56]           = "line_flag_gpatt_equal";
  char static_flag_gpatt_equal[56]         = "static_flag_gpatt_equal";
  char run_state_flag_gpatt_equal[56]      = "run_state_flag_gpatt_equal";
  char ins_gpatt_equal[56]                 = "ins_gpatt_equal";

  // F=Fix=31
  char fix_angle_flag_gpatt_over[56]       = "fix_angle_flag_gpatt_over";
  char fix_angle_flag_gpatt_under[56]      = "fix_angle_flag_gpatt_under[";
  char fix_angle_flag_gpatt_equal[56]      = "fix_angle_flag_gpatt_equal";
  char fix_angle_flag_gpatt_in_range[56]   = "fix_angle_flag_gpatt_in_range";

  char time_save_num_gpatt_over[56]        = "time_save_num_gpatt_over";
  char time_save_num_gpatt_under[56]       = "time_save_num_gpatt_under[";
  char time_save_num_gpatt_equal[56]       = "time_save_num_gpatt_equal";
  char time_save_num_gpatt_in_range[56]    = "time_save_num_gpatt_in_range";

  char mileage_gpatt_over[56]              = "mileage_gpatt_over";
  char mileage_gpatt_under[56]             = "mileage_gpatt_under[";
  char mileage_gpatt_equal[56]             = "mileage_gpatt_equal";
  char mileage_gpatt_in_range[56]          = "mileage_gpatt_in_range";

  char gst_data_gpatt_over[56]             = "gst_data_gpatt_over";
  char gst_data_gpatt_under[56]            = "gst_data_gpatt_under[";
  char gst_data_gpatt_equal[56]            = "gst_data_gpatt_equal";
  char gst_data_gpatt_in_range[56]         = "gst_data_gpatt_in_range";

  char yaw_gpatt_over[56]                  = "yaw_gpatt_over";
  char yaw_gpatt_under[56]                 = "yaw_gpatt_under[";
  char yaw_gpatt_equal[56]                 = "yaw_gpatt_equal";
  char yaw_gpatt_in_range[56]              = "yaw_gpatt_in_range";

  char roll_gpatt_over[56]                 = "roll_gpatt_over";
  char roll_gpatt_under[56]                = "roll_gpatt_under[";
  char roll_gpatt_equal[56]                = "roll_gpatt_equal";
  char roll_gpatt_in_range[56]             = "roll_gpatt_in_range";

  char pitch_gpatt_over[56]                = "pitch_gpatt_over";
  char pitch_gpatt_under[56]               = "pitch_gpatt_under[";
  char pitch_gpatt_equal[56]               = "pitch_gpatt_equal";
  char pitch_gpatt_in_range[56]            = "pitch_gpatt_in_range";

  char heading_gnrmc_over[56]              = "heading_gnrmc_over";
  char heading_gnrmc_under[56]             = "heading_gnrmc_under";
  char heading_gnrmc_equal[56]             = "heading_gnrmc_equal";
  char heading_gnrmc_in_range[56]          = "heading_gnrmc_in_range";

  char satellite_count_gngga_over[56]      = "satellite_count_gngga_over";
  char satellite_count_gngga_under[56]     = "satellite_count_gngga_under";
  char satellite_count_gngga_equal[56]     = "satellite_count_gngga_equal";

  char satellite_time_over[56]             = "satellite_time_over";
  char satellite_time_under[56]            = "satellite_time_under";
  char satellite_time_equal[56]            = "satellite_time_equal";
  char satellite_time_in_range[56]         = "satellite_time_in_range";

  char satellite_coord_gngga_over[56]      = "satellite_coord_gngga_over";
  char satellite_coord_gngga_under[56]     = "satellite_coord_gngga_under";
  char satellite_coord_gngga_equal[56]     = "satellite_coord_gngga_equal";
  char satellite_coord_gngga_in_range[56]  = "satellite_coord_gngga_in_range";

  char hemisphere_gngga_N[56]              = "hemisphere_gngga_N";
  char hemisphere_gngga_E[56]              = "hemisphere_gngga_E";
  char hemisphere_gngga_S[56]              = "hemisphere_gngga_S";
  char hemisphere_gngga_W[56]              = "hemisphere_gngga_W";

  char hemisphere_gngga_NE[56]             = "hemisphere_gngga_NE";
  char hemisphere_gngga_SE[56]             = "hemisphere_gngga_SE";
  char hemisphere_gngga_NW[56]             = "hemisphere_gngga_NW";
  char hemisphere_gngga_SW[56]             = "hemisphere_gngga_SW";

  char precision_factor_gngga_over[56]     = "precision_factor_gngga_over";
  char precision_factor_gngga_under[56]    = "precision_factor_gngga_under";
  char precision_factor_gngga_equal[56]    = "precision_factor_gngga_equal";
  char precision_factor_gngga_in_range[56] = "precision_factor_gngga_in_range";

  char altitude_gngga_over[56]             = "altitude_gngga_over";
  char altitude_gngga_under[56]            = "altitude_gngga_under";
  char altitude_gngga_equal[56]            = "altitude_gngga_equal";
  char altitude_gngga_in_range[56]         = "altitude_gngga_in_range";

  char ground_speed_gnrmc_over[56]         = "ground_speed_gnrmc_over";
  char ground_speed_gnrmc_under[56]        = "ground_speed_gnrmc_under";
  char ground_speed_gnrmc_equal[56]        = "ground_speed_gnrmc_equal";
  char ground_speed_gnrmc_in_range[56]     = "ground_speed_gnrmc_in_range";

  char utc_time_speed_over[56]             = "utc_time_speed_over";
  char utc_time_speed_under[56]            = "utc_time_speed_under";
  char utc_time_speed_equal[56]            = "utc_time_speed_equal";
  char utc_time_speed_in_range[56]         = "utc_time_speed_in_range";

  char ground_speed_speed_over[56]         = "ground_speed_speed_over";
  char ground_speed_speed_under[56]        = "ground_speed_speed_under";
  char ground_speed_speed_equal[56]        = "ground_speed_speed_equal";
  char ground_speed_speed_in_range[56]     = "ground_speed_speed_in_range";

  char status_speed_over[56]               = "status_speed_over";
  char status_speed_under[56]              = "status_speed_under";
  char status_speed_equal[56]              = "status_speed_equal";
  char status_speed_in_range[56]           = "status_speed_in_range";

  char acc_X_speed_over[56]                = "acc_X_speed_over";
  char acc_X_speed_under[56]               = "acc_X_speed_under";
  char acc_X_speed_equal[56]               = "acc_X_speed_equal";
  char acc_X_speed_in_range[56]            = "acc_X_speed_in_range";

  char acc_Y_speed_over[56]                = "acc_Y_speed_over";
  char acc_Y_speed_under[56]               = "acc_Y_speed_under";
  char acc_Y_speed_equal[56]               = "acc_Y_speed_equal";
  char acc_Y_speed_in_range[56]            = "acc_Y_speed_in_range";

  char acc_Z_speed_over[56]                = "acc_Z_speed_over";
  char acc_Z_speed_under[56]               = "acc_Z_speed_under";
  char acc_Z_speed_equal[56]               = "acc_Z_speed_equal";
  char acc_Z_speed_in_range[56]            = "acc_Z_speed_in_range";

  char gyro_X_speed_over[56]               = "gyro_X_speed_over";
  char gyro_X_speed_under[56]              = "gyro_X_speed_under";
  char gyro_X_speed_equal[56]              = "gyro_X_speed_equal";
  char gyro_X_speed_in_range[56]           = "gyro_X_speed_in_range";

  char gyro_Y_speed_over[56]               = "gyro_Y_speed_over";
  char gyro_Y_speed_under[56]              = "gyro_Y_speed_under";
  char gyro_Y_speed_equal[56]              = "gyro_Y_speed_equal";
  char gyro_Y_speed_in_range[56]           = "gyro_Y_speed_in_range";

  char gyro_Z_speed_over[56]               = "gyro_Z_speed_over";
  char gyro_Z_speed_under[56]              = "gyro_Z_speed_under";
  char gyro_Z_speed_equal[56]              = "gyro_Z_speed_equal";
  char gyro_Z_speed_in_range[56]           = "gyro_Z_speed_in_range";

  char ubi_state_flag_speed_over[56]       = "ubi_state_flag_speed_over";
  char ubi_state_flag_speed_under[56]      = "ubi_state_flag_speed_under";
  char ubi_state_flag_speed_equal[56]      = "ubi_state_flag_speed_equal";
  char ubi_state_flag_speed_in_range[56]   = "gyro_Z_speed_in_range";

  char ubi_state_kind_speed_over[56]       = "ubi_state_kind_speed_over";
  char ubi_state_kind_speed_under[56]      = "ubi_state_kind_speed_under";
  char ubi_state_kind_speed_equal[56]      = "ubi_state_kind_speed_equal";
  char ubi_state_kind_speed_in_range[56]   = "ubi_state_kind_speed_in_range";

  char ubi_state_value_speed_over[56]      = "ubi_state_value_speed_over";
  char ubi_state_value_speed_under[56]     = "ubi_state_value_speed_under";
  char ubi_state_value_speed_equal[56]     = "ubi_state_value_speed_equal";
  char ubi_state_value_speed_in_range[56]  = "ubi_state_value_speed_in_range";

  char utc_time_error_over[56]             = "utc_time_error_over";
  char utc_time_error_under[56]            = "utc_time_error_under";
  char utc_time_error_equal[56]            = "utc_time_error_equal";
  char utc_time_error_in_range[56]         = "utc_time_error_in_range";

  char code_flag_error_over[56]            = "code_flag_error_over";
  char code_flag_error_under[56]           = "code_flag_error_under";
  char code_flag_error_equal[56]           = "code_flag_error_equal";
  char code_flag_error_in_range[56]        = "code_flag_error_in_range";

  char gset_flag_error_equal[56]           = "gset_flag_error_equal";
  char sset_flag_error_equal[56]           = "sset_flag_error_equal";

  char coll_T_heading_debug_over[56]       = "coll_T_heading_debug_over";
  char coll_T_heading_debug_under[56]      = "coll_T_heading_debug_under";
  char coll_T_heading_debug_equal[56]      = "coll_T_heading_debug_equal";
  char coll_T_heading_debug_in_range[56]   = "coll_T_heading_debug_in_range";

  char coll_T_data_debug_over[56]          = "coll_T_data_debug_over";
  char coll_T_data_debug_under[56]         = "coll_T_data_debug_under";
  char coll_T_data_debug_equal[56]         = "coll_T_data_debug_equal";
  char coll_T_data_debug_in_range[56]      = "coll_T_data_debug_in_range";

  char ubi_valid_debug_equal[56]           = "ubi_valid_debug_equal";

  char ins_flag_debug_over[56]             = "ins_flag_debug_over";
  char ins_flag_debug_under[56]            = "ins_flag_debug_under";
  char ins_flag_debug_equal[56]            = "ins_flag_debug_equal";
  char ins_flag_debug_in_range[56]         = "ins_flag_debug_in_range";

  char car_speed_debug_over[56]            = "car_speed_debug_over";
  char car_speed_debug_under[56]           = "car_speed_debug_under";
  char car_speed_debug_equal[56]           = "car_speed_debug_equal";
  char car_speed_debug_in_range[56]        = "car_speed_debug_in_range";

  char yaw_angle_debug_over[56]            = "yaw_angle_debug_over";
  char yaw_angle_debug_under[56]           = "yaw_angle_debug_under";
  char yaw_angle_debug_equal[56]           = "yaw_angle_debug_equal";
  char yaw_angle_debug_in_range[56]        = "yaw_angle_debug_in_range";

  char roll_angle_debug_over[56]           = "roll_angle_debug_over";
  char roll_angle_debug_under[56]          = "roll_angle_debug_under";
  char roll_angle_debug_equal[56]          = "roll_angle_debug_equal";
  char roll_angle_debug_in_range[56]       = "roll_angle_debug_in_range";

  char pitch_angle_debug_over[56]          = "pitch_angle_debug_over";
  char pitch_angle_debug_under[56]         = "pitch_angle_debug_under";
  char pitch_angle_debug_equal[56]         = "pitch_angle_debug_equal";
  char pitch_angle_debug_in_range[56]      = "pitch_angle_debug_in_range";

  char ang_dget_flag_debug_equal[56]       = "ang_dget_flag_debug_equal";
  char ins_run_flag_debug_equal[56]        = "ins_run_flag_debug_equal";
  char fix_roll_flag_debug_equal[56]       = "fix_roll_flag_debug_equal";
  char fix_pitch_flag_debug_equal[56]      = "fix_pitch_flag_debug_equal";
  char ubi_kind_flag_debug_equal[56]       = "ubi_kind_flag_debug_equal";
  
  char ubi_on_flag_debug_over[56]          = "ubi_on_flag_debug_over";
  char ubi_on_flag_debug_under[56]         = "ubi_on_flag_debug_under";
  char ubi_on_flag_debug_equal[56]         = "ubi_on_flag_debug_equal";
  char ubi_on_flag_debug_in_range[56]      = "ubi_on_flag_debug_in_range";

  char ubi_a_set_debug_over[56]            = "ubi_a_set_debug_over";
  char ubi_a_set_debug_under[56]           = "ubi_a_set_debug_under";
  char ubi_a_set_debug_equal[56]           = "ubi_a_set_debug_equal";
  char ubi_a_set_debug_in_range[56]        = "ubi_a_set_debug_in_range";

  char ubi_b_set_debug_over[56]            = "ubi_b_set_debug_over";
  char ubi_b_set_debug_under[56]           = "ubi_b_set_debug_under";
  char ubi_b_set_debug_equal[56]           = "ubi_b_set_debug_equal";
  char ubi_b_set_debug_in_range[56]        = "ubi_b_set_debug_in_range";

  char acc_X_data_debug_over[56]           = "acc_X_data_debug_over";
  char acc_X_data_debug_under[56]          = "acc_X_data_debug_under";
  char acc_X_data_debug_equal[56]          = "acc_X_data_debug_equal";
  char acc_X_data_debug_in_range[56]       = "acc_X_data_debug_in_range";

  char acc_Y_data_debug_over[56]           = "acc_Y_data_debug_over";
  char acc_Y_data_debug_under[56]          = "acc_Y_data_debug_under";
  char acc_Y_data_debug_equal[56]          = "acc_Y_data_debug_equal";
  char acc_Y_data_debug_in_range[56]       = "acc_Y_data_debug_in_range";

  char gyro_Z_data_debug_over[56]          = "gyro_Z_data_debug_over";
  char gyro_Z_data_debug_under[56]         = "gyro_Z_data_debug_under";
  char gyro_Z_data_debug_equal[56]         = "gyro_Z_data_debug_equal";
  char gyro_Z_data_debug_in_range[56]      = "gyro_Z_data_debug_in_range";
};
RelayStruct relayData;


// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                    MATRIX CHECKS: PRIMITIVES

// calculate if x1 in range of specified x0 +- ( specified range / 2 )
bool in_range_check(double n0, double n1, double r) {
  if (n0  >=  n1 - r/2) {if (n0  <= n1 + r/2) {return true;}}
  else {return false;}
}

bool in_ranges_check(char * Fn0, char * Fn1, int Ri, int Fi) {
  Serial.println("[CHECKING] in_ranges_check");
  if (in_range_check(atoi(Fn0), relayData.relays_data[Ri][Fi][3], relayData.relays_data[Ri][Fi][5]) == true) {
    if (in_range_check(atoi(Fn1), relayData.relays_data[Ri][Fi][4], relayData.relays_data[Ri][Fi][5]) == true) {return true;}}
  else {return false;}
}

bool check_over(char * Fn, int Ri, int Fi) {
  Serial.println("[CHECKING] " + String(Fn) + " > " + String(relayData.relays_data[Ri][Fi][0]));
  if (atoi(Fn) > relayData.relays_data[Ri][Fi][0]) {return true;}
  else {return false;}
}

bool check_under(char * Fn, int Ri, int Fi) {
  Serial.println("[CHECKING] " + String(Fn) + " < " + String(relayData.relays_data[Ri][Fi][1]));
  if (atoi(Fn) < relayData.relays_data[Ri][Fi][1]) {return true;}
  else {return false;}
}

bool check_equal(char * Fn, int Ri, int Fi) {
  Serial.println("[CHECKING] " + String(Fn) + " == " + String(relayData.relays_data[Ri][Fi][2]));
  if (atoi(Fn) == relayData.relays_data[Ri][Fi][2]) {return true;}
  else {return false;}
}

// check range from specified x to specify y
bool check_in_range(char * Fn, int Ri, int Fi) {
  Serial.println("[CHECKING] " + String(Fn) + " > " + String(relayData.relays_data[Ri][Fi][3]) + " && " + String(Fn) + " < " + String(relayData.relays_data[Ri][Fi][4]));
  if ((atoi(Fn) >= relayData.relays_data[Ri][Fi][3]) && (atoi(Fn) <= relayData.relays_data[Ri][Fi][4])) {return true;}
  else {return false;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                          CHECKS: HEMISPHERES

bool hemisphere_gngga_N(int Ri, int Fi) {
  Serial.println("[CHECKING] hemisphere_gngga_N");
  if (strcmp(gnggaData.latitude_hemisphere, "N") == 0 ) {return true;}
  else {return false;}
}

bool hemisphere_gngga_E(int Ri, int Fi) {
  Serial.println("[CHECKING] hemisphere_gngga_E");
  if (strcmp(gnggaData.longitude_hemisphere, "E") == 0 ) {return true;}
  else {return false;}
}

bool hemisphere_gngga_S(int Ri, int Fi) {
  Serial.println("[CHECKING] hemisphere_gngga_S");
  if (strcmp(gnggaData.latitude_hemisphere, "S") == 0 ) {return true;}
  else {return false;}
}

bool hemisphere_gngga_W(int Ri, int Fi) {
  Serial.println("[CHECKING] hemisphere_gngga_W");
  if (strcmp(gnggaData.longitude_hemisphere, "W") == 0 ) {return true;}
  else {return false;}
}

bool hemisphere_gngga_NE(int Ri, int Fi) {
  Serial.println("[CHECKING] hemisphere_gngga_NE");
  if ((strcmp(gnggaData.latitude_hemisphere, "N") == 0 ) && (strcmp(gnggaData.longitude_hemisphere, "E") == 0)) {return true;}
  else {return false;}
}

bool hemisphere_gngga_SE(int Ri, int Fi) {
  Serial.println("[CHECKING] hemisphere_gngga_SE");
  if ((strcmp(gnggaData.latitude_hemisphere, "S") == 0 ) && (strcmp(gnggaData.longitude_hemisphere, "E") == 0)) {return true;}
  else {return false;}
}

bool hemisphere_gngga_NW(int Ri, int Fi) {
  Serial.println("[CHECKING] hemisphere_gngga_NW");
  if ((strcmp(gnggaData.latitude_hemisphere, "N") == 0 ) && (strcmp(gnggaData.longitude_hemisphere, "W") == 0)) {return true;}
  else {return false;}
}

bool hemisphere_gngga_SW(int Ri, int Fi) {
  Serial.println("[CHECKING] hemisphere_gngga_SW");
  if ((strcmp(gnggaData.latitude_hemisphere, "S") == 0 ) && (strcmp(gnggaData.longitude_hemisphere, "W") == 0)) {return true;}
  else {return false;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                               SYSTEMS CHECKS

/*
Check each relays key and run a function for each relays corresponding key. First check $NONE.
*/

void systems_Check() {

  /*
  Compound conditions can be created for each zero/one result at the final_bool. This allows for trillions of combinations with
  the current data alone.
  */

  // system test (simulate interface with matrix because there is no control panel/other HID yet):                      
  strcpy(relayData.relays[0][0], relayData.satellite_count_gngga_over); // 1: set relay zero's first check condition.
  relayData.relays_data[0][0][0]  = 1;                                  // 2: set relays first function data. in this case we will use the column 'over' element.
  relayData.relays_data[0][10][0] = 1;                                  // 3: lastly, soft enable the check/relay (IMPORTANT: ensure soft enable is zero if not in use)

  strcpy(relayData.relays[0][1], relayData.hemisphere_gngga_N);         // 1: optionally set relay zero's second check condition (because checks can be elemental or compounded).
  relayData.relays_data[0][10][0] = 1;                                  // 2: lastly, soft enable the check/relay (IMPORTANT: ensure soft enable is zero if not in use)

  // iterate over each relay array
  for (int Ri = 0; Ri < relayData.MAX_RELAYS; Ri++) {

    // Serial.println("[Ri] " + String(Ri) + " [ENABLED] " + String(relayData.relays_data[Ri][10][0]));
    if (relayData.relays_data[Ri][10][0] == 1) {

      // temporary switch must be zero each time. allows for polynomial expressions.
      bool tmp_matrix[1][30] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      int count_none_function = 0;

      // iterate over each function name for current relay, building the temporary matrix switch according to check reults
      for (int Fi = 0; Fi < 30; Fi++) {

        /*
        Possible combinations example: 100 checks ^ 10 functions = 100,000,000,000,000,000,000 combinations.

        Put true in the temporary matrix if no function is specified ($NONE). this allows 1-N elemental conditions to be set and result in true/false at the final bool.
        IMPORTANT: this also means if soft enable true, then final_bool defaults to true if no function at all is specified within a switches matrix. There is one check to catch you if you do soft enable with no functions set.
        */
        if (strcmp(relayData.relays[Ri][Fi], relayData.default_relay_function) == 0) {tmp_matrix[0][Fi] = 1; count_none_function++;}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                        SYSTEMS CHECKS: GNGGA

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.satellite_coord_gngga_in_range) == 0) {tmp_matrix[0][Fi] = in_ranges_check(satData.location_latitude_gngga_str, satData.location_longitude_gngga_str, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.satellite_count_gngga_over) == 0) {tmp_matrix[0][Fi] = check_over(gnggaData.satellite_count_gngga, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.satellite_count_gngga_under) == 0) {tmp_matrix[0][Fi] = check_under(gnggaData.satellite_count_gngga, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.satellite_count_gngga_equal) == 0) {tmp_matrix[0][Fi] = check_equal(gnggaData.satellite_count_gngga, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.satellite_time_over) == 0) {tmp_matrix[0][Fi] = check_over(satData.sat_time_stamp_string, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.satellite_time_under) == 0) {tmp_matrix[0][Fi] = check_under(satData.sat_time_stamp_string, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.satellite_time_equal) == 0) {tmp_matrix[0][Fi] = check_equal(satData.sat_time_stamp_string, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.satellite_time_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(satData.sat_time_stamp_string, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.hemisphere_gngga_N) == 0) {tmp_matrix[0][Fi] = hemisphere_gngga_N(Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.hemisphere_gngga_E) == 0) {tmp_matrix[0][Fi] = hemisphere_gngga_E(Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.hemisphere_gngga_S) == 0) {tmp_matrix[0][Fi] = hemisphere_gngga_S(Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.hemisphere_gngga_W) == 0) {tmp_matrix[0][Fi] = hemisphere_gngga_W(Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.hemisphere_gngga_NE) == 0) {tmp_matrix[0][Fi] = hemisphere_gngga_NE(Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.hemisphere_gngga_NW) == 0) {tmp_matrix[0][Fi] = hemisphere_gngga_NW(Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.hemisphere_gngga_SE) == 0) {tmp_matrix[0][Fi] = hemisphere_gngga_SE(Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.hemisphere_gngga_SW) == 0) {tmp_matrix[0][Fi] = hemisphere_gngga_SW(Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.precision_factor_gngga_over) == 0) {tmp_matrix[0][Fi] = check_over(gnggaData.hdop_precision_factor, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.precision_factor_gngga_under) == 0) {tmp_matrix[0][Fi] = check_under(gnggaData.hdop_precision_factor, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.precision_factor_gngga_equal) == 0) {tmp_matrix[0][Fi] = check_equal(gnggaData.hdop_precision_factor, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.precision_factor_gngga_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(gnggaData.hdop_precision_factor, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.altitude_gngga_over) == 0) {tmp_matrix[0][Fi] = check_over(gnggaData.altitude, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.altitude_gngga_under) == 0) {tmp_matrix[0][Fi] = check_under(gnggaData.altitude, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.altitude_gngga_equal) == 0) {tmp_matrix[0][Fi] = check_equal(gnggaData.altitude, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.altitude_gngga_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(gnggaData.altitude, Ri, Fi);}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                        SYSTEMS CHECKS: GNRMC

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ground_speed_gnrmc_over) == 0) {tmp_matrix[0][Fi] = check_over(gnrmcData.ground_speed, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ground_speed_gnrmc_under) == 0) {tmp_matrix[0][Fi] = check_under(gnrmcData.ground_speed, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ground_speed_gnrmc_equal) == 0) {tmp_matrix[0][Fi] = check_equal(gnrmcData.ground_speed, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ground_speed_gnrmc_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(gnrmcData.ground_speed, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.heading_gnrmc_over) == 0) {tmp_matrix[0][Fi] = check_over(gnrmcData.ground_heading, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.heading_gnrmc_under) == 0) {tmp_matrix[0][Fi] = check_under(gnrmcData.ground_heading, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.heading_gnrmc_equal) == 0) {tmp_matrix[0][Fi] = check_equal(gnrmcData.ground_heading, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.heading_gnrmc_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(gnrmcData.ground_heading, Ri, Fi);}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                        SYSTEMS CHECKS: GPATT

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.pitch_gpatt_over) == 0) {tmp_matrix[0][Fi] = check_over(gpattData.pitch, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.pitch_gpatt_under) == 0) {tmp_matrix[0][Fi] = check_under(gpattData.pitch, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.pitch_gpatt_equal) == 0) {tmp_matrix[0][Fi] = check_equal(gpattData.pitch, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.pitch_gpatt_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(gpattData.pitch, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.roll_gpatt_over) == 0) {tmp_matrix[0][Fi] = check_over(gpattData.roll, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.roll_gpatt_under) == 0) {tmp_matrix[0][Fi] = check_under(gpattData.roll, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.roll_gpatt_equal) == 0) {tmp_matrix[0][Fi] = check_equal(gpattData.roll, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.roll_gpatt_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(gpattData.roll, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.yaw_gpatt_over) == 0) {tmp_matrix[0][Fi] = check_over(gpattData.yaw, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.yaw_gpatt_under) == 0) {tmp_matrix[0][Fi] = check_under(gpattData.yaw, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.yaw_gpatt_equal) == 0) {tmp_matrix[0][Fi] = check_equal(gpattData.yaw, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.yaw_gpatt_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(gpattData.yaw, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ins_gpatt_equal) == 0) {tmp_matrix[0][Fi] = check_equal(gpattData.ins, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.run_state_flag_gpatt_equal) == 0) {tmp_matrix[0][Fi] = check_equal(gpattData.run_state_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.static_flag_gpatt_equal) == 0) {tmp_matrix[0][Fi] = check_equal(gpattData.static_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gst_data_gpatt_over) == 0) {tmp_matrix[0][Fi] = check_over(gpattData.gst_data, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gst_data_gpatt_under) == 0) {tmp_matrix[0][Fi] = check_under(gpattData.gst_data, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gst_data_gpatt_equal) == 0) {tmp_matrix[0][Fi] = check_equal(gpattData.gst_data, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gst_data_gpatt_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(gpattData.gst_data, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.line_flag_gpatt_equal) == 0) {tmp_matrix[0][Fi] = check_equal(gpattData.line_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.mileage_gpatt_over) == 0) {tmp_matrix[0][Fi] = check_over(gpattData.mileage, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.mileage_gpatt_under) == 0) {tmp_matrix[0][Fi] = check_under(gpattData.mileage, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.mileage_gpatt_equal) == 0) {tmp_matrix[0][Fi] = check_equal(gpattData.mileage, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.mileage_gpatt_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(gpattData.mileage, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.run_inetial_flag_gpatt_equal) == 0) {tmp_matrix[0][Fi] = check_equal(gpattData.run_inetial_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.time_save_num_gpatt_over) == 0) {tmp_matrix[0][Fi] = check_over(gpattData.time_save_num, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.time_save_num_gpatt_under) == 0) {tmp_matrix[0][Fi] = check_under(gpattData.time_save_num, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.time_save_num_gpatt_equal) == 0) {tmp_matrix[0][Fi] = check_equal(gpattData.time_save_num, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.time_save_num_gpatt_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(gpattData.time_save_num, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.fix_angle_flag_gpatt_over) == 0) {tmp_matrix[0][Fi] = check_over(gpattData.fix_angle_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.fix_angle_flag_gpatt_under) == 0) {tmp_matrix[0][Fi] = check_under(gpattData.fix_angle_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.fix_angle_flag_gpatt_equal) == 0) {tmp_matrix[0][Fi] = check_equal(gpattData.fix_angle_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.fix_angle_flag_gpatt_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(gpattData.fix_angle_flag, Ri, Fi);}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                        SYSTEMS CHECKS: SPEED

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_state_value_speed_over) == 0) {tmp_matrix[0][Fi] = check_over(speedData.ubi_state_value, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_state_value_speed_under) == 0) {tmp_matrix[0][Fi] = check_under(speedData.ubi_state_value, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_state_value_speed_equal) == 0) {tmp_matrix[0][Fi] = check_equal(speedData.ubi_state_value, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_state_value_speed_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(speedData.ubi_state_value, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_state_kind_speed_over) == 0) {tmp_matrix[0][Fi] = check_over(speedData.ubi_state_kind, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_state_kind_speed_under) == 0) {tmp_matrix[0][Fi] = check_under(speedData.ubi_state_kind, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_state_kind_speed_equal) == 0) {tmp_matrix[0][Fi] = check_equal(speedData.ubi_state_kind, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_state_kind_speed_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(speedData.ubi_state_kind, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_state_flag_speed_over) == 0) {tmp_matrix[0][Fi] = check_over(speedData.ubi_state_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_state_flag_speed_under) == 0) {tmp_matrix[0][Fi] = check_under(speedData.ubi_state_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_state_flag_speed_equal) == 0) {tmp_matrix[0][Fi] = check_equal(speedData.ubi_state_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_state_flag_speed_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(speedData.ubi_state_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_Z_speed_over) == 0) {tmp_matrix[0][Fi] = check_over(speedData.gyro_Z, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_Z_speed_under) == 0) {tmp_matrix[0][Fi] = check_under(speedData.gyro_Z, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_Z_speed_equal) == 0) {tmp_matrix[0][Fi] = check_equal(speedData.gyro_Z, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_Z_speed_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(speedData.gyro_Z, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_Y_speed_over) == 0) {tmp_matrix[0][Fi] = check_over(speedData.gyro_Y, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_Y_speed_under) == 0) {tmp_matrix[0][Fi] = check_under(speedData.gyro_Y, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_Y_speed_equal) == 0) {tmp_matrix[0][Fi] = check_equal(speedData.gyro_Y, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_Y_speed_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(speedData.gyro_Y, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_X_speed_over) == 0) {tmp_matrix[0][Fi] = check_over(speedData.gyro_X, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_X_speed_under) == 0) {tmp_matrix[0][Fi] = check_under(speedData.gyro_X, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_X_speed_equal) == 0) {tmp_matrix[0][Fi] = check_equal(speedData.gyro_X, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_X_speed_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(speedData.gyro_X, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_Z_speed_over) == 0) {tmp_matrix[0][Fi] = check_over(speedData.acc_Z, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_Z_speed_under) == 0) {tmp_matrix[0][Fi] = check_under(speedData.acc_Z, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_Z_speed_equal) == 0) {tmp_matrix[0][Fi] = check_equal(speedData.acc_Z, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_Z_speed_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(speedData.acc_Z, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_Y_speed_over) == 0) {tmp_matrix[0][Fi] = check_over(speedData.acc_Y, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_Y_speed_under) == 0) {tmp_matrix[0][Fi] = check_under(speedData.acc_Y, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_Y_speed_equal) == 0) {tmp_matrix[0][Fi] = check_equal(speedData.acc_Y, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_Y_speed_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(speedData.acc_Y, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_X_speed_over) == 0) {tmp_matrix[0][Fi] = check_over(speedData.acc_X, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_X_speed_under) == 0) {tmp_matrix[0][Fi] = check_under(speedData.acc_X, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_X_speed_equal) == 0) {tmp_matrix[0][Fi] = check_equal(speedData.acc_X, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_X_speed_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(speedData.acc_X, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.status_speed_over) == 0) {tmp_matrix[0][Fi] = check_over(speedData.status, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.status_speed_under) == 0) {tmp_matrix[0][Fi] = check_under(speedData.status, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.status_speed_equal) == 0) {tmp_matrix[0][Fi] = check_equal(speedData.status, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.status_speed_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(speedData.status, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ground_speed_speed_over) == 0) {tmp_matrix[0][Fi] = check_over(speedData.speed, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ground_speed_speed_under) == 0) {tmp_matrix[0][Fi] = check_under(speedData.speed, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ground_speed_speed_equal) == 0) {tmp_matrix[0][Fi] = check_equal(speedData.speed, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ground_speed_speed_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(speedData.speed, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_time_speed_over) == 0) {tmp_matrix[0][Fi] = check_over(speedData.utc_time, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_time_speed_under) == 0) {tmp_matrix[0][Fi] = check_under(speedData.utc_time, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_time_speed_equal) == 0) {tmp_matrix[0][Fi] = check_equal(speedData.utc_time, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_time_speed_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(speedData.utc_time, Ri, Fi);}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                        SYSTEMS CHECKS: ERROR

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gset_flag_error_equal) == 0) {tmp_matrix[0][Fi] = check_over(errorData.gset_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.sset_flag_error_equal) == 0) {tmp_matrix[0][Fi] = check_under(errorData.sset_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.code_flag_error_over) == 0) {tmp_matrix[0][Fi] = check_over(errorData.code_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.code_flag_error_under) == 0) {tmp_matrix[0][Fi] = check_under(errorData.code_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.code_flag_error_equal) == 0) {tmp_matrix[0][Fi] = check_equal(errorData.code_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.code_flag_error_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(errorData.code_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_time_error_over) == 0) {tmp_matrix[0][Fi] = check_over(errorData.utc, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_time_error_under) == 0) {tmp_matrix[0][Fi] = check_under(errorData.utc, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_time_error_equal) == 0) {tmp_matrix[0][Fi] = check_equal(errorData.utc, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_time_error_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(errorData.utc, Ri, Fi);}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                        SYSTEMS CHECKS: DEBUG

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.coll_T_heading_debug_over) == 0) {tmp_matrix[0][Fi] = check_over(debugData.coll_T_heading, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.coll_T_heading_debug_under) == 0) {tmp_matrix[0][Fi] = check_under(debugData.coll_T_heading, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.coll_T_heading_debug_equal) == 0) {tmp_matrix[0][Fi] = check_equal(debugData.coll_T_heading, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.coll_T_heading_debug_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(debugData.coll_T_heading, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.coll_T_data_debug_over) == 0) {tmp_matrix[0][Fi] = check_over(debugData.coll_T_data, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.coll_T_data_debug_under) == 0) {tmp_matrix[0][Fi] = check_under(debugData.coll_T_data, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.coll_T_data_debug_equal) == 0) {tmp_matrix[0][Fi] = check_equal(debugData.coll_T_data, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.coll_T_data_debug_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(debugData.coll_T_data, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_valid_debug_equal) == 0) {tmp_matrix[0][Fi] = check_equal(debugData.ubi_valid, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ins_flag_debug_over) == 0) {tmp_matrix[0][Fi] = check_over(debugData.ins_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ins_flag_debug_under) == 0) {tmp_matrix[0][Fi] = check_under(debugData.ins_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ins_flag_debug_equal) == 0) {tmp_matrix[0][Fi] = check_equal(debugData.ins_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ins_flag_debug_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(debugData.ins_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.car_speed_debug_over) == 0) {tmp_matrix[0][Fi] = check_over(debugData.car_speed, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.car_speed_debug_under) == 0) {tmp_matrix[0][Fi] = check_under(debugData.car_speed, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.car_speed_debug_equal) == 0) {tmp_matrix[0][Fi] = check_equal(debugData.car_speed, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.car_speed_debug_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(debugData.car_speed, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.yaw_angle_debug_over) == 0) {tmp_matrix[0][Fi] = check_over(debugData.yaw_angle, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.yaw_angle_debug_under) == 0) {tmp_matrix[0][Fi] = check_under(debugData.yaw_angle, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.yaw_angle_debug_equal) == 0) {tmp_matrix[0][Fi] = check_equal(debugData.yaw_angle, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.yaw_angle_debug_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(debugData.yaw_angle, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.roll_angle_debug_over) == 0) {tmp_matrix[0][Fi] = check_over(debugData.roll_angle, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.roll_angle_debug_under) == 0) {tmp_matrix[0][Fi] = check_under(debugData.roll_angle, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.roll_angle_debug_equal) == 0) {tmp_matrix[0][Fi] = check_equal(debugData.roll_angle, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.roll_angle_debug_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(debugData.roll_angle, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.pitch_angle_debug_over) == 0) {tmp_matrix[0][Fi] = check_over(debugData.pitch_angle, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.pitch_angle_debug_under) == 0) {tmp_matrix[0][Fi] = check_under(debugData.pitch_angle, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.pitch_angle_debug_equal) == 0) {tmp_matrix[0][Fi] = check_equal(debugData.pitch_angle, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.pitch_angle_debug_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(debugData.pitch_angle, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ang_dget_flag_debug_equal) == 0) {tmp_matrix[0][Fi] = check_equal(debugData.ang_dget_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ins_run_flag_debug_equal) == 0) {tmp_matrix[0][Fi] = check_equal(debugData.ins_run_flag, Ri, Fi);}

         // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.fix_roll_flag_debug_equal) == 0) {tmp_matrix[0][Fi] = check_equal(debugData.fix_roll_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.fix_pitch_flag_debug_equal) == 0) {tmp_matrix[0][Fi] = check_equal(debugData.fix_pitch_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_kind_flag_debug_equal) == 0) {tmp_matrix[0][Fi] = check_equal(debugData.ubi_kind_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_on_flag_debug_over) == 0) {tmp_matrix[0][Fi] = check_over(debugData.ubi_on_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_on_flag_debug_under) == 0) {tmp_matrix[0][Fi] = check_under(debugData.ubi_on_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_on_flag_debug_equal) == 0) {tmp_matrix[0][Fi] = check_equal(debugData.ubi_on_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_on_flag_debug_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(debugData.ubi_on_flag, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_a_set_debug_over) == 0) {tmp_matrix[0][Fi] = check_over(debugData.ubi_a_set, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_a_set_debug_under) == 0) {tmp_matrix[0][Fi] = check_under(debugData.ubi_a_set, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_a_set_debug_equal) == 0) {tmp_matrix[0][Fi] = check_equal(debugData.ubi_a_set, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_a_set_debug_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(debugData.ubi_a_set, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_b_set_debug_over) == 0) {tmp_matrix[0][Fi] = check_over(debugData.ubi_b_set, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_b_set_debug_under) == 0) {tmp_matrix[0][Fi] = check_under(debugData.ubi_b_set, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_b_set_debug_equal) == 0) {tmp_matrix[0][Fi] = check_equal(debugData.ubi_b_set, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_b_set_debug_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(debugData.ubi_b_set, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_X_data_debug_over) == 0) {tmp_matrix[0][Fi] = check_over(debugData.acc_X_data, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_X_data_debug_under) == 0) {tmp_matrix[0][Fi] = check_under(debugData.acc_X_data, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_X_data_debug_equal) == 0) {tmp_matrix[0][Fi] = check_equal(debugData.acc_X_data, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_X_data_debug_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(debugData.acc_X_data, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_Y_data_debug_over) == 0) {tmp_matrix[0][Fi] = check_over(debugData.acc_Y_data, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_Y_data_debug_under) == 0) {tmp_matrix[0][Fi] = check_under(debugData.acc_Y_data, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_Y_data_debug_equal) == 0) {tmp_matrix[0][Fi] = check_equal(debugData.acc_Y_data, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_Y_data_debug_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(debugData.acc_Y_data, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_Z_data_debug_over) == 0) {tmp_matrix[0][Fi] = check_over(debugData.gyro_Z_data, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_Z_data_debug_under) == 0) {tmp_matrix[0][Fi] = check_under(debugData.gyro_Z_data, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_Z_data_debug_equal) == 0) {tmp_matrix[0][Fi] = check_equal(debugData.gyro_Z_data, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_Z_data_debug_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(debugData.gyro_Z_data, Ri, Fi);}

        /*
        the above checks are the 'basic' checks. with that out the way, we can now build more 'advanced' checks/calculations, using the same format.
        a way to make this more efficient would also be as preferrable as the degree to which we can currently formulate compund expressions via the matrices and matrix switch.
        if the data/system is critical then this all needs to happen as fast as possible. 
        */

        // Serial.println("[tmp_matrix] " + String(Fi) + " [DAT] " + String(tmp_matrix[0][Fi]));
      }
      
      // safety layer: disengage if all entries are $NONE. if you enabled a master switch with no functions set then this could save you.
      if (count_none_function <= 29) {

        // default final bool is true and if a single false is found final bool should be set to false and remain false
        bool final_bool = true;
        for (int FC = 0; FC < 30; FC++) {if (tmp_matrix[0][FC] == 0) {final_bool = false;}}
        // Serial.println("[FINAL_BOOL] " + String(final_bool));

        /*
        Remember always: why do you think you can trust this data? are you transmitting this data to yourelf (from satellite or not)?
                         how critical are your system(s)?
                         once you plug something into this, the 'satellites' are in control unless you have a way to override.
        */

        // activate/deactivate relay Ri (Ri=pinN): pin number matrix required for relay selcection via Ri->PIN column access in non-linear form (multiplex relays)
        if (final_bool == false) {Serial.println("[RELAY " + String(Ri) + "] de-activating");}
        else if (final_bool == true) {Serial.println("[RELAY " + String(Ri) + "] activating");}
      }
      else {Serial.println("[RELAY " + String(Ri) + "] WARNING: Matrix checks are enabled for an non configured matrix!");}
    }
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    MAIN LOOP

void loop() {
  readRXD_1();
  // extrapulatedSatData();
  // SSD_Display_4();
  // SSD_Display_5();
  // SSD_Display_6();
  // SSD_Display_7();
  // systems_Check();

  delay(1);
}

// ----------------------------------------------------------------------------------------------------------------------------


