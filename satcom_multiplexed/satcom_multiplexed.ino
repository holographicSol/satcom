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
  bool gngga_sentence = false;
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

bool val_utc_date(char * data) {
  bool check_pass = false;
  if (strlen(data) == 6) {
    if ((atoi(data) >= 0.0) && (atoi(data) <= 999999)) {check_pass = true;}
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
    if ((strcmp(data, "N") == 0) || (strcmp(data, "S") == 0)) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_longitude_H(char * data) {
  bool check_pass = false;
  if (strlen(data) == 1) {
    if ((strcmp(data, "E") == 0) || (strcmp(data, "W") == 0)) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_positioning_status_gngga(char * data) {
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
    if (strcmp(data, "M") == 0) {
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
    if (strcmp(data, "M") == 0) {
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

bool val_positioning_status_gnrmc(char * data) {
  bool check_pass = false;
  if (strlen(data) == 1) {
    if ((strcmp(data, "A") == 0) || (strcmp(data, "V") == 0)) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_ground_speed(char * data) {
  bool check_pass = false;
  if (atoi(data) >= 0){
    check_pass = true;
  }
  return check_pass;
}

bool val_ground_heading(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= 0) && (atoi(data) <= 360)) {
    check_pass = true;
  }
  return check_pass;
}

bool val_installation_angle(char * data) {
  bool check_pass = false;
  if (atoi(data) >= 0) {
    check_pass = true;
  }
  return check_pass;
}

bool val_installation_angle_direction(char * data) {
  bool check_pass = false;
  if (strlen(data) == 1) {
    if ((strcmp(data, "E") == 0) || (strcmp(data, "W") == 0) || (strcmp(data, "M") == 0)) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_mode_indication(char * data) {
  bool check_pass = false;
  if (strlen(data) == 1) {
    if ((strcmp(data, "A") == 0) || (strcmp(data, "D") == 0) || (strcmp(data, "E") == 0) || (strcmp(data, "N") == 0)) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_pitch_gpatt(char * data) {
  bool check_pass = false;
  if (atoi(data) >= 0) {check_pass = true;}
  return check_pass;
}

bool val_roll_gpatt(char * data) {
  bool check_pass = false;
  if (atoi(data) >= 0) {check_pass = true;}
  return check_pass;
}

bool val_yaw_gpatt(char * data) {
  bool check_pass = false;
  if (atoi(data) >= 0) {check_pass = true;}
  return check_pass;
}


bool val_angle_channle_p_gpatt(char * data) {
  bool check_pass = false;
  if (strcmp(data, "p") == 0) {check_pass = true;}
  return check_pass;
}

bool val_angle_channle_r_gpatt(char * data) {
  bool check_pass = false;
  if (strcmp(data, "r") == 0) {check_pass = true;}
  return check_pass;
}

bool val_angle_channle_y_gpatt(char * data) {
  bool check_pass = false;
  if (strcmp(data, "y") == 0) {check_pass = true;}
  return check_pass;
}

bool val_version_channel_s_gpatt(char * data) {
  bool check_pass = false;
  if (strcmp(data, "s") == 0) {check_pass = true;}
  return check_pass;
}

bool val_software_version_gpatt(char * data) {
  bool check_pass = false;
  if (atoi(data) == 20230219) {check_pass = true;}
  return check_pass;
}

bool val_product_id_gpatt(char * data) {
  bool check_pass = false;
  if (strcmp(data, "003E009") == 0) {check_pass = true;}
  return check_pass;
}

bool val_id_channel_gpatt(char * data) {
  bool check_pass = false;
  if (strcmp(data, "ID") == 0) {check_pass = true;}
  return check_pass;
}

bool val_ins_gpatt(char * data) {
  bool check_pass = false;
  if ((atoi(data) == 0) || (atoi(data) == 1)) {check_pass = true;}
  return check_pass;
}

bool val_ins_channel_gpatt(char * data) {
  bool check_pass = false;
  if (strcmp(data, "INS") == 0) {check_pass = true;}
  return check_pass;
}

bool val_hardware_version_gpatt(char * data) {
  bool check_pass = false;
  if (strcmp(data, "3335") == 0) {check_pass = true;}
  return check_pass;
}

bool val_run_state_flag_gpatt(char * data) {
  bool check_pass = false;
  if ((strcmp(data, "01") == 0) || (strcmp(data, "02") == 0) || (strcmp(data, "03") == 0)) {check_pass = true;}
  return check_pass;
}

bool val_mis_angle_num_gpatt(char * data) {
  bool check_pass = false;
  if (atoi(data) >= 0) {check_pass = true;}
  return check_pass;
}

bool val_static_flag_gpatt(char * data) {
  bool check_pass = false;
  if ((atoi(data) == 0) || (atoi(data) == 1)) {check_pass = true;}
  return check_pass;
}

bool val_user_code_gpatt(char * data) {
  bool check_pass = false;
  if (atoi(data) >= 0) {check_pass = true;}
  return check_pass;
}

bool val_gst_data_gpatt(char * data) {
  bool check_pass = false;
  if (atoi(data) >= 0) {check_pass = true;}
  return check_pass;
}

bool val_line_flag_gpatt(char * data) {
  bool check_pass = false;
  if ((atoi(data) == 0) || (atoi(data) == 1)) {check_pass = true;}
  return check_pass;
}

bool val_mis_att_flag_gpatt(char * data) {
  bool check_pass = false;
  if ((atoi(data) == 0) || (atoi(data) == 1)) {check_pass = true;}
  return check_pass;
}

bool val_imu_kind_gpatt(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= 0) && (atoi(data) <= 7)) {check_pass = true;}
  return check_pass;
}

bool val_ubi_car_kind_gpatt(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= 1) && (atoi(data) <= 4)) {check_pass = true;}
  return check_pass;
}

bool val_mileage_gpatt(char * data) {
  bool check_pass = false;
  if (atoi(data) >= 0) {check_pass = true;}
  return check_pass;
}

bool val_run_inetial_flag_gpatt(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= 0) && (atoi(data) <= 4)) {check_pass = true;}
  return check_pass;
}

bool val_speed_enable_gpatt(char * data) {
  bool check_pass = false;
  if ((atoi(data) == 0) || (atoi(data) == 1)) {check_pass = true;}
  return check_pass;
}

bool val_speed_num_gpatt(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= 0) && (atoi(data) <= 99)) {check_pass = true;}
  return check_pass;
}

bool val_speed_status(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= 0) && (atoi(data) <= 2)) {check_pass = true;}
  return check_pass;
}

bool val_accelleration_delimiter(char * data) {
  bool check_pass = false;
  if (strcmp(data, "A") == 0) {check_pass = true;}
  return check_pass;
}

bool val_axis_accelleration(char * data) {
  bool check_pass = false;
  if (atoi(data) >= -1000000) {check_pass = true;}
  return check_pass;
}

bool val_angular_velocity_delimiter(char * data) {
  bool check_pass = false;
  if (strcmp(data, "G") == 0) {check_pass = true;}
  return check_pass;
}

bool val_gyro_angular_velocity(char * data) {
  bool check_pass = false;
  if (atoi(data) >= -1000000) {check_pass = true;}
  return check_pass;
}

bool val_status_delimiter(char * data) {
  bool check_pass = false;
  if (strcmp(data, "S") == 0) {check_pass = true;}
  return check_pass;
}

bool val_ubi_state_flag(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  return check_pass;
}

bool val_ubi_state_kind_flag(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= 0) && (atoi(data) <= 8)) {check_pass = true;}
  return check_pass;
}

bool val_code_flag(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  return check_pass;
}

bool val_gset_flag(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  return check_pass;
}

bool val_sset_flag(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  return check_pass;
}

bool val_ang_dget_flag(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  return check_pass;
}

bool val_ins_run_flag(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  return check_pass;
}

bool val_fix_kind_flag(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  return check_pass;
}

bool val_fix_roll_flag(char * data) {
  bool check_pass = false;
  if (atoi(data) >= -1000000) {check_pass = true;}
  return check_pass;
}

bool val_fix_pitch_flag(char * data) {
  bool check_pass = false;
  if (atoi(data) >= -1000000) {check_pass = true;}
  return check_pass;
}

bool val_ubi_on_flag(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= 0) && (atoi(data) <= 8)) {check_pass = true;}
  return check_pass;
}

bool val_ubi_kind_flag(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= 0) && (atoi(data) <= 2)) {check_pass = true;}
  return check_pass;
}

bool val_ubi_a_set(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= 0) && (atoi(data) <= 19)) {check_pass = true;}
  return check_pass;
}

bool val_ubi_b_set(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= 0) && (atoi(data) <= 19)) {check_pass = true;}
  return check_pass;
}

bool val_acc_X_data(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= -400) && (atoi(data) <= 400)) {check_pass = true;}
  return check_pass;
}

bool val_acc_Y_data(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= -400) && (atoi(data) <= 400)) {check_pass = true;}
  return check_pass;
}

bool val_gyro_Z_data(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= -250) && (atoi(data) <= 250)) {check_pass = true;}
  return check_pass;
}

bool val_pitch_angle(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= -180) && (atoi(data) <= 180)) {check_pass = true;}
  return check_pass;
}

bool val_roll_angle(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= -180) && (atoi(data) <= 180)) {check_pass = true;}
  return check_pass;
}

bool val_yaw_angle(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= -180) && (atoi(data) <= 180)) {check_pass = true;}
  return check_pass;
}

bool val_car_speed(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= 0) && (atoi(data) <= 100)) {check_pass = true;}
  return check_pass;
}

bool val_ins_flag(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= 0) && (atoi(data) <= 4)) {check_pass = true;}
  return check_pass;
}

bool val_ubi_num(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= 0) && (atoi(data) <= 65536)) {check_pass = true;}
  return check_pass;
}

bool val_ubi_valid(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  return check_pass;
}

bool val_coll_T_data(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= -800) && (atoi(data) <= 800)) {check_pass = true;}
  return check_pass;
}

bool val_coll_T_heading(char * data) {
  bool check_pass = false;
  if ((atoi(data) >= -180) && (atoi(data) <= 180)) {check_pass = true;}
  return check_pass;
}

bool val_custom_flag(char * data) {
  bool check_pass = false;
  if (strlen(data) >= 1) {check_pass = true;}
  return check_pass;
}

bool val_checksum(char * data) {
  bool check_pass = false;
  if (strlen(data) == 3) {check_pass = true;}
  return check_pass;
}

bool val_scalable(char * data) {
  bool check_pass = false;
  if (strlen(data) >= 1) {check_pass = true;}
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
  char temporary_data_1[56];
  int check_data = 0;                   // should result in 16
  unsigned long bad_utc_time_i;
  unsigned long bad_latitude_i;
  unsigned long bad_latitude_hemisphere_i;
  unsigned long bad_longitude_i;
  unsigned long bad_longitude_hemisphere_i;
  unsigned long bad_positioning_status_i;
  unsigned long bad_satellite_count_gngga_i;
  unsigned long bad_hdop_precision_factor_i;
  unsigned long bad_altitude_i;
  unsigned long bad_altitude_units_i;
  unsigned long bad_geoidal_i;
  unsigned long bad_geoidal_units_i;
  unsigned long bad_differential_delay_i;
  unsigned long bad_id_i;
  unsigned long bad_check_sum_i;
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
    if     (serialData.iter_token == 0)                                                               {strcpy(gnggaData.tag, "GNGGA");                            gnggaData.check_data++;}
    else if (serialData.iter_token ==1)  {if (val_utc_time(serialData.token) == true)                 {strcpy(gnggaData.utc_time, serialData.token);              gnggaData.check_data++;} else {gnggaData.bad_utc_time_i++;}}
    else if (serialData.iter_token ==2)  {if (val_latitude(serialData.token) == true)                 {strcpy(gnggaData.latitude, serialData.token);              gnggaData.check_data++;} else {gnggaData.bad_latitude_i++;}}
    else if (serialData.iter_token ==3)  {if (val_latitude_H(serialData.token) == true)               {strcpy(gnggaData.latitude_hemisphere, serialData.token);   gnggaData.check_data++;} else {gnggaData.bad_latitude_hemisphere_i++;}}
    else if (serialData.iter_token ==4)  {if (val_longitude(serialData.token) == true)                {strcpy(gnggaData.longitude, serialData.token);             gnggaData.check_data++;} else {gnggaData.bad_longitude_i++;}}
    else if (serialData.iter_token ==5)  {if (val_longitude_H(serialData.token) == true)              {strcpy(gnggaData.longitude_hemisphere, serialData.token);  gnggaData.check_data++;} else {gnggaData.bad_longitude_hemisphere_i++;}}
    else if (serialData.iter_token ==6)  {if (val_positioning_status_gngga(serialData.token) == true) {strcpy(gnggaData.positioning_status, serialData.token);    gnggaData.check_data++;} else {gnggaData.bad_positioning_status_i++;}}
    else if (serialData.iter_token ==7)  {if (val_satellite_count(serialData.token) == true)          {strcpy(gnggaData.satellite_count_gngga, serialData.token); gnggaData.check_data++;} else {gnggaData.bad_satellite_count_gngga_i++;}}
    else if (serialData.iter_token ==8)  {if (val_hdop_precision_factor(serialData.token) == true)    {strcpy(gnggaData.hdop_precision_factor, serialData.token); gnggaData.check_data++;} else {gnggaData.bad_hdop_precision_factor_i++;}}
    else if (serialData.iter_token ==9)  {if (val_altitude(serialData.token) == true)                 {strcpy(gnggaData.altitude, serialData.token);              gnggaData.check_data++;} else {gnggaData.bad_altitude_i++;}}
    else if (serialData.iter_token ==10) {if (val_altitude_units(serialData.token) == true)           {strcpy(gnggaData.altitude_units, serialData.token);        gnggaData.check_data++;} else {gnggaData.bad_altitude_units_i++;}}
    else if (serialData.iter_token ==11) {if (val_geoidal(serialData.token) == true)                  {strcpy(gnggaData.geoidal, serialData.token);               gnggaData.check_data++;} else {gnggaData.bad_geoidal_i++;}}
    else if (serialData.iter_token ==12) {if (val_geoidal_units(serialData.token) == true)            {strcpy(gnggaData.geoidal_units, serialData.token);         gnggaData.check_data++;} else {gnggaData.bad_geoidal_units_i++;}}
    else if (serialData.iter_token ==13) {if (val_differential_delay(serialData.token) == true)       {strcpy(gnggaData.differential_delay, serialData.token);    gnggaData.check_data++;} else {gnggaData.bad_differential_delay_i++;}}
    else if (serialData.iter_token ==14) {
      strcpy(gnggaData.temporary_data, strtok(serialData.token, "*"));
      if (val_basestation_id(gnggaData.temporary_data) == true)                                       {strcpy(gnggaData.id, gnggaData.temporary_data);            gnggaData.check_data++;} else {gnggaData.bad_id_i++;}
      serialData.token = strtok(NULL, "*");
      strcpy(gnggaData.temporary_data_1, strtok(serialData.token, "*"));
      if (val_checksum(gnggaData.temporary_data_1) == true)                                           {strcpy(gnggaData.check_sum, gnggaData.temporary_data_1);   gnggaData.check_data++;} else {gnggaData.bad_check_sum_i++;}}
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
    Serial.println("[gnggaData.geoidal] "                 + String(gnggaData.geoidal));
    Serial.println("[gnggaData.geoidal_units] "           + String(gnggaData.geoidal_units));
    Serial.println("[gnggaData.differential_delay] "      + String(gnggaData.differential_delay));
    Serial.println("[gnggaData.id] "                      + String(gnggaData.id));
    Serial.println("[gnggaData.check_sum] "               + String(gnggaData.check_sum));
    Serial.println("[gnggaData.check_data] "              + String(gnggaData.check_data));
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   GNRMC DATA

struct GNRMCStruct {
  char tag[56];                                                                              // <0> Log header
  char utc_time[56];                       unsigned long bad_utc_time_i;                     // <1> UTC time, the format is hhmmss.sss
  char positioning_status[56];             unsigned long bad_positioning_status_i;           // <2> Positioning status, A=effective positioning, V=invalid positioning
  char latitude[56];                       unsigned long bad_latitude_i;                     // <3> Latitude, the format is  ddmm.mmmmmmm
  char latitude_hemisphere[56];            unsigned long bad_latitude_hemisphere_i;          // <4> Latitude hemisphere, N or S (north latitude or south latitude)
  char longitude[56];                      unsigned long bad_longitude_i;                    // <5> Longitude, the format is dddmm.mmmmmmm
  char longitude_hemisphere[56];           unsigned long bad_longitude_hemisphere_i;         // <6> Longitude hemisphere, E or W (east longitude or west longitude)
  char ground_speed[56];                   unsigned long bad_ground_speed_i;                 // <7> Ground speed
  char ground_heading[56];                 unsigned long bad_ground_heading_i;               // <8> Ground heading (take true north as the reference datum)
  char utc_date[56];                       unsigned long bad_utc_date_i;                     // <9> UTC date, the format is ddmmyy (day, month, year)
  char installation_angle[56];             unsigned long bad_installation_angle_i;           // <10> Magnetic declination (000.0~180.0 degrees)
  char installation_angle_direction[56];   unsigned long bad_installation_angle_direction_i; // <11> Magnetic declination direction, E (east) or W (west)
  char mode_indication[56];                unsigned long bad_mode_indication_i;              // <12> Mode indication (A=autonomous positioning, D=differential E=estimation, N=invalid data) */
  char check_sum[56];                      unsigned long bad_check_sum_i;                    // <13> XOR check value of all bytes starting from $ to *
  char temporary_data[56];
  char temporary_data_1[56];
  int check_data = 0; // should result in 14
};
GNRMCStruct gnrmcData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        GNRMC

void GNRMC() {
  
  gnrmcData.check_data = 0;
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
  memset(gnrmcData.installation_angle, 0, 56);
  memset(gnrmcData.installation_angle_direction, 0, 56);
  memset(gnrmcData.mode_indication, 0, 56);
  memset(gnrmcData.check_sum, 0, 56);
  serialData.iter_token = 0;
  serialData.token = strtok(serialData.BUFFER, ",");
  while( serialData.token != NULL ) {
    if     (serialData.iter_token == 0)                                                                   {strcpy(gnrmcData.tag, "GNRMC");                                   gnrmcData.check_data++;}
    else if (serialData.iter_token ==1)  {if (val_utc_time(serialData.token) == true)                     {strcpy(gnrmcData.utc_time, serialData.token);                     gnrmcData.check_data++;} else {gnrmcData.bad_utc_time_i++;}}
    else if (serialData.iter_token ==2)  {if (val_positioning_status_gnrmc(serialData.token) == true)     {strcpy(gnrmcData.positioning_status, serialData.token);           gnrmcData.check_data++;} else {gnrmcData.bad_positioning_status_i++;}}
    else if (serialData.iter_token ==3)  {if (val_latitude(serialData.token) == true)                     {strcpy(gnrmcData.latitude, serialData.token);                     gnrmcData.check_data++;} else {gnrmcData.bad_latitude_i++;}}
    else if (serialData.iter_token ==4)  {if (val_latitude_H(serialData.token) == true)                   {strcpy(gnrmcData.latitude_hemisphere, serialData.token);          gnrmcData.check_data++;} else {gnrmcData.bad_latitude_hemisphere_i++;}}
    else if (serialData.iter_token ==5)  {if (val_longitude(serialData.token) == true)                    {strcpy(gnrmcData.longitude, serialData.token);                    gnrmcData.check_data++;} else {gnrmcData.bad_longitude_i++;}}
    else if (serialData.iter_token ==6)  {if (val_longitude_H(serialData.token) == true)                  {strcpy(gnrmcData.longitude_hemisphere, serialData.token);         gnrmcData.check_data++;} else {gnrmcData.bad_longitude_hemisphere_i++;}}
    else if (serialData.iter_token ==7)  {if (val_ground_speed(serialData.token) == true)                 {strcpy(gnrmcData.ground_speed, serialData.token);                 gnrmcData.check_data++;} else {gnrmcData.bad_ground_speed_i++;}}
    else if (serialData.iter_token ==8)  {if (val_ground_heading(serialData.token) == true)               {strcpy(gnrmcData.ground_heading, serialData.token);               gnrmcData.check_data++;} else {gnrmcData.bad_ground_heading_i++;}}
    else if (serialData.iter_token ==9)  {if (val_utc_date(serialData.token) == true)                     {strcpy(gnrmcData.utc_date, serialData.token);                     gnrmcData.check_data++;} else {gnrmcData.bad_utc_date_i++;}}
    else if (serialData.iter_token ==10) {if (val_installation_angle(serialData.token) == true)           {strcpy(gnrmcData.installation_angle, serialData.token);           gnrmcData.check_data++;} else {gnrmcData.bad_installation_angle_i++;}}
    else if (serialData.iter_token ==11) {if (val_installation_angle_direction(serialData.token) == true) {strcpy(gnrmcData.installation_angle_direction, serialData.token); gnrmcData.check_data++;} else {gnrmcData.bad_installation_angle_direction_i++;}}
    else if (serialData.iter_token ==12) {
      strcpy(gnrmcData.temporary_data, strtok(serialData.token, "*"));
      if (val_mode_indication(gnrmcData.temporary_data) == true)                                          {strcpy(gnrmcData.mode_indication, gnrmcData.temporary_data);      gnrmcData.check_data++;} else {gnrmcData.bad_mode_indication_i++;}
      serialData.token = strtok(NULL, "*");
      strcpy(gnrmcData.temporary_data_1, strtok(serialData.token, "*"));
      if (val_checksum(gnrmcData.temporary_data_1) == true)                                               {strcpy(gnrmcData.check_sum, gnrmcData.temporary_data_1);          gnrmcData.check_data++;} else {gnrmcData.bad_check_sum_i++;}}

    serialData.token = strtok(NULL, ",");
    serialData.iter_token++;
  }
  if (sysDebugData.gnrmc_sentence == true) {
    Serial.println("[gnrmcData.tag] "                          + String(gnrmcData.tag));
    Serial.println("[gnrmcData.utc_time] "                     + String(gnrmcData.utc_time));
    Serial.println("[gnrmcData.positioning_status] "           + String(gnrmcData.positioning_status));
    Serial.println("[gnrmcData.latitude] "                     + String(gnrmcData.latitude));
    Serial.println("[gnrmcData.latitude_hemisphere] "          + String(gnrmcData.latitude_hemisphere));
    Serial.println("[gnrmcData.longitude] "                    + String(gnrmcData.longitude));
    Serial.println("[gnrmcData.longitude_hemisphere] "         + String(gnrmcData.longitude_hemisphere));
    Serial.println("[gnrmcData.ground_speed] "                 + String(gnrmcData.ground_speed));
    Serial.println("[gnrmcData.ground_heading] "               + String(gnrmcData.ground_heading));
    Serial.println("[gnrmcData.utc_date] "                     + String(gnrmcData.utc_date));
    Serial.println("[gnrmcData.installation_angle] "           + String(gnrmcData.installation_angle));
    Serial.println("[gnrmcData.installation_angle_direction] " + String(gnrmcData.installation_angle_direction));
    Serial.println("[gnrmcData.mode_indication] "              + String(gnrmcData.mode_indication));
    Serial.println("[gnrmcData.check_sum] "                    + String(gnrmcData.check_sum));
    Serial.println("[gnrmcData.check_data] "                   + String(gnrmcData.check_data));
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   GPATT DATA

struct GPATTStruct {
  char tag[56];                                                    // <0> Log header
  char pitch[56];            unsigned long bad_pitch_i;            // <1> pitch angle
  char angle_channel_0[56];  unsigned long bad_angle_channel_0_i;  // <2> P
  char roll[56];             unsigned long bad_roll_i;             // <3> Roll angle
  char angle_channel_1[56];  unsigned long bad_angle_channel_1_i;  // <4> R
  char yaw[56];              unsigned long bad_yaw_i;              // <5> Yaw angle
  char angle_channel_2[56];  unsigned long bad_angle_channel_2_i;  // <6> Y
  char software_version[56]; unsigned long bad_software_version_i; // <7> software verion
  char version_channel[56];  unsigned long bad_version_channel_i;  // <8> S
  char product_id[56];       unsigned long bad_product_id_i;       // <9> Product ID: 96 bit unique ID
  char id_channel[56];       unsigned long bad_id_channel_i;       // <10> ID 
  char ins[56];              unsigned long bad_ins_i;              // <11> INS Default open inertial navigation system
  char ins_channel[56];      unsigned long bad_ins_channel_i;      // <12> whether inertial navigation open
  char hardware_version[56]; unsigned long bad_hardware_version_i; // <13> Named after master chip
  char run_state_flag[56];   unsigned long bad_run_state_flag_i;   // <14> Algorithm status flag: 1->3
  char mis_angle_num[56];    unsigned long bad_mis_angle_num_i;    // <15> number of Installation
  char custom_logo_0[56];    unsigned long bad_custom_logo_0_i;    // <16>
  char custom_logo_1[56];    unsigned long bad_custom_logo_1_i;    // <17>
  char custom_logo_2[56];    unsigned long bad_custom_logo_2_i;    // <18>
  char static_flag[56];      unsigned long bad_static_flag_i;      // <19> 1:Static 0：dynamic
  char user_code[56];        unsigned long bad_user_code_i;        // <20> 1：Normal user X：Customuser
  char gst_data[56];         unsigned long bad_gst_data_i;         // <21> User satellite accuracy
  char line_flag[56];        unsigned long bad_line_flag_i;        // <22> 1：straight driving，0：curve driving
  char custom_logo_3[56];    unsigned long bad_custom_logo_3_i;    // <23>
  char mis_att_flag[56];     unsigned long bad_mis_att_flag_i;     // <24> 
  char imu_kind[56];         unsigned long bad_imu_kind_i;         // <25> Sensor Type: 0->BIM055; 1->BMI160; 2->LSM6DS3TR-C; 3->LSM6DSOW 4->ICM-40607; 5->ICM-40608 6->ICM-42670; 7->LSM6DSR
  char ubi_car_kind[56];     unsigned long bad_ubi_car_kind_i;     // <26> 1: small car, 2: big car
  char mileage[56];          unsigned long bad_mileage_i;          // <27> kilometers: max 9999 kilometers
  char custom_logo_4[56];    unsigned long bad_custom_logo_4_i;    // <28>
  char custom_logo_5[56];    unsigned long bad_custom_logo_5_i;    // <29>
  char run_inetial_flag[56]; unsigned long bad_run_inetial_flag_i; // <30> 1->4
  char custom_logo_6[56];    unsigned long bad_custom_logo_6_i;    // <31>
  char custom_logo_7[56];    unsigned long bad_custom_logo_7_i;    // <32>
  char custom_logo_8[56];    unsigned long bad_custom_logo_8_i;    // <33>
  char custom_logo_9[56];    unsigned long bad_custom_logo_9_i;    // <34>
  char speed_enable[56];     unsigned long bad_speed_enable_i;     // <35> 
  char custom_logo_10[56];   unsigned long bad_custom_logo_10_i;   // <36>
  char custom_logo_11[56];   unsigned long bad_custom_logo_11_i;   // <37>
  char speed_num[56];        unsigned long bad_speed_num_i;        // <38> 1：fixed setting，0：Self adaptive installation
  char scalable[56];         unsigned long bad_scalable_i;         // <39> 
  char check_sum[56];        unsigned long bad_check_sum_i;        // <40> XOR check value of all bytes starting from $ to *
  char temporary_data[56];
  char temporary_data_1[56];
  int check_data = 0;        // should result in 40
};
GPATTStruct gpattData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        GPATT

void GPATT() {
  gpattData.check_data = 0;
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
  memset(gpattData.custom_logo_0, 0, 56);
  memset(gpattData.custom_logo_1, 0, 56);
  memset(gpattData.custom_logo_2, 0, 56);
  memset(gpattData.static_flag, 0, 56);
  memset(gpattData.user_code, 0, 56);
  memset(gpattData.gst_data, 0, 56);
  memset(gpattData.line_flag, 0, 56);
  memset(gpattData.custom_logo_3, 0, 56);
  memset(gpattData.mis_att_flag, 0, 56);
  memset(gpattData.imu_kind, 0, 56);
  memset(gpattData.ubi_car_kind, 0, 56);
  memset(gpattData.mileage, 0, 56);
  memset(gpattData.custom_logo_4, 0, 56);
  memset(gpattData.custom_logo_5, 0, 56);
  memset(gpattData.run_inetial_flag, 0, 56);
  memset(gpattData.custom_logo_6, 0, 56);
  memset(gpattData.custom_logo_7, 0, 56);
  memset(gpattData.custom_logo_8, 0, 56);
  memset(gpattData.speed_enable, 0, 56);
  memset(gpattData.custom_logo_4, 0, 56);
  memset(gpattData.custom_logo_5, 0, 56);
  memset(gpattData.speed_num, 0, 56);
  memset(gpattData.scalable, 0, 56);
  memset(gpattData.check_sum, 0, 56);
  serialData.iter_token = 0;
  serialData.token = strtok(serialData.BUFFER, ",");
  while( serialData.token != NULL ) {
    if      (serialData.iter_token == 0)                                                             {strcpy(gpattData.tag, "GPATT");                       gpattData.check_data++;}
    else if (serialData.iter_token == 1) {if (val_pitch_gpatt(serialData.token) == true)             {strcpy(gpattData.pitch, serialData.token);            gpattData.check_data++;} else {gpattData.bad_pitch_i++;}}
    else if (serialData.iter_token == 2) {if (val_angle_channle_p_gpatt(serialData.token) == true)   {strcpy(gpattData.angle_channel_0, serialData.token);  gpattData.check_data++;} else {gpattData.bad_angle_channel_0_i++;}}
    else if (serialData.iter_token == 3) {if (val_roll_gpatt(serialData.token) == true)              {strcpy(gpattData.roll, serialData.token);             gpattData.check_data++;} else {gpattData.bad_roll_i++;}}
    else if (serialData.iter_token == 4) {if (val_angle_channle_r_gpatt(serialData.token) == true)   {strcpy(gpattData.angle_channel_1, serialData.token);  gpattData.check_data++;} else {gpattData.bad_angle_channel_1_i++;}}
    else if (serialData.iter_token == 5) {if (val_yaw_gpatt(serialData.token) == true)               {strcpy(gpattData.yaw, serialData.token);              gpattData.check_data++;} else {gpattData.bad_yaw_i++;}}
    else if (serialData.iter_token == 6) {if (val_angle_channle_y_gpatt(serialData.token) == true)   {strcpy(gpattData.angle_channel_2, serialData.token);  gpattData.check_data++;} else {gpattData.bad_angle_channel_2_i++;}}
    else if (serialData.iter_token == 7) {if (val_software_version_gpatt(serialData.token) == true)  {strcpy(gpattData.software_version, serialData.token); gpattData.check_data++;} else {gpattData.bad_software_version_i++;}}
    else if (serialData.iter_token == 8) {if (val_version_channel_s_gpatt(serialData.token) == true) {strcpy(gpattData.version_channel, serialData.token);  gpattData.check_data++;} else {gpattData.bad_version_channel_i++;}}
    else if (serialData.iter_token == 9) {if (val_product_id_gpatt(serialData.token) == true)        {strcpy(gpattData.product_id, serialData.token);       gpattData.check_data++;} else {gpattData.bad_product_id_i++;}}
    else if (serialData.iter_token == 10) {if (val_id_channel_gpatt(serialData.token) == true)       {strcpy(gpattData.id_channel, serialData.token);       gpattData.check_data++;} else {gpattData.bad_id_channel_i++;}}
    else if (serialData.iter_token == 11) {if (val_ins_gpatt(serialData.token) == true)              {strcpy(gpattData.ins, serialData.token);              gpattData.check_data++;} else {gpattData.bad_ins_i++;}}
    else if (serialData.iter_token == 12) {if (val_ins_channel_gpatt(serialData.token) == true)      {strcpy(gpattData.ins_channel, serialData.token);      gpattData.check_data++;} else {gpattData.bad_ins_channel_i++;}}
    else if (serialData.iter_token == 13) {if (val_hardware_version_gpatt(serialData.token) == true) {strcpy(gpattData.hardware_version, serialData.token); gpattData.check_data++;} else {gpattData.bad_hardware_version_i++;}}
    else if (serialData.iter_token == 14) {if (val_run_state_flag_gpatt(serialData.token) == true)   {strcpy(gpattData.run_state_flag, serialData.token);   gpattData.check_data++;} else {gpattData.bad_run_state_flag_i++;}}
    else if (serialData.iter_token == 15) {if (val_mis_angle_num_gpatt(serialData.token) == true)    {strcpy(gpattData.mis_angle_num, serialData.token);    gpattData.check_data++;} else {gpattData.bad_mis_angle_num_i++;}}
    else if (serialData.iter_token == 16) {if (val_custom_flag(serialData.token) == true)            {strcpy(gpattData.custom_logo_0, serialData.token);    gpattData.check_data++;} else {gpattData.bad_custom_logo_0_i++;}}
    else if (serialData.iter_token == 17) {if (val_custom_flag(serialData.token) == true)            {strcpy(gpattData.custom_logo_1, serialData.token);    gpattData.check_data++;} else {gpattData.bad_custom_logo_1_i++;}}
    else if (serialData.iter_token == 18) {if (val_custom_flag(serialData.token) == true)            {strcpy(gpattData.custom_logo_2, serialData.token);    gpattData.check_data++;} else {gpattData.bad_custom_logo_2_i++;}}
    else if (serialData.iter_token == 19) {if (val_static_flag_gpatt(serialData.token) == true)      {strcpy(gpattData.static_flag, serialData.token);      gpattData.check_data++;} else {gpattData.bad_static_flag_i++;}}
    else if (serialData.iter_token == 20) {if (val_user_code_gpatt(serialData.token) == true)        {strcpy(gpattData.user_code, serialData.token);        gpattData.check_data++;} else {gpattData.bad_user_code_i++;}}
    else if (serialData.iter_token == 21) {if (val_gst_data_gpatt(serialData.token) == true)         {strcpy(gpattData.gst_data, serialData.token);         gpattData.check_data++;} else {gpattData.bad_gst_data_i++;}}
    else if (serialData.iter_token == 22) {if (val_line_flag_gpatt(serialData.token) == true)        {strcpy(gpattData.line_flag, serialData.token);        gpattData.check_data++;} else {gpattData.bad_line_flag_i++;}}
    else if (serialData.iter_token == 23) {if (val_custom_flag(serialData.token) == true)            {strcpy(gpattData.custom_logo_3, serialData.token);    gpattData.check_data++;} else {gpattData.bad_custom_logo_3_i++;}}
    else if (serialData.iter_token == 24) {if (val_mis_att_flag_gpatt(serialData.token) == true)     {strcpy(gpattData.mis_att_flag, serialData.token);     gpattData.check_data++;} else {gpattData.bad_mis_att_flag_i++;}}
    else if (serialData.iter_token == 25) {if (val_imu_kind_gpatt(serialData.token) == true)         {strcpy(gpattData.imu_kind, serialData.token);         gpattData.check_data++;} else {gpattData.bad_imu_kind_i++;}}
    else if (serialData.iter_token == 26) {if (val_ubi_car_kind_gpatt(serialData.token) == true)     {strcpy(gpattData.ubi_car_kind, serialData.token);     gpattData.check_data++;} else {gpattData.bad_ubi_car_kind_i++;}}
    else if (serialData.iter_token == 27) {if (val_mileage_gpatt(serialData.token) == true)          {strcpy(gpattData.mileage, serialData.token);          gpattData.check_data++;} else {gpattData.bad_mileage_i++;}}
    else if (serialData.iter_token == 28) {if (val_custom_flag(serialData.token) == true)            {strcpy(gpattData.custom_logo_4, serialData.token);    gpattData.check_data++;} else {gpattData.bad_custom_logo_4_i++;}}
    else if (serialData.iter_token == 29) {if (val_custom_flag(serialData.token) == true)            {strcpy(gpattData.custom_logo_5, serialData.token);    gpattData.check_data++;} else {gpattData.bad_custom_logo_5_i++;}}
    else if (serialData.iter_token == 30) {if (val_run_inetial_flag_gpatt(serialData.token) == true) {strcpy(gpattData.run_inetial_flag, serialData.token); gpattData.check_data++;} else {gpattData.bad_run_inetial_flag_i++;}}
    else if (serialData.iter_token == 31) {if (val_custom_flag(serialData.token) == true)            {strcpy(gpattData.custom_logo_6, serialData.token);    gpattData.check_data++;} else {gpattData.bad_custom_logo_6_i++;}}
    else if (serialData.iter_token == 32) {if (val_custom_flag(serialData.token) == true)            {strcpy(gpattData.custom_logo_7, serialData.token);    gpattData.check_data++;} else {gpattData.bad_custom_logo_7_i++;}}
    else if (serialData.iter_token == 33) {if (val_custom_flag(serialData.token) == true)            {strcpy(gpattData.custom_logo_8, serialData.token);    gpattData.check_data++;} else {gpattData.bad_custom_logo_8_i++;}}
    else if (serialData.iter_token == 34) {if (val_custom_flag(serialData.token) == true)            {strcpy(gpattData.custom_logo_9, serialData.token);    gpattData.check_data++;} else {gpattData.bad_custom_logo_9_i++;}}
    else if (serialData.iter_token == 35) {if (val_speed_enable_gpatt(serialData.token) == true)     {strcpy(gpattData.speed_enable, serialData.token);     gpattData.check_data++;} else {gpattData.bad_speed_enable_i++;}}
    else if (serialData.iter_token == 36) {if (val_custom_flag(serialData.token) == true)            {strcpy(gpattData.custom_logo_10, serialData.token);   gpattData.check_data++;} else {gpattData.bad_custom_logo_10_i++;}}
    else if (serialData.iter_token == 37) {if (val_custom_flag(serialData.token) == true)            {strcpy(gpattData.custom_logo_11, serialData.token);   gpattData.check_data++;} else {gpattData.bad_custom_logo_11_i++;}}
    else if (serialData.iter_token == 38) {if (val_speed_num_gpatt(serialData.token) == true)        {strcpy(gpattData.speed_num, serialData.token);        gpattData.check_data++;} else {gpattData.bad_speed_num_i++;}}
    else if (serialData.iter_token == 39) {
      strcpy(gpattData.temporary_data, strtok(serialData.token, "*"));
      if (val_scalable(gpattData.temporary_data) == true)                                            {strcpy(gpattData.scalable, gpattData.temporary_data); gpattData.check_data++;} else {gpattData.bad_scalable_i++;}
      serialData.token = strtok(NULL, "*");
      strcpy(gpattData.temporary_data_1, strtok(serialData.token, "*"));
      if (val_checksum(gpattData.temporary_data_1) == true)                                          {strcpy(gpattData.check_sum, gpattData.temporary_data_1); gpattData.check_data++;} else {gpattData.bad_check_sum_i++;}}
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
    Serial.println("[gpattData.custom_logo_0] "    + String(gpattData.custom_logo_0));
    Serial.println("[gpattData.custom_logo_1] "    + String(gpattData.custom_logo_1));
    Serial.println("[gpattData.custom_logo_2] "    + String(gpattData.custom_logo_2));
    Serial.println("[gpattData.static_flag] "      + String(gpattData.static_flag));
    Serial.println("[gpattData.user_code] "        + String(gpattData.user_code));
    Serial.println("[gpattData.gst_data] "         + String(gpattData.gst_data));
    Serial.println("[gpattData.line_flag] "        + String(gpattData.line_flag));
    Serial.println("[gpattData.custom_logo_3] "    + String(gpattData.custom_logo_3));
    Serial.println("[gpattData.imu_kind] "         + String(gpattData.imu_kind));
    Serial.println("[gpattData.ubi_car_kind] "     + String(gpattData.ubi_car_kind));
    Serial.println("[gpattData.mileage] "          + String(gpattData.mileage));
    Serial.println("[gpattData.custom_logo_4] "    + String(gpattData.custom_logo_4));
    Serial.println("[gpattData.custom_logo_5] "    + String(gpattData.custom_logo_5));
    Serial.println("[gpattData.run_inetial_flag] " + String(gpattData.run_inetial_flag));
    Serial.println("[gpattData.custom_logo_6] "    + String(gpattData.custom_logo_6));
    Serial.println("[gpattData.custom_logo_7] "    + String(gpattData.custom_logo_7));
    Serial.println("[gpattData.custom_logo_8] "    + String(gpattData.custom_logo_8));
    Serial.println("[gpattData.custom_logo_9] "    + String(gpattData.custom_logo_9));
    Serial.println("[gpattData.speed_enable] "     + String(gpattData.speed_enable));
    Serial.println("[gpattData.custom_logo_10] "   + String(gpattData.custom_logo_10));
    Serial.println("[gpattData.custom_logo_11] "   + String(gpattData.custom_logo_11));
    Serial.println("[gpattData.speed_num] "        + String(gpattData.speed_num));
    Serial.println("[gpattData.scalable] "         + String(gpattData.scalable)); // intentionally unpopulated
    Serial.println("[gpattData.check_sum] "        + String(gpattData.check_sum));
    Serial.println("[gpattData.check_data] "        + String(gpattData.check_data));
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
  char tag[56];                                                                              // <0> log header
  char utc_time[56];                         unsigned long bad_utc_time_i;                   // <1> utc time
  char speed[56];                            unsigned long bad_speed_i;                      // <2> ground speed: knots
  char status[56];                           unsigned long bad_status_i;                     // <3> 0=invalid data, 1=converging, 2=valid data
  char acceleration_delimiter[56];           unsigned long bad_acceleration_delimiter_i;     // <4> represents acceleration
  char acc_X[56];                            unsigned long bad_acc_X_i;                      // <5> x-axis acceleration
  char acc_Y[56];                            unsigned long bad_acc_Y_i;                      // <6> y-axis acceleration
  char acc_Z[56];                            unsigned long bad_acc_Z_i;                      // <7> z-axis acceleration
  char angular_velocity_delimiter[56] = "0"; unsigned long bad_angular_velocity_delimiter_i; // <8> represents angular velocity
  char gyro_X[56];                           unsigned long bad_gyro_X_i;                     // <9> x-axis angular velocity
  char gyro_Y[56];                           unsigned long bad_gyro_Y_i;                     // <10> y-axis angular velocity
  char gyro_Z[56];                           unsigned long bad_gyro_Z_i;                     // <11> z-axis angular velocity
  char status_delimiter[56];                 unsigned long bad_status_delimiter_i;           // <12> represents status
  char ubi_state_flag[56];                   unsigned long bad_ubi_state_flag_i;             // <13> 0=smooth driving, 1=unsteady driving
  char ubi_state_kind[56];                   unsigned long bad_ubi_state_kind_i;             // <14> status tyoe: see ubi_state_kind table
  char ubi_state_value[56];                  unsigned long bad_ubi_state_value_i;            // <15> status threshold: see ubi_state_kind table
  char check_sum[56];                        unsigned long bad_check_sum_i;                  // <16> XOR check value of all bytes starting from $ to *
  char temporary_data[56];
  char temporary_data_1[56];
  int check_data = 0; // should result in 40
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
    if      (serialData.iter_token == 0)                                                                 {strcpy(speedData.tag, "SPEED");                                 speedData.check_data++;}
    else if (serialData.iter_token == 1)  {if (val_utc_time(serialData.token) == true)                   {strcpy(speedData.utc_time, serialData.token);                   speedData.check_data++;} else {speedData.bad_utc_time_i++;}}
    else if (serialData.iter_token == 2)  {if (val_ground_speed(serialData.token) == true)               {strcpy(speedData.speed, serialData.token);                      speedData.check_data++;} else {speedData.bad_speed_i++;}}
    else if (serialData.iter_token == 3)  {if (val_speed_status(serialData.token) == true)               {strcpy(speedData.status, serialData.token);                     speedData.check_data++;} else {speedData.bad_status_i++;}}
    else if (serialData.iter_token == 4)  {if (val_accelleration_delimiter(serialData.token) == true)    {strcpy(speedData.acceleration_delimiter, serialData.token);     speedData.check_data++;} else {speedData.bad_acceleration_delimiter_i++;}}
    else if (serialData.iter_token == 5)  {if (val_axis_accelleration(serialData.token) == true)         {strcpy(speedData.acc_X, serialData.token);                      speedData.check_data++;} else {speedData.bad_acc_X_i++;}}
    else if (serialData.iter_token == 6)  {if (val_axis_accelleration(serialData.token) == true)         {strcpy(speedData.acc_Y, serialData.token);                      speedData.check_data++;} else {speedData.bad_acc_Y_i++;}}
    else if (serialData.iter_token == 7)  {if (val_axis_accelleration(serialData.token) == true)         {strcpy(speedData.acc_Z, serialData.token);                      speedData.check_data++;} else {speedData.bad_acc_Z_i++;}}
    else if (serialData.iter_token == 8)  {if (val_angular_velocity_delimiter(serialData.token) == true) {strcpy(speedData.angular_velocity_delimiter, serialData.token); speedData.check_data++;} else {speedData.bad_angular_velocity_delimiter_i++;}}
    else if (serialData.iter_token == 9)  {if (val_gyro_angular_velocity(serialData.token) == true)      {strcpy(speedData.gyro_X, serialData.token);                     speedData.check_data++;} else {speedData.bad_gyro_X_i++;}}
    else if (serialData.iter_token == 10) {if (val_gyro_angular_velocity(serialData.token) == true)      {strcpy(speedData.gyro_Y, serialData.token);                     speedData.check_data++;} else {speedData.bad_gyro_Y_i++;}}
    else if (serialData.iter_token == 11) {if (val_gyro_angular_velocity(serialData.token) == true)      {strcpy(speedData.gyro_Z, serialData.token);                     speedData.check_data++;} else {speedData.bad_gyro_Z_i++;}}
    else if (serialData.iter_token == 12) {if (val_status_delimiter(serialData.token) == true)           {strcpy(speedData.status_delimiter, serialData.token);           speedData.check_data++;} else {speedData.bad_status_delimiter_i++;}}
    else if (serialData.iter_token == 13) {if (val_ubi_state_flag(serialData.token) == true)             {strcpy(speedData.ubi_state_flag, serialData.token);             speedData.check_data++;} else {speedData.bad_ubi_state_flag_i++;}}
    else if (serialData.iter_token == 14) {if (val_ubi_state_kind_flag(serialData.token) == true)        {strcpy(speedData.ubi_state_kind, serialData.token);             speedData.check_data++;} else {speedData.bad_ubi_state_kind_i++;}}
    else if (serialData.iter_token == 15) {
      strcpy(speedData.temporary_data, strtok(serialData.token, "*"));
      if (val_scalable(speedData.temporary_data) == true)                                                {strcpy(speedData.ubi_state_value, speedData.temporary_data);    speedData.check_data++;} else {speedData.bad_ubi_state_value_i++;}
      serialData.token = strtok(NULL, "*");
      strcpy(speedData.temporary_data_1, strtok(serialData.token, "*"));
      if (val_checksum(speedData.temporary_data_1) == true)                                              {strcpy(speedData.check_sum, speedData.temporary_data_1);        speedData.check_data++;} else {speedData.bad_check_sum_i++;}}
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
    Serial.println("[speedData.check_data] "                 + String(speedData.check_data));
  }
}


// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   ERROR DATA

struct ERRORStruct {
  char tag[56];                                             // <0> Log header
  char utc[56];            unsigned long bad_utc_i;         // <1> utc time
  char code_flag[56];      unsigned long bad_code_flag_i;   // <2> encryption chip: 1=problem, 0=normal
  char gset_flag[56];      unsigned long bad_gset_flag_i;   // <3> positioning chip: 1=problem, 0=normal
  char sset_flag[56];      unsigned long bad_sset_flag_i;   // <4> sensor chip: 1=problem, 0=normal
  char customize_0[56];    unsigned long bad_customize_0_i; // <5> customize 0-20
  char customize_1[56];    unsigned long bad_customize_1_i; // <6> customize float
  char check_sum[56];      unsigned long bad_check_sum_i;   // <7> XOR check value of all bytes starting from $ to *
  char temporary_data[56];
  char temporary_data_1[56];
  int check_data = 0; // should result in 8
};
ERRORStruct errorData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        ERROR

void ERROR() {
  errorData.check_data = 0;
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
    if      (serialData.iter_token == 0)                                                 {strcpy(errorData.tag, "ERROR");                  errorData.check_data++;}
    else if (serialData.iter_token == 1) {if (val_utc_time(serialData.token) == true)    {strcpy(errorData.utc, serialData.token);         errorData.check_data++;} else {errorData.bad_utc_i++;}}
    else if (serialData.iter_token == 2) {if (val_code_flag(serialData.token) == true)   {strcpy(errorData.code_flag, serialData.token);   errorData.check_data++;} else {errorData.bad_code_flag_i++;}}
    else if (serialData.iter_token == 3) {if (val_gset_flag(serialData.token) == true)   {strcpy(errorData.gset_flag, serialData.token);   errorData.check_data++;} else {errorData.bad_gset_flag_i++;}}
    else if (serialData.iter_token == 4) {if (val_sset_flag(serialData.token) == true)   {strcpy(errorData.sset_flag, serialData.token);   errorData.check_data++;} else {errorData.bad_sset_flag_i++;}}
    else if (serialData.iter_token == 5) {if (val_custom_flag(serialData.token) == true) {strcpy(errorData.customize_0, serialData.token); errorData.check_data++;} else {errorData.bad_customize_0_i++;}}
    else if (serialData.iter_token == 6) {
      strcpy(errorData.temporary_data, strtok(serialData.token, "*"));
      if (val_scalable(errorData.temporary_data) == true)                                {strcpy(errorData.customize_1, errorData.temporary_data); errorData.check_data++;} else {errorData.bad_customize_1_i++;}
      serialData.token = strtok(NULL, "*");
      strcpy(errorData.temporary_data_1, strtok(serialData.token, "*"));
      if (val_checksum(errorData.temporary_data_1) == true)                              {strcpy(errorData.check_sum, errorData.temporary_data_1); errorData.check_data++;} else {errorData.bad_check_sum_i++;}}
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
    Serial.println("[errorData.check_data] "  + String(errorData.check_data));
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   DEBUG DATA

struct DEBUGStruct {
  char tag[56];            // <0> log header
  char ang_dget_flag[56];  unsigned long bad_ang_dget_flag_i;  // <1> installation azimuth: 1=with azimuth, 0=without azimuth
  char fix_kind_flag[56];  unsigned long bad_fix_kind_flag_i;  // <2> type of installed coordinate system
  char ins_run_flag[56];   unsigned long bad_ins_run_flag_i;   // <3> forced ins: 1=forced, 0=normal
  char fix_roll_flag[56];  unsigned long bad_fix_roll_flag_i;  // <4> installation roll angle
  char fix_pitch_flag[56]; unsigned long bad_fix_pitch_flag_i; // <5> installation pitch angle
  char ubi_on_flag[56];    unsigned long bad_ubi_on_flag_i;    // <6> 0 to 8
  char ubi_kind_flag[56];  unsigned long bad_ubi_kind_flag_i;  // <7> 0=none, 1=ubi event, 2=ubi alarm
  char ubi_a_set[56];      unsigned long bad_ubi_a_set_i;      // <8> ubi a parameter setting value
  char ubi_b_set[56];      unsigned long bad_ubi_b_set_i;      // <9> ubi b parameter setting value
  char acc_X_data[56];     unsigned long bad_acc_X_data_i;     // <10> vehicle longitudinal acceleration: 0.1m/s2
  char acc_Y_data[56];     unsigned long bad_acc_Y_data_i;     // <11> vehicle lateral acceleration: 0.1m/s2
  char gyro_Z_data[56];    unsigned long bad_gyro_Z_data_i;    // <12> vehicle z axis angular velocity: degrees
  char pitch_angle[56];    unsigned long bad_pitch_angle_i;    // <13> vehicle pitch angle: degrees
  char roll_angle[56];     unsigned long bad_roll_angle_i;     // <14> vehicle roll angle: degrees
  char yaw_angle[56];      unsigned long bad_yaw_angle_i;      // <15> vehicle direction change angle: degrees
  char car_speed[56];      unsigned long bad_car_speed_i;      // <16> vehicle speed: m/s
  char ins_flag[56];       unsigned long bad_ins_flag_i;       // <17> intertial navigation convergence flag
  char ubi_num[56];        unsigned long bad_ubi_num_i;        // <18> serial number
  char ubi_valid[56];      unsigned long bad_ubi_valid_i;      // <19> ubi valid flag: 1=valid, 0=invalid
  char coll_T_data[56];    unsigned long bad_coll_T_data_i;    // <20> collision factor
  char coll_T_heading[56]; unsigned long bad_coll_T_heading_i; // <21> collision direction
  char custom_logo_0[56];  unsigned long bad_custom_logo_0_i;  // <22> 
  char custom_logo_1[56];  unsigned long bad_custom_logo_1_i;  // <23> 
  char custom_logo_2[56];  unsigned long bad_custom_logo_2_i;  // <24> 
  char custom_logo_3[56];  unsigned long bad_custom_logo_3_i;  // <25> 
  char custom_logo_4[56];  unsigned long bad_custom_logo_4_i;  // <26> 
  char custom_logo_5[56];  unsigned long bad_custom_logo_5_i;  // <27> 
  char check_sum[56];      unsigned long bad_check_sum_i; // <28> XOR check value of all bytes starting from $ to *
  char temporary_data[56];
  char temporary_data_1[56];
  int check_data = 0;      // should result in 29
};
DEBUGStruct debugData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        DEBUG

void DEBUG() {
  debugData.check_data = 0;
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
    if      (serialData.iter_token == 0)                                                     {strcpy(debugData.tag, "DEBUG");                     debugData.check_data++;}
    else if (serialData.iter_token == 1)  {if (val_ang_dget_flag(serialData.token) == true)  {strcpy(debugData.ang_dget_flag, serialData.token);  debugData.check_data++;} else {debugData.bad_ang_dget_flag_i++;}}
    else if (serialData.iter_token == 2)  {if (val_fix_kind_flag(serialData.token) == true)  {strcpy(debugData.fix_kind_flag, serialData.token);  debugData.check_data++;} else {debugData.bad_fix_kind_flag_i++;}}
    else if (serialData.iter_token == 3)  {if (val_ins_run_flag(serialData.token) == true)   {strcpy(debugData.ins_run_flag, serialData.token);   debugData.check_data++;} else {debugData.bad_ins_run_flag_i++;}}
    else if (serialData.iter_token == 4)  {if (val_fix_roll_flag(serialData.token) == true)  {strcpy(debugData.fix_roll_flag, serialData.token);  debugData.check_data++;} else {debugData.bad_fix_roll_flag_i++;}}
    else if (serialData.iter_token == 5)  {if (val_fix_pitch_flag(serialData.token) == true) {strcpy(debugData.fix_pitch_flag, serialData.token); debugData.check_data++;} else {debugData.bad_fix_pitch_flag_i++;}}
    else if (serialData.iter_token == 6)  {if (val_ubi_on_flag(serialData.token) == true)    {strcpy(debugData.ubi_on_flag, serialData.token);    debugData.check_data++;} else {debugData.bad_ubi_on_flag_i++;}}
    else if (serialData.iter_token == 7)  {if (val_ubi_kind_flag(serialData.token) == true)  {strcpy(debugData.ubi_kind_flag, serialData.token);  debugData.check_data++;} else {debugData.bad_ubi_kind_flag_i++;}}
    else if (serialData.iter_token == 8)  {if (val_ubi_a_set(serialData.token) == true)      {strcpy(debugData.ubi_a_set, serialData.token);      debugData.check_data++;} else {debugData.bad_ubi_a_set_i++;}}
    else if (serialData.iter_token == 9)  {if (val_ubi_b_set(serialData.token) == true)      {strcpy(debugData.ubi_b_set, serialData.token);      debugData.check_data++;} else {debugData.bad_ubi_b_set_i++;}}
    else if (serialData.iter_token == 10) {if (val_acc_X_data(serialData.token) == true)     {strcpy(debugData.acc_X_data, serialData.token);     debugData.check_data++;} else {debugData.bad_acc_X_data_i++;}}
    else if (serialData.iter_token == 11) {if (val_acc_Y_data(serialData.token) == true)     {strcpy(debugData.acc_Y_data, serialData.token);     debugData.check_data++;} else {debugData.bad_acc_Y_data_i++;}}
    else if (serialData.iter_token == 12) {if (val_gyro_Z_data(serialData.token) == true)    {strcpy(debugData.gyro_Z_data, serialData.token);    debugData.check_data++;} else {debugData.bad_gyro_Z_data_i++;}}
    else if (serialData.iter_token == 13) {if (val_pitch_angle(serialData.token) == true)    {strcpy(debugData.pitch_angle, serialData.token);    debugData.check_data++;} else {debugData.bad_pitch_angle_i++;}}
    else if (serialData.iter_token == 14) {if (val_roll_angle(serialData.token) == true)     {strcpy(debugData.roll_angle, serialData.token);     debugData.check_data++;} else {debugData.bad_roll_angle_i++;}}
    else if (serialData.iter_token == 15) {if (val_yaw_angle(serialData.token) == true)      {strcpy(debugData.yaw_angle, serialData.token);      debugData.check_data++;} else {debugData.bad_yaw_angle_i++;}}
    else if (serialData.iter_token == 16) {if (val_car_speed(serialData.token) == true)      {strcpy(debugData.car_speed, serialData.token);      debugData.check_data++;} else {debugData.bad_car_speed_i++;}}
    else if (serialData.iter_token == 17) {if (val_ins_flag(serialData.token) == true)       {strcpy(debugData.ins_flag, serialData.token);       debugData.check_data++;} else {debugData.bad_ins_flag_i++;}}
    else if (serialData.iter_token == 18) {if (val_ubi_num(serialData.token) == true)        {strcpy(debugData.ubi_num, serialData.token);        debugData.check_data++;} else {debugData.bad_ubi_num_i++;}}
    else if (serialData.iter_token == 19) {if (val_ubi_valid(serialData.token) == true)      {strcpy(debugData.ubi_valid, serialData.token);      debugData.check_data++;} else {debugData.bad_ubi_valid_i++;}}
    else if (serialData.iter_token == 20) {if (val_coll_T_data(serialData.token) == true)    {strcpy(debugData.coll_T_data, serialData.token);    debugData.check_data++;} else {debugData.bad_coll_T_data_i++;}}
    else if (serialData.iter_token == 21) {if (val_coll_T_heading(serialData.token) == true) {strcpy(debugData.coll_T_heading, serialData.token); debugData.check_data++;} else {debugData.bad_coll_T_heading_i++;}}
    else if (serialData.iter_token == 22) {if (val_custom_flag(serialData.token) == true)    {strcpy(debugData.custom_logo_0, serialData.token);  debugData.check_data++;} else {debugData.bad_custom_logo_0_i++;}}
    else if (serialData.iter_token == 23) {if (val_custom_flag(serialData.token) == true)    {strcpy(debugData.custom_logo_1, serialData.token);  debugData.check_data++;} else {debugData.bad_custom_logo_1_i++;}}
    else if (serialData.iter_token == 24) {if (val_custom_flag(serialData.token) == true)    {strcpy(debugData.custom_logo_2, serialData.token);  debugData.check_data++;} else {debugData.bad_custom_logo_2_i++;}}
    else if (serialData.iter_token == 25) {if (val_custom_flag(serialData.token) == true)    {strcpy(debugData.custom_logo_3, serialData.token);  debugData.check_data++;} else {debugData.bad_custom_logo_3_i++;}}
    else if (serialData.iter_token == 26) {if (val_custom_flag(serialData.token) == true)    {strcpy(debugData.custom_logo_4, serialData.token);  debugData.check_data++;} else {debugData.bad_custom_logo_4_i++;}}
    else if (serialData.iter_token == 27) {
      strcpy(debugData.temporary_data, strtok(serialData.token, "*"));
      if (val_scalable(debugData.temporary_data) == true)                                    {strcpy(debugData.custom_logo_5, debugData.temporary_data); debugData.check_data++;} else {debugData.bad_custom_logo_5_i++;}
      serialData.token = strtok(NULL, "*");
      strcpy(debugData.temporary_data_1, strtok(serialData.token, "*"));
      if (val_checksum(debugData.temporary_data_1) == true)                                  {strcpy(debugData.check_sum, debugData.temporary_data_1); debugData.check_data++;} else {debugData.bad_check_sum_i++;}}
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
    Serial.println("[debugData.check_data] "     + String(debugData.check_data));
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
        Serial.print(""); Serial.println(serialData.BUFFER);
        GNRMC();
      }
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                                    GPATT

    else if (strncmp(serialData.BUFFER, "$GPATT", 6) == 0) {
      if ((serialData.nbytes == 136) || (serialData.nbytes == 189)) {
        Serial.print(""); Serial.println(serialData.BUFFER);
        GPATT();
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

  char speed_num_gpatt_over[56]            = "speed_num_gpatt_over";
  char speed_num_gpatt_under[56]           = "speed_num_gpatt_under[";
  char speed_num_gpatt_equal[56]           = "speed_num_gpatt_equal";
  char speed_num_gpatt_in_range[56]        = "speed_num_gpatt_in_range";

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
        else if (strcmp(relayData.relays[Ri][Fi], relayData.speed_num_gpatt_over) == 0) {tmp_matrix[0][Fi] = check_over(gpattData.speed_num, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.speed_num_gpatt_under) == 0) {tmp_matrix[0][Fi] = check_under(gpattData.speed_num, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.speed_num_gpatt_equal) == 0) {tmp_matrix[0][Fi] = check_equal(gpattData.speed_num, Ri, Fi);}

        // put true or false in the temporary matrix
        else if (strcmp(relayData.relays[Ri][Fi], relayData.speed_num_gpatt_in_range) == 0) {tmp_matrix[0][Fi] = check_in_range(gpattData.speed_num, Ri, Fi);}


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
  extrapulatedSatData();
  SSD_Display_4();
  SSD_Display_5();
  SSD_Display_6();
  SSD_Display_7();
  systems_Check();

  delay(1);
}

// ----------------------------------------------------------------------------------------------------------------------------


