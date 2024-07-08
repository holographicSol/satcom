/*

                                           SATCOM - Written by Benjamin Jack Cullen.

                                                                                                                       
                                     Receives and Processes Transmissions from Satellites.
                   Possible combinations example: 100 checks ^ 10 functions = 100,000,000,000,000,000,000 combinations.
                                                                              (100 Quintillion)
                                    Calibratable matrix for a wide range of applications.
    A global location Sensor & Time Sensor. For projects requiring location data and or time to be positionally aware and syncronized.
         Tested with WTGPS300P (WITMOTION) Satellite module this project receives data from satellites, sorts that data into
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
        as they would be seen as if the WTGPS300P is plugged in via USB. Dumping more (sentences/information) and dumping in
                                        a standard way and for generic use.

                                        
                                                       Compatibility
                     To Be Potentially Anything And Function As A Hardware Plugin For Other Projects
                                         Headless / Standalone / Serial / Remote


                              Wiring for Optional Multiplexed OLED Displays (SSD1306 Monochromes)
                                       WTGPS300P TX              --> ESP32 io26 as RXD
                                       WTGPS300P VCC             --> ESP32 3.3/5v
                                       TCA9548A i2C Multiplexer  --> ESP32 i2C
                                       x3 SSD1306                --> TCA9548A i2C Multiplexer


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
*/

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    LIBRARIES

#include <stdio.h>
#include <string.h>
#include <Wire.h>
#include <SSD1306Wire.h>   // SSD1306Wire                                https://gitlab.com/alexpr0/ssd1306wire
#include <OLEDDisplayUi.h> // ESP8266 and ESP32 OLED driver for SSD1306  https://github.com/ThingPulse/esp8266-oled-ssd1306
#include <Timezone.h>      // Timezone                                   https://github.com/JChristensen/Timezone
#include <SdFat.h>

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                      DEFINES

#define TCAADDR 0x70

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                       SDCARD
#define SPI_DRIVER_SELECT 2  // Must be set in SdFat/SdFatConfig.h

// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h. 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 0

// Chip select may be constant or RAM variable.
const uint8_t SD_CS_PIN = 5;

// Pin numbers in templates must be constants.
const uint8_t SOFT_MISO_PIN = 19;
const uint8_t SOFT_MOSI_PIN = 23;
const uint8_t SOFT_SCK_PIN = 18;

// SdFat software SPI template
SoftSpiDriver<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> softSpi;

// Speed argument is ignored for software SPI.
#if ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(5, DEDICATED_SPI, SD_SCK_MHZ(0), &softSpi)
#else
#define SD_CONFIG SdSpiConfig(5, SHARED_SPI, SD_SCK_MHZ(0), &softSpi)
#endif

#if SD_FAT_TYPE == 0
SdFat sd;
File file;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 file;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile file;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile file;
#else
#error Invalid SD_FAT_TYPE
#endif

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                       WIRING

SSD1306Wire   display_7(0x3c, SDA, SCL); // let SSD1306Wire wire up our SSD1306 on the i2C bus
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
  bool speed_sentence = false;
  bool error_sentence = false;
  bool debug_sentence = false;
  bool serial_0_sentence = true;
};
sysDebugStruct sysDebugData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                SERIAL 0 DATA

struct Serial0Struct {
  unsigned long nbytes;
  unsigned long iter_token;
  char BUFFER[2048];
  char * token = strtok(BUFFER, ",");
  char data_0[56];
  char data_1[56];
  char data_2[56];
  char data_3[56];
  char data_4[56];
  char data_5[56];
  char data_6[56];
  int check_data_R;
};
Serial0Struct serial0Data;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                SERIAL 1 DATA

struct Serial1Struct {
  unsigned long nbytes;
  unsigned long iter_token;
  char BUFFER[2048];
  char * token = strtok(BUFFER, ",");
  bool rcv = false;
  unsigned long badrcv_i;
};
Serial1Struct serial1Data;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                  SDCARD DATA

struct SDCardStruct {
  unsigned long nbytes;
  unsigned long iter_token;
  char BUFFER[2048];
  String SBUFFER;
  char * token = strtok(BUFFER, ",");
  bool rcv = false;
  unsigned long badrcv_i;
  char data_0[56];
  char data_1[56];
  char data_2[56];
  char data_3[56];
  char data_4[56];
  char data_5[56];
  char data_6[56];
  int check_data_R;
  char file_data[256];
  char delim[2] = ",";
  char tmp[256];
  char tag_0[56] = "r";
  char tag_1[56] = "e";
  File current_file;
};
SDCardStruct sdcardData;

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
//                                                                                                            VALIDATION STRUCT

struct validationStruct {
  bool preliminary_check = true;
  int  valid_i = 0;
  bool valid_b = true;
  char *find_char;
  int  index;
};
validationStruct validData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                          CHECKSUM VALIDATION

int getCheckSum(char *string) {
    int i;
    int XOR;
    int c;
    for (XOR = 0, i = 0; i < strlen(string); i++) {
      c = (unsigned char)string[i];
      if (c == '*') break;
      if (c != '$') XOR ^= c;
    }
    return XOR;
}

uint8_t h2d(char hex) {if(hex > 0x39) hex -= 7; return(hex & 0xf);}

uint8_t h2d2(char h1, char h2) {return (h2d(h1)<<4) | h2d(h2);}

bool validateChecksum(char * buffer) {
  char gotSum[2];
  gotSum[0] = buffer[strlen(buffer) - 3];
  gotSum[1] = buffer[strlen(buffer) - 2];
  uint8_t checksum_of_buffer =  getCheckSum(buffer);
  uint8_t checksum_in_buffer = h2d2(gotSum[0], gotSum[1]);
  if (checksum_of_buffer == checksum_in_buffer) {return true;} else {return false;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                  SENTENCE ELEMENT VALIDATION

/*
checks can be ellaborated upon individually.
each sentence has a checksum that is for checking if the payload is more or less intact, while in contrast checks below are for
sanitizing each element of a sentence. thorough testing is required to ensure no false negatives are encountered but its worth
the extra work, rather than assuming all elements will be what we expect every time.
*/


bool count_digits(char * data, int expected) {
  validData.valid_i = 0;
  for (int i = 0; i < strlen(data); i++) {if (isdigit(data[i]) == 1) {validData.valid_i++;}}
  if (validData.valid_i == expected) {return true;} else {return false;}
}

bool count_alpha(char * data, int expected) {
  validData.valid_i = 0;
  for (int i = 0; i < strlen(data); i++) {if (isalpha(data[i]) == 1) {validData.valid_i++;}}
  if (validData.valid_i == expected) {return true;} else {return false;}
}

bool is_all_digits(char * data) {
  validData.valid_b = true;
  for (int i = 0; i < strlen(data); i++) {if (isdigit(data[i]) == 0) {validData.valid_b = false;}}
  return validData.valid_b;
}

bool is_all_digits_plus_char(char * data, char * find_char) {
  // designed to check all chars are digits except one period and is more general purpose than just accepting a period
  validData.valid_b = true;
  validData.find_char = strchr(data, * find_char);
  validData.index = (int)(validData.find_char - data);
  for (int i = 0; i < strlen(data); i++) {if (isdigit(data[i]) == 0) {if (i != validData.index) {validData.valid_b = false;}}}
  return validData.valid_b;
}

bool is_all_alpha(char * data) {
  validData.valid_b = true;
  for (int i = 0; i < strlen(data); i++) {if (isalpha(data[i]) == 0) {validData.valid_b = false;}}
  return validData.valid_b;
}

bool val_utc_time(char * data) {
  bool check_pass = false;
  if (strlen(data) == 9) {
    if (data[6] == '.') {
      if (count_digits(data, 8) == true) {
        if ((atoi(data) >= 0.0) && (atoi(data) <= 235959.99)) {check_pass = true;}
      }
    }
  }
  return check_pass;
}

bool val_utc_date(char * data) {
  bool check_pass = false;
  if (strlen(data) == 6) {
    if (is_all_digits(data) == true) {
      if ((atoi(data) >= 0.0) && (atoi(data) <= 999999)) {check_pass = true;}
    }
  }
  return check_pass;
}

bool val_latitude(char * data) {
  bool check_pass = false;
  if (strlen(data) == 13) {
    if (data[4] == '.') {
      if (count_digits(data, 12) == true) {
        check_pass = true;
      }
    }
  }
  return check_pass;
}

bool val_longitude(char * data) {
  bool check_pass = false;
  if (strlen(data) == 14) {
    if (data[5] == '.') {
      if (count_digits(data, 13) == true) {
        check_pass = true;
      }
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
    if (is_all_digits(data) == true) {
      if ((atoi(data) >= 0) && (atoi(data) <= 6)) {
        check_pass = true;
      }
    }
  }
  return check_pass;
}

bool val_satellite_count(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if (atoi(data) >= 0){
      check_pass = true;
      }
  }
  return check_pass;
}

bool val_hdop_precision_factor(char * data) {
  bool check_pass = false;
  if (is_all_digits_plus_char(data, ".") == true) {
    if (atoi(data) >= 0){
      check_pass = true;
  }
  }
  return check_pass;
}

bool val_altitude(char * data) {
  // account for decimal point
  bool check_pass = false;
  if (is_all_digits_plus_char(data, ".") == true) {
    if (atoi(data) >= -20000000){
        check_pass = true;
    }
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
  if (is_all_digits_plus_char(data, ".") == true) {
    if (atoi(data) >= 0){
      check_pass = true;
    }
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
  if (is_all_digits_plus_char(data, ".") == true) {
    if (atoi(data) >= 0){
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_basestation_id(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if (strlen(data) == 4) {
      check_pass = true;
    }
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
  if (is_all_digits_plus_char(data, ".") == true) {
    if (atoi(data) >= 0){
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_ground_heading(char * data) {
  bool check_pass = false;
  if (is_all_digits_plus_char(data, ".") == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 360)) {
      check_pass = true;
    }
  }
  return check_pass;
}

// todo
bool val_installation_angle(char * data) {
  bool check_pass = false;
  if (is_all_digits_plus_char(data, ".") == true) {
    if (atoi(data) >= 0) {
      check_pass = true;
    }
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
  if (is_all_digits_plus_char(data, ".") == true) {
    if (atoi(data) >= 0) {check_pass = true;}
  }
  return check_pass;
}

bool val_roll_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits_plus_char(data, ".") == true) {
    if (atoi(data) >= 0) {check_pass = true;}
  }
  return check_pass;
}

bool val_yaw_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits_plus_char(data, ".") == true) {
    if (atoi(data) >= 0) {check_pass = true;}
  }
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
  if (strcmp(data, "S") == 0) {check_pass = true;}
  return check_pass;
}

bool val_software_version_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if (atoi(data) == 20230219) {check_pass = true;}
  }
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
  if (is_all_digits(data) == true) {
    if ((atoi(data) == 0) || (atoi(data) == 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ins_channel_gpatt(char * data) {
  bool check_pass = false;
  if (strcmp(data, "INS") == 0) {check_pass = true;}
  return check_pass;
}

bool val_hardware_version_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if (strcmp(data, "3335") == 0) {check_pass = true;}
  }
  return check_pass;
}

bool val_run_state_flag_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((strcmp(data, "01") == 0) || (strcmp(data, "02") == 0) || (strcmp(data, "03") == 0)) {check_pass = true;}
  }
  return check_pass;
}

// todo
bool val_mis_angle_num_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits_plus_char(data, ".") == true) {
    if (atoi(data) >= 0) {check_pass = true;}
  }
  return check_pass;
}

bool val_static_flag_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) == 0) || (atoi(data) == 1)) {check_pass = true;}
  }
  return check_pass;
}

// todo
bool val_user_code_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits_plus_char(data, ".") == true) {
    if (atoi(data) >= 0) {check_pass = true;}
  }
  return check_pass;
}

bool val_gst_data_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if (atoi(data) >= 0) {check_pass = true;}
  }
  return check_pass;
}

bool val_line_flag_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) == 0) || (atoi(data) == 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_mis_att_flag_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) == 0) || (atoi(data) == 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_imu_kind_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 8)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ubi_car_kind_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 1) && (atoi(data) <= 4)) {check_pass = true;}
  }
  return check_pass;
}

bool val_mileage_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits_plus_char(data, ".") == true) {
    if (atoi(data) >= 0) {check_pass = true;}
  }
  return check_pass;
}

bool val_run_inetial_flag_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 4)) {check_pass = true;}
  }
  return check_pass;
}

bool val_speed_enable_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) == 0) || (atoi(data) == 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_speed_num_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 99)) {check_pass = true;}
  }
  return check_pass;
}

bool val_speed_status(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 2)) {check_pass = true;}
  }
  return check_pass;
}

bool val_accelleration_delimiter(char * data) {
  bool check_pass = false;
  if (strcmp(data, "A") == 0) {check_pass = true;}
  return check_pass;
}

bool val_axis_accelleration(char * data) {
  bool check_pass = false;
  if (is_all_digits_plus_char(data, ".") == true) {
    if (atoi(data) >= -1000000) {check_pass = true;}
  }
  return check_pass;
}

bool val_angular_velocity_delimiter(char * data) {
  bool check_pass = false;
  if (strcmp(data, "G") == 0) {check_pass = true;}
  return check_pass;
}

bool val_gyro_angular_velocity(char * data) {
  bool check_pass = false;
  if (is_all_digits_plus_char(data, ".") == true) {
    if (atoi(data) >= -1000000) {check_pass = true;}
  }
  return check_pass;
}

bool val_status_delimiter(char * data) {
  bool check_pass = false;
  if (strcmp(data, "S") == 0) {check_pass = true;}
  return check_pass;
}

bool val_ubi_state_flag(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ubi_state_kind_flag(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 8)) {check_pass = true;}
  }
  return check_pass;
}

bool val_code_flag(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_gset_flag(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_sset_flag(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ang_dget_flag(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ins_run_flag(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_fix_kind_flag(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_fix_roll_flag(char * data) {
  bool check_pass = false;
  if (is_all_digits_plus_char(data, ".") == true) {
    if (atoi(data) >= -1000000) {check_pass = true;}
  }
  return check_pass;
}

bool val_fix_pitch_flag(char * data) {
  bool check_pass = false;
  if (is_all_digits_plus_char(data, ".") == true) {
    if (atoi(data) >= -1000000) {check_pass = true;}
  }
  return check_pass;
}

bool val_ubi_on_flag(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 8)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ubi_kind_flag(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 2)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ubi_a_set(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
  if ((atoi(data) >= 0) && (atoi(data) <= 19)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ubi_b_set(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
  if ((atoi(data) >= 0) && (atoi(data) <= 19)) {check_pass = true;}
  }
  return check_pass;
}

bool val_acc_X_data(char * data) {
  bool check_pass = false;
  if (is_all_digits_plus_char(data, ".") == true) {
    if ((atoi(data) >= -400) && (atoi(data) <= 400)) {check_pass = true;}
  }
  return check_pass;
}

bool val_acc_Y_data(char * data) {
  bool check_pass = false;
  if (is_all_digits_plus_char(data, ".") == true) {
    if ((atoi(data) >= -400) && (atoi(data) <= 400)) {check_pass = true;}
  }
  return check_pass;
}

bool val_gyro_Z_data(char * data) {
  bool check_pass = false;
  if (is_all_digits_plus_char(data, ".") == true) {
    if ((atoi(data) >= -250) && (atoi(data) <= 250)) {check_pass = true;}
  }
  return check_pass;
}

bool val_pitch_angle(char * data) {
  bool check_pass = false;
  if (is_all_digits_plus_char(data, ".") == true) {
    if ((atoi(data) >= -180) && (atoi(data) <= 180)) {check_pass = true;}
  }
  return check_pass;
}

bool val_roll_angle(char * data) {
  bool check_pass = false;
  if (is_all_digits_plus_char(data, ".") == true) {
    if ((atoi(data) >= -180) && (atoi(data) <= 180)) {check_pass = true;}
  }
  return check_pass;
}

bool val_yaw_angle(char * data) {
  bool check_pass = false;
  if (is_all_digits_plus_char(data, ".") == true) {
    if ((atoi(data) >= -180) && (atoi(data) <= 180)) {check_pass = true;}
  }
  return check_pass;
}

bool val_car_speed(char * data) {
  bool check_pass = false;
  if (is_all_digits_plus_char(data, ".") == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 100)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ins_flag(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 4)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ubi_num(char * data) {
  bool check_pass = false;
  if (is_all_digits_plus_char(data, ".") == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 65536)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ubi_valid(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
  if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_coll_T_data(char * data) {
  bool check_pass = false;
  if (is_all_digits_plus_char(data, ".") == true) {
    if ((atoi(data) >= -800) && (atoi(data) <= 800)) {check_pass = true;}
  }
  return check_pass;
}

bool val_coll_T_heading(char * data) {
  bool check_pass = false;
  if (is_all_digits_plus_char(data, ".") == true) {
    if ((atoi(data) >= -180) && (atoi(data) <= 180)) {check_pass = true;}
  }
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
//                                                                                                                  RELAYS DATA

struct RelayStruct {

  int MAX_RELAYS = 40;
  int MAX_RELAY_ELEMENTS = 10;

  bool relays_bool[1][40] = {
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      }
  };
  
  char relays[40][10][100] = {
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 1
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 2
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 3
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 4
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 5
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 6
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 7
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 8
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 9
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 10
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 11
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 12
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 13
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 14
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 15
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 16
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 17
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 18
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 19
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 20
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 21
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 22
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 23
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 24
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 25
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 26
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 27
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 28
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 29
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 30
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 31
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 32
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 33
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 34
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 35
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 36
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 37
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 38
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 39
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 40
     },
    };


  /*
  Calibratable matrix. empty by default.
  consider overhead for fallback logic (no/bad satellite data --> untrained INS --> other sensor floor).

  Matrix containing sets of values per relay.
  X: use with/without  Y,Z.
  Y: necessary if comparing to X.
  Z: necessary if comparing to X/Y in range.  
                
                0                   30
          0     1     2     
          X     Y     Z    
  {  {   0.0,  0.0,  0.0   }       {0}          }
                              Enabled/Disabled
  */
  double relays_data[40][10+1][3] = {
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 1
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 2
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 3
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 4
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 5
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 6
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 7
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 8
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 9
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 10
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 11
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 12
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 13
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 14
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 15
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 16
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 17
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 18
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 19
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 20
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 21
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 22
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 23
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 24
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 25
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 26
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 27
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 28
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 29
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 30
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 31
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 32
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 33
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 34
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 35
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 36
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 37
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 38
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 39
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 40
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0},
    },
  };

  // default and specifiable value to indicate a relay should not be activated/deactivated if all functions in relays expression are $NONE
  char default_relay_function[56]          = "$NONE";
  char default_enable_relay_function[56]   = "$ENABLED";

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                  SATCOM DATA

  char latitude_satcom_gngga_over[56]             = "latitude_satcom_gngga_over";
  char latitude_satcom_gngga_under[56]            = "latitude_satcom_gngga_under";
  char latitude_satcom_gngga_equal[56]            = "latitude_satcom_gngga_equal";
  char latitude_satcom_gngga_in_range[56]         = "latitude_satcom_gngga_in_range";
  char satcom_in_range_gngga[56]                  = "satcom_in_range_gngga";

  char longitude_satcom_gngga_over[56]            = "longitude_satcom_gngga_over";
  char longitude_satcom_gngga_under[56]           = "longitude_satcom_gngga_under";
  char longitude_satcom_gngga_equal[56]           = "longitude_satcom_gngga_equal";
  char longitude_satcom_gngga_in_range[56]        = "longitude_satcom_gngga_in_range";

  char latitude_satcom_gnrmc_over[56]             = "latitude_satcom_gnrmc_over";
  char latitude_satcom_gnrmc_under[56]            = "latitude_satcom_gnrmc_under";
  char latitude_satcom_gnrmc_equal[56]            = "latitude_satcom_gnrmc_equal";
  char latitude_satcom_gnrmc_in_range[56]         = "latitude_satcom_gnrmc_in_range";

  char longitude_satcom_gnrmc_over[56]            = "longitude_satcom_gnrmc_over";
  char longitude_satcom_gnrmc_under[56]           = "longitude_satcom_gnrmc_under";
  char longitude_satcom_gnrmc_equal[56]           = "longitude_satcom_gnrmc_equal";
  char longitude_satcom_gnrmc_in_range[56]        = "longitude_satcom_gnrmc_in_ranged";

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                   GNGGA DATA

  char utc_time_gngga_over[56]             = "utc_time_gngga_over";
  char utc_time_gngga_under[56]            = "utc_time_gngga_under";
  char utc_time_gngga_equal[56]            = "utc_time_gngga_equal";
  char utc_time_gngga_in_range[56]         = "utc_time_gngga_in_range";

  char latitude_gngga_over[56]             = "latitude_gngga_over";
  char latitude_gngga_under[56]            = "latitude_gngga_under";
  char latitude_gngga_equal[56]            = "latitude_gngga_equal";
  char latitude_gngga_in_range[56]         = "latitude_gngga_in_range";

  char longitude_gngga_over[56]            = "longitude_gngga_over";
  char longitude_gngga_under[56]           = "longitude_gngga_under";
  char longitude_gngga_equal[56]           = "longitude_gngga_equal";
  char longitude_gngga_in_range[56]        = "longitude_gngga_in_range";

  char positioning_status_gngga_equal[56]  = "positioning_status_gngga_equal";

  char satellite_count_gngga_over[56]      = "satellite_count_gngga_over";
  char satellite_count_gngga_under[56]     = "satellite_count_gngga_under";
  char satellite_count_gngga_equal[56]     = "satellite_count_gngga_equal";
  char satellite_count_gngga_in_range[56]  = "satellite_count_gngga_in_range";

  char hemisphere_gngga_N[56]              = "hemisphere_gngga_N";
  char hemisphere_gngga_E[56]              = "hemisphere_gngga_E";
  char hemisphere_gngga_S[56]              = "hemisphere_gngga_S";
  char hemisphere_gngga_W[56]              = "hemisphere_gngga_W";

  char hdop_precision_factor_gngga_over[56]     = "hdop_precision_factor_gngga_over";
  char hdop_precision_factor_gngga_under[56]    = "hdop_precision_factor_gngga_under";
  char hdop_precision_factor_gngga_equal[56]    = "hdop_precision_factor_gngga_equal";
  char hdop_precision_factor_gngga_in_range[56] = "hdop_precision_factor_gngga_in_range";

  char altitude_gngga_over[56]             = "altitude_gngga_over";
  char altitude_gngga_under[56]            = "altitude_gngga_under";
  char altitude_gngga_equal[56]            = "altitude_gngga_equal";
  char altitude_gngga_in_range[56]         = "altitude_gngga_in_range";

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                   GNRMC DATA

  char utc_time_gnrmc_over[56]       = "utc_time_gnrmc_over";
  char utc_time_gnrmc_under[56]      = "utc_time_gnrmc_under";
  char utc_time_gnrmc_equal[56]      = "utc_time_gnrmc_equal";
  char utc_time_gnrmc_in_range[56]   = "utc_time_gnrmc_in_range";

  char positioning_status_gnrmc_equal_A[56]  = "positioning_status_gnrmc_equal_A";
  char positioning_status_gnrmc_equal_V[56]  = "positioning_status_gnrmc_equal_V";

  char mode_indication_gnrmc_equal_A[56]     = "mode_indication_gnrmc_equal_A";
  char mode_indication_gnrmc_equal_D[56]     = "mode_indication_gnrmc_equal_D";
  char mode_indication_gnrmc_equal_N[56]     = "mode_indication_gnrmc_equal_N";

  char latitude_gnrmc_over[56]             = "latitude_gnrmc_over";
  char latitude_gnrmc_under[56]            = "latitude_gnrmc_under";
  char latitude_gnrmc_equal[56]            = "latitude_gnrmc_equal";
  char latitude_gnrmc_in_range[56]         = "latitude_gnrmc_in_range";

  char longitude_gnrmc_over[56]            = "longitude_gnrmc_over";
  char longitude_gnrmc_under[56]           = "longitude_gnrmc_under";
  char longitude_gnrmc_equal[56]           = "longitude_gnrmc_equal";
  char longitude_gnrmc_in_range[56]        = "longitude_gnrmc_in_range";

  char hemisphere_gnrmc_N[56]              = "hemisphere_gnrmc_N";
  char hemisphere_gnrmc_E[56]              = "hemisphere_gnrmc_E";
  char hemisphere_gnrmc_S[56]              = "hemisphere_gnrmc_S";
  char hemisphere_gnrmc_W[56]              = "hemisphere_gnrmc_W";

  char ground_speed_gnrmc_over[56]         = "ground_speed_gnrmc_over";
  char ground_speed_gnrmc_under[56]        = "ground_speed_gnrmc_under";
  char ground_speed_gnrmc_equal[56]        = "ground_speed_gnrmc_equal";
  char ground_speed_gnrmc_in_range[56]     = "ground_speed_gnrmc_in_range";

  char heading_gnrmc_over[56]              = "heading_gnrmc_over";
  char heading_gnrmc_under[56]             = "heading_gnrmc_under";
  char heading_gnrmc_equal[56]             = "heading_gnrmc_equal";
  char heading_gnrmc_in_range[56]          = "heading_gnrmc_in_range";

  char utc_date_gnrmc_over[56]       = "utc_date_gnrmc_over";
  char utc_date_gnrmc_under[56]      = "utc_date_gnrmc_under";
  char utc_date_gnrmc_equal[56]      = "utc_date_gnrmc_equal";
  char utc_date_gnrmc_in_range[56]   = "utc_date_gnrmc_in_range";

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                   GPATT DATA

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

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                   SPEED DATA

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

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                   ERROR DATA

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

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                   DEBUG DATA

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

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                VALIDITY DATA

  char gngga_valid_checksum[56] = "gngga_valid_checksum";
  char gnrmc_valid_checksum[56] = "gnrmc_valid_checksum";
  char gpatt_valid_checksum[56] = "gpatt_valid_checksum";
  char desbi_valid_checksum[56] = "desbi_valid_checksum";
  char speed_valid_checksum[56] = "speed_valid_checksum";
  char error_valid_checksum[56] = "error_valid_checksum";
  char debug_valid_checksum[56] = "debug_valid_checksum";
  char gngga_invalid_checksum[56] = "gngga_invalid_checksum";
  char gnrmc_invalid_checksum[56] = "gnrmc_invalid_checksum";
  char gpatt_invalid_checksum[56] = "gpatt_invalid_checksum";
  char desbi_invalid_checksum[56] = "desbi_invalid_checksum";
  char speed_invalid_checksum[56] = "speed_invalid_checksum";
  char error_invalid_checksum[56] = "error_invalid_checksum";
  char debug_invalid_checksum[56] = "debug_invalid_checksum";

  char gngga_valid_check_data[56] = "gngga_valid_check_data";
  char gnrmc_valid_check_data[56] = "gnrmc_valid_check_data";
  char gpatt_valid_check_data[56] = "gpatt_valid_check_data";
  char desbi_valid_check_data[56] = "desbi_valid_check_data";
  char speed_valid_check_data[56] = "speed_valid_check_data";
  char error_valid_check_data[56] = "error_valid_check_data";
  char debug_valid_check_data[56] = "debug_valid_check_data";
  char gngga_invalid_check_data[56] = "gngga_invalid_check_data";
  char gnrmc_invalid_check_data[56] = "gnrmc_invalid_check_data";
  char gpatt_invalid_check_data[56] = "gpatt_invalid_check_data";
  char desbi_invalid_check_data[56] = "desbi_invalid_check_data";
  char speed_invalid_check_data[56] = "speed_invalid_check_data";
  char error_invalid_check_data[56] = "error_invalid_check_data";
  char debug_invalid_check_data[56] = "debug_invalid_check_data";

};
RelayStruct relayData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   GNGGA DATA

struct GNGGAStruct {
  char tag[56];                                                                                                           // <0> Log header
  char utc_time[56];                    unsigned long bad_utc_time_i;              bool bad_utc_time = true;              // <1> UTC time, the format is hhmmss.sss
  char latitude[56];                    unsigned long bad_latitude_i;              bool bad_latitude = true;              // <2> Latitude, the format is  ddmm.mmmmmmm
  char latitude_hemisphere[56];         unsigned long bad_latitude_hemisphere_i;   bool bad_latitude_hemisphere = true;   // <3> Latitude hemisphere, N or S (north latitude or south latitude)
  char longitude[56];                   unsigned long bad_longitude_i;             bool bad_longitude = true;             // <4> Longitude, the format is dddmm.mmmmmmm
  char longitude_hemisphere[56];        unsigned long bad_longitude_hemisphere_i;  bool bad_longitude_hemisphere = true;  // <5> Longitude hemisphere, E or W (east longitude or west longitude)
  char positioning_status[56];          unsigned long bad_positioning_status_i;    bool bad_positioning_status = true;    // <6> GNSS positioning status: 0 not positioned, 1 single point positioning, 2: pseudorange difference, 6: pure INS */
  char satellite_count_gngga[56] = "0"; unsigned long bad_satellite_count_gngga_i; bool bad_satellite_count_gngga = true; // <7> Number of satellites used
  char hdop_precision_factor[56];       unsigned long bad_hdop_precision_factor_i; bool bad_hdop_precision_factor = true; // <8> HDOP level precision factor
  char altitude[56];                    unsigned long bad_altitude_i;              bool bad_altitude = true;              // <9> Altitude
  char altitude_units[56];              unsigned long bad_altitude_units_i;        bool bad_altitude_units = true;        // <10> 
  char geoidal[56];                     unsigned long bad_geoidal_i;               bool bad_geoidal = true;               // <11> The height of the earth ellipsoid relative to the geoid 
  char geoidal_units[56];               unsigned long bad_geoidal_units_i;         bool bad_geoidal_units = true;         // <12> 
  char differential_delay[56];          unsigned long bad_differential_delay_i;    bool bad_differential_delay = true;    // <13>
  char id[56];                          unsigned long bad_id_i;                    bool bad_id = true;                    // <14> base station ID
  char check_sum[56];                   unsigned long bad_check_sum_i;             bool bad_check_sum = true;             // <15> XOR check value of all bytes starting from $ to *
  int check_data = 0;                   unsigned long bad_checksum_validity;       bool valid_checksum = false;           // Checksum validity bool, counters and a counter for how many elements passed further testing (gngga check_data should result in 16)
  char temporary_data[56];
  char temporary_data_1[56];
  
};
GNGGAStruct gnggaData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        GNGGA

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
  serial1Data.iter_token = 0;
  serial1Data.token = strtok(serial1Data.BUFFER, ",");
  while( serial1Data.token != NULL ) {
    if     (serial1Data.iter_token == 0)                                                               {strcpy(gnggaData.tag, "GNGGA");                            gnggaData.check_data++;}
    else if (serial1Data.iter_token ==1)  {if (val_utc_time(serial1Data.token) == true)                 {strcpy(gnggaData.utc_time, serial1Data.token);              gnggaData.check_data++; gnggaData.bad_utc_time = false;}              else {gnggaData.bad_utc_time_i++;              gnggaData.bad_utc_time = true;}}
    else if (serial1Data.iter_token ==2)  {if (val_latitude(serial1Data.token) == true)                 {strcpy(gnggaData.latitude, serial1Data.token);              gnggaData.check_data++; gnggaData.bad_latitude = false;}              else {gnggaData.bad_latitude_i++;              gnggaData.bad_latitude = true;}}
    else if (serial1Data.iter_token ==3)  {if (val_latitude_H(serial1Data.token) == true)               {strcpy(gnggaData.latitude_hemisphere, serial1Data.token);   gnggaData.check_data++; gnggaData.bad_latitude_hemisphere = false;}   else {gnggaData.bad_latitude_hemisphere_i++;   gnggaData.bad_latitude_hemisphere = true;}}
    else if (serial1Data.iter_token ==4)  {if (val_longitude(serial1Data.token) == true)                {strcpy(gnggaData.longitude, serial1Data.token);             gnggaData.check_data++; gnggaData.bad_longitude = false;}             else {gnggaData.bad_longitude_i++;             gnggaData.bad_longitude = true;}}
    else if (serial1Data.iter_token ==5)  {if (val_longitude_H(serial1Data.token) == true)              {strcpy(gnggaData.longitude_hemisphere, serial1Data.token);  gnggaData.check_data++; gnggaData.bad_longitude_hemisphere = false;}  else {gnggaData.bad_longitude_hemisphere_i++;  gnggaData.bad_longitude_hemisphere = true;}}
    else if (serial1Data.iter_token ==6)  {if (val_positioning_status_gngga(serial1Data.token) == true) {strcpy(gnggaData.positioning_status, serial1Data.token);    gnggaData.check_data++; gnggaData.bad_positioning_status = false;}    else {gnggaData.bad_positioning_status_i++;    gnggaData.bad_positioning_status = true;}}
    else if (serial1Data.iter_token ==7)  {if (val_satellite_count(serial1Data.token) == true)          {strcpy(gnggaData.satellite_count_gngga, serial1Data.token); gnggaData.check_data++; gnggaData.bad_satellite_count_gngga = false;} else {gnggaData.bad_satellite_count_gngga_i++; gnggaData.bad_satellite_count_gngga = true;}}
    else if (serial1Data.iter_token ==8)  {if (val_hdop_precision_factor(serial1Data.token) == true)    {strcpy(gnggaData.hdop_precision_factor, serial1Data.token); gnggaData.check_data++; gnggaData.bad_hdop_precision_factor = false;} else {gnggaData.bad_hdop_precision_factor_i++; gnggaData.bad_hdop_precision_factor = true;}}
    else if (serial1Data.iter_token ==9)  {if (val_altitude(serial1Data.token) == true)                 {strcpy(gnggaData.altitude, serial1Data.token);              gnggaData.check_data++; gnggaData.bad_altitude = false;}              else {gnggaData.bad_altitude_i++;              gnggaData.bad_altitude = true;}}
    else if (serial1Data.iter_token ==10) {if (val_altitude_units(serial1Data.token) == true)           {strcpy(gnggaData.altitude_units, serial1Data.token);        gnggaData.check_data++; gnggaData.bad_altitude_units = false;}        else {gnggaData.bad_altitude_units_i++;        gnggaData.bad_altitude_units = true;}}
    else if (serial1Data.iter_token ==11) {if (val_geoidal(serial1Data.token) == true)                  {strcpy(gnggaData.geoidal, serial1Data.token);               gnggaData.check_data++; gnggaData.bad_geoidal = false;}               else {gnggaData.bad_geoidal_i++;               gnggaData.bad_geoidal = true;}}
    else if (serial1Data.iter_token ==12) {if (val_geoidal_units(serial1Data.token) == true)            {strcpy(gnggaData.geoidal_units, serial1Data.token);         gnggaData.check_data++; gnggaData.bad_geoidal_units = false;}         else {gnggaData.bad_geoidal_units_i++;         gnggaData.bad_geoidal_units = true;}}
    else if (serial1Data.iter_token ==13) {if (val_differential_delay(serial1Data.token) == true)       {strcpy(gnggaData.differential_delay, serial1Data.token);    gnggaData.check_data++; gnggaData.bad_differential_delay = false;}    else {gnggaData.bad_differential_delay_i++;    gnggaData.bad_differential_delay = true;}}
    else if (serial1Data.iter_token ==14) {
      strcpy(gnggaData.temporary_data, strtok(serial1Data.token, "*"));
      if (val_basestation_id(gnggaData.temporary_data) == true)                                       {strcpy(gnggaData.id, gnggaData.temporary_data);            gnggaData.check_data++; gnggaData.bad_id = false;}                    else {gnggaData.bad_id_i++;                    gnggaData.bad_id = true;}
      serial1Data.token = strtok(NULL, "*");
      strcpy(gnggaData.temporary_data_1, strtok(serial1Data.token, "*"));
      if (val_checksum(gnggaData.temporary_data_1) == true)                                           {strcpy(gnggaData.check_sum, gnggaData.temporary_data_1);   gnggaData.check_data++; gnggaData.bad_check_sum = false;}             else {gnggaData.bad_check_sum_i++;             gnggaData.bad_check_sum = true;}}
    serial1Data.token = strtok(NULL, ",");
    serial1Data.iter_token++;
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
  char tag[56];                                                                                                                            // <0> Log header
  char utc_time[56];                       unsigned long bad_utc_time_i;                     bool bad_utc_time = true;                     // <1> UTC time, the format is hhmmss.sss
  char positioning_status[56];             unsigned long bad_positioning_status_i;           bool bad_positioning_status = true;           // <2> Positioning status, A=effective positioning, V=invalid positioning
  char latitude[56];                       unsigned long bad_latitude_i;                     bool bad_latitude = true;                     // <3> Latitude, the format is  ddmm.mmmmmmm
  char latitude_hemisphere[56];            unsigned long bad_latitude_hemisphere_i;          bool bad_latitude_hemisphere = true;          // <4> Latitude hemisphere, N or S (north latitude or south latitude)
  char longitude[56];                      unsigned long bad_longitude_i;                    bool bad_longitude = true;                    // <5> Longitude, the format is dddmm.mmmmmmm
  char longitude_hemisphere[56];           unsigned long bad_longitude_hemisphere_i;         bool bad_longitude_hemisphere = true;         // <6> Longitude hemisphere, E or W (east longitude or west longitude)
  char ground_speed[56];                   unsigned long bad_ground_speed_i;                 bool bad_ground_speed = true;                 // <7> Ground speed
  char ground_heading[56];                 unsigned long bad_ground_heading_i;               bool bad_ground_heading = true;               // <8> Ground heading (take true north as the reference datum)
  char utc_date[56];                       unsigned long bad_utc_date_i;                     bool bad_utc_date = true;                     // <9> UTC date, the format is ddmmyy (day, month, year)
  char installation_angle[56];             unsigned long bad_installation_angle_i;           bool bad_installation_angle = true;           // <10> Magnetic declination (000.0~180.0 degrees)
  char installation_angle_direction[56];   unsigned long bad_installation_angle_direction_i; bool bad_installation_angle_direction = true; // <11> Magnetic declination direction, E (east) or W (west)
  char mode_indication[56];                unsigned long bad_mode_indication_i;              bool bad_mode_indication = true;              // <12> Mode indication (A=autonomous positioning, D=differential E=estimation, N=invalid data) */
  char check_sum[56];                      unsigned long bad_check_sum_i;                    bool bad_check_sum = true;                    // <13> XOR check value of all bytes starting from $ to *
  int check_data = 0;                      unsigned long bad_checksum_validity;              bool valid_checksum = false;                  // Checksum validity bool, counters and a counter for how many elements passed further testing (gnrmc check_data should result in 14)
  char temporary_data[56];
  char temporary_data_1[56];
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
  serial1Data.iter_token = 0;
  serial1Data.token = strtok(serial1Data.BUFFER, ",");
  while( serial1Data.token != NULL ) {
    if      (serial1Data.iter_token == 0)                                                                  {strcpy(gnrmcData.tag, "GNRMC");                                   gnrmcData.check_data++;}
    else if (serial1Data.iter_token ==1)  {if (val_utc_time(serial1Data.token) == true)                     {strcpy(gnrmcData.utc_time, serial1Data.token);                     gnrmcData.check_data++; gnrmcData.bad_utc_time = false;}                     else {gnrmcData.bad_utc_time_i++;                     gnrmcData.bad_utc_time = true;}}
    else if (serial1Data.iter_token ==2)  {if (val_positioning_status_gnrmc(serial1Data.token) == true)     {strcpy(gnrmcData.positioning_status, serial1Data.token);           gnrmcData.check_data++; gnrmcData.bad_positioning_status = false;}           else {gnrmcData.bad_positioning_status_i++;           gnrmcData.bad_positioning_status = true;}}
    else if (serial1Data.iter_token ==3)  {if (val_latitude(serial1Data.token) == true)                     {strcpy(gnrmcData.latitude, serial1Data.token);                     gnrmcData.check_data++; gnrmcData.bad_latitude = false;}                     else {gnrmcData.bad_latitude_i++;                     gnrmcData.bad_latitude = true;}}
    else if (serial1Data.iter_token ==4)  {if (val_latitude_H(serial1Data.token) == true)                   {strcpy(gnrmcData.latitude_hemisphere, serial1Data.token);          gnrmcData.check_data++; gnrmcData.bad_latitude_hemisphere = false;}          else {gnrmcData.bad_latitude_hemisphere_i++;          gnrmcData.bad_latitude_hemisphere = true;}}
    else if (serial1Data.iter_token ==5)  {if (val_longitude(serial1Data.token) == true)                    {strcpy(gnrmcData.longitude, serial1Data.token);                    gnrmcData.check_data++; gnrmcData.bad_longitude = false;}                    else {gnrmcData.bad_longitude_i++;                    gnrmcData.bad_longitude = true;}}
    else if (serial1Data.iter_token ==6)  {if (val_longitude_H(serial1Data.token) == true)                  {strcpy(gnrmcData.longitude_hemisphere, serial1Data.token);         gnrmcData.check_data++; gnrmcData.bad_longitude_hemisphere = false;}         else {gnrmcData.bad_longitude_hemisphere_i++;         gnrmcData.bad_longitude_hemisphere = true;}}
    else if (serial1Data.iter_token ==7)  {if (val_ground_speed(serial1Data.token) == true)                 {strcpy(gnrmcData.ground_speed, serial1Data.token);                 gnrmcData.check_data++; gnrmcData.bad_ground_speed = false;}                 else {gnrmcData.bad_ground_speed_i++;                 gnrmcData.bad_ground_speed = true;}}
    else if (serial1Data.iter_token ==8)  {if (val_ground_heading(serial1Data.token) == true)               {strcpy(gnrmcData.ground_heading, serial1Data.token);               gnrmcData.check_data++; gnrmcData.bad_ground_heading = false;}               else {gnrmcData.bad_ground_heading_i++;               gnrmcData.bad_ground_heading = true;}}
    else if (serial1Data.iter_token ==9)  {if (val_utc_date(serial1Data.token) == true)                     {strcpy(gnrmcData.utc_date, serial1Data.token);                     gnrmcData.check_data++; gnrmcData.bad_utc_date = false;}                     else {gnrmcData.bad_utc_date_i++;                     gnrmcData.bad_utc_date = true;}}
    else if (serial1Data.iter_token ==10) {if (val_installation_angle(serial1Data.token) == true)           {strcpy(gnrmcData.installation_angle, serial1Data.token);           gnrmcData.check_data++; gnrmcData.bad_installation_angle = false;}           else {gnrmcData.bad_installation_angle_i++;           gnrmcData.bad_installation_angle = true;}}
    else if (serial1Data.iter_token ==11) {if (val_installation_angle_direction(serial1Data.token) == true) {strcpy(gnrmcData.installation_angle_direction, serial1Data.token); gnrmcData.check_data++; gnrmcData.bad_installation_angle_direction = false;} else {gnrmcData.bad_installation_angle_direction_i++; gnrmcData.bad_installation_angle_direction = true;}}
    else if (serial1Data.iter_token ==12) {
      strcpy(gnrmcData.temporary_data, strtok(serial1Data.token, "*"));
      if (val_mode_indication(gnrmcData.temporary_data) == true)                                          {strcpy(gnrmcData.mode_indication, gnrmcData.temporary_data);      gnrmcData.check_data++; gnrmcData.bad_mode_indication = false;}              else {gnrmcData.bad_mode_indication_i++;              gnrmcData.bad_mode_indication = true;}
      serial1Data.token = strtok(NULL, "*");
      strcpy(gnrmcData.temporary_data_1, strtok(serial1Data.token, "*"));
      if (val_checksum(gnrmcData.temporary_data_1) == true)                                               {strcpy(gnrmcData.check_sum, gnrmcData.temporary_data_1);          gnrmcData.check_data++; gnrmcData.bad_check_sum = false;}                    else {gnrmcData.bad_check_sum_i++;                    gnrmcData.bad_check_sum = true;}}
    serial1Data.token = strtok(NULL, ",");
    serial1Data.iter_token++;
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
  char tag[56];                                                                                      // <0> Log header
  char pitch[56];            unsigned long bad_pitch_i;            bool bad_pitch = true;            // <1> pitch angle
  char angle_channel_0[56];  unsigned long bad_angle_channel_0_i;  bool bad_angle_channel_0 = true;  // <2> P
  char roll[56];             unsigned long bad_roll_i;             bool bad_roll = true;             // <3> Roll angle
  char angle_channel_1[56];  unsigned long bad_angle_channel_1_i;  bool bad_angle_channel_1 = true;  // <4> R
  char yaw[56];              unsigned long bad_yaw_i;              bool bad_yaw = true;              // <5> Yaw angle
  char angle_channel_2[56];  unsigned long bad_angle_channel_2_i;  bool bad_angle_channel_2 = true;  // <6> Y
  char software_version[56]; unsigned long bad_software_version_i; bool bad_software_version = true; // <7> software verion
  char version_channel[56];  unsigned long bad_version_channel_i;  bool bad_version_channel = true;  // <8> S
  char product_id[56];       unsigned long bad_product_id_i;       bool bad_product_id = true;       // <9> Product ID: 96 bit unique ID
  char id_channel[56];       unsigned long bad_id_channel_i;       bool bad_id_channel = true;       // <10> ID 
  char ins[56];              unsigned long bad_ins_i;              bool bad_ins = true;              // <11> INS Default open inertial navigation system
  char ins_channel[56];      unsigned long bad_ins_channel_i;      bool bad_ins_channel = true;      // <12> whether inertial navigation open
  char hardware_version[56]; unsigned long bad_hardware_version_i; bool bad_hardware_version = true; // <13> Named after master chip
  char run_state_flag[56];   unsigned long bad_run_state_flag_i;   bool bad_run_state_flag = true;   // <14> Algorithm status flag: 1->3
  char mis_angle_num[56];    unsigned long bad_mis_angle_num_i;    bool bad_mis_angle_num = true;    // <15> number of Installation
  char custom_logo_0[56];    unsigned long bad_custom_logo_0_i;    bool bad_custom_logo_0 = true;    // <16>
  char custom_logo_1[56];    unsigned long bad_custom_logo_1_i;    bool bad_custom_logo_1 = true;    // <17>
  char custom_logo_2[56];    unsigned long bad_custom_logo_2_i;    bool bad_custom_logo_2 = true;    // <18>
  char static_flag[56];      unsigned long bad_static_flag_i;      bool bad_static_flag = true;      // <19> 1:Static 0：dynamic
  char user_code[56];        unsigned long bad_user_code_i;        bool bad_user_code = true;        // <20> 1：Normal user X：Customuser
  char gst_data[56];         unsigned long bad_gst_data_i;         bool bad_gst_data = true;         // <21> User satellite accuracy
  char line_flag[56];        unsigned long bad_line_flag_i;        bool bad_line_flag = true;        // <22> 1：straight driving，0：curve driving
  char custom_logo_3[56];    unsigned long bad_custom_logo_3_i;    bool bad_custom_logo_3 = true;    // <23>
  char mis_att_flag[56];     unsigned long bad_mis_att_flag_i;     bool bad_mis_att_flag = true;     // <24> 
  char imu_kind[56];         unsigned long bad_imu_kind_i;         bool bad_imu_kind = true;         // <25> Sensor Type: 0->BIM055; 1->BMI160; 2->LSM6DS3TR-C; 3->LSM6DSOW 4->ICM-40607; 5->ICM-40608 6->ICM-42670; 7->LSM6DSR
  char ubi_car_kind[56];     unsigned long bad_ubi_car_kind_i;     bool bad_ubi_car_kind = true;     // <26> 1: small car, 2: big car
  char mileage[56];          unsigned long bad_mileage_i;          bool bad_mileage = true;          // <27> kilometers: max 9999 kilometers
  char custom_logo_4[56];    unsigned long bad_custom_logo_4_i;    bool bad_custom_logo_4 = true;    // <28>
  char custom_logo_5[56];    unsigned long bad_custom_logo_5_i;    bool bad_custom_logo_5 = true;    // <29>
  char run_inetial_flag[56]; unsigned long bad_run_inetial_flag_i; bool bad_run_inetial_flag = true; // <30> 1->4
  char custom_logo_6[56];    unsigned long bad_custom_logo_6_i;    bool bad_custom_logo_6 = true;    // <31>
  char custom_logo_7[56];    unsigned long bad_custom_logo_7_i;    bool bad_custom_logo_7 = true;    // <32>
  char custom_logo_8[56];    unsigned long bad_custom_logo_8_i;    bool bad_custom_logo_8 = true;    // <33>
  char custom_logo_9[56];    unsigned long bad_custom_logo_9_i;    bool bad_custom_logo_9 = true;    // <34>
  char speed_enable[56];     unsigned long bad_speed_enable_i;     bool bad_speed_enable = true;     // <35> 
  char custom_logo_10[56];   unsigned long bad_custom_logo_10_i;   bool bad_custom_logo_10 = true;   // <36>
  char custom_logo_11[56];   unsigned long bad_custom_logo_11_i;   bool bad_custom_logo_11 = true;   // <37>
  char speed_num[56];        unsigned long bad_speed_num_i;        bool bad_speed_num = true;        // <38> 1：fixed setting，0：Self adaptive installation
  char scalable[56];         unsigned long bad_scalable_i;         bool bad_scalable = true;         // <39> 
  char check_sum[56];        unsigned long bad_check_sum_i;        bool bad_check_sum = true;        // <40> XOR check value of all bytes starting from $ to *
  int check_data = 0;        unsigned long bad_checksum_validity;  bool valid_checksum = false;      // Checksum validity bool, counters and a counter for how many elements passed further testing (gnrmc check_data should result in 41)
  char temporary_data[56];
  char temporary_data_1[56];
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
  serial1Data.iter_token = 0;
  serial1Data.token = strtok(serial1Data.BUFFER, ",");
  while( serial1Data.token != NULL ) {
    if      (serial1Data.iter_token == 0)                                                             {strcpy(gpattData.tag, "GPATT");                       gpattData.check_data++;}
    else if (serial1Data.iter_token == 1) {if (val_pitch_gpatt(serial1Data.token) == true)             {strcpy(gpattData.pitch, serial1Data.token);            gpattData.check_data++; gpattData.bad_pitch = false;}            else {gpattData.bad_pitch_i++;            gpattData.bad_pitch = true;}}
    else if (serial1Data.iter_token == 2) {if (val_angle_channle_p_gpatt(serial1Data.token) == true)   {strcpy(gpattData.angle_channel_0, serial1Data.token);  gpattData.check_data++; gpattData.bad_angle_channel_0 = false;}  else {gpattData.bad_angle_channel_0_i++;  gpattData.bad_angle_channel_0 = true;}}
    else if (serial1Data.iter_token == 3) {if (val_roll_gpatt(serial1Data.token) == true)              {strcpy(gpattData.roll, serial1Data.token);             gpattData.check_data++; gpattData.bad_roll = false;}             else {gpattData.bad_roll_i++;             gpattData.bad_roll = true;}}
    else if (serial1Data.iter_token == 4) {if (val_angle_channle_r_gpatt(serial1Data.token) == true)   {strcpy(gpattData.angle_channel_1, serial1Data.token);  gpattData.check_data++; gpattData.bad_angle_channel_1 = false;}  else {gpattData.bad_angle_channel_1_i++;  gpattData.bad_angle_channel_1 = true;}}
    else if (serial1Data.iter_token == 5) {if (val_yaw_gpatt(serial1Data.token) == true)               {strcpy(gpattData.yaw, serial1Data.token);              gpattData.check_data++; gpattData.bad_yaw = false;}              else {gpattData.bad_yaw_i++;              gpattData.bad_yaw = true;}}
    else if (serial1Data.iter_token == 6) {if (val_angle_channle_y_gpatt(serial1Data.token) == true)   {strcpy(gpattData.angle_channel_2, serial1Data.token);  gpattData.check_data++; gpattData.bad_angle_channel_2 = false;}  else {gpattData.bad_angle_channel_2_i++;  gpattData.bad_angle_channel_2 = true;}}
    else if (serial1Data.iter_token == 7) {if (val_software_version_gpatt(serial1Data.token) == true)  {strcpy(gpattData.software_version, serial1Data.token); gpattData.check_data++; gpattData.bad_software_version = false;} else {gpattData.bad_software_version_i++; gpattData.bad_software_version = true;}}
    else if (serial1Data.iter_token == 8) {if (val_version_channel_s_gpatt(serial1Data.token) == true) {strcpy(gpattData.version_channel, serial1Data.token);  gpattData.check_data++; gpattData.bad_version_channel = false;}  else {gpattData.bad_version_channel_i++;  gpattData.bad_version_channel = true;}}
    else if (serial1Data.iter_token == 9) {if (val_product_id_gpatt(serial1Data.token) == true)        {strcpy(gpattData.product_id, serial1Data.token);       gpattData.check_data++; gpattData.bad_product_id = false;}       else {gpattData.bad_product_id_i++;       gpattData.bad_product_id = true;}}
    else if (serial1Data.iter_token == 10) {if (val_id_channel_gpatt(serial1Data.token) == true)       {strcpy(gpattData.id_channel, serial1Data.token);       gpattData.check_data++; gpattData.bad_id_channel = false;}       else {gpattData.bad_id_channel_i++;       gpattData.bad_id_channel = true;}}
    else if (serial1Data.iter_token == 11) {if (val_ins_gpatt(serial1Data.token) == true)              {strcpy(gpattData.ins, serial1Data.token);              gpattData.check_data++; gpattData.bad_ins = false;}              else {gpattData.bad_ins_i++;              gpattData.bad_ins = true;}}
    else if (serial1Data.iter_token == 12) {if (val_ins_channel_gpatt(serial1Data.token) == true)      {strcpy(gpattData.ins_channel, serial1Data.token);      gpattData.check_data++; gpattData.bad_ins_channel = false;}      else {gpattData.bad_ins_channel_i++;      gpattData.bad_ins_channel = true;}}
    else if (serial1Data.iter_token == 13) {if (val_hardware_version_gpatt(serial1Data.token) == true) {strcpy(gpattData.hardware_version, serial1Data.token); gpattData.check_data++; gpattData.bad_hardware_version = false;} else {gpattData.bad_hardware_version_i++; gpattData.bad_hardware_version = true;}}
    else if (serial1Data.iter_token == 14) {if (val_run_state_flag_gpatt(serial1Data.token) == true)   {strcpy(gpattData.run_state_flag, serial1Data.token);   gpattData.check_data++; gpattData.bad_run_state_flag = false;}   else {gpattData.bad_run_state_flag_i++;   gpattData.bad_run_state_flag = true;}}
    else if (serial1Data.iter_token == 15) {if (val_mis_angle_num_gpatt(serial1Data.token) == true)    {strcpy(gpattData.mis_angle_num, serial1Data.token);    gpattData.check_data++; gpattData.bad_mis_angle_num = false;}    else {gpattData.bad_mis_angle_num_i++;    gpattData.bad_mis_angle_num = true;}}
    else if (serial1Data.iter_token == 16) {if (val_custom_flag(serial1Data.token) == true)            {strcpy(gpattData.custom_logo_0, serial1Data.token);    gpattData.check_data++; gpattData.bad_custom_logo_0 = false;}    else {gpattData.bad_custom_logo_0_i++;    gpattData.bad_custom_logo_0 = true;}}
    else if (serial1Data.iter_token == 17) {if (val_custom_flag(serial1Data.token) == true)            {strcpy(gpattData.custom_logo_1, serial1Data.token);    gpattData.check_data++; gpattData.bad_custom_logo_1 = false;}    else {gpattData.bad_custom_logo_1_i++;    gpattData.bad_custom_logo_1 = true;}}
    else if (serial1Data.iter_token == 18) {if (val_custom_flag(serial1Data.token) == true)            {strcpy(gpattData.custom_logo_2, serial1Data.token);    gpattData.check_data++; gpattData.bad_custom_logo_2 = false;}    else {gpattData.bad_custom_logo_2_i++;    gpattData.bad_custom_logo_2 = true;}}
    else if (serial1Data.iter_token == 19) {if (val_static_flag_gpatt(serial1Data.token) == true)      {strcpy(gpattData.static_flag, serial1Data.token);      gpattData.check_data++; gpattData.bad_static_flag = false;}      else {gpattData.bad_static_flag_i++;      gpattData.bad_static_flag = true;}}
    else if (serial1Data.iter_token == 20) {if (val_user_code_gpatt(serial1Data.token) == true)        {strcpy(gpattData.user_code, serial1Data.token);        gpattData.check_data++; gpattData.bad_user_code = false;}        else {gpattData.bad_user_code_i++;        gpattData.bad_user_code = true;}}
    else if (serial1Data.iter_token == 21) {if (val_gst_data_gpatt(serial1Data.token) == true)         {strcpy(gpattData.gst_data, serial1Data.token);         gpattData.check_data++; gpattData.bad_gst_data = false;}         else {gpattData.bad_gst_data_i++;         gpattData.bad_gst_data = true;}}
    else if (serial1Data.iter_token == 22) {if (val_line_flag_gpatt(serial1Data.token) == true)        {strcpy(gpattData.line_flag, serial1Data.token);        gpattData.check_data++; gpattData.bad_line_flag = false;}        else {gpattData.bad_line_flag_i++;        gpattData.bad_line_flag = true;}}
    else if (serial1Data.iter_token == 23) {if (val_custom_flag(serial1Data.token) == true)            {strcpy(gpattData.custom_logo_3, serial1Data.token);    gpattData.check_data++; gpattData.bad_custom_logo_3 = false;}    else {gpattData.bad_custom_logo_3_i++;    gpattData.bad_custom_logo_3 = true;}}
    else if (serial1Data.iter_token == 24) {if (val_mis_att_flag_gpatt(serial1Data.token) == true)     {strcpy(gpattData.mis_att_flag, serial1Data.token);     gpattData.check_data++; gpattData.bad_mis_att_flag = false;}     else {gpattData.bad_mis_att_flag_i++;     gpattData.bad_mis_att_flag = true;}}
    else if (serial1Data.iter_token == 25) {if (val_imu_kind_gpatt(serial1Data.token) == true)         {strcpy(gpattData.imu_kind, serial1Data.token);         gpattData.check_data++; gpattData.bad_imu_kind = false;}         else {gpattData.bad_imu_kind_i++;         gpattData.bad_imu_kind = true;}}
    else if (serial1Data.iter_token == 26) {if (val_ubi_car_kind_gpatt(serial1Data.token) == true)     {strcpy(gpattData.ubi_car_kind, serial1Data.token);     gpattData.check_data++; gpattData.bad_ubi_car_kind = false;}     else {gpattData.bad_ubi_car_kind_i++;     gpattData.bad_ubi_car_kind = true;}}
    else if (serial1Data.iter_token == 27) {if (val_mileage_gpatt(serial1Data.token) == true)          {strcpy(gpattData.mileage, serial1Data.token);          gpattData.check_data++; gpattData.bad_mileage = false;}          else {gpattData.bad_mileage_i++;          gpattData.bad_mileage = true;}}
    else if (serial1Data.iter_token == 28) {if (val_custom_flag(serial1Data.token) == true)            {strcpy(gpattData.custom_logo_4, serial1Data.token);    gpattData.check_data++; gpattData.bad_custom_logo_4 = false;}    else {gpattData.bad_custom_logo_4_i++;    gpattData.bad_custom_logo_4 = true;}}
    else if (serial1Data.iter_token == 29) {if (val_custom_flag(serial1Data.token) == true)            {strcpy(gpattData.custom_logo_5, serial1Data.token);    gpattData.check_data++; gpattData.bad_custom_logo_5 = false;}    else {gpattData.bad_custom_logo_5_i++;    gpattData.bad_custom_logo_5 = true;}}
    else if (serial1Data.iter_token == 30) {if (val_run_inetial_flag_gpatt(serial1Data.token) == true) {strcpy(gpattData.run_inetial_flag, serial1Data.token); gpattData.check_data++; gpattData.bad_run_inetial_flag = false;} else {gpattData.bad_run_inetial_flag_i++; gpattData.bad_run_inetial_flag = true;}}
    else if (serial1Data.iter_token == 31) {if (val_custom_flag(serial1Data.token) == true)            {strcpy(gpattData.custom_logo_6, serial1Data.token);    gpattData.check_data++; gpattData.bad_custom_logo_6 = false;}    else {gpattData.bad_custom_logo_6_i++;    gpattData.bad_custom_logo_6 = true;}}
    else if (serial1Data.iter_token == 32) {if (val_custom_flag(serial1Data.token) == true)            {strcpy(gpattData.custom_logo_7, serial1Data.token);    gpattData.check_data++; gpattData.bad_custom_logo_7 = false;}    else {gpattData.bad_custom_logo_7_i++;    gpattData.bad_custom_logo_7 = true;}}
    else if (serial1Data.iter_token == 33) {if (val_custom_flag(serial1Data.token) == true)            {strcpy(gpattData.custom_logo_8, serial1Data.token);    gpattData.check_data++; gpattData.bad_custom_logo_8 = false;}    else {gpattData.bad_custom_logo_8_i++;    gpattData.bad_custom_logo_8 = true;}}
    else if (serial1Data.iter_token == 34) {if (val_custom_flag(serial1Data.token) == true)            {strcpy(gpattData.custom_logo_9, serial1Data.token);    gpattData.check_data++; gpattData.bad_custom_logo_9 = false;}    else {gpattData.bad_custom_logo_9_i++;    gpattData.bad_custom_logo_9 = true;}}
    else if (serial1Data.iter_token == 35) {if (val_speed_enable_gpatt(serial1Data.token) == true)     {strcpy(gpattData.speed_enable, serial1Data.token);     gpattData.check_data++; gpattData.bad_speed_enable = false;}     else {gpattData.bad_speed_enable_i++;     gpattData.bad_speed_enable = true;}}
    else if (serial1Data.iter_token == 36) {if (val_custom_flag(serial1Data.token) == true)            {strcpy(gpattData.custom_logo_10, serial1Data.token);   gpattData.check_data++; gpattData.bad_custom_logo_10 = false;}   else {gpattData.bad_custom_logo_10_i++;   gpattData.bad_custom_logo_10 = true;}}
    else if (serial1Data.iter_token == 37) {if (val_custom_flag(serial1Data.token) == true)            {strcpy(gpattData.custom_logo_11, serial1Data.token);   gpattData.check_data++; gpattData.bad_custom_logo_11 = false;}   else {gpattData.bad_custom_logo_11_i++;   gpattData.bad_custom_logo_11 = true;}}
    else if (serial1Data.iter_token == 38) {if (val_speed_num_gpatt(serial1Data.token) == true)        {strcpy(gpattData.speed_num, serial1Data.token);        gpattData.check_data++; gpattData.bad_speed_num = false;}        else {gpattData.bad_speed_num_i++;        gpattData.bad_speed_num = true;}}
    else if (serial1Data.iter_token == 39) {
      strcpy(gpattData.temporary_data, strtok(serial1Data.token, "*"));
      if (val_scalable(gpattData.temporary_data) == true)                                            {strcpy(gpattData.scalable, gpattData.temporary_data); gpattData.check_data++; gpattData.bad_scalable = false;}         else {gpattData.bad_scalable_i++;         gpattData.bad_scalable = true;}
      serial1Data.token = strtok(NULL, "*");
      strcpy(gpattData.temporary_data_1, strtok(serial1Data.token, "*"));
      if (val_checksum(gpattData.temporary_data_1) == true)                                          {strcpy(gpattData.check_sum, gpattData.temporary_data_1); gpattData.check_data++; gpattData.bad_check_sum = false;}     else {gpattData.bad_check_sum_i++;        gpattData.bad_check_sum = true;}}
    serial1Data.token = strtok(NULL, ",");
    serial1Data.iter_token++;
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
  char tag[56];                                                                                                                          // <0> log header
  char utc_time[56];                         unsigned long bad_utc_time_i;                   bool bad_utc_time = true;                   // <1> utc time
  char speed[56];                            unsigned long bad_speed_i;                      bool bad_speed = true;                      // <2> ground speed: knots
  char status[56];                           unsigned long bad_status_i;                     bool bad_status = true;                     // <3> 0=invalid data, 1=converging, 2=valid data
  char acceleration_delimiter[56];           unsigned long bad_acceleration_delimiter_i;     bool bad_acceleration_delimiter = true;     // <4> represents acceleration
  char acc_X[56];                            unsigned long bad_acc_X_i;                      bool bad_acc_X = true;                      // <5> x-axis acceleration
  char acc_Y[56];                            unsigned long bad_acc_Y_i;                      bool bad_acc_Y = true;                      // <6> y-axis acceleration
  char acc_Z[56];                            unsigned long bad_acc_Z_i;                      bool bad_acc_Z = true;                      // <7> z-axis acceleration
  char angular_velocity_delimiter[56] = "0"; unsigned long bad_angular_velocity_delimiter_i; bool bad_angular_velocity_delimiter = true; // <8> represents angular velocity
  char gyro_X[56];                           unsigned long bad_gyro_X_i;                     bool bad_gyro_X = true;                     // <9> x-axis angular velocity
  char gyro_Y[56];                           unsigned long bad_gyro_Y_i;                     bool bad_gyro_Y = true;                     // <10> y-axis angular velocity
  char gyro_Z[56];                           unsigned long bad_gyro_Z_i;                     bool bad_gyro_Z = true;                     // <11> z-axis angular velocity
  char status_delimiter[56];                 unsigned long bad_status_delimiter_i;           bool bad_status_delimiter = true;           // <12> represents status
  char ubi_state_flag[56];                   unsigned long bad_ubi_state_flag_i;             bool bad_ubi_state_flag = true;             // <13> 0=smooth driving, 1=unsteady driving
  char ubi_state_kind[56];                   unsigned long bad_ubi_state_kind_i;             bool bad_ubi_state_kind = true;             // <14> status tyoe: see ubi_state_kind table
  char ubi_state_value[56];                  unsigned long bad_ubi_state_value_i;            bool bad_ubi_state_value = true;            // <15> status threshold: see ubi_state_kind table
  char check_sum[56];                        unsigned long bad_check_sum_i;                  bool bad_check_sum = true;                  // <16> XOR check value of all bytes starting from $ to *
  int check_data = 0;                        unsigned long bad_checksum_validity;            bool valid_checksum = false;                // Checksum validity bool, counters and a counter for how many elements passed further testing (gnrmc check_data should result in 17)
  char temporary_data[56];
  char temporary_data_1[56];
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
  serial1Data.iter_token = 0;
  serial1Data.token = strtok(serial1Data.BUFFER, ",");
  while( serial1Data.token != NULL ) {
    if      (serial1Data.iter_token == 0)                                                                 {strcpy(speedData.tag, "SPEED");                                 speedData.check_data++;}
    else if (serial1Data.iter_token == 1)  {if (val_utc_time(serial1Data.token) == true)                   {strcpy(speedData.utc_time, serial1Data.token);                   speedData.check_data++; speedData.bad_utc_time = false;}                   else {speedData.bad_utc_time_i++;                   speedData.bad_utc_time = true;}}
    else if (serial1Data.iter_token == 2)  {if (val_ground_speed(serial1Data.token) == true)               {strcpy(speedData.speed, serial1Data.token);                      speedData.check_data++; speedData.bad_speed = false;}                      else {speedData.bad_speed_i++;                      speedData.bad_speed = true;}}
    else if (serial1Data.iter_token == 3)  {if (val_speed_status(serial1Data.token) == true)               {strcpy(speedData.status, serial1Data.token);                     speedData.check_data++; speedData.bad_status = false;}                     else {speedData.bad_status_i++;                     speedData.bad_status = true;}}
    else if (serial1Data.iter_token == 4)  {if (val_accelleration_delimiter(serial1Data.token) == true)    {strcpy(speedData.acceleration_delimiter, serial1Data.token);     speedData.check_data++; speedData.bad_acceleration_delimiter = false;}     else {speedData.bad_acceleration_delimiter_i++;     speedData.bad_acceleration_delimiter = true;}}
    else if (serial1Data.iter_token == 5)  {if (val_axis_accelleration(serial1Data.token) == true)         {strcpy(speedData.acc_X, serial1Data.token);                      speedData.check_data++; speedData.bad_acc_X = false;}                      else {speedData.bad_acc_X_i++;                      speedData.bad_acc_X = true;}}
    else if (serial1Data.iter_token == 6)  {if (val_axis_accelleration(serial1Data.token) == true)         {strcpy(speedData.acc_Y, serial1Data.token);                      speedData.check_data++; speedData.bad_acc_Y = false;}                      else {speedData.bad_acc_Y_i++;                      speedData.bad_acc_Y = true;}}
    else if (serial1Data.iter_token == 7)  {if (val_axis_accelleration(serial1Data.token) == true)         {strcpy(speedData.acc_Z, serial1Data.token);                      speedData.check_data++; speedData.bad_acc_Z = false;}                      else {speedData.bad_acc_Z_i++;                      speedData.bad_acc_Z = true;}}
    else if (serial1Data.iter_token == 8)  {if (val_angular_velocity_delimiter(serial1Data.token) == true) {strcpy(speedData.angular_velocity_delimiter, serial1Data.token); speedData.check_data++; speedData.bad_angular_velocity_delimiter = false;} else {speedData.bad_angular_velocity_delimiter_i++; speedData.bad_angular_velocity_delimiter = true;}}
    else if (serial1Data.iter_token == 9)  {if (val_gyro_angular_velocity(serial1Data.token) == true)      {strcpy(speedData.gyro_X, serial1Data.token);                     speedData.check_data++; speedData.bad_gyro_X = false;}                     else {speedData.bad_gyro_X_i++;                     speedData.bad_gyro_X = true;}}
    else if (serial1Data.iter_token == 10) {if (val_gyro_angular_velocity(serial1Data.token) == true)      {strcpy(speedData.gyro_Y, serial1Data.token);                     speedData.check_data++; speedData.bad_gyro_Y = false;}                     else {speedData.bad_gyro_Y_i++;                     speedData.bad_gyro_Y = true;}}
    else if (serial1Data.iter_token == 11) {if (val_gyro_angular_velocity(serial1Data.token) == true)      {strcpy(speedData.gyro_Z, serial1Data.token);                     speedData.check_data++; speedData.bad_gyro_Z = false;}                     else {speedData.bad_gyro_Z_i++;                     speedData.bad_gyro_Z = true;}}
    else if (serial1Data.iter_token == 12) {if (val_status_delimiter(serial1Data.token) == true)           {strcpy(speedData.status_delimiter, serial1Data.token);           speedData.check_data++; speedData.bad_status_delimiter = false;}           else {speedData.bad_status_delimiter_i++;           speedData.bad_status_delimiter = true;}}
    else if (serial1Data.iter_token == 13) {if (val_ubi_state_flag(serial1Data.token) == true)             {strcpy(speedData.ubi_state_flag, serial1Data.token);             speedData.check_data++; speedData.bad_ubi_state_flag = false;}             else {speedData.bad_ubi_state_flag_i++;             speedData.bad_ubi_state_flag = true;}}
    else if (serial1Data.iter_token == 14) {if (val_ubi_state_kind_flag(serial1Data.token) == true)        {strcpy(speedData.ubi_state_kind, serial1Data.token);             speedData.check_data++; speedData.bad_ubi_state_kind = false;}             else {speedData.bad_ubi_state_kind_i++;             speedData.bad_ubi_state_kind = true;}}
    else if (serial1Data.iter_token == 15) {
      strcpy(speedData.temporary_data, strtok(serial1Data.token, "*"));
      if (val_scalable(speedData.temporary_data) == true)                                                {strcpy(speedData.ubi_state_value, speedData.temporary_data);    speedData.check_data++; speedData.bad_ubi_state_value = false;}            else {speedData.bad_ubi_state_value_i++;            speedData.bad_ubi_state_value = true;}
      serial1Data.token = strtok(NULL, "*");
      strcpy(speedData.temporary_data_1, strtok(serial1Data.token, "*"));
      if (val_checksum(speedData.temporary_data_1) == true)                                              {strcpy(speedData.check_sum, speedData.temporary_data_1);        speedData.check_data++; speedData.bad_check_sum = false;}                  else {speedData.bad_check_sum_i++;                  speedData.bad_check_sum = true;}}
    serial1Data.token = strtok(NULL, ",");
    serial1Data.iter_token++;
  }
  if (sysDebugData.speed_sentence == true) {
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
  char tag[56];                                                                              // <0> Log header
  char utc[56];            unsigned long bad_utc_i;             bool bad_utc = true;         // <1> utc time
  char code_flag[56];      unsigned long bad_code_flag_i;       bool bad_code_flag = true;   // <2> encryption chip: 1=problem, 0=normal
  char gset_flag[56];      unsigned long bad_gset_flag_i;       bool bad_gset_flag = true;   // <3> positioning chip: 1=problem, 0=normal
  char sset_flag[56];      unsigned long bad_sset_flag_i;       bool bad_sset_flag = true;   // <4> sensor chip: 1=problem, 0=normal
  char customize_0[56];    unsigned long bad_customize_0_i;     bool bad_customize_0 = true; // <5> customize 0-20
  char customize_1[56];    unsigned long bad_customize_1_i;     bool bad_customize_1 = true; // <6> customize float
  char check_sum[56];      unsigned long bad_check_sum_i;       bool bad_check_sum = true;   // <7> XOR check value of all bytes starting from $ to *
  int check_data = 0;      unsigned long bad_checksum_validity; bool valid_checksum = false; // Checksum validity bool, counters and a counter for how many elements passed further testing (gnrmc check_data should result in 8)
  char temporary_data[56];
  char temporary_data_1[56];
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
  serial1Data.iter_token = 0;
  serial1Data.token = strtok(serial1Data.BUFFER, ",");
  while( serial1Data.token != NULL ) {
    if      (serial1Data.iter_token == 0)                                                 {strcpy(errorData.tag, "ERROR");                  errorData.check_data++;}
    else if (serial1Data.iter_token == 1) {if (val_utc_time(serial1Data.token) == true)    {strcpy(errorData.utc, serial1Data.token);         errorData.check_data++; errorData.bad_utc = false;}         else {errorData.bad_utc_i++;         errorData.bad_utc = true;}}
    else if (serial1Data.iter_token == 2) {if (val_code_flag(serial1Data.token) == true)   {strcpy(errorData.code_flag, serial1Data.token);   errorData.check_data++; errorData.bad_code_flag = false;}   else {errorData.bad_code_flag_i++;   errorData.bad_code_flag = true;}}
    else if (serial1Data.iter_token == 3) {if (val_gset_flag(serial1Data.token) == true)   {strcpy(errorData.gset_flag, serial1Data.token);   errorData.check_data++; errorData.bad_gset_flag = false;}   else {errorData.bad_gset_flag_i++;   errorData.bad_gset_flag = true;}}
    else if (serial1Data.iter_token == 4) {if (val_sset_flag(serial1Data.token) == true)   {strcpy(errorData.sset_flag, serial1Data.token);   errorData.check_data++; errorData.bad_sset_flag = false;}   else {errorData.bad_sset_flag_i++;   errorData.bad_sset_flag = true;}}
    else if (serial1Data.iter_token == 5) {if (val_custom_flag(serial1Data.token) == true) {strcpy(errorData.customize_0, serial1Data.token); errorData.check_data++; errorData.bad_customize_0 = false;} else {errorData.bad_customize_0_i++; errorData.bad_customize_0 = true;}}
    else if (serial1Data.iter_token == 6) {
      strcpy(errorData.temporary_data, strtok(serial1Data.token, "*"));
      if (val_scalable(errorData.temporary_data) == true)                                {strcpy(errorData.customize_1, errorData.temporary_data); errorData.check_data++; errorData.bad_customize_1 = false;} else {errorData.bad_customize_1_i++; errorData.bad_customize_1 = true;}
      serial1Data.token = strtok(NULL, "*");
      strcpy(errorData.temporary_data_1, strtok(serial1Data.token, "*"));
      if (val_checksum(errorData.temporary_data_1) == true)                              {strcpy(errorData.check_sum, errorData.temporary_data_1); errorData.check_data++; errorData.bad_check_sum = false;}   else {errorData.bad_check_sum_i++;   errorData.bad_check_sum = true;}}
    serial1Data.token = strtok(NULL, ",");
    serial1Data.iter_token++;
  }
  if (sysDebugData.error_sentence == true) {
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
  char tag[56];                                                                                 // <0> log header
  char ang_dget_flag[56];  unsigned long bad_ang_dget_flag_i;   bool bad_ang_dget_flag = true;  // <1> installation azimuth: 1=with azimuth, 0=without azimuth
  char fix_kind_flag[56];  unsigned long bad_fix_kind_flag_i;   bool bad_fix_kind_flag = true;  // <2> type of installed coordinate system
  char ins_run_flag[56];   unsigned long bad_ins_run_flag_i;    bool bad_ins_run_flag = true;   // <3> forced ins: 1=forced, 0=normal
  char fix_roll_flag[56];  unsigned long bad_fix_roll_flag_i;   bool bad_fix_roll_flag = true;  // <4> installation roll angle
  char fix_pitch_flag[56]; unsigned long bad_fix_pitch_flag_i;  bool bad_fix_pitch_flag = true; // <5> installation pitch angle
  char ubi_on_flag[56];    unsigned long bad_ubi_on_flag_i;     bool bad_ubi_on_flag = true;    // <6> 0 to 8
  char ubi_kind_flag[56];  unsigned long bad_ubi_kind_flag_i;   bool bad_ubi_kind_flag = true;  // <7> 0=none, 1=ubi event, 2=ubi alarm
  char ubi_a_set[56];      unsigned long bad_ubi_a_set_i;       bool bad_ubi_a_set = true;      // <8> ubi a parameter setting value
  char ubi_b_set[56];      unsigned long bad_ubi_b_set_i;       bool bad_ubi_b_set = true;      // <9> ubi b parameter setting value
  char acc_X_data[56];     unsigned long bad_acc_X_data_i;      bool bad_acc_X_data = true;     // <10> vehicle longitudinal acceleration: 0.1m/s2
  char acc_Y_data[56];     unsigned long bad_acc_Y_data_i;      bool bad_acc_Y_data = true;     // <11> vehicle lateral acceleration: 0.1m/s2
  char gyro_Z_data[56];    unsigned long bad_gyro_Z_data_i;     bool bad_gyro_Z_data = true;    // <12> vehicle z axis angular velocity: degrees
  char pitch_angle[56];    unsigned long bad_pitch_angle_i;     bool bad_pitch_angle = true;    // <13> vehicle pitch angle: degrees
  char roll_angle[56];     unsigned long bad_roll_angle_i;      bool bad_roll_angle = true;     // <14> vehicle roll angle: degrees
  char yaw_angle[56];      unsigned long bad_yaw_angle_i;       bool bad_yaw_angle = true;      // <15> vehicle direction change angle: degrees
  char car_speed[56];      unsigned long bad_car_speed_i;       bool bad_car_speed = true;      // <16> vehicle speed: m/s
  char ins_flag[56];       unsigned long bad_ins_flag_i;        bool bad_ins_flag = true;       // <17> intertial navigation convergence flag
  char ubi_num[56];        unsigned long bad_ubi_num_i;         bool bad_ubi_num = true;        // <18> serial number
  char ubi_valid[56];      unsigned long bad_ubi_valid_i;       bool bad_ubi_valid = true;      // <19> ubi valid flag: 1=valid, 0=invalid
  char coll_T_data[56];    unsigned long bad_coll_T_data_i;     bool bad_coll_T_data = true;    // <20> collision factor
  char coll_T_heading[56]; unsigned long bad_coll_T_heading_i;  bool bad_coll_T_heading = true; // <21> collision direction
  char custom_logo_0[56];  unsigned long bad_custom_logo_0_i;   bool bad_custom_logo_0 = true;  // <22> 
  char custom_logo_1[56];  unsigned long bad_custom_logo_1_i;   bool bad_custom_logo_1 = true;  // <23> 
  char custom_logo_2[56];  unsigned long bad_custom_logo_2_i;   bool bad_custom_logo_2 = true;  // <24> 
  char custom_logo_3[56];  unsigned long bad_custom_logo_3_i;   bool bad_custom_logo_3 = true;  // <25> 
  char custom_logo_4[56];  unsigned long bad_custom_logo_4_i;   bool bad_custom_logo_4 = true;  // <26> 
  char custom_logo_5[56];  unsigned long bad_custom_logo_5_i;   bool bad_custom_logo_5 = true;  // <27> 
  char check_sum[56];      unsigned long bad_check_sum_i;       bool bad_check_sum = true;      // <28> XOR check value of all bytes starting from $ to *
  int check_data = 0;      unsigned long bad_checksum_validity; bool valid_checksum = false;    // Checksum validity bool, counters and a counter for how many elements passed further testing (gnrmc check_data should result in 29)
  char temporary_data[56];
  char temporary_data_1[56];
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
  serial1Data.iter_token = 0;
  serial1Data.token = strtok(serial1Data.BUFFER, ",");
  while( serial1Data.token != NULL ) {
    if      (serial1Data.iter_token == 0)                                                     {strcpy(debugData.tag, "DEBUG");                     debugData.check_data++;}
    else if (serial1Data.iter_token == 1)  {if (val_ang_dget_flag(serial1Data.token) == true)  {strcpy(debugData.ang_dget_flag, serial1Data.token);  debugData.check_data++; debugData.bad_ang_dget_flag = false;}  else {debugData.bad_ang_dget_flag_i++;  debugData.bad_ang_dget_flag = true;}}
    else if (serial1Data.iter_token == 2)  {if (val_fix_kind_flag(serial1Data.token) == true)  {strcpy(debugData.fix_kind_flag, serial1Data.token);  debugData.check_data++; debugData.bad_fix_kind_flag = false;}  else {debugData.bad_fix_kind_flag_i++;  debugData.bad_fix_kind_flag = true;}}
    else if (serial1Data.iter_token == 3)  {if (val_ins_run_flag(serial1Data.token) == true)   {strcpy(debugData.ins_run_flag, serial1Data.token);   debugData.check_data++; debugData.bad_ins_run_flag = false;}   else {debugData.bad_ins_run_flag_i++;   debugData.bad_ins_run_flag = true;}}
    else if (serial1Data.iter_token == 4)  {if (val_fix_roll_flag(serial1Data.token) == true)  {strcpy(debugData.fix_roll_flag, serial1Data.token);  debugData.check_data++; debugData.bad_fix_roll_flag = false;}  else {debugData.bad_fix_roll_flag_i++;  debugData.bad_fix_roll_flag = true;}}
    else if (serial1Data.iter_token == 5)  {if (val_fix_pitch_flag(serial1Data.token) == true) {strcpy(debugData.fix_pitch_flag, serial1Data.token); debugData.check_data++; debugData.bad_fix_pitch_flag = false;} else {debugData.bad_fix_pitch_flag_i++; debugData.bad_fix_pitch_flag = true;}}
    else if (serial1Data.iter_token == 6)  {if (val_ubi_on_flag(serial1Data.token) == true)    {strcpy(debugData.ubi_on_flag, serial1Data.token);    debugData.check_data++; debugData.bad_ubi_on_flag = false;}    else {debugData.bad_ubi_on_flag_i++;    debugData.bad_ubi_on_flag = true;}}
    else if (serial1Data.iter_token == 7)  {if (val_ubi_kind_flag(serial1Data.token) == true)  {strcpy(debugData.ubi_kind_flag, serial1Data.token);  debugData.check_data++; debugData.bad_ubi_kind_flag = false;}  else {debugData.bad_ubi_kind_flag_i++;  debugData.bad_ubi_kind_flag = true;}}
    else if (serial1Data.iter_token == 8)  {if (val_ubi_a_set(serial1Data.token) == true)      {strcpy(debugData.ubi_a_set, serial1Data.token);      debugData.check_data++; debugData.bad_ubi_a_set = false;}      else {debugData.bad_ubi_a_set_i++;      debugData.bad_ubi_a_set = true;}}
    else if (serial1Data.iter_token == 9)  {if (val_ubi_b_set(serial1Data.token) == true)      {strcpy(debugData.ubi_b_set, serial1Data.token);      debugData.check_data++; debugData.bad_ubi_b_set = false;}      else {debugData.bad_ubi_b_set_i++;      debugData.bad_ubi_b_set = true;}}
    else if (serial1Data.iter_token == 10) {if (val_acc_X_data(serial1Data.token) == true)     {strcpy(debugData.acc_X_data, serial1Data.token);     debugData.check_data++; debugData.bad_acc_X_data = false;}     else {debugData.bad_acc_X_data_i++;     debugData.bad_acc_X_data = true;}}
    else if (serial1Data.iter_token == 11) {if (val_acc_Y_data(serial1Data.token) == true)     {strcpy(debugData.acc_Y_data, serial1Data.token);     debugData.check_data++; debugData.bad_acc_Y_data = false;}     else {debugData.bad_acc_Y_data_i++;     debugData.bad_acc_Y_data = true;}}
    else if (serial1Data.iter_token == 12) {if (val_gyro_Z_data(serial1Data.token) == true)    {strcpy(debugData.gyro_Z_data, serial1Data.token);    debugData.check_data++; debugData.bad_gyro_Z_data = false;}    else {debugData.bad_gyro_Z_data_i++;    debugData.bad_gyro_Z_data = true;}}
    else if (serial1Data.iter_token == 13) {if (val_pitch_angle(serial1Data.token) == true)    {strcpy(debugData.pitch_angle, serial1Data.token);    debugData.check_data++; debugData.bad_pitch_angle = false;}    else {debugData.bad_pitch_angle_i++;    debugData.bad_pitch_angle = true;}}
    else if (serial1Data.iter_token == 14) {if (val_roll_angle(serial1Data.token) == true)     {strcpy(debugData.roll_angle, serial1Data.token);     debugData.check_data++; debugData.bad_roll_angle = false;}     else {debugData.bad_roll_angle_i++;     debugData.bad_roll_angle = true;}}
    else if (serial1Data.iter_token == 15) {if (val_yaw_angle(serial1Data.token) == true)      {strcpy(debugData.yaw_angle, serial1Data.token);      debugData.check_data++; debugData.bad_yaw_angle = false;}      else {debugData.bad_yaw_angle_i++;      debugData.bad_yaw_angle = true;}}
    else if (serial1Data.iter_token == 16) {if (val_car_speed(serial1Data.token) == true)      {strcpy(debugData.car_speed, serial1Data.token);      debugData.check_data++; debugData.bad_car_speed = false;}      else {debugData.bad_car_speed_i++;      debugData.bad_car_speed = true;}}
    else if (serial1Data.iter_token == 17) {if (val_ins_flag(serial1Data.token) == true)       {strcpy(debugData.ins_flag, serial1Data.token);       debugData.check_data++; debugData.bad_ins_flag = false;}       else {debugData.bad_ins_flag_i++;       debugData.bad_ins_flag = true;}}
    else if (serial1Data.iter_token == 18) {if (val_ubi_num(serial1Data.token) == true)        {strcpy(debugData.ubi_num, serial1Data.token);        debugData.check_data++; debugData.bad_ubi_num = false;}        else {debugData.bad_ubi_num_i++;        debugData.bad_ubi_num = true;}}
    else if (serial1Data.iter_token == 19) {if (val_ubi_valid(serial1Data.token) == true)      {strcpy(debugData.ubi_valid, serial1Data.token);      debugData.check_data++; debugData.bad_ubi_valid = false;}      else {debugData.bad_ubi_valid_i++;      debugData.bad_ubi_valid = true;}}
    else if (serial1Data.iter_token == 20) {if (val_coll_T_data(serial1Data.token) == true)    {strcpy(debugData.coll_T_data, serial1Data.token);    debugData.check_data++; debugData.bad_coll_T_data = false;}    else {debugData.bad_coll_T_data_i++;    debugData.bad_coll_T_data = true;}}
    else if (serial1Data.iter_token == 21) {if (val_coll_T_heading(serial1Data.token) == true) {strcpy(debugData.coll_T_heading, serial1Data.token); debugData.check_data++; debugData.bad_coll_T_heading = false;} else {debugData.bad_coll_T_heading_i++; debugData.bad_coll_T_heading = true;}}
    else if (serial1Data.iter_token == 22) {if (val_custom_flag(serial1Data.token) == true)    {strcpy(debugData.custom_logo_0, serial1Data.token);  debugData.check_data++; debugData.bad_custom_logo_0 = false;}  else {debugData.bad_custom_logo_0_i++;  debugData.bad_custom_logo_0 = true;}}
    else if (serial1Data.iter_token == 23) {if (val_custom_flag(serial1Data.token) == true)    {strcpy(debugData.custom_logo_1, serial1Data.token);  debugData.check_data++; debugData.bad_custom_logo_1 = false;}  else {debugData.bad_custom_logo_1_i++;  debugData.bad_custom_logo_1 = true;}}
    else if (serial1Data.iter_token == 24) {if (val_custom_flag(serial1Data.token) == true)    {strcpy(debugData.custom_logo_2, serial1Data.token);  debugData.check_data++; debugData.bad_custom_logo_2 = false;}  else {debugData.bad_custom_logo_2_i++;  debugData.bad_custom_logo_2 = true;}}
    else if (serial1Data.iter_token == 25) {if (val_custom_flag(serial1Data.token) == true)    {strcpy(debugData.custom_logo_3, serial1Data.token);  debugData.check_data++; debugData.bad_custom_logo_3 = false;}  else {debugData.bad_custom_logo_3_i++;  debugData.bad_custom_logo_3 = true;}}
    else if (serial1Data.iter_token == 26) {if (val_custom_flag(serial1Data.token) == true)    {strcpy(debugData.custom_logo_4, serial1Data.token);  debugData.check_data++; debugData.bad_custom_logo_4 = false;}  else {debugData.bad_custom_logo_4_i++;  debugData.bad_custom_logo_4 = true;}}
    else if (serial1Data.iter_token == 27) {
      strcpy(debugData.temporary_data, strtok(serial1Data.token, "*"));
      if (val_scalable(debugData.temporary_data) == true)                                    {strcpy(debugData.custom_logo_5, debugData.temporary_data); debugData.check_data++; debugData.bad_custom_logo_5 = false;} else {debugData.bad_custom_logo_5_i++; debugData.bad_custom_logo_5 = true;}
      serial1Data.token = strtok(NULL, "*");
      strcpy(debugData.temporary_data_1, strtok(serial1Data.token, "*"));
      if (val_checksum(debugData.temporary_data_1) == true)                                  {strcpy(debugData.check_sum, debugData.temporary_data_1); debugData.check_data++; debugData.bad_check_sum = false;} else {debugData.bad_check_sum_i++; debugData.bad_check_sum = true;}}
    serial1Data.token = strtok(NULL, ",");
    serial1Data.iter_token++;
  }
  if (sysDebugData.debug_sentence == true) {
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
  char checksum_str[56];
  int checksum_i;
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
  strcat(satData.satcom_sentence, "*");
  satData.checksum_i = getCheckSum(satData.satcom_sentence);
  itoa(satData.checksum_i, satData.checksum_str, 10);
  strcat(satData.satcom_sentence, satData.checksum_str);
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

void SSD_Display_3() {
  tcaselect(3);
  display_3.setTextAlignment(TEXT_ALIGN_CENTER);
  display_3.setColor(WHITE);
  display_3.clear();
  display_3.drawString(display_3.getWidth()/2, 0, "MATRIX");
  display_3.drawString(display_3.getWidth()/2, 14, "   1 2 3 4 5 6 7 8 9 10");
  display_3.drawString(display_3.getWidth()/2,24,"0 "+String(relayData.relays_bool[0][0])+" "+String(relayData.relays_bool[0][1])+" "+String(relayData.relays_bool[0][2])+" "+String(relayData.relays_bool[0][3])+" "+String(relayData.relays_bool[0][4])+" "+String(relayData.relays_bool[0][5])+" "+String(relayData.relays_bool[0][6])+" "+String(relayData.relays_bool[0][7])+" "+String(relayData.relays_bool[0][8])+" "+String(relayData.relays_bool[0][9]));
  display_3.drawString(display_3.getWidth()/2,34,"1 "+String(relayData.relays_bool[0][10])+" "+String(relayData.relays_bool[0][11])+" "+String(relayData.relays_bool[0][12])+" "+String(relayData.relays_bool[0][13])+" "+String(relayData.relays_bool[0][14])+" "+String(relayData.relays_bool[0][15])+" "+String(relayData.relays_bool[0][16])+" "+String(relayData.relays_bool[0][17])+" "+String(relayData.relays_bool[0][18])+" "+String(relayData.relays_bool[0][19]));
  display_3.drawString(display_3.getWidth()/2,44,"2 "+String(relayData.relays_bool[0][20])+" "+String(relayData.relays_bool[0][21])+" "+String(relayData.relays_bool[0][22])+" "+String(relayData.relays_bool[0][23])+" "+String(relayData.relays_bool[0][24])+" "+String(relayData.relays_bool[0][25])+" "+String(relayData.relays_bool[0][26])+" "+String(relayData.relays_bool[0][27])+" "+String(relayData.relays_bool[0][28])+" "+String(relayData.relays_bool[0][29]));
  display_3.drawString(display_3.getWidth()/2,54,"3 "+String(relayData.relays_bool[0][30])+" "+String(relayData.relays_bool[0][31])+" "+String(relayData.relays_bool[0][32])+" "+String(relayData.relays_bool[0][33])+" "+String(relayData.relays_bool[0][34])+" "+String(relayData.relays_bool[0][35])+" "+String(relayData.relays_bool[0][36])+" "+String(relayData.relays_bool[0][37])+" "+String(relayData.relays_bool[0][38])+" "+String(relayData.relays_bool[0][39]));
  display_3.display();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                            INITIALIZE SDCARD

bool init_sdcard() {
  Serial.println("[sdcard] attempting to initialize");
  if (!sd.begin(SD_CONFIG)) {
    Serial.println("[sdcard] failed to initialize");
    return false;
  }
  else {Serial.println("[sdcard] initialized successfully"); return true;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                PRINT FILE CONTENTS TO SERIAL

bool sdcard_read_to_serial(char * file) {
  sdcardData.current_file = sd.open(file);
  sdcardData.current_file.rewind();
  if (sdcardData.current_file) {
    while (sdcardData.current_file.available()) {Serial.write(sdcardData.current_file.read());}
    sdcardData.current_file.close(); return true;
  }
  else {sdcardData.current_file.close(); return false;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                 LOAD MATRIX DATA FROM SDCARD 

bool sdcard_load_matrix(char * file) {

  // ensure cleared
  memset(sdcardData.data_0, 0, 56);
  memset(sdcardData.data_1, 0, 56);
  memset(sdcardData.data_2, 0, 56);
  memset(sdcardData.data_3, 0, 56);
  memset(sdcardData.data_4, 0, 56);
  memset(sdcardData.data_5, 0, 56);
  memset(sdcardData.data_6, 0, 56);

  // open file to read
  sdcardData.current_file = sd.open(file); 
  if (sdcardData.current_file) {
    sdcardData.current_file.rewind();
    while (sdcardData.current_file.available()) {

      // read line
      sdcardData.SBUFFER = "";
      memset(sdcardData.BUFFER, 0, 2048);
      sdcardData.SBUFFER = sdcardData.current_file.readStringUntil('\n');
      sdcardData.SBUFFER.toCharArray(sdcardData.BUFFER, sdcardData.SBUFFER.length()+1);
      Serial.println("[sdcard] [reading] " + String(sdcardData.BUFFER));
      
      if (strncmp(sdcardData.BUFFER, "r", 1) == 0) {

        // split line on delimiter
        sdcardData.token = strtok(sdcardData.BUFFER, ",");

        // relay index
        sdcardData.token = strtok(NULL, ",");
        memset(sdcardData.data_0, 0, 56);
        strcpy(sdcardData.data_0, sdcardData.token);
        Serial.println("[Ri] " +String(sdcardData.data_0));

        // relay function index
        sdcardData.token = strtok(NULL, ",");
        memset(sdcardData.data_1, 0, 56);
        strcpy(sdcardData.data_1, sdcardData.token);
        Serial.println("[Fi] " +String(sdcardData.data_1));
        
        // relay function name
        sdcardData.token = strtok(NULL, ",");
        memset(sdcardData.data_2, 0, 56);
        strcpy(sdcardData.data_2, sdcardData.token);
        memset(relayData.relays[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)], 0, 56);
        strcpy(relayData.relays[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)], sdcardData.data_2);
        Serial.println("[Fn] " +String(relayData.relays[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)]));

        // relay function data: x
        sdcardData.token = strtok(NULL, ",");
        memset(sdcardData.data_3, 0, 56);
        strcpy(sdcardData.data_3, sdcardData.token);
        relayData.relays_data[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][0] = atol(sdcardData.data_3);
        Serial.println("[X] " +String(relayData.relays_data[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][0]));

        // relay function data: y
        sdcardData.token = strtok(NULL, ",");
        memset(sdcardData.data_4, 0, 56);
        strcpy(sdcardData.data_4, sdcardData.token);
        relayData.relays_data[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][1] = atol(sdcardData.data_4);
        Serial.println("[Y] " +String(relayData.relays_data[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][1]));

        // relay function data: z
        sdcardData.token = strtok(NULL, ",");
        memset(sdcardData.data_5, 0, 56);
        strcpy(sdcardData.data_5, sdcardData.token);
        relayData.relays_data[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][2] = atol(sdcardData.data_5);
        Serial.println("[Z] " +String(relayData.relays_data[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][2]));
      }

      else if (strncmp(sdcardData.BUFFER, "e", 1) == 0) {

        // split line on delimiter
        sdcardData.token = strtok(sdcardData.BUFFER, ",");

        // relay index
        sdcardData.token = strtok(NULL, ",");
        memset(sdcardData.data_0, 0, 56);
        strcpy(sdcardData.data_0, sdcardData.token);
        Serial.println("[Ri] " +String(sdcardData.data_0));

        // relay enabled/disabled
        sdcardData.token = strtok(NULL, ",");
        memset(sdcardData.data_6, 0, 56);
        strcpy(sdcardData.data_6, sdcardData.token);
        relayData.relays_data[atoi(sdcardData.data_0)][10][0] = (int)atoi(sdcardData.data_6);
        Serial.println("[E] " + String(relayData.relays_data[atoi(sdcardData.data_0)][10][0]));

        // ensure cleared
        memset(sdcardData.data_0, 0, 56);
        memset(sdcardData.data_1, 0, 56);
        memset(sdcardData.data_2, 0, 56);
        memset(sdcardData.data_3, 0, 56);
        memset(sdcardData.data_4, 0, 56);
        memset(sdcardData.data_5, 0, 56);
        memset(sdcardData.data_6, 0, 56);
      }
    }
    sdcardData.current_file.close();
    return true;
  }
  else {sdcardData.current_file.close(); return false;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                  WRITE MATRIX DATA TO SDCARD 

bool sdcard_write_matrix(char * file) {
  // sdcardData
  sdcardData.current_file = sd.open(file, FILE_WRITE);
  sdcardData.current_file.rewind();
  if (sdcardData.current_file) {
    for (int Ri = 0; Ri < relayData.MAX_RELAYS; Ri++) {
      for (int Fi = 0; Fi < relayData.MAX_RELAY_ELEMENTS; Fi++) {
        memset(sdcardData.file_data, 0 , 256);
        // tag 0
        strcat(sdcardData.file_data, sdcardData.tag_0); strcat(sdcardData.file_data, sdcardData.delim);
        // Ri
        memset(sdcardData.tmp, 0 , 256);
        sprintf(sdcardData.tmp, "%d", Ri);
        strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
        // Fi
        memset(sdcardData.tmp, 0 , 256);
        sprintf(sdcardData.tmp, "%d", Fi);
        strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
        // function name
        strcat(sdcardData.file_data, relayData.relays[Ri][Fi]); strcat(sdcardData.file_data, sdcardData.delim);
        // function value x
        memset(sdcardData.tmp, 0 , 256);
        sprintf(sdcardData.tmp, "%f", relayData.relays_data[Ri][Fi][0]);
        strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
        // function value y
        memset(sdcardData.tmp, 0 , 256);
        sprintf(sdcardData.tmp, "%f", relayData.relays_data[Ri][Fi][1]);
        strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
        // function value z
        memset(sdcardData.tmp, 0 , 256);
        sprintf(sdcardData.tmp, "%f", relayData.relays_data[Ri][Fi][2]);
        strcat(sdcardData.file_data, sdcardData.tmp);
        // write line
        Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
        sdcardData.current_file.println(sdcardData.file_data);
      }
      memset(sdcardData.file_data, 0 , 256);
      // tag 1
      strcat(sdcardData.file_data, sdcardData.tag_1); strcat(sdcardData.file_data, sdcardData.delim);
      // Ri
      memset(sdcardData.tmp, 0 , 256);
      sprintf(sdcardData.tmp, "%d", Ri);
      strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
      // Ri enabled 0/1
      memset(sdcardData.tmp, 0 , 256);
      itoa(relayData.relays_data[Ri][10][0], sdcardData.tmp, 10);
      strcat(sdcardData.file_data, sdcardData.tmp);
      // write line
      Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
      sdcardData.current_file.println(sdcardData.file_data);
    }
    sdcardData.current_file.close();
    return true;
  }
  else {sdcardData.current_file.close(); return false;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                             MATRIX SET ENTRY

/*
                                        R F Function Name              X Y Z Enable/Disable 
example test command: $MATRIX_SET_ENTRY,0,0,satellite_count_gngga_over,1,0,0,1
example test command: $MATRIX_SET_ENTRY,0,0,satellite_count_gngga_over,-1,0,0,0
clear test command:   $MATRIX_SET_ENTRY,0,0,$NONE,0,0,0,0
*/

void matrix_set_entry() {
  Serial.println("[RXD0_matrix_interface] connected");
  serial0Data.check_data_R = 0;
  memset(serial0Data.data_0, 0, 56);
  memset(serial0Data.data_1, 0, 56);
  memset(serial0Data.data_2, 0, 56);
  memset(serial0Data.data_3, 0, 56);
  memset(serial0Data.data_4, 0, 56);
  memset(serial0Data.data_5, 0, 56);
  memset(serial0Data.data_6, 0, 56);
  serial0Data.iter_token = 0;
  serial0Data.token = strtok(serial0Data.BUFFER, ",");
  while( serial0Data.token != NULL ) {
    if      (serial0Data.iter_token == 0) {}
    else if (serial0Data.iter_token == 1) {strcpy(serial0Data.data_0, serial0Data.token);} // rn
    else if (serial0Data.iter_token == 2) {strcpy(serial0Data.data_1, serial0Data.token);} // fn
    else if (serial0Data.iter_token == 3) {strcpy(serial0Data.data_2, serial0Data.token);} // function
    else if (serial0Data.iter_token == 4) {strcpy(serial0Data.data_3, serial0Data.token);} // x
    else if (serial0Data.iter_token == 5) {strcpy(serial0Data.data_4, serial0Data.token);} // y
    else if (serial0Data.iter_token == 6) {strcpy(serial0Data.data_5, serial0Data.token);} // z
    else if (serial0Data.iter_token == 7) {strcpy(serial0Data.data_6, serial0Data.token);} // 0/1
    serial0Data.token = strtok(NULL, ",");
    serial0Data.iter_token++;
  }
  if (sysDebugData.serial_0_sentence == true) {
    Serial.println("[serial0Data.data_0] "         + String(serial0Data.data_0));
    Serial.println("[serial0Data.data_1] "         + String(serial0Data.data_1));
    Serial.println("[serial0Data.data_2] "         + String(serial0Data.data_2));
    Serial.println("[serial0Data.data_3] "         + String(serial0Data.data_3));
    Serial.println("[serial0Data.data_4] "         + String(serial0Data.data_4));
    Serial.println("[serial0Data.data_5] "         + String(serial0Data.data_5));
    Serial.println("[serial0Data.data_6] "         + String(serial0Data.data_6));
  }
  //                      [           RN          ][          FN            ][      VALUE        ]
  strcpy(relayData.relays[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)], serial0Data.data_2);      // set function
  relayData.relays_data[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)][0]=atol(serial0Data.data_3); // set function value x
  relayData.relays_data[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)][1]=atol(serial0Data.data_4); // set function value y
  relayData.relays_data[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)][2]=atol(serial0Data.data_5); // set function value z
  relayData.relays_data[atoi(serial0Data.data_0)][10][0]                      =atol(serial0Data.data_6); // set enable/disable

  Serial.println("[Fi] " +String(serial0Data.data_0));
  Serial.println("[Fi] " +String(serial0Data.data_1));
  Serial.println("[Fn] " +String(relayData.relays[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)]));
  Serial.println("[X] " +String(relayData.relays_data[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)][0]));
  Serial.println("[Y] " +String(relayData.relays_data[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)][1]));
  Serial.println("[Z] " +String(relayData.relays_data[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)][2]));
  Serial.println("[E] " + String(relayData.relays_data[atoi(serial0Data.data_0)][10][0]));
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                              MATRIX OVERRIDE

// disable all matrix entries.

void matrix_override() {for (int Ri = 0; Ri < relayData.MAX_RELAYS; Ri++) {relayData.relays_data[Ri][10][0]=0;}}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                            MATRIX ENABLE ALL


// enable all matrix entries. will result in warnings for matrix entries with no function(s) set. this is expected, required and desirable behaviour

void matrix_enable_all() {for (int Ri = 0; Ri < relayData.MAX_RELAYS; Ri++) {relayData.relays_data[Ri][10][0]=1;}}


// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        SETUP

void setup() {

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                               SETUP SERIAL

  Serial.begin(115200);
  Serial1.begin(115200); // ( RXD from WTGPS300P's TXD. io26 on ESP32 )

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                 SETUP WIRE

  Wire.begin();

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                               SETUP SDCARD

  init_sdcard();
  sdcard_load_matrix("matrix.txt");

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
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                    MATRIX CHECKS: PRIMITIVES

// calculate if x1 in range of specified x0 +- ( specified range / 2 )
bool in_range_check(double n0, double n1, double r) {
  if (n0  >=  n1 - r/2) {if (n0  <= n1 + r/2) {return true;}}
  else {return false;}
}

bool in_ranges_check(double x0, double x1, double y0, double y1, double r) {
  // Serial.println("[CHECKING] in_ranges_check");
  if (in_range_check(x0, x1, r) == true) {
    if (in_range_check(y0, y1, r) == true) {return true;}}
  else {return false;}
}

bool check_over(char * Fn, int Ri, int Fi) {
  // Serial.println("[CHECKING] " + String(Fn) + " > " + String(relayData.relays_data[Ri][Fi][0]));
  if (atol(Fn) > relayData.relays_data[Ri][Fi][0]) {return true;}
  else {return false;}
}

bool check_under(char * Fn, int Ri, int Fi) {
  // Serial.println("[CHECKING] " + String(Fn) + " < " + String(relayData.relays_data[Ri][Fi][0]));
  if (atol(Fn) < relayData.relays_data[Ri][Fi][0]) {return true;}
  else {return false;}
}

bool check_equal(char * Fn, int Ri, int Fi) {
  // Serial.println("[CHECKING] " + String(Fn) + " == " + String(relayData.relays_data[Ri][Fi][0]));
  if (atol(Fn) == relayData.relays_data[Ri][Fi][0]) {return true;}
  else {return false;}
}

// check range from specified x to specify y
bool check_in_range(char * Fn, int Ri, int Fi) {
  // Serial.println("[CHECKING] " + String(Fn) + " > " + String(relayData.relays_data[Ri][Fi][0]) + " && " + String(Fn) + " < " + String(relayData.relays_data[Ri][Fi][1]));
  if ((atol(Fn) >= relayData.relays_data[Ri][Fi][0]) && (atol(Fn) <= relayData.relays_data[Ri][Fi][1])) {return true;}
  else {return false;}
}

bool check_strncmp_true(char * C0, char * C1, int N) {
  // Serial.println("[CHECKING] " + String(C0) + " == " + String(C1));
  if (strncmp(C0, C1, N) == 0) {return true;}
  else {return false;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                 CHECKS: BOOL

/*
the ploynomial matrix switch requires all true to result in a final truth. two polynomials can be checking the
same bool in a way that always returns true no matter which polynomial is true. this allows us to check say current validity of a
value, return true to either polynomial in the matrix and potentially do something differently. one polynomial is checking for
is false while another polynimial is checking for is true. example: fallback from sat nav to trained INS, no trained INS, fallback
to other sensors etc.
*/

bool is_false(bool _bool) {
  if (_bool == false) {return true;} else {return false;}
}

bool is_true(bool _bool) {
  if (_bool == true) {return true;} else {return false;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                            CHECKS: EQUAL NUM

bool is_N_false(int n0, int n1) {
  if (n0 != n1) {return true;} else {return false;}
}

bool is_N_true(int n0, int n1) {
  if (n0 == n1) {return true;} else {return false;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                   PRELIMINARY SYSTEMS CHECKS

bool preliminary_check() {
  validData.preliminary_check = true;
  Serial.println("[preliminary_check.gnggaData.check_data]     " + String(gnggaData.check_data) + "/16");
  Serial.println("[preliminary_check.gnggaData.valid_checksum] " + String(gnggaData.valid_checksum));
  Serial.println("[preliminary_check.gnrmcData.check_data]     " + String(gnrmcData.check_data) + "/14");
  Serial.println("[preliminary_check.gnrmcData.valid_checksum] " + String(gnrmcData.valid_checksum));
  Serial.println("[preliminary_check.gpattData.check_data]     " + String(gpattData.check_data) + "/41");
  Serial.println("[preliminary_check.gpattData.valid_checksum] " + String(gpattData.valid_checksum));
  if (gnggaData.check_data != 16) {validData.preliminary_check = false;}
  if (gnggaData.valid_checksum == false) {validData.preliminary_check = false;}
  if (gnrmcData.check_data != 14) {validData.preliminary_check = false;}
  if (gnrmcData.valid_checksum == false) {validData.preliminary_check = false;}
  if (gpattData.check_data != 41) {validData.preliminary_check = false;}
  if (gpattData.valid_checksum == false) {validData.preliminary_check = false;}
  return validData.preliminary_check;
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                MATRIX SWITCH

void matrixSwitch() {

  /*
  compound conditions are checked, each resulting in zero/one at the final_bool. This currently allows for sextillions of combinations with
  the current data alone.
  */

  // iterate over each relay matrix
  for (int Ri = 0; Ri < relayData.MAX_RELAYS; Ri++) {

    if (relayData.relays_data[Ri][10][0] == 1) {

      // temporary switch must be zero each time
      bool tmp_matrix[relayData.MAX_RELAY_ELEMENTS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      int count_none_function = 0;

      // iterate over each function name in the current relay matrix
      for (int Fi = 0; Fi < relayData.MAX_RELAY_ELEMENTS; Fi++) {

        // uncomment to debug
        // Serial.println("[Ri] " + String(Ri));
        // Serial.println("[Fi] " + String(Fi));
        // Serial.println("[relayData.relays[Ri][Fi]] " + String(relayData.relays[Ri][Fi]));

        // for perfromance reasons logic may prefer adding functions from position zero else if position zero not populated then break to next inner matrix
        if ((strcmp(relayData.relays[Ri][Fi], relayData.default_relay_function) == 0) && (Fi == 0)) {break;}

        // put true in temporary matrix for functions set to none. there is one check to catch you if you do soft enable with no functions set.
        else if (strcmp(relayData.relays[Ri][Fi], relayData.default_relay_function) == 0) {tmp_matrix[Fi] = 1; count_none_function++;}

        // put true in temporary matrix if switch is enabled regardless of data. allows final bool true with no further requirements, even if all set $ENABLED, unlike if all set $NONE
        else if (strcmp(relayData.relays[Ri][Fi], relayData.default_enable_relay_function) == 0) {tmp_matrix[Fi] = 1;}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                       SYSTEMS CHECKS: SATCOM

        // SATCOM: GNGGA
        else if (strcmp(relayData.relays[Ri][Fi], relayData.latitude_satcom_gngga_over) == 0) {tmp_matrix[Fi] = check_over(satData.location_latitude_gngga_str, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.latitude_satcom_gngga_under) == 0) {tmp_matrix[Fi] = check_under(satData.location_latitude_gngga_str, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.latitude_satcom_gngga_equal) == 0) {tmp_matrix[Fi] = check_equal(satData.location_latitude_gngga_str, Ri, Fi);}
        // check latitude range: matrix {x, z}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.latitude_satcom_gngga_in_range) == 0) {tmp_matrix[Fi] = in_range_check(satData.location_latitude_gngga, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);} // is n in [2]range of [0]x

        else if (strcmp(relayData.relays[Ri][Fi], relayData.longitude_satcom_gngga_over) == 0) {tmp_matrix[Fi] = check_over(satData.location_longitude_gngga_str, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.longitude_satcom_gngga_under) == 0) {tmp_matrix[Fi] = check_under(satData.location_longitude_gngga_str, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.longitude_satcom_gngga_equal) == 0) {tmp_matrix[Fi] = check_equal(satData.location_longitude_gngga_str, Ri, Fi);}
        // check longitude range: matrix {x, z}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.longitude_satcom_gngga_in_range) == 0) {tmp_matrix[Fi] = in_range_check(satData.location_longitude_gngga, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);} // is n in [2]range of [0]x
        // check latitude and longitude in range: matrix {x, y, z}
        //                                                                                                                               x0                              x1                                 y0                               y1                               r   
        else if (strcmp(relayData.relays[Ri][Fi], relayData.satcom_in_range_gngga) == 0) {tmp_matrix[Fi] = in_ranges_check(satData.location_latitude_gngga, relayData.relays_data[Ri][Fi][0], satData.location_longitude_gngga, relayData.relays_data[Ri][Fi][1], relayData.relays_data[Ri][Fi][2]);}

        // SATCOM: GNRMC
        else if (strcmp(relayData.relays[Ri][Fi], relayData.latitude_satcom_gnrmc_over) == 0) {tmp_matrix[Fi] = check_over(satData.location_latitude_gnrmc_str, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.latitude_satcom_gnrmc_under) == 0) {tmp_matrix[Fi] = check_under(satData.location_latitude_gnrmc_str, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.latitude_satcom_gnrmc_equal) == 0) {tmp_matrix[Fi] = check_equal(satData.location_latitude_gnrmc_str, Ri, Fi);}
        // check latitude range: matrix {x, z}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.latitude_satcom_gnrmc_in_range) == 0) {tmp_matrix[Fi] = in_range_check(satData.location_latitude_gnrmc, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);} // is n in [2]range of [0]x

        else if (strcmp(relayData.relays[Ri][Fi], relayData.longitude_satcom_gnrmc_over) == 0) {tmp_matrix[Fi] = check_over(satData.location_longitude_gnrmc_str, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.longitude_satcom_gnrmc_under) == 0) {tmp_matrix[Fi] = check_under(satData.location_longitude_gnrmc_str, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.longitude_satcom_gnrmc_equal) == 0) {tmp_matrix[Fi] = check_equal(satData.location_longitude_gnrmc_str, Ri, Fi);}
        // check longitude range: matrix {x, z}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.longitude_satcom_gnrmc_in_range) == 0) {tmp_matrix[Fi] = in_range_check(satData.location_longitude_gnrmc, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);} // is n in [2]range of [0]x
        // check latitude and longitude in range: matrix {x, y, z}
        //                                                                                                                               x0                              x1                                 y0                               y1                               r   
        else if (strcmp(relayData.relays[Ri][Fi], relayData.satcom_in_range_gngga) == 0) {tmp_matrix[Fi] = in_ranges_check(satData.location_latitude_gnrmc, relayData.relays_data[Ri][Fi][0], satData.location_longitude_gnrmc, relayData.relays_data[Ri][Fi][1], relayData.relays_data[Ri][Fi][2]);}



        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                        SYSTEMS CHECKS: GNGGA

        else if (strcmp(relayData.relays[Ri][Fi], relayData.latitude_gngga_over) == 0) {tmp_matrix[Fi] = check_over(gnggaData.latitude, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.latitude_gngga_under) == 0) {tmp_matrix[Fi] = check_under(gnggaData.latitude, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.latitude_gngga_equal) == 0) {tmp_matrix[Fi] = check_equal(gnggaData.latitude, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.latitude_gngga_in_range) == 0) {tmp_matrix[Fi] = check_in_range(gnggaData.latitude, Ri, Fi);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.longitude_gngga_over) == 0) {tmp_matrix[Fi] = check_over(gnggaData.longitude, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.longitude_gngga_under) == 0) {tmp_matrix[Fi] = check_under(gnggaData.longitude, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.longitude_gngga_equal) == 0) {tmp_matrix[Fi] = check_equal(gnggaData.longitude, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.longitude_gngga_in_range) == 0) {tmp_matrix[Fi] = check_in_range(gnggaData.longitude, Ri, Fi);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_time_gngga_over) == 0) {tmp_matrix[Fi] = check_over(gnggaData.utc_time, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_time_gngga_under) == 0) {tmp_matrix[Fi] = check_under(gnggaData.utc_time, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_time_gngga_equal) == 0) {tmp_matrix[Fi] = check_equal(gnggaData.utc_time, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_time_gngga_in_range) == 0) {tmp_matrix[Fi] = check_in_range(gnggaData.utc_time, Ri, Fi);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.positioning_status_gngga_equal) == 0) {tmp_matrix[Fi] = check_equal(gnggaData.positioning_status, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.satellite_count_gngga_over) == 0) {tmp_matrix[Fi] = check_over(gnggaData.satellite_count_gngga, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.satellite_count_gngga_under) == 0) {tmp_matrix[Fi] = check_under(gnggaData.satellite_count_gngga, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.satellite_count_gngga_equal) == 0) {tmp_matrix[Fi] = check_equal(gnggaData.satellite_count_gngga, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.satellite_count_gngga_in_range) == 0) {tmp_matrix[Fi] = check_in_range(gnggaData.satellite_count_gngga, Ri, Fi);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.hemisphere_gngga_N) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnggaData.latitude_hemisphere, "N", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.hemisphere_gngga_E) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnggaData.longitude_hemisphere, "E", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.hemisphere_gngga_S) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnggaData.latitude_hemisphere, "S", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.hemisphere_gngga_W) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnggaData.longitude_hemisphere, "W", 1);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.hdop_precision_factor_gngga_over) == 0) {tmp_matrix[Fi] = check_over(gnggaData.hdop_precision_factor, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.hdop_precision_factor_gngga_under) == 0) {tmp_matrix[Fi] = check_under(gnggaData.hdop_precision_factor, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.hdop_precision_factor_gngga_equal) == 0) {tmp_matrix[Fi] = check_equal(gnggaData.hdop_precision_factor, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.hdop_precision_factor_gngga_in_range) == 0) {tmp_matrix[Fi] = check_in_range(gnggaData.hdop_precision_factor, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.altitude_gngga_over) == 0) {tmp_matrix[Fi] = check_over(gnggaData.altitude, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.altitude_gngga_under) == 0) {tmp_matrix[Fi] = check_under(gnggaData.altitude, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.altitude_gngga_equal) == 0) {tmp_matrix[Fi] = check_equal(gnggaData.altitude, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.altitude_gngga_in_range) == 0) {tmp_matrix[Fi] = check_in_range(gnggaData.altitude, Ri, Fi);}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                        SYSTEMS CHECKS: GNRMC

        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_time_gnrmc_over) == 0) {tmp_matrix[Fi] = check_over(gnrmcData.utc_time, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_time_gnrmc_under) == 0) {tmp_matrix[Fi] = check_under(gnrmcData.utc_time, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_time_gnrmc_equal) == 0) {tmp_matrix[Fi] = check_equal(gnrmcData.utc_time, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_time_gnrmc_in_range) == 0) {tmp_matrix[Fi] = check_in_range(gnrmcData.utc_time, Ri, Fi);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.latitude_gnrmc_over) == 0) {tmp_matrix[Fi] = check_over(gnrmcData.latitude, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.latitude_gnrmc_under) == 0) {tmp_matrix[Fi] = check_under(gnrmcData.latitude, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.latitude_gnrmc_equal) == 0) {tmp_matrix[Fi] = check_equal(gnrmcData.latitude, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.latitude_gnrmc_in_range) == 0) {tmp_matrix[Fi] = check_in_range(gnrmcData.latitude, Ri, Fi);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.longitude_gnrmc_over) == 0) {tmp_matrix[Fi] = check_over(gnrmcData.longitude, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.longitude_gnrmc_under) == 0) {tmp_matrix[Fi] = check_under(gnrmcData.longitude, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.longitude_gnrmc_equal) == 0) {tmp_matrix[Fi] = check_equal(gnrmcData.longitude, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.longitude_gnrmc_in_range) == 0) {tmp_matrix[Fi] = check_in_range(gnrmcData.longitude, Ri, Fi);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.hemisphere_gnrmc_N) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.latitude_hemisphere, "N", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.hemisphere_gnrmc_E) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.longitude_hemisphere, "E", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.hemisphere_gnrmc_S) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.latitude_hemisphere, "S", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.hemisphere_gnrmc_W) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.longitude_hemisphere, "W", 1);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.ground_speed_gnrmc_over) == 0) {tmp_matrix[Fi] = check_over(gnrmcData.ground_speed, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ground_speed_gnrmc_under) == 0) {tmp_matrix[Fi] = check_under(gnrmcData.ground_speed, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ground_speed_gnrmc_equal) == 0) {tmp_matrix[Fi] = check_equal(gnrmcData.ground_speed, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ground_speed_gnrmc_in_range) == 0) {tmp_matrix[Fi] = check_in_range(gnrmcData.ground_speed, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.heading_gnrmc_over) == 0) {tmp_matrix[Fi] = check_over(gnrmcData.ground_heading, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.heading_gnrmc_under) == 0) {tmp_matrix[Fi] = check_under(gnrmcData.ground_heading, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.heading_gnrmc_equal) == 0) {tmp_matrix[Fi] = check_equal(gnrmcData.ground_heading, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.heading_gnrmc_in_range) == 0) {tmp_matrix[Fi] = check_in_range(gnrmcData.ground_heading, Ri, Fi);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_date_gnrmc_over) == 0) {tmp_matrix[Fi] = check_over(gnrmcData.utc_date, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_date_gnrmc_under) == 0) {tmp_matrix[Fi] = check_under(gnrmcData.utc_date, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_date_gnrmc_equal) == 0) {tmp_matrix[Fi] = check_equal(gnrmcData.utc_date, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_date_gnrmc_in_range) == 0) {tmp_matrix[Fi] = check_in_range(gnrmcData.utc_date, Ri, Fi);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.positioning_status_gnrmc_equal_A) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.positioning_status, "A", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.positioning_status_gnrmc_equal_V) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.positioning_status, "V", 1);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.mode_indication_gnrmc_equal_A) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.mode_indication, "A", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.mode_indication_gnrmc_equal_D) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.mode_indication, "D", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.mode_indication_gnrmc_equal_N) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.mode_indication, "N", 1);}


        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                        SYSTEMS CHECKS: GPATT

        else if (strcmp(relayData.relays[Ri][Fi], relayData.pitch_gpatt_over) == 0) {tmp_matrix[Fi] = check_over(gpattData.pitch, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.pitch_gpatt_under) == 0) {tmp_matrix[Fi] = check_under(gpattData.pitch, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.pitch_gpatt_in_range) == 0) {tmp_matrix[Fi] = check_in_range(gpattData.pitch, Ri, Fi);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.roll_gpatt_over) == 0) {tmp_matrix[Fi] = check_over(gpattData.roll, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.roll_gpatt_under) == 0) {tmp_matrix[Fi] = check_under(gpattData.roll, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.roll_gpatt_equal) == 0) {tmp_matrix[Fi] = check_equal(gpattData.roll, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.roll_gpatt_in_range) == 0) {tmp_matrix[Fi] = check_in_range(gpattData.roll, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.yaw_gpatt_over) == 0) {tmp_matrix[Fi] = check_over(gpattData.yaw, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.yaw_gpatt_under) == 0) {tmp_matrix[Fi] = check_under(gpattData.yaw, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.yaw_gpatt_equal) == 0) {tmp_matrix[Fi] = check_equal(gpattData.yaw, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.yaw_gpatt_in_range) == 0) {tmp_matrix[Fi] = check_in_range(gpattData.yaw, Ri, Fi);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.gst_data_gpatt_over) == 0) {tmp_matrix[Fi] = check_over(gpattData.gst_data, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gst_data_gpatt_under) == 0) {tmp_matrix[Fi] = check_under(gpattData.gst_data, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gst_data_gpatt_equal) == 0) {tmp_matrix[Fi] = check_equal(gpattData.gst_data, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gst_data_gpatt_in_range) == 0) {tmp_matrix[Fi] = check_in_range(gpattData.gst_data, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.mileage_gpatt_over) == 0) {tmp_matrix[Fi] = check_over(gpattData.mileage, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.mileage_gpatt_under) == 0) {tmp_matrix[Fi] = check_under(gpattData.mileage, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.mileage_gpatt_equal) == 0) {tmp_matrix[Fi] = check_equal(gpattData.mileage, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.mileage_gpatt_in_range) == 0) {tmp_matrix[Fi] = check_in_range(gpattData.mileage, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.speed_num_gpatt_over) == 0) {tmp_matrix[Fi] = check_over(gpattData.speed_num, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.speed_num_gpatt_under) == 0) {tmp_matrix[Fi] = check_under(gpattData.speed_num, Ri, Fi);}  
        else if (strcmp(relayData.relays[Ri][Fi], relayData.speed_num_gpatt_equal) == 0) {tmp_matrix[Fi] = check_equal(gpattData.speed_num, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.speed_num_gpatt_in_range) == 0) {tmp_matrix[Fi] = check_in_range(gpattData.speed_num, Ri, Fi);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.line_flag_gpatt_equal) == 0) {tmp_matrix[Fi] = check_equal(gpattData.line_flag, Ri, Fi);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.run_inetial_flag_gpatt_equal) == 0) {tmp_matrix[Fi] = check_equal(gpattData.run_inetial_flag, Ri, Fi);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.ins_gpatt_equal) == 0) {tmp_matrix[Fi] = check_equal(gpattData.ins, Ri, Fi);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.run_state_flag_gpatt_equal) == 0) {tmp_matrix[Fi] = check_equal(gpattData.run_state_flag, Ri, Fi);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.static_flag_gpatt_equal) == 0) {tmp_matrix[Fi] = check_equal(gpattData.static_flag, Ri, Fi);}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                        SYSTEMS CHECKS: SPEED

        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_state_value_speed_over) == 0) {tmp_matrix[Fi] = check_over(speedData.ubi_state_value, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_state_value_speed_under) == 0) {tmp_matrix[Fi] = check_under(speedData.ubi_state_value, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_state_value_speed_equal) == 0) {tmp_matrix[Fi] = check_equal(speedData.ubi_state_value, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_state_value_speed_in_range) == 0) {tmp_matrix[Fi] = check_in_range(speedData.ubi_state_value, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_state_kind_speed_over) == 0) {tmp_matrix[Fi] = check_over(speedData.ubi_state_kind, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_state_kind_speed_under) == 0) {tmp_matrix[Fi] = check_under(speedData.ubi_state_kind, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_state_kind_speed_equal) == 0) {tmp_matrix[Fi] = check_equal(speedData.ubi_state_kind, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_state_kind_speed_in_range) == 0) {tmp_matrix[Fi] = check_in_range(speedData.ubi_state_kind, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_state_flag_speed_over) == 0) {tmp_matrix[Fi] = check_over(speedData.ubi_state_flag, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_state_flag_speed_under) == 0) {tmp_matrix[Fi] = check_under(speedData.ubi_state_flag, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_state_flag_speed_equal) == 0) {tmp_matrix[Fi] = check_equal(speedData.ubi_state_flag, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_state_flag_speed_in_range) == 0) {tmp_matrix[Fi] = check_in_range(speedData.ubi_state_flag, Ri, Fi);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_Z_speed_over) == 0) {tmp_matrix[Fi] = check_over(speedData.gyro_Z, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_Z_speed_under) == 0) {tmp_matrix[Fi] = check_under(speedData.gyro_Z, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_Z_speed_equal) == 0) {tmp_matrix[Fi] = check_equal(speedData.gyro_Z, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_Z_speed_in_range) == 0) {tmp_matrix[Fi] = check_in_range(speedData.gyro_Z, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_Y_speed_over) == 0) {tmp_matrix[Fi] = check_over(speedData.gyro_Y, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_Y_speed_under) == 0) {tmp_matrix[Fi] = check_under(speedData.gyro_Y, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_Y_speed_equal) == 0) {tmp_matrix[Fi] = check_equal(speedData.gyro_Y, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_Y_speed_in_range) == 0) {tmp_matrix[Fi] = check_in_range(speedData.gyro_Y, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_X_speed_over) == 0) {tmp_matrix[Fi] = check_over(speedData.gyro_X, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_X_speed_under) == 0) {tmp_matrix[Fi] = check_under(speedData.gyro_X, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_X_speed_equal) == 0) {tmp_matrix[Fi] = check_equal(speedData.gyro_X, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_X_speed_in_range) == 0) {tmp_matrix[Fi] = check_in_range(speedData.gyro_X, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_Z_speed_over) == 0) {tmp_matrix[Fi] = check_over(speedData.acc_Z, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_Z_speed_under) == 0) {tmp_matrix[Fi] = check_under(speedData.acc_Z, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_Z_speed_equal) == 0) {tmp_matrix[Fi] = check_equal(speedData.acc_Z, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_Z_speed_in_range) == 0) {tmp_matrix[Fi] = check_in_range(speedData.acc_Z, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_Y_speed_over) == 0) {tmp_matrix[Fi] = check_over(speedData.acc_Y, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_Y_speed_under) == 0) {tmp_matrix[Fi] = check_under(speedData.acc_Y, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_Y_speed_equal) == 0) {tmp_matrix[Fi] = check_equal(speedData.acc_Y, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_Y_speed_in_range) == 0) {tmp_matrix[Fi] = check_in_range(speedData.acc_Y, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_X_speed_over) == 0) {tmp_matrix[Fi] = check_over(speedData.acc_X, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_X_speed_under) == 0) {tmp_matrix[Fi] = check_under(speedData.acc_X, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_X_speed_equal) == 0) {tmp_matrix[Fi] = check_equal(speedData.acc_X, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_X_speed_in_range) == 0) {tmp_matrix[Fi] = check_in_range(speedData.acc_X, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.status_speed_over) == 0) {tmp_matrix[Fi] = check_over(speedData.status, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.status_speed_under) == 0) {tmp_matrix[Fi] = check_under(speedData.status, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.status_speed_equal) == 0) {tmp_matrix[Fi] = check_equal(speedData.status, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.status_speed_in_range) == 0) {tmp_matrix[Fi] = check_in_range(speedData.status, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ground_speed_speed_over) == 0) {tmp_matrix[Fi] = check_over(speedData.speed, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ground_speed_speed_under) == 0) {tmp_matrix[Fi] = check_under(speedData.speed, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ground_speed_speed_equal) == 0) {tmp_matrix[Fi] = check_equal(speedData.speed, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ground_speed_speed_in_range) == 0) {tmp_matrix[Fi] = check_in_range(speedData.speed, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_time_speed_over) == 0) {tmp_matrix[Fi] = check_over(speedData.utc_time, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_time_speed_under) == 0) {tmp_matrix[Fi] = check_under(speedData.utc_time, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_time_speed_equal) == 0) {tmp_matrix[Fi] = check_equal(speedData.utc_time, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_time_speed_in_range) == 0) {tmp_matrix[Fi] = check_in_range(speedData.utc_time, Ri, Fi);}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                        SYSTEMS CHECKS: ERROR
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gset_flag_error_equal) == 0) {tmp_matrix[Fi] = check_over(errorData.gset_flag, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.sset_flag_error_equal) == 0) {tmp_matrix[Fi] = check_under(errorData.sset_flag, Ri, Fi);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.code_flag_error_over) == 0) {tmp_matrix[Fi] = check_over(errorData.code_flag, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.code_flag_error_under) == 0) {tmp_matrix[Fi] = check_under(errorData.code_flag, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.code_flag_error_equal) == 0) {tmp_matrix[Fi] = check_equal(errorData.code_flag, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.code_flag_error_in_range) == 0) {tmp_matrix[Fi] = check_in_range(errorData.code_flag, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_time_error_over) == 0) {tmp_matrix[Fi] = check_over(errorData.utc, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_time_error_under) == 0) {tmp_matrix[Fi] = check_under(errorData.utc, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_time_error_equal) == 0) {tmp_matrix[Fi] = check_equal(errorData.utc, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.utc_time_error_in_range) == 0) {tmp_matrix[Fi] = check_in_range(errorData.utc, Ri, Fi);}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                        SYSTEMS CHECKS: DEBUG
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.coll_T_heading_debug_over) == 0) {tmp_matrix[Fi] = check_over(debugData.coll_T_heading, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.coll_T_heading_debug_under) == 0) {tmp_matrix[Fi] = check_under(debugData.coll_T_heading, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.coll_T_heading_debug_equal) == 0) {tmp_matrix[Fi] = check_equal(debugData.coll_T_heading, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.coll_T_heading_debug_in_range) == 0) {tmp_matrix[Fi] = check_in_range(debugData.coll_T_heading, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.coll_T_data_debug_over) == 0) {tmp_matrix[Fi] = check_over(debugData.coll_T_data, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.coll_T_data_debug_under) == 0) {tmp_matrix[Fi] = check_under(debugData.coll_T_data, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.coll_T_data_debug_equal) == 0) {tmp_matrix[Fi] = check_equal(debugData.coll_T_data, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.coll_T_data_debug_in_range) == 0) {tmp_matrix[Fi] = check_in_range(debugData.coll_T_data, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_valid_debug_equal) == 0) {tmp_matrix[Fi] = check_equal(debugData.ubi_valid, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ins_flag_debug_over) == 0) {tmp_matrix[Fi] = check_over(debugData.ins_flag, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ins_flag_debug_under) == 0) {tmp_matrix[Fi] = check_under(debugData.ins_flag, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ins_flag_debug_equal) == 0) {tmp_matrix[Fi] = check_equal(debugData.ins_flag, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ins_flag_debug_in_range) == 0) {tmp_matrix[Fi] = check_in_range(debugData.ins_flag, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.car_speed_debug_over) == 0) {tmp_matrix[Fi] = check_over(debugData.car_speed, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.car_speed_debug_under) == 0) {tmp_matrix[Fi] = check_under(debugData.car_speed, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.car_speed_debug_equal) == 0) {tmp_matrix[Fi] = check_equal(debugData.car_speed, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.car_speed_debug_in_range) == 0) {tmp_matrix[Fi] = check_in_range(debugData.car_speed, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.yaw_angle_debug_over) == 0) {tmp_matrix[Fi] = check_over(debugData.yaw_angle, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.yaw_angle_debug_under) == 0) {tmp_matrix[Fi] = check_under(debugData.yaw_angle, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.yaw_angle_debug_equal) == 0) {tmp_matrix[Fi] = check_equal(debugData.yaw_angle, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.yaw_angle_debug_in_range) == 0) {tmp_matrix[Fi] = check_in_range(debugData.yaw_angle, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.roll_angle_debug_over) == 0) {tmp_matrix[Fi] = check_over(debugData.roll_angle, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.roll_angle_debug_under) == 0) {tmp_matrix[Fi] = check_under(debugData.roll_angle, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.roll_angle_debug_equal) == 0) {tmp_matrix[Fi] = check_equal(debugData.roll_angle, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.roll_angle_debug_in_range) == 0) {tmp_matrix[Fi] = check_in_range(debugData.roll_angle, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.pitch_angle_debug_over) == 0) {tmp_matrix[Fi] = check_over(debugData.pitch_angle, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.pitch_angle_debug_under) == 0) {tmp_matrix[Fi] = check_under(debugData.pitch_angle, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.pitch_angle_debug_equal) == 0) {tmp_matrix[Fi] = check_equal(debugData.pitch_angle, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.pitch_angle_debug_in_range) == 0) {tmp_matrix[Fi] = check_in_range(debugData.pitch_angle, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_on_flag_debug_over) == 0) {tmp_matrix[Fi] = check_over(debugData.ubi_on_flag, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_on_flag_debug_under) == 0) {tmp_matrix[Fi] = check_under(debugData.ubi_on_flag, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_on_flag_debug_equal) == 0) {tmp_matrix[Fi] = check_equal(debugData.ubi_on_flag, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_on_flag_debug_in_range) == 0) {tmp_matrix[Fi] = check_in_range(debugData.ubi_on_flag, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_a_set_debug_over) == 0) {tmp_matrix[Fi] = check_over(debugData.ubi_a_set, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_a_set_debug_under) == 0) {tmp_matrix[Fi] = check_under(debugData.ubi_a_set, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_a_set_debug_equal) == 0) {tmp_matrix[Fi] = check_equal(debugData.ubi_a_set, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_a_set_debug_in_range) == 0) {tmp_matrix[Fi] = check_in_range(debugData.ubi_a_set, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_b_set_debug_over) == 0) {tmp_matrix[Fi] = check_over(debugData.ubi_b_set, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_b_set_debug_under) == 0) {tmp_matrix[Fi] = check_under(debugData.ubi_b_set, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_b_set_debug_equal) == 0) {tmp_matrix[Fi] = check_equal(debugData.ubi_b_set, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_b_set_debug_in_range) == 0) {tmp_matrix[Fi] = check_in_range(debugData.ubi_b_set, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_X_data_debug_over) == 0) {tmp_matrix[Fi] = check_over(debugData.acc_X_data, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_X_data_debug_under) == 0) {tmp_matrix[Fi] = check_under(debugData.acc_X_data, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_X_data_debug_equal) == 0) {tmp_matrix[Fi] = check_equal(debugData.acc_X_data, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_X_data_debug_in_range) == 0) {tmp_matrix[Fi] = check_in_range(debugData.acc_X_data, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_Y_data_debug_over) == 0) {tmp_matrix[Fi] = check_over(debugData.acc_Y_data, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_Y_data_debug_under) == 0) {tmp_matrix[Fi] = check_under(debugData.acc_Y_data, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_Y_data_debug_equal) == 0) {tmp_matrix[Fi] = check_equal(debugData.acc_Y_data, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.acc_Y_data_debug_in_range) == 0) {tmp_matrix[Fi] = check_in_range(debugData.acc_Y_data, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_Z_data_debug_over) == 0) {tmp_matrix[Fi] = check_over(debugData.gyro_Z_data, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_Z_data_debug_under) == 0) {tmp_matrix[Fi] = check_under(debugData.gyro_Z_data, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_Z_data_debug_equal) == 0) {tmp_matrix[Fi] = check_equal(debugData.gyro_Z_data, Ri, Fi);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gyro_Z_data_debug_in_range) == 0) {tmp_matrix[Fi] = check_in_range(debugData.gyro_Z_data, Ri, Fi);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.ang_dget_flag_debug_equal) == 0) {tmp_matrix[Fi] = check_equal(debugData.ang_dget_flag, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ins_run_flag_debug_equal) == 0) {tmp_matrix[Fi] = check_equal(debugData.ins_run_flag, Ri, Fi);}
         
        else if (strcmp(relayData.relays[Ri][Fi], relayData.fix_roll_flag_debug_equal) == 0) {tmp_matrix[Fi] = check_equal(debugData.fix_roll_flag, Ri, Fi);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.fix_pitch_flag_debug_equal) == 0) {tmp_matrix[Fi] = check_equal(debugData.fix_pitch_flag, Ri, Fi);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.ubi_kind_flag_debug_equal) == 0) {tmp_matrix[Fi] = check_equal(debugData.ubi_kind_flag, Ri, Fi);}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                     SYSTEMS CHECKS: VALIDITY

        /*
        intended to conditionally switch datasets, making it possible to continue performing the same task and or other tasks instead, but with the option of
        relying on different data in the event data becomes unavailable/unreliable/etc. this can allow for fallback functions to be considered in the matrix switch.

        example: if is_true(checksum)  then A using data X is active/on and B is inactive/off
                 if is_false(checksum) then B using data Y is active/on and A is inactive/off
                 A and B may even be plugged into the same endpoint, and now that endpoint is on/off predicated upon different data/conditions.
        */
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gngga_valid_checksum) == 0) {tmp_matrix[Fi] = is_true(gnggaData.valid_checksum);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gngga_invalid_checksum) == 0) {tmp_matrix[Fi] = is_false(gnggaData.valid_checksum);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gnrmc_valid_checksum) == 0) {tmp_matrix[Fi] = is_true(gnrmcData.valid_checksum);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gnrmc_invalid_checksum) == 0) {tmp_matrix[Fi] = is_false(gnrmcData.valid_checksum);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gpatt_valid_checksum) == 0) {tmp_matrix[Fi] = is_true(gpattData.valid_checksum);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gpatt_invalid_checksum) == 0) {tmp_matrix[Fi] = is_false(gpattData.valid_checksum);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.speed_valid_checksum) == 0) {tmp_matrix[Fi] = is_true(speedData.valid_checksum);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.speed_invalid_checksum) == 0) {tmp_matrix[Fi] = is_false(speedData.valid_checksum);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.error_valid_checksum) == 0) {tmp_matrix[Fi] = is_true(errorData.valid_checksum);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.error_invalid_checksum) == 0) {tmp_matrix[Fi] = is_false(errorData.valid_checksum);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.debug_valid_checksum) == 0) {tmp_matrix[Fi] = is_true(debugData.valid_checksum);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.debug_invalid_checksum) == 0) {tmp_matrix[Fi] = is_false(debugData.valid_checksum);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gngga_valid_check_data) == 0) {tmp_matrix[Fi] = is_N_true(gnggaData.check_data, 16);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gngga_invalid_check_data) == 0) {tmp_matrix[Fi] = is_N_false(gnggaData.check_data, 16);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gnrmc_valid_check_data) == 0) {tmp_matrix[Fi] = is_N_true(gnrmcData.check_data, 14);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gnrmc_invalid_check_data) == 0) {tmp_matrix[Fi] = is_N_false(gnrmcData.check_data, 14);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gpatt_valid_check_data) == 0) {tmp_matrix[Fi] = is_N_true(gpattData.check_data, 41);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gpatt_invalid_check_data) == 0) {tmp_matrix[Fi] = is_N_false(gpattData.check_data, 41);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.speed_valid_check_data) == 0) {tmp_matrix[Fi] = is_N_true(speedData.check_data, 17);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.speed_invalid_check_data) == 0) {tmp_matrix[Fi] = is_N_false(speedData.check_data, 17);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.error_valid_check_data) == 0) {tmp_matrix[Fi] = is_N_true(errorData.check_data, 8);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.error_invalid_check_data) == 0) {tmp_matrix[Fi] = is_N_false(errorData.check_data, 8);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.debug_valid_check_data) == 0) {tmp_matrix[Fi] = is_N_true(debugData.check_data, 29);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.debug_invalid_check_data) == 0) {tmp_matrix[Fi] = is_N_false(debugData.check_data, 29);}
      }
      
      // Safety Layer: Disengage if all entries are $NONE
      if (count_none_function <= relayData.MAX_RELAY_ELEMENTS-1) {

        // Default final bool default is true: If a single false is found then final bool should be set to false and remain false
        bool final_bool = true;

        // debug (same as line below but with output)
        // for (int FC = 0; FC < relayData.MAX_RELAY_ELEMENTS-1; FC++) {Serial.println("[tmp_matrix[FC]] " + String(tmp_matrix[FC])); if (tmp_matrix[FC] == 0) {final_bool = false;}}

        for (int FC = 0; FC < relayData.MAX_RELAY_ELEMENTS-1; FC++) {if (tmp_matrix[FC] == 0) {final_bool = false; break;}}

        /*
        Remember Always: why do you think you can trust this data? are you transmitting this data to yourelf (from satellite or not)?
                         how critical are your system(s)?
                         once you plug something into this, the 'satellites' are in control unless you have a way to override.

        Activate/Deactivate relay with Ri mapped to pinN: pin number matrix required for relay selcection
        */

        // debug (same as line below but with output)
        // if (final_bool == false) {Serial.println("[RELAY " + String(Ri) + "] inactive"); relayData.relays_bool[0][Ri] = 0;}
        // else if (final_bool == true) {Serial.println("[RELAY " + String(Ri) + "] active"); relayData.relays_bool[0][Ri] = 1;}

        if (final_bool == false) {relayData.relays_bool[0][Ri] = 0;}
        else if (final_bool == true) {relayData.relays_bool[0][Ri] = 1;}
      }
      else {Serial.println("[RELAY " + String(Ri) + "] WARNING: Matrix checks are enabled for an non configured matrix!");}
    }
    // handle Ri's that are disbaled
    else {relayData.relays_bool[0][Ri] = 0;}
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   READ RXD 1

void readRXD_1() {

  serial1Data.rcv = false;

  if (Serial1.available() > 0) {
    
    memset(serial1Data.BUFFER, 0, 2048);
    serial1Data.nbytes = (Serial1.readBytesUntil('\n', serial1Data.BUFFER, sizeof(serial1Data.BUFFER)));
    // Serial.println(serial1Data.nbytes); // debug

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                                    GNGGA

    if (strncmp(serial1Data.BUFFER, "$GNGGA", 6) == 0) {
      if ((serial1Data.nbytes == 94) || (serial1Data.nbytes == 90) ) {
        serial1Data.rcv = true;
        Serial.print(""); Serial.println(serial1Data.BUFFER);
        gnggaData.valid_checksum = validateChecksum(serial1Data.BUFFER);
        if (gnggaData.valid_checksum == true) {GNGGA();}
        else {gnggaData.bad_checksum_validity++;}
      }
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                                    GNRMC

    else if (strncmp(serial1Data.BUFFER, "$GNRMC", 6) == 0) {
      if ((serial1Data.nbytes == 78) || (serial1Data.nbytes == 80)) {
        serial1Data.rcv = true;
        Serial.print(""); Serial.println(serial1Data.BUFFER);
        gnrmcData.valid_checksum = validateChecksum(serial1Data.BUFFER);
        if (gnrmcData.valid_checksum == true) {GNRMC();}
        else {gnrmcData.bad_checksum_validity++;}
      }
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                                    GPATT

    else if (strncmp(serial1Data.BUFFER, "$GPATT", 6) == 0) {
      if ((serial1Data.nbytes == 136) || (serial1Data.nbytes == 189)) {
        serial1Data.rcv = true;
        Serial.print(""); Serial.println(serial1Data.BUFFER);
        gpattData.valid_checksum = validateChecksum(serial1Data.BUFFER);
        if (gpattData.valid_checksum == true) {GPATT();}
        else {gpattData.bad_checksum_validity++;}
      }
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                                    DESBI

    else if (strncmp(serial1Data.BUFFER, "$DESBI", 6) == 0) {
      // serial1Data.rcv = true;
      // Serial.print(""); Serial.println(serial1Data.BUFFER);
      // awaiting length checks and clarification: wait for clarification, take a ride with the laptop
      // DESBI();
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                                    SPEED

    else if (strncmp(serial1Data.BUFFER, "$SPEED", 6) == 0) {
      // serial1Data.rcv = true;
      // Serial.print(""); Serial.println(serial1Data.BUFFER);
      // awaiting length checks: take a ride with the laptop
      // SPEED();
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                                    ERROR

    else if (strncmp(serial1Data.BUFFER, "$ERROR", 6) == 0) {
      // serial1Data.rcv = true;
      // Serial.print(""); Serial.println(serial1Data.BUFFER);
      // awaiting length checks: take a ride with the laptop
      // ERROR();
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                                    DEBUG

    else if (strncmp(serial1Data.BUFFER, "$DEBUG", 6) == 0) {
      // serial1Data.rcv = true;
      // Serial.print(""); Serial.println(serial1Data.BUFFER);
      // awaiting length checks: take a ride with the laptop
      // DEBUG();
    }

    // else {
    //   Serial.println("[unknown] " + String(serial1Data.BUFFER));
    // }
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   READ RXD 0

void readRXD_0() {

  

  if (Serial.available() > 0) {
    
    memset(serial0Data.BUFFER, 0, 2048);
    serial0Data.nbytes = (Serial.readBytesUntil('\n', serial0Data.BUFFER, sizeof(serial0Data.BUFFER)));
    // Serial.println(serial0Data.nbytes); // debug

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                        MATRIX: SET ENTRY

    if (strncmp(serial0Data.BUFFER, "$MATRIX_SET_ENTRY", 17) == 0) {
      matrix_set_entry();
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                         MATRIX: OVERRIDE

    else if (strcmp(serial0Data.BUFFER, "$MATRIX_OVERRIDE") == 0) {
      matrix_override();
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                       MATRIX: ENABLE ALL

    else if (strcmp(serial0Data.BUFFER, "$MATRIX_ENABLE_ALL") == 0) {
      matrix_enable_all();
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                         MATRIX: SERIAL PRINT MATRIX FILE

    else if (strcmp(serial0Data.BUFFER, "$SDCARD_READ_MATRIX") == 0) {
      sdcard_read_to_serial("matrix.txt");
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                           MATRIX: WRITE MATRIX TO SDCARD

    else if (strcmp(serial0Data.BUFFER, "$SDCARD_WRITE_MATRIX") == 0) {
      sdcard_write_matrix("matrix.txt");
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                      MATRIX: LOAD MATRIX

    else if (strcmp(serial0Data.BUFFER, "$SDCARD_LOAD_MATRIX") == 0) {
      sdcard_load_matrix("matrix.txt");
    }

    // ------------------------------------------------------------------------------------------------------------------------

    else {
      Serial.println("[unknown] " + String(serial0Data.BUFFER));
    }
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    MAIN LOOP

void loop() {

  // check serial input commands
  readRXD_0();

  // check satellite receiver
  readRXD_1();

  /*
  for performance/efficiency only do the following if data is received OR if there may be an issue receiving, this way the matrix
  switch can remain operational for data that is not received while also performing better overall. (tunable)
  */  
  if ((serial1Data.rcv == true) || (serial1Data.badrcv_i >= 10)) {serial1Data.badrcv_i=0; extrapulatedSatData();
    matrixSwitch();
    SSD_Display_3();
    SSD_Display_4();
    SSD_Display_5();
    SSD_Display_6();
    SSD_Display_7();
  }
  else {serial1Data.badrcv_i++;}

  delay(1);
}

// ----------------------------------------------------------------------------------------------------------------------------


