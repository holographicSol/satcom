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


                                                          Wiring
                                  WTGPS300P TX               --> ESP32 io26 as RXD
                                  WTGPS300P VCC              --> ESP32 3.3/5v
                                  TCA9548A i2C Multiplexer   --> ESP32 i2C (3.3v) (Optional)
                                  x6 SSD1306 (blue & yellow) --> TCA9548A i2C Multiplexer (Optional)
                                  SDCARD Adapter HW-125      --> CS 05, SCK 18, MOSI 23, MISO 19, VCC 5v (Optional)


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

                        WARNING! the burn in is for real. auto display dim/off is not yet implemented.
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

SSD1306Wire   display_6(0x3c, SDA, SCL); // let SSD1306Wire wire up our SSD1306 on the i2C bus
SSD1306Wire   display_7(0x3c, SDA, SCL); // let SSD1306Wire wire up our SSD1306 on the i2C bus
SSD1306Wire   display_5(0x3c, SDA, SCL); // let SSD1306Wire wire up our SSD1306 on the i2C bus
SSD1306Wire   display_4(0x3c, SDA, SCL); // let SSD1306Wire wire up our SSD1306 on the i2C bus
SSD1306Wire   display_3(0x3c, SDA, SCL); // let SSD1306Wire wire up our SSD1306 on the i2C bus
SSD1306Wire   display_2(0x3c, SDA, SCL); // let SSD1306Wire wire up our SSD1306 on the i2C bus

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                  SYSTEM DATA

struct systemStruct {
  bool satcom_enabled = true;
  bool gngga_enabled = true;
  bool gnrmc_enabled = true;
  bool gpatt_enabled = true;
  bool speed_enabled = true;
  bool error_enabled = true;
  bool debug_enabled = true;
  bool matrix_enabled = true;
  char translate_enable_bool[1][2][56] = { {"DISABLED", "ENABLED"} };
};
systemStruct systemData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   DEBUG DATA

struct sysDebugStruct {
  bool gngga_sentence = false;
  bool gnrmc_sentence = false;
  bool gpatt_sentence = false;
  bool speed_sentence = false;
  bool error_sentence = false;
  bool debug_sentence = false;
  bool serial_0_sentence = true;
};
sysDebugStruct sysDebugData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    MENU DATA

struct menuStruct {
  int y = 0;
  int x = 1;
  int menu_max_y0 = 6;
  int menu_max_x0 = 3;
  int numpad_y = 0;
  int numpad_x = 0;
  int menu_numpad_max_y0 = 6;
  int menu_numpad_max_x0 = 3;
  char input[256];
  int numpad_key = NULL;
  bool select = false;
  int page = 0;
  int page_max = 7;
  int relay_select = 0;
  int relay_function_select = 0;
  int function_index = 0;
  
};
menuStruct menuData;

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
//                                                                                                                    TIME DATA

struct TimeStruct {
  unsigned long ms0;
  unsigned long ms1;
  unsigned long milliseconds;
  unsigned long seconds;
  unsigned long mainLoopTimeTaken;
  unsigned long mainLoopTimeStart;
};
TimeStruct timeData;

void time_counter() {
  timeData.ms0 += timeData.mainLoopTimeTaken;
  if (timeData.ms0 >= (timeData.ms1 + 1000)) {timeData.ms1 = timeData.ms0; timeData.seconds++;}
}

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
  display_7.setContrast(255, 241);
  display_7.setFont(ArialMT_Plain_10);
  display_7.cls();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                          INITIALIZE DISPLAY

void initDisplay6() {
  display_6.init();
  display_6.setContrast(255, 241);
  display_6.setFont(ArialMT_Plain_10);
  display_6.cls();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                          INITIALIZE DISPLAY

void initDisplay5() {
  display_5.init();
  display_5.setContrast(255, 241);
  display_5.setFont(ArialMT_Plain_10);
  display_5.cls();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                          INITIALIZE DISPLAY

void initDisplay4() {
  display_4.init();
  display_4.setContrast(255, 241);
  display_4.setFont(ArialMT_Plain_10);
  display_4.cls();
}


// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                          INITIALIZE DISPLAY

void initDisplay3() {
  display_3.init();
  display_3.setContrast(255, 241);
  display_3.setFont(ArialMT_Plain_10);
  display_3.cls();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                          INITIALIZE DISPLAY

void initDisplay2() {
  display_2.init();
  display_2.setContrast(255, 241);
  display_2.setFont(ArialMT_Plain_10);
  display_2.cls();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                            VALIDATION STRUCT

struct validationStruct {
  int  valid_i = 0;
  bool valid_b = true;
  char *find_char;
  int  index;
  bool bool_data_0 = false;
  bool bool_data_1 = false;
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

bool is_positive_negative_num(char * data) {
  // designed to check all chars are digits except one period and the signed bit. allows positive/negative floats, doubles and ints
  // allow 1 period anywhere.
  // allow 1 - sign at index zero.
  validData.valid_b = true;
  validData.find_char = strchr(data, '.');
  validData.index = (int)(validData.find_char - data);
  for (int i = 0; i < strlen(data); i++) {if (isdigit(data[i]) == 0) {if (i != validData.index) {if ((data[i] != '-') && (i > 0)) {validData.valid_b = false;}}}}
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
        if (is_positive_negative_num(data) == true) {
          check_pass = true;
        }
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
        if (is_positive_negative_num(data) == true) {
          check_pass = true;
        }
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
  if (is_positive_negative_num(data) == true) {
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
  if (is_positive_negative_num(data) == true) {
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
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
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
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
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
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_roll_gpatt(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_yaw_gpatt(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
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
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
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
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
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
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
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
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
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
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
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
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_fix_pitch_flag(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
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
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_acc_Y_data(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_gyro_Z_data(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_pitch_angle(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_roll_angle(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_yaw_angle(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
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
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
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
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_coll_T_heading(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
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

  int relays_enable[1][40] = {
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      }
  };

  unsigned long relays_timing[1][40] = {
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

char function_names[252][56] = {
        "$NONE",
        "$ENABLED",
        "SecondsTimer",
        "DegreesLatGNGGAOver",
        "DegreesLatGNGGAUnder",
        "DegreesLatGNGGAEqual",
        "DegreesLatGNGGARange",
        "DegreesGNGGARange",
        "DegreesLonGNGGAOver",
        "DegreesLonGNGGAUnder",
        "DegreesLonGNGGAEqual",
        "DegreesLonGNGGARange",
        "DegreesLatGNRMCOver",
        "DegreesLatGNRMCUnder",
        "DegreesLatGNRMCEqual",
        "DegreesLatGNRMCRange",
        "DegreesLonGNRMCOver",
        "DegreesLonGNRMCUnder",
        "DegreesLonGNRMCEqual",
        "DegreesGNRMCRange",

        "UTCTimeGNGGAOver",
        "UTCTimeGNGGAUnder",
        "UTCTimeGNGGAEqual",
        "UTCTimeGNGGARange",
        "LatGNGGAOver",
        "LatGNGGAUnder",
        "LatGNGGAEqual",
        "LatGNGGARange",
        "LonGNGGAOver",
        "LonGNGGAUnder",
        "LonGNGGAEqual",
        "LonGNGGARange",
        "PositioningStatusGNGGA",
        "SatelliteCountOver",
        "SatelliteCountUnder",
        "SatelliteCountEqual",
        "SatelliteCountRange",
        "HemisphereGNGGANorth",
        "HemisphereGNGGAEast",
        "HemisphereGNGGASouth",
        "HemisphereGNGGAWest",
        "GPSPrecisionOver",
        "GPSPrecisionUnder",
        "GPSPrecisionEqual",
        "GPSPrecisionRange",
        "AltitudeGNGGAOver",
        "AltitudeGNGGAUnder",
        "AltitudeGNGGAEqual",
        "AltitudeGNGGARange",

        "UTCTimeGNRMCOver",
        "UTCTimeGNRMCUnder",
        "UTCTimeGNRMCEqual",
        "UTCTimeGNRMCRange",
        "PositioningStatusGNRMCA",
        "PositioningStatusGNRMCV",
        "ModeGNRMCA",
        "ModeGNRMCD",
        "ModeGNRMCN",
        "LatGNRMCOver",
        "LatGNRMCUnder",
        "LatGNRMCEqual",
        "LatGNRMCRange",
        "LonGNRMCOver",
        "LonGNRMCUnder",
        "LonGNRMCEqual",
        "LonGNRMCRange",
        "HemisphereGNRMCNorth",
        "HemisphereGNRMCEast",
        "HemisphereGNRMCSouth",
        "HemisphereGNRMCWest",
        "GroundSpeedGNRMCOver",
        "GroundSpeedGNRMCUnder",
        "GroundSpeedGNRMCEqual",
        "GroundSpeedGNRMCRange",
        "HeadingGNRMCOver",
        "HeadingGNRMCUnder",
        "HeadingGNRMCEqual",
        "HeadingGNRMCRange",
        "UTCDateGNRMCOver",
        "UTCDateGNRMCUnder",
        "UTCDateGNRMCEqual",
        "UTCDateGNRMCRange",

        "InertialFlagGPATTEqual",
        "LineFlagGPATTEqual",
        "StaticFlagGPATTEqual",
        "RunStateFlagGPATTEqual",
        "INSGPATTEqual",
        "SpeedNumGPATTOver",
        "SpeedNumGPATTUnder",
        "SpeedNumGPATTEqual",
        "SpeedNumGPATTRange",
        "MileageGPATTOver",
        "MileageGPATTUnder",
        "MileageGPATTEqual",
        "MileageGPATTRange",
        "GSTDataGPATTOver",
        "GSTDataGPATTUnder",
        "GSTDataGPATTEqual",
        "GSTDataGPATTRange",
        "YawGPATTOver",
        "YawGPATTUnder",
        "YawGPATTEqual",
        "YawGPATTRange",
        "RollGPATTOver",
        "RollGPATTUnder",
        "RollGPATTEqual",
        "RollGPATTRange",
        "PitchGPATTOver",
        "PitchGPATTUnder",
        "PitchGPATTEqual",
        "PitchGPATTRange",

        "UTCTimeSPEEDOver",
        "UTCTimeSPEEDUnder",
        "UTCTimeSPEEDEqual",
        "UTCTimeSPEEDRange",
        "GroundSpeedSPEEDOver",
        "GroundSpeedSPEEDUnder",
        "GroundSpeedSPEEDEqual",
        "GroundSpeedSPEEDRange",
        "StatusSPEEDOver",
        "StatusSPEEDUnder",
        "StatusSPEEDEqual",
        "StatusSPEEDRange",
        "AccXSPEEDOver",
        "AccXSPEEDUnder",
        "AccXSPEEDEqual",
        "AccXSPEEDRange",
        "AccYSPEEDOver",
        "AccYSPEEDUnder",
        "AccYSPEEDEqual",
        "AccYSPEEDRange",
        "AccZSPEEDOver",
        "AccZSPEEDUnder",
        "AccZSPEEDEqual",
        "AccZSPEEDRange",
        "GyroXSPEEDOver",
        "GyroXSPEEDUnder",
        "GyroXSPEEDEqual",
        "GyroXSPEEDRange",
        "GyroYSPEEDOver",
        "GyroYSPEEDUnder",
        "GyroYSPEEDEqual",
        "GyroYSPEEDRange",
        "GyroZSPEEDOver",
        "GyroZSPEEDUnder",
        "GyroZSPEEDEqual",
        "GyroZSPEEDRange",
        "UBIStateFlagOver",
        "UBIStateFlagUnder",
        "UBIStateFlagEqual",
        "GyroZSPEEDRange",
        "UBIStateKindOver",
        "UBIStateKindUnder",
        "UBIStateKindEqual",
        "UBIStateKindRange",
        "UBIStateValueOver",
        "UBIStateValueUnder",
        "UBIStateValueEqual",
        "UBIStateValueRange",

        "ERRORUTCTimeOver",
        "ERRORUTCTimeUnder",
        "ERRORUTCTimeEqual",
        "ERRORUTCTimeRange",
        "ERRORCodeFlagEqual",
        "ERRORGSetFlagEqual",
        "ERRORSSetFlagEqual",

        "CollisionTHeadingOver",
        "CollisionTHeadingUnder",
        "CollisionTHeadingEqual",
        "CollisionTHeadingRange",
        "CollisionTDataOver",
        "CollisionTDataUnder",
        "CollisionTDataEqual",
        "CollisionTDataRange",
        "UBIValidEqual",
        "INSFlagOver",
        "INSFlagUnder",
        "INSFlagEqual",
        "INSFlagRange",
        "CarSpeedOver",
        "CarSpeedUnder",
        "CarSpeedEqual",
        "CarSpeedRange",
        "YawAngleOver",
        "YawAngleUnder",
        "YawAngleEqual",
        "YawAngleRange",
        "RollAngleOver",
        "RollAngleUnder",
        "RollAngleEqual",
        "RollAngleRange",
        "PitchAngleOver",
        "PitchAngleUnder",
        "PitchAngleEqual",
        "PitchAngleRange",
        "AngDGetFlagEqual",
        "INSRunFlagEqual",
        "FixRollFlagEqual",
        "FixPitchFlagEqual",
        "UBIKindFlagEqual",
        "UBIOnFlagOver",
        "UBIOnFlagUnder",
        "UBIOnFlagEqual",
        "UBIOnFlagRange",
        "UBIASetOver",
        "UBIASetUnder",
        "UBIASetEqual",
        "UBIASetRange",
        "UBIBSetOver",
        "UBIBSetUnder",
        "UBIBSetEqual",
        "UBIBSetRange",
        "AccXDataOver",
        "AccXDataUnder",
        "AccXDataEqual",
        "AccXDataRange",
        "AccYDataOver",
        "AccYDataUnder",
        "AccYDataEqual",
        "AccYDataRange",
        "GyroZDataOver",
        "GyroZDataUnder",
        "GyroZDataEqual",
        "GyroZDataRange",

        "gngga_valid_checksum",
        "gnrmc_valid_checksum",
        "gpatt_valid_checksum",
        "speed_valid_checksum",
        "error_valid_checksum",
        "debug_valid_checksum",
        "gngga_invalid_checksum",
        "gnrmc_invalid_checksum",
        "gpatt_invalid_checksum",
        "speed_invalid_checksum",
        "error_invalid_checksum",
        "debug_invalid_checksum",
        "gngga_valid_check_data",
        "gnrmc_valid_check_data",
        "gpatt_valid_check_data",
        "speed_valid_check_data",
        "error_valid_check_data",
        "debug_valid_check_data",
        "gngga_invalid_check_data",
        "gnrmc_invalid_check_data",
        "gpatt_invalid_check_data",
        "speed_invalid_check_data",
        "error_invalid_check_data",
        "debug_invalid_check_data"
    };

    

  // todo: CamelCase and when necessary shorten function names below ready to be displayed

  // default and specifiable value to indicate a relay should not be activated/deactivated if all functions in relays expression are $NONE
  char default_relay_function[56]          = "$NONE";
  char default_enable_relay_function[56]   = "$ENABLED";

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                  SATCOM DATA

  char SecondsTimer[56] = "SecondsTimer";

  char DegreesLatGNGGAOver[56]             = "DegreesLatGNGGAOver";
  char DegreesLatGNGGAUnder[56]            = "DegreesLatGNGGAUnder";
  char DegreesLatGNGGAEqual[56]            = "DegreesLatGNGGAEqual";
  char DegreesLatGNGGARange[56]            = "DegreesLatGNGGARange";
  char DegreesGNGGARange[56]               = "DegreesGNGGARange";
  char DegreesLonGNGGAOver[56]             = "DegreesLonGNGGAOver";
  char DegreesLonGNGGAUnder[56]            = "DegreesLonGNGGAUnder";
  char DegreesLonGNGGAEqual[56]            = "DegreesLonGNGGAEqual";
  char DegreesLonGNGGARange[56]            = "DegreesLonGNGGARange";
  char DegreesLatGNRMCOver[56]             = "DegreesLatGNRMCOver";
  char DegreesLatGNRMCUnder[56]            = "DegreesLatGNRMCUnder";
  char DegreesLatGNRMCEqual[56]            = "DegreesLatGNRMCEqual";
  char DegreesLatGNRMCRange[56]            = "DegreesLatGNRMCRange";
  char DegreesLonGNRMCOver[56]             = "DegreesLonGNRMCOver";
  char DegreesLonGNRMCUnder[56]            = "DegreesLonGNRMCUnder";
  char DegreesLonGNRMCEqual[56]            = "DegreesLonGNRMCEqual";
  char longitude_satcom_gnrmc_in_range[56] = "DegreesGNRMCRange";

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                   GNGGA DATA

  char UTCTimeGNGGAOver[56]       = "UTCTimeGNGGAOver";
  char UTCTimeGNGGAUnder[56]      = "UTCTimeGNGGAUnder";
  char UTCTimeGNGGAEqual[56]      = "UTCTimeGNGGAEqual";
  char UTCTimeGNGGARange[56]      = "UTCTimeGNGGARange";
  char LatGNGGAOver[56]           = "LatGNGGAOver";
  char LatGNGGAUnder[56]          = "LatGNGGAUnder";
  char LatGNGGAEqual[56]          = "LatGNGGAEqual";
  char LatGNGGARange[56]          = "LatGNGGARange";
  char LonGNGGAOver[56]           = "LonGNGGAOver";
  char LonGNGGAUnder[56]          = "LonGNGGAUnder";
  char LonGNGGAEqual[56]          = "LonGNGGAEqual";
  char LonGNGGARange[56]          = "LonGNGGARange";
  char PositioningStatusGNGGA[56] = "PositioningStatusGNGGA";
  char SatelliteCountOver[56]     = "SatelliteCountOver";
  char SatelliteCountUnder[56]    = "SatelliteCountUnder";
  char SatelliteCountEqual[56]    = "SatelliteCountEqual";
  char SatelliteCountRange[56]    = "SatelliteCountRange";
  char HemisphereGNGGANorth[56]   = "HemisphereGNGGANorth";
  char HemisphereGNGGAEast[56]    = "HemisphereGNGGAEast";
  char HemisphereGNGGASouth[56]   = "HemisphereGNGGASouth";
  char HemisphereGNGGAWest[56]    = "HemisphereGNGGAWest";
  char GPSPrecisionOver[56]       = "GPSPrecisionOver";
  char GPSPrecisionUnder[56]      = "GPSPrecisionUnder";
  char GPSPrecisionEqual[56]      = "GPSPrecisionEqual";
  char GPSPrecisionRange[56]      = "GPSPrecisionRange";
  char AltitudeGNGGAOver[56]      = "AltitudeGNGGAOver";
  char AltitudeGNGGAUnder[56]     = "AltitudeGNGGAUnder";
  char AltitudeGNGGAEqual[56]     = "AltitudeGNGGAEqual";
  char AltitudeGNGGARange[56]     = "AltitudeGNGGARange";

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                   GNRMC DATA

  char UTCTimeGNRMCOver[56]         = "UTCTimeGNRMCOver";
  char UTCTimeGNRMCUnder[56]        = "UTCTimeGNRMCUnder";
  char UTCTimeGNRMCEqual[56]        = "UTCTimeGNRMCEqual";
  char UTCTimeGNRMCRange[56]        = "UTCTimeGNRMCRange";
  char PositioningStatusGNRMCA[56]  = "PositioningStatusGNRMCA";
  char PositioningStatusGNRMCV[56]  = "PositioningStatusGNRMCV";
  char ModeGNRMCA[56]               = "ModeGNRMCA";
  char ModeGNRMCD[56]               = "ModeGNRMCD";
  char ModeGNRMCN[56]               = "ModeGNRMCN";
  char LatGNRMCOver[56]             = "LatGNRMCOver";
  char LatGNRMCUnder[56]            = "LatGNRMCUnder";
  char LatGNRMCEqual[56]            = "LatGNRMCEqual";
  char LatGNRMCRange[56]            = "LatGNRMCRange";
  char LonGNRMCOver[56]             = "LonGNRMCOver";
  char LonGNRMCUnder[56]            = "LonGNRMCUnder";
  char LonGNRMCEqual[56]            = "LonGNRMCEqual";
  char LonGNRMCRange[56]            = "LonGNRMCRange";
  char HemisphereGNRMCNorth[56]     = "HemisphereGNRMCNorth";
  char HemisphereGNRMCEast[56]      = "HemisphereGNRMCEast";
  char HemisphereGNRMCSouth[56]     = "HemisphereGNRMCSouth";
  char HemisphereGNRMCWest[56]      = "HemisphereGNRMCWest";
  char GroundSpeedGNRMCOver[56]     = "GroundSpeedGNRMCOver";
  char GroundSpeedGNRMCUnder[56]    = "GroundSpeedGNRMCUnder";
  char GroundSpeedGNRMCEqual[56]    = "GroundSpeedGNRMCEqual";
  char GroundSpeedGNRMCRange[56]    = "GroundSpeedGNRMCRange";
  char HeadingGNRMCOver[56]         = "HeadingGNRMCOver";
  char HeadingGNRMCUnder[56]        = "HeadingGNRMCUnder";
  char HeadingGNRMCEqual[56]        = "HeadingGNRMCEqual";
  char HeadingGNRMCRange[56]        = "HeadingGNRMCRange";
  char UTCDateGNRMCOver[56]         = "UTCDateGNRMCOver";
  char UTCDateGNRMCUnder[56]        = "UTCDateGNRMCUnder";
  char UTCDateGNRMCEqual[56]        = "UTCDateGNRMCEqual";
  char UTCDateGNRMCRange[56]        = "UTCDateGNRMCRange";

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                   GPATT DATA

  char InertialFlagGPATTEqual[56] = "InertialFlagGPATTEqual";
  char LineFlagGPATTEqual[56]     = "LineFlagGPATTEqual";
  char StaticFlagGPATTEqual[56]   = "StaticFlagGPATTEqual";
  char RunStateFlagGPATTEqual[56] = "RunStateFlagGPATTEqual";
  char INSGPATTEqual[56]          = "INSGPATTEqual";
  char SpeedNumGPATTOver[56]      = "SpeedNumGPATTOver";
  char SpeedNumGPATTUnder[56]     = "SpeedNumGPATTUnder[";
  char SpeedNumGPATTEqual[56]     = "SpeedNumGPATTEqual";
  char SpeedNumGPATTRange[56]     = "SpeedNumGPATTRange";
  char MileageGPATTOver[56]       = "MileageGPATTOver";
  char MileageGPATTUnder[56]      = "MileageGPATTUnder[";
  char MileageGPATTEqual[56]      = "MileageGPATTEqual";
  char MileageGPATTRange[56]      = "MileageGPATTRange";
  char GSTDataGPATTOver[56]       = "GSTDataGPATTOver";
  char GSTDataGPATTUnder[56]      = "GSTDataGPATTUnder[";
  char GSTDataGPATTEqual[56]      = "GSTDataGPATTEqual";
  char GSTDataGPATTRange[56]      = "GSTDataGPATTRange";
  char YawGPATTOver[56]           = "YawGPATTOver";
  char YawGPATTUnder[56]          = "YawGPATTUnder[";
  char YawGPATTEqual[56]          = "YawGPATTEqual";
  char YawGPATTRange[56]          = "YawGPATTRange";
  char RollGPATTOver[56]          = "RollGPATTOver";
  char RollGPATTUnder[56]         = "RollGPATTUnder[";
  char RollGPATTEqual[56]         = "RollGPATTEqual";
  char RollGPATTRange[56]         = "RollGPATTRange";
  char PitchGPATTOver[56]         = "PitchGPATTOver";
  char PitchGPATTUnder[56]        = "PitchGPATTUnder[";
  char PitchGPATTEqual[56]        = "PitchGPATTEqual";
  char PitchGPATTRange[56]        = "PitchGPATTRange";

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                   SPEED DATA

  char UTCTimeSPEEDOver[56]      = "UTCTimeSPEEDOver";
  char UTCTimeSPEEDUnder[56]     = "UTCTimeSPEEDUnder";
  char UTCTimeSPEEDEqual[56]     = "UTCTimeSPEEDEqual";
  char UTCTimeSPEEDRange[56]     = "UTCTimeSPEEDRange";
  char GroundSpeedSPEEDOver[56]  = "GroundSpeedSPEEDOver";
  char GroundSpeedSPEEDUnder[56] = "GroundSpeedSPEEDUnder";
  char GroundSpeedSPEEDEqual[56] = "GroundSpeedSPEEDEqual";
  char GroundSpeedSPEEDRange[56] = "GroundSpeedSPEEDRange";
  char StatusSPEEDOver[56]       = "StatusSPEEDOver";
  char StatusSPEEDUnder[56]      = "StatusSPEEDUnder";
  char StatusSPEEDEqual[56]      = "StatusSPEEDEqual";
  char StatusSPEEDRange[56]      = "StatusSPEEDRange";
  char AccXSPEEDOver[56]         = "AccXSPEEDOver";
  char AccXSPEEDUnder[56]        = "AccXSPEEDUnder";
  char AccXSPEEDEqual[56]        = "AccXSPEEDEqual";
  char AccXSPEEDRange[56]        = "AccXSPEEDRange";
  char AccYSPEEDOver[56]         = "AccYSPEEDOver";
  char AccYSPEEDUnder[56]        = "AccYSPEEDUnder";
  char AccYSPEEDEqual[56]        = "AccYSPEEDEqual";
  char AccYSPEEDRange[56]        = "AccYSPEEDRange";
  char AccZSPEEDOver[56]         = "AccZSPEEDOver";
  char AccZSPEEDUnder[56]        = "AccZSPEEDUnder";
  char AccZSPEEDEqual[56]        = "AccZSPEEDEqual";
  char AccZSPEEDRange[56]        = "AccZSPEEDRange";
  char GyroXSPEEDOver[56]        = "GyroXSPEEDOver";
  char GyroXSPEEDUnder[56]       = "GyroXSPEEDUnder";
  char GyroXSPEEDEqual[56]       = "GyroXSPEEDEqual";
  char GyroXSPEEDRange[56]       = "GyroXSPEEDRange";
  char GyroYSPEEDOver[56]        = "GyroYSPEEDOver";
  char GyroYSPEEDUnder[56]       = "GyroYSPEEDUnder";
  char GyroYSPEEDEqual[56]       = "GyroYSPEEDEqual";
  char GyroYSPEEDRange[56]       = "GyroYSPEEDRange";
  char GyroZSPEEDOver[56]        = "GyroZSPEEDOver";
  char GyroZSPEEDUnder[56]       = "GyroZSPEEDUnder";
  char GyroZSPEEDEqual[56]       = "GyroZSPEEDEqual";
  char GyroZSPEEDRange[56]       = "GyroZSPEEDRange";
  char UBIStateFlagOver[56]      = "UBIStateFlagOver";
  char UBIStateFlagUnder[56]     = "UBIStateFlagUnder";
  char UBIStateFlagEqual[56]     = "UBIStateFlagEqual";
  char UBIStateFlagRange[56]     = "GyroZSPEEDRange";
  char UBIStateKindOver[56]      = "UBIStateKindOver";
  char UBIStateKindUnder[56]     = "UBIStateKindUnder";
  char UBIStateKindEqual[56]     = "UBIStateKindEqual";
  char UBIStateKindRange[56]     = "UBIStateKindRange";
  char UBIStateValueOver[56]     = "UBIStateValueOver";
  char UBIStateValueUnder[56]    = "UBIStateValueUnder";
  char UBIStateValueEqual[56]    = "UBIStateValueEqual";
  char UBIStateValueRange[56]    = "UBIStateValueRange";

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                   ERROR DATA

  char ERRORUTCTimeOver[56]   = "ERRORUTCTimeOver";
  char ERRORUTCTimeUnder[56]  = "ERRORUTCTimeUnder";
  char ERRORUTCTimeEqual[56]  = "ERRORUTCTimeEqual";
  char ERRORUTCTimeRange[56]  = "ERRORUTCTimeRange";
  char ERRORCodeFlagEqual[56] = "ERRORCodeFlagEqual";
  char ERRORGSetFlagEqual[56] = "ERRORGSetFlagEqual";
  char ERRORSSetFlagEqual[56] = "ERRORSSetFlagEqual";

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                   DEBUG DATA

  char CollisionTHeadingOver[56]  = "CollisionTHeadingOver";
  char CollisionTHeadingUnder[56] = "CollisionTHeadingUnder";
  char CollisionTHeadingEqual[56] = "CollisionTHeadingEqual";
  char CollisionTHeadingRange[56] = "CollisionTHeadingRange";
  char CollisionTDataOver[56]     = "CollisionTDataOver";
  char CollisionTDataUnder[56]    = "CollisionTDataUnder";
  char CollisionTDataEqual[56]    = "CollisionTDataEqual";
  char CollisionTDataRange[56]    = "CollisionTDataRange";
  char UBIValidEqual[56]          = "UBIValidEqual";
  char INSFlagOver[56]            = "INSFlagOver";
  char INSFlagUnder[56]           = "INSFlagUnder";
  char INSFlagEqual[56]           = "INSFlagEqual";
  char INSFlagRange[56]           = "INSFlagRange";
  char CarSpeedOver[56]           = "CarSpeedOver";
  char CarSpeedUnder[56]          = "CarSpeedUnder";
  char CarSpeedEqual[56]          = "CarSpeedEqual";
  char CarSpeedRange[56]          = "CarSpeedRange";
  char YawAngleOver[56]           = "YawAngleOver";
  char YawAngleUnder[56]          = "YawAngleUnder";
  char YawAngleEqual[56]          = "YawAngleEqual";
  char YawAngleRange[56]          = "YawAngleRange";
  char RollAngleOver[56]          = "RollAngleOver";
  char RollAngleUnder[56]         = "RollAngleUnder";
  char RollAngleEqual[56]         = "RollAngleEqual";
  char RollAngleRange[56]         = "RollAngleRange";
  char PitchAngleOver[56]         = "PitchAngleOver";
  char PitchAngleUnder[56]        = "PitchAngleUnder";
  char PitchAngleEqual[56]        = "PitchAngleEqual";
  char PitchAngleRange[56]        = "PitchAngleRange";
  char AngDGetFlagEqual[56]       = "AngDGetFlagEqual";
  char INSRunFlagEqual[56]        = "INSRunFlagEqual";
  char FixRollFlagEqual[56]       = "FixRollFlagEqual";
  char FixPitchFlagEqual[56]      = "FixPitchFlagEqual";
  char UBIKindFlagEqual[56]       = "UBIKindFlagEqual";
  char UBIOnFlagOver[56]          = "UBIOnFlagOver";
  char UBIOnFlagUnder[56]         = "UBIOnFlagUnder";
  char UBIOnFlagEqual[56]         = "UBIOnFlagEqual";
  char UBIOnFlagRange[56]         = "UBIOnFlagRange";
  char UBIASetOver[56]            = "UBIASetOver";
  char UBIASetUnder[56]           = "UBIASetUnder";
  char UBIASetEqual[56]           = "UBIASetEqual";
  char UBIASetRange[56]           = "UBIASetRange";
  char UBIBSetOver[56]            = "UBIBSetOver";
  char UBIBSetUnder[56]           = "UBIBSetUnder";
  char UBIBSetEqual[56]           = "UBIBSetEqual";
  char UBIBSetRange[56]           = "UBIBSetRange";
  char AccXDataOver[56]           = "AccXDataOver";
  char AccXDataUnder[56]          = "AccXDataUnder";
  char AccXDataEqual[56]          = "AccXDataEqual";
  char AccXDataRange[56]          = "AccXDataRange";
  char AccYDataOver[56]           = "AccYDataOver";
  char AccYDataUnder[56]          = "AccYDataUnder";
  char AccYDataEqual[56]          = "AccYDataEqual";
  char AccYDataRange[56]          = "AccYDataRange";
  char GyroZDataOver[56]          = "GyroZDataOver";
  char GyroZDataUnder[56]         = "GyroZDataUnder";
  char GyroZDataEqual[56]         = "GyroZDataEqual";
  char GyroZDataRange[56]         = "GyroZDataRange";

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                VALIDITY DATA

  char gngga_valid_checksum[56] = "gngga_valid_checksum";
  char gnrmc_valid_checksum[56] = "gnrmc_valid_checksum";
  char gpatt_valid_checksum[56] = "gpatt_valid_checksum";
  char speed_valid_checksum[56] = "speed_valid_checksum";
  char error_valid_checksum[56] = "error_valid_checksum";
  char debug_valid_checksum[56] = "debug_valid_checksum";
  char gngga_invalid_checksum[56] = "gngga_invalid_checksum";
  char gnrmc_invalid_checksum[56] = "gnrmc_invalid_checksum";
  char gpatt_invalid_checksum[56] = "gpatt_invalid_checksum";
  char speed_invalid_checksum[56] = "speed_invalid_checksum";
  char error_invalid_checksum[56] = "error_invalid_checksum";
  char debug_invalid_checksum[56] = "debug_invalid_checksum";
  char gngga_valid_check_data[56] = "gngga_valid_check_data";
  char gnrmc_valid_check_data[56] = "gnrmc_valid_check_data";
  char gpatt_valid_check_data[56] = "gpatt_valid_check_data";
  char speed_valid_check_data[56] = "speed_valid_check_data";
  char error_valid_check_data[56] = "error_valid_check_data";
  char debug_valid_check_data[56] = "debug_valid_check_data";
  char gngga_invalid_check_data[56] = "gngga_invalid_check_data";
  char gnrmc_invalid_check_data[56] = "gnrmc_invalid_check_data";
  char gpatt_invalid_check_data[56] = "gpatt_invalid_check_data";
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
  char imu_kind[56];         unsigned long bad_imu_kind_i;         bool bad_imu_kind = true;         // <25> Sensor Type: 0->BIms055; 1->BMI160; 2->LSM6DS3TR-C; 3->LSM6DSOW 4->ICM-40607; 5->ICM-40608 6->ICM-42670; 7->LSM6DSR
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
  bool convert_coordinates            = true;
  char coordinate_conversion_mode[10] = "GNGGA";                   // choose a sentence that degrees/decimal coordinates will be created from
  double latitude_meter               = 0.0000100;                // one meter (tune)
  double longitude_meter              = 0.0000100;                // one meter (tune)
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
    satData.degreesLat = trunc(satData.temp_latitude_gngga / 100);
    satData.minutesLat = satData.temp_latitude_gngga - (satData.degreesLat * 100);
    satData.secondsLat = (satData.minutesLat - trunc(satData.minutesLat)) * 60;
    satData.millisecondsLat = (satData.secondsLat - trunc(satData.secondsLat)) * 1000;
    satData.minutesLat = trunc(satData.minutesLat);
    satData.secondsLat = trunc(satData.secondsLat);
    satData.location_latitude_gngga =
    satData.degreesLat + satData.minutesLat / 60 + satData.secondsLat / 3600 + satData.millisecondsLat / 3600000;
    if (strcmp(gnggaData.latitude_hemisphere, "S") == 0) {
      satData.location_latitude_gngga = 0 - satData.location_latitude_gngga;
    }
    scanf("%f17", &satData.location_latitude_gngga);
    sprintf(satData.location_latitude_gngga_str, "%f", satData.location_latitude_gngga);

    // convert GNGGA longitude
    satData.temp_longitude_gngga = satData.abs_longitude_gngga_0;
    satData.degreesLong = trunc(satData.temp_longitude_gngga / 100);
    satData.minutesLong = satData.temp_longitude_gngga - (satData.degreesLong * 100);
    satData.secondsLong = (satData.minutesLong - trunc(satData.minutesLong)) * 60;
    satData.millisecondsLong = (satData.secondsLong - trunc(satData.secondsLong)) * 1000;
    satData.minutesLong = trunc(satData.minutesLong);
    satData.secondsLong = trunc(satData.secondsLong);
    satData.location_longitude_gngga =
    satData.degreesLong + satData.minutesLong / 60 + satData.secondsLong / 3600 + satData.millisecondsLong / 3600000;
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
    satData.degreesLat = trunc(satData.temp_latitude_gnrmc / 100);
    satData.minutesLat = satData.temp_latitude_gnrmc - (satData.degreesLat * 100);
    satData.secondsLat = (satData.minutesLat - (satData.minutesLat)) * 60;
    satData.millisecondsLat = (satData.secondsLat - trunc(satData.secondsLat)) * 1000;
    satData.minutesLat = trunc(satData.minutesLat);
    satData.secondsLat = trunc(satData.secondsLat);
    satData.location_latitude_gnrmc =
    satData.degreesLat + satData.minutesLat / 60 + satData.secondsLat / 3600 + satData.millisecondsLat / 3600000;
    if (strcmp(gnrmcData.latitude_hemisphere, "S") == 0) {
      satData.location_latitude_gnrmc = 0 - satData.location_latitude_gnrmc;
    }
    scanf("%f17", &satData.location_latitude_gnrmc);
    sprintf(satData.location_latitude_gnrmc_str, "%f", satData.location_latitude_gnrmc);

    // convert GNRMC longitude
    satData.temp_longitude_gnrmc = satData.abs_longitude_gnrmc_0;
    satData.degreesLong = trunc(satData.temp_longitude_gnrmc / 100);
    satData.minutesLong = satData.temp_longitude_gnrmc - (satData.degreesLong * 100);
    satData.secondsLong = (satData.minutesLong - trunc(satData.minutesLong)) * 60;
    satData.millisecondsLong = (satData.secondsLong - trunc(satData.secondsLong)) * 1000;
    satData.minutesLong = trunc(satData.minutesLong);
    satData.secondsLong = trunc(satData.secondsLong);
    satData.location_longitude_gnrmc =
    satData.degreesLong + satData.minutesLong / 60 + satData.secondsLong / 3600 + satData.millisecondsLong / 3600000;
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

  if (satData.convert_coordinates == true) {
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
  }
  else {strcat(satData.satcom_sentence, "0.0,0.0,");}

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                       SATCOM SENTENCE: END
  strcat(satData.satcom_sentence, "*");
  satData.checksum_i = getCheckSum(satData.satcom_sentence);
  itoa(satData.checksum_i, satData.checksum_str, 10);
  strcat(satData.satcom_sentence, satData.checksum_str);
  Serial.println(satData.satcom_sentence);
  }

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    DISPLAY 2

void SSD_Display_2_Menu() {
  tcaselect(2);
  display_2.setTextAlignment(TEXT_ALIGN_CENTER);

  // numpad
  if (menuData.page == 10) {
    display_2.setColor(BLACK); display_2.fillRect(0, 0, display_2.getWidth(), display_2.getHeight());
    display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, String(menuData.input));
    // none selected.
    if (menuData.numpad_y == 0) {
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 14, "7");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 14, "8");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 14, "9");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 24, "4");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "5");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 24, "6");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 34, "1");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "2");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 34, "3");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 44, "0");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, ".");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 44, "-");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 54, "ENTER");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 54, "DELETE");
    }
    // 7
    if ((menuData.numpad_y == 1) && (menuData.numpad_x == 0)) {
      display_2.setColor(WHITE); display_2.fillRect(2, 16, display_2.getWidth()/3, 10);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(BLACK); display_2.drawString((display_2.getWidth()/3)/2, 14, "7");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 14, "8");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 14, "9");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 24, "4");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "5");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 24, "6");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 34, "1");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "2");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 34, "3");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 44, "0");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, ".");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 44, "-");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 54, "ENTER");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 54, "DELETE");
    }
    // 8
    if ((menuData.numpad_y == 1) && (menuData.numpad_x == 1)) {
      display_2.setColor(WHITE); display_2.fillRect(display_2.getWidth()/3, 16, display_2.getWidth()/3, 10);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 14, "7");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 14, "8");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 14, "9");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 24, "4");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "5");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 24, "6");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 34, "1");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "2");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 34, "3");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 44, "0");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, ".");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 44, "-");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 54, "ENTER");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 54, "DELETE");
    }
    // 9
    if ((menuData.numpad_y == 1) && (menuData.numpad_x == 2)) {
      display_2.setColor(WHITE); display_2.fillRect(display_2.getWidth()-(display_2.getWidth()/3)-2, 16, display_2.getWidth()/3, 10);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 14, "7");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 14, "8");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(BLACK); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 14, "9");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 24, "4");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "5");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 24, "6");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 34, "1");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "2");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 34, "3");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 44, "0");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, ".");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 44, "-");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 54, "ENTER");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 54, "DELETE");
    }
    // 4
    if ((menuData.numpad_y == 2) && (menuData.numpad_x == 0)) {
      display_2.setColor(WHITE); display_2.fillRect(2, 26, display_2.getWidth()/3, 10);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 14, "7");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 14, "8");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 14, "9");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(BLACK); display_2.drawString((display_2.getWidth()/3)/2, 24, "4");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "5");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 24, "6");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 34, "1");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "2");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 34, "3");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 44, "0");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, ".");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 44, "-");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 54, "ENTER");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 54, "DELETE");
    }
    // 5
    if ((menuData.numpad_y == 2) && (menuData.numpad_x == 1)) {
      display_2.setColor(WHITE); display_2.fillRect(display_2.getWidth()/3, 26, display_2.getWidth()/3, 10);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 14, "7");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 14, "8");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 14, "9");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 24, "4");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 24, "5");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 24, "6");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 34, "1");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "2");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 34, "3");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 44, "0");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, ".");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 44, "-");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 54, "ENTER");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 54, "DELETE");
    }
    // 6
    if ((menuData.numpad_y == 2) && (menuData.numpad_x == 2)) {
      display_2.setColor(WHITE); display_2.fillRect(display_2.getWidth()-(display_2.getWidth()/3)-2, 26, display_2.getWidth()/3, 10);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 14, "7");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 14, "8");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 14, "9");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 24, "4");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "5");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(BLACK); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 24, "6");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 34, "1");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "2");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 34, "3");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 44, "0");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, ".");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 44, "-");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 54, "ENTER");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 54, "DELETE");
    }
    // 1
    if ((menuData.numpad_y == 3) && (menuData.numpad_x == 0)) {
      display_2.setColor(WHITE); display_2.fillRect(2, 36, display_2.getWidth()/3, 10);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 14, "7");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 14, "8");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 14, "9");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 24, "4");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "5");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 24, "6");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(BLACK); display_2.drawString((display_2.getWidth()/3)/2, 34, "1");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "2");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 34, "3");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 44, "0");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, ".");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 44, "-");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 54, "ENTER");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 54, "DELETE");
    }
    // 2
    if ((menuData.numpad_y == 3) && (menuData.numpad_x == 1)) {
      display_2.setColor(WHITE); display_2.fillRect(display_2.getWidth()/3, 36, display_2.getWidth()/3, 10);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 14, "7");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 14, "8");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 14, "9");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 24, "4");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "5");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 24, "6");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 34, "1");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 34, "2");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 34, "3");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 44, "0");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, ".");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 44, "-");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 54, "ENTER");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 54, "DELETE");
    }
    // 3
    if ((menuData.numpad_y == 3) && (menuData.numpad_x == 2)) {
      display_2.setColor(WHITE); display_2.fillRect(display_2.getWidth()-(display_2.getWidth()/3)-2, 36, display_2.getWidth()/3, 10);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 14, "7");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 14, "8");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 14, "9");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 24, "4");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "5");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 24, "6");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 34, "1");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "2");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(BLACK); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 34, "3");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 44, "0");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, ".");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 44, "-");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 54, "ENTER");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 54, "DELETE");
    }
    // 0
    if ((menuData.numpad_y == 4) && (menuData.numpad_x == 0)) {
      display_2.setColor(WHITE); display_2.fillRect(2, 46, display_2.getWidth()/3, 10);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 14, "7");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 14, "8");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 14, "9");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 24, "4");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "5");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 24, "6");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 34, "1");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "2");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 34, "3");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(BLACK); display_2.drawString((display_2.getWidth()/3)/2, 44, "0");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, ".");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 44, "-");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 54, "ENTER");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 54, "DELETE");
    }
    // .
    if ((menuData.numpad_y == 4) && (menuData.numpad_x == 1)) {
      display_2.setColor(WHITE); display_2.fillRect(display_2.getWidth()/3, 46, display_2.getWidth()/3, 10);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 14, "7");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 14, "8");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 14, "9");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 24, "4");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "5");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 24, "6");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 34, "1");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "2");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 34, "3");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 44, "0");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 44, ".");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 44, "-");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 54, "ENTER");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 54, "DELETE");
    }
    // minus
    if ((menuData.numpad_y == 4) && (menuData.numpad_x == 2)) {
      display_2.setColor(WHITE); display_2.fillRect(display_2.getWidth()-(display_2.getWidth()/3)-2, 46, display_2.getWidth()/3, 10);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 14, "7");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 14, "8");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 14, "9");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 24, "4");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "5");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 24, "6");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 34, "1");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "2");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 34, "3");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 44, "0");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, ".");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(BLACK); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 44, "-");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 54, "ENTER");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 54, "DELETE");
    }
    // enter
    if ((menuData.numpad_y == 5) && (menuData.numpad_x == 0)) {
      display_2.setColor(WHITE); display_2.fillRect(2, 56, display_2.getWidth()/2, 10);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 14, "7");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 14, "8");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 14, "9");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 24, "4");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "5");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 24, "6");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 34, "1");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "2");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 34, "3");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 44, "0");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, ".");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 44, "-");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(BLACK); display_2.drawString(2, 54, "ENTER");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 54, "DELETE");
    }
    // delete
    if (((menuData.numpad_y == 5) && (menuData.numpad_x == 1)) || ((menuData.numpad_y == 5) && (menuData.numpad_x == 2))) {
      display_2.setColor(WHITE); display_2.fillRect(display_2.getWidth()-(display_2.getWidth()/2), 56, display_2.getWidth()/2, 10);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 14, "7");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 14, "8");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 14, "9");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 24, "4");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "5");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 24, "6");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 34, "1");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "2");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 34, "3");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()/3)/2, 44, "0");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, ".");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString((display_2.getWidth()-(display_2.getWidth()/3)/2), 44, "-");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 54, "ENTER");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()-7, 54, "DELETE");
    }
  }

  if (menuData.page == 0) {
    display_2.clear();
    // select top center
    if ((menuData.y == 0) && (menuData.x == 1)) {
      display_2.setColor(WHITE); display_2.fillRect(display_2.getWidth()/4, 0, display_2.getWidth()/2, 14);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(0, 16, display_2.getWidth(), display_2.getHeight()-16);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 15, String(systemData.translate_enable_bool[0][relayData.relays_enable[0][menuData.relay_select]]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 15, "R " + String(menuData.relay_select));
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 15, "F " + String(menuData.relay_function_select));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "" + String(relayData.relays[menuData.relay_select][menuData.relay_function_select]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 33, "X " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][0]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 42, "Y " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][1]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 51, "Z " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][2]));
      }
    // select top left
    if ((menuData.y == 0) && (menuData.x == 0)) {
      display_2.setColor(WHITE); display_2.fillRect(2, 0, display_2.getWidth()/8, 14);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(BLACK); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(0, 16, display_2.getWidth(), display_2.getHeight()-16);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 15, String(systemData.translate_enable_bool[0][relayData.relays_enable[0][menuData.relay_select]]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 15, "R " + String(menuData.relay_select));
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 15, "F " + String(menuData.relay_function_select));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "" + String(relayData.relays[menuData.relay_select][menuData.relay_function_select]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 33, "X " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][0]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 42, "Y " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][1]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 51, "Z " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][2]));
      }
    // select top right
    if ((menuData.y == 0) && (menuData.x == 2)) {
      display_2.setColor(WHITE); display_2.fillRect(display_2.getWidth()-(display_2.getWidth()/8)-2, 0, display_2.getWidth()/8, 14);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(0, 16, display_2.getWidth(), display_2.getHeight()-16);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 15, String(systemData.translate_enable_bool[0][relayData.relays_enable[0][menuData.relay_select]]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 15, "R " + String(menuData.relay_select));
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 15, "F " + String(menuData.relay_function_select));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "" + String(relayData.relays[menuData.relay_select][menuData.relay_function_select]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 33, "X " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][0]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 42, "Y " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][1]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 51, "Z " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][2]));
      }
    // select row 1 center
    if ((menuData.y == 1) && (menuData.x == 1)) {
      display_2.setColor(WHITE); display_2.fillRect(display_2.getWidth()/4, 16, display_2.getWidth()/2, 10);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(0, 16, display_2.getWidth(), display_2.getHeight()-16);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 15, String(systemData.translate_enable_bool[0][relayData.relays_enable[0][menuData.relay_select]]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 15, "R " + String(menuData.relay_select));
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 15, "F " + String(menuData.relay_function_select));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "" + String(relayData.relays[menuData.relay_select][menuData.relay_function_select]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 33, "X " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][0]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 42, "Y " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][1]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 51, "Z " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][2]));
      }
    // select row 1 left
    if ((menuData.y == 1) && (menuData.x == 0)) {
      display_2.setColor(WHITE); display_2.fillRect(0, 16, display_2.getWidth()/4, 9);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(0, 16, display_2.getWidth(), display_2.getHeight()-16);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 15, String(systemData.translate_enable_bool[0][relayData.relays_enable[0][menuData.relay_select]]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(BLACK); display_2.drawString(2, 15, "R " + String(menuData.relay_select));
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 15, "F " + String(menuData.relay_function_select));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "" + String(relayData.relays[menuData.relay_select][menuData.relay_function_select]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 33, "X " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][0]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 42, "Y " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][1]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 51, "Z " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][2]));
      }
    // select row 1 right
    if ((menuData.y == 1) && (menuData.x == 2)) {
      display_2.setColor(WHITE); display_2.fillRect(display_2.getWidth()-(display_2.getWidth()/4), 16, display_2.getWidth()/4, 9);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(0, 16, display_2.getWidth(), display_2.getHeight()-16);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 15, String(systemData.translate_enable_bool[0][relayData.relays_enable[0][menuData.relay_select]]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 15, "R " + String(menuData.relay_select));
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()-7, 15, "F " + String(menuData.relay_function_select));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "" + String(relayData.relays[menuData.relay_select][menuData.relay_function_select]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 33, "X " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][0]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 42, "Y " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][1]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 51, "Z " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][2]));
      }
    // select row 2 center
    if (menuData.y == 2) {
      display_2.setColor(WHITE); display_2.fillRect(0, 26, display_2.getWidth(), 9);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(0, 16, display_2.getWidth(), display_2.getHeight()-16);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 15, String(systemData.translate_enable_bool[0][relayData.relays_enable[0][menuData.relay_select]]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 15, "R " + String(menuData.relay_select));
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 15, "F " + String(menuData.relay_function_select));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 24, "" + String(relayData.relays[menuData.relay_select][menuData.relay_function_select]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 33, "X " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][0]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 42, "Y " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][1]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 51, "Z " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][2]));
      }
    // select row 3 center
    if (menuData.y == 3) {
      display_2.setColor(WHITE); display_2.fillRect(0, 35, display_2.getWidth(), 9);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(0, 16, display_2.getWidth(), display_2.getHeight()-16);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 15, String(systemData.translate_enable_bool[0][relayData.relays_enable[0][menuData.relay_select]]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 15, "R " + String(menuData.relay_select));
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 15, "F " + String(menuData.relay_function_select));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "" + String(relayData.relays[menuData.relay_select][menuData.relay_function_select]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 33, "X " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][0]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 42, "Y " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][1]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 51, "Z " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][2]));
      }
    // select row 4 center
    if (menuData.y == 4) {
      display_2.setColor(WHITE); display_2.fillRect(0, 44, display_2.getWidth(), 9);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(0, 16, display_2.getWidth(), display_2.getHeight()-16);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 15, String(systemData.translate_enable_bool[0][relayData.relays_enable[0][menuData.relay_select]]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 15, "R " + String(menuData.relay_select));
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 15, "F " + String(menuData.relay_function_select));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "" + String(relayData.relays[menuData.relay_select][menuData.relay_function_select]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 33, "X " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][0]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 42, "Y " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][1]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 51, "Z " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][2]));
      }
    // select row 5 center
    if (menuData.y == 5) {
      display_2.setColor(WHITE); display_2.fillRect(0, 53, display_2.getWidth(), 10);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(0, 16, display_2.getWidth(), display_2.getHeight()-16);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 15, String(systemData.translate_enable_bool[0][relayData.relays_enable[0][menuData.relay_select]]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 15, "R " + String(menuData.relay_select));
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 15, "F " + String(menuData.relay_function_select));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "" + String(relayData.relays[menuData.relay_select][menuData.relay_function_select]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 33, "X " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][0]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 42, "Y " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][1]));
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 51, "Z " + String(relayData.relays_data[menuData.relay_select][menuData.relay_function_select][2]));
      }
  }

  if (menuData.page == 1) {
    display_2.clear();
    // select top center
    if ((menuData.y == 0) && (menuData.x == 1)) {
      display_2.setColor(WHITE); display_2.fillRect(display_2.getWidth()/4, 0, display_2.getWidth()/2, 14);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 0, "SATELLITE");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(0, 16, display_2.getWidth(), display_2.getHeight()-16);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(4, 18, "SATCOM ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-2, 18, String(systemData.translate_enable_bool[0][systemData.satcom_enabled]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(4, 28, "GNGGA ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-2, 28, String(systemData.translate_enable_bool[0][systemData.gngga_enabled]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(4, 38, "GNRMC ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-2, 38, String(systemData.translate_enable_bool[0][systemData.gnrmc_enabled]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(4, 48, "GPATT ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-2, 48, String(systemData.translate_enable_bool[0][systemData.gpatt_enabled]));
      }
    // select top left
    if ((menuData.y == 0) && (menuData.x == 0)) {
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "SATELLITE");
      display_2.setColor(WHITE); display_2.fillRect(2, 0, display_2.getWidth()/8, 14);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(BLACK); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(0, 16, display_2.getWidth(), display_2.getHeight()-16);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(4, 18, "SATCOM ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-2, 18, String(systemData.translate_enable_bool[0][systemData.satcom_enabled]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(4, 28, "GNGGA ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-2, 28, String(systemData.translate_enable_bool[0][systemData.gngga_enabled]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(4, 38, "GNRMC ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-2, 38, String(systemData.translate_enable_bool[0][systemData.gnrmc_enabled]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(4, 48, "GPATT ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-2, 48, String(systemData.translate_enable_bool[0][systemData.gpatt_enabled]));
      }
    // select top right
    if ((menuData.y == 0) && (menuData.x == 2)) {
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "SATELLITE");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setColor(WHITE); display_2.fillRect(display_2.getWidth()-(display_2.getWidth()/8)-2, 0, display_2.getWidth()/8, 14);
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(0, 16, display_2.getWidth(), display_2.getHeight()-16);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(4, 18, "SATCOM ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-2, 18, String(systemData.translate_enable_bool[0][systemData.satcom_enabled]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(4, 28, "GNGGA ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-2, 28, String(systemData.translate_enable_bool[0][systemData.gngga_enabled]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(4, 38, "GNRMC ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-2, 38, String(systemData.translate_enable_bool[0][systemData.gnrmc_enabled]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(4, 48, "GPATT ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-2, 48, String(systemData.translate_enable_bool[0][systemData.gpatt_enabled]));
      }
    // select row 1 center
    if (menuData.y == 1) {
      display_2.setColor(WHITE); display_2.fillRect(2, 20, display_2.getWidth(), 9);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "SATELLITE");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(0, 16, display_2.getWidth(), display_2.getHeight()-16);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(BLACK); display_2.drawString(4, 18, "SATCOM ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()-2, 18, String(systemData.translate_enable_bool[0][systemData.satcom_enabled]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(4, 28, "GNGGA ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-2, 28, String(systemData.translate_enable_bool[0][systemData.gngga_enabled]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(4, 38, "GNRMC ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-2, 38, String(systemData.translate_enable_bool[0][systemData.gnrmc_enabled]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(4, 48, "GPATT ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-2, 48, String(systemData.translate_enable_bool[0][systemData.gpatt_enabled]));
      }
    // select row 2 center
    if (menuData.y == 2) {
      display_2.setColor(WHITE); display_2.fillRect(2, 30, display_2.getWidth(), 9);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "SATELLITE");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(0, 16, display_2.getWidth(), display_2.getHeight()-16);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(4, 18, "SATCOM ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-2, 18, String(systemData.translate_enable_bool[0][systemData.satcom_enabled]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(BLACK); display_2.drawString(4, 28, "GNGGA ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()-2, 28, String(systemData.translate_enable_bool[0][systemData.gngga_enabled]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(4, 38, "GNRMC ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-2, 38, String(systemData.translate_enable_bool[0][systemData.gnrmc_enabled]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(4, 48, "GPATT ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-2, 48, String(systemData.translate_enable_bool[0][systemData.gpatt_enabled]));
      }
    // select row 3 center
    if (menuData.y == 3) {
      display_2.setColor(WHITE); display_2.fillRect(2, 40, display_2.getWidth(), 9);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "SATELLITE");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(0, 16, display_2.getWidth(), display_2.getHeight()-16);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(4, 18, "SATCOM ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-2, 18, String(systemData.translate_enable_bool[0][systemData.satcom_enabled]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(4, 28, "GNGGA ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-2, 28, String(systemData.translate_enable_bool[0][systemData.gngga_enabled]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(BLACK); display_2.drawString(4, 38, "GNRMC ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()-2, 38, String(systemData.translate_enable_bool[0][systemData.gnrmc_enabled]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(4, 48, "GPATT ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-2, 48, String(systemData.translate_enable_bool[0][systemData.gpatt_enabled]));
      }
    // select row 4 center
    if (menuData.y == 4) {
      display_2.setColor(WHITE); display_2.fillRect(2, 50, display_2.getWidth(), 9);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "SATELLITE");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(0, 16, display_2.getWidth(), display_2.getHeight()-16);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(4, 18, "SATCOM ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-2, 18, String(systemData.translate_enable_bool[0][systemData.satcom_enabled]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(4, 28, "GNGGA ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-2, 28, String(systemData.translate_enable_bool[0][systemData.gngga_enabled]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(4, 38, "GNRMC ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-2, 38, String(systemData.translate_enable_bool[0][systemData.gnrmc_enabled]));
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(BLACK); display_2.drawString(4, 48, "GPATT ");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()-2, 48, String(systemData.translate_enable_bool[0][systemData.gpatt_enabled]));
      }
    // null unless more entries
    if (menuData.y >= 5) {menuData.y = 0;}
  }

  if (menuData.page == 2) {
    display_2.clear();
    // select top center
    if ((menuData.y == 0) && (menuData.x == 1)) {
      display_2.setColor(WHITE); display_2.fillRect(display_2.getWidth()/4, 0, display_2.getWidth()/2, 14);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(2, 16, display_2.getWidth()-4, display_2.getHeight()-18);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "MATRIX " + String(systemData.translate_enable_bool[0][systemData.matrix_enabled]));
      }
    // select top left
    if ((menuData.y == 0) && (menuData.x == 0)) {
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setColor(WHITE); display_2.fillRect(2, 0, display_2.getWidth()/8, 14);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(BLACK); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(2, 16, display_2.getWidth()-4, display_2.getHeight()-18);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "MATRIX " + String(systemData.translate_enable_bool[0][systemData.matrix_enabled]));
      }
    // select top right
    if ((menuData.y == 0) && (menuData.x == 2)) {
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setColor(WHITE); display_2.fillRect(display_2.getWidth()-(display_2.getWidth()/8)-2, 0, display_2.getWidth()/8, 14);
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(2, 16, display_2.getWidth()-4, display_2.getHeight()-18);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "MATRIX " + String(systemData.translate_enable_bool[0][systemData.matrix_enabled]));
      }
    // select row 1 center
    if (menuData.y != 0) {
      display_2.setColor(WHITE); display_2.fillRect(2, 16, display_2.getWidth(), 10);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.fillRect(2, 16, display_2.getWidth()-4, display_2.getHeight()-18);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 24, "MATRIX " + String(systemData.translate_enable_bool[0][systemData.matrix_enabled]));
      }
    if (menuData.y >= 2) {menuData.y = 0;}
  }
  
  if (menuData.page == 3) {
    display_2.clear();
    // select top center
    if ((menuData.y == 0) && (menuData.x == 1)) {
      display_2.setColor(WHITE); display_2.fillRect(display_2.getWidth()/4, 0, display_2.getWidth()/2, 14);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(2, 16, display_2.getWidth()-4, display_2.getHeight()-18);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "DISABLE");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "ALL");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, "RELAYS");
      }
    // select top left
    if ((menuData.y == 0) && (menuData.x == 0)) {
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setColor(WHITE); display_2.fillRect(2, 0, display_2.getWidth()/8, 14);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(BLACK); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(2, 16, display_2.getWidth()-4, display_2.getHeight()-18);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "DISABLE");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "ALL");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, "RELAYS");
      }
    // select top right
    if ((menuData.y == 0) && (menuData.x == 2)) {
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setColor(WHITE); display_2.fillRect(display_2.getWidth()-(display_2.getWidth()/8)-2, 0, display_2.getWidth()/8, 14);
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(2, 16, display_2.getWidth()-4, display_2.getHeight()-18);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "DISABLE");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "ALL");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, "RELAYS");
      }
    // select row 1 center
    if (menuData.y != 0) {
      display_2.setColor(WHITE); display_2.fillRect(2, 16, display_2.getWidth(), 10);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.fillRect(2, 16, display_2.getWidth()-4, display_2.getHeight()-18);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 24, "DISABLE");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 34, "ALL");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 44, "RELAYS");
      }
    if (menuData.y >= 2) {menuData.y = 0;}
  }
    
  if (menuData.page == 4) {
    display_2.clear();
    // select top center
    if ((menuData.y == 0) && (menuData.x == 1)) {
      display_2.setColor(WHITE); display_2.fillRect(display_2.getWidth()/4, 0, display_2.getWidth()/2, 14);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(2, 16, display_2.getWidth()-4, display_2.getHeight()-18);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "DEACTIVATE");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "ALL");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, "RELAYS");
      }
    // select top left
    if ((menuData.y == 0) && (menuData.x == 0)) {
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setColor(WHITE); display_2.fillRect(2, 0, display_2.getWidth()/8, 14);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(BLACK); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(2, 16, display_2.getWidth()-4, display_2.getHeight()-18);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "DEACTIVATE");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "ALL");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, "RELAYS");
      }
    // select top right
    if ((menuData.y == 0) && (menuData.x == 2)) {
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setColor(WHITE); display_2.fillRect(display_2.getWidth()-(display_2.getWidth()/8)-2, 0, display_2.getWidth()/8, 14);
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(2, 16, display_2.getWidth()-4, display_2.getHeight()-18);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "DEACTIVATE");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "ALL");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, "RELAYS");
      }
    // select row 1 center
    if (menuData.y != 0) {
      display_2.setColor(WHITE); display_2.fillRect(2, 16, display_2.getWidth(), 10);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.fillRect(2, 16, display_2.getWidth()-4, display_2.getHeight()-18);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 24, "DEACTIVATE");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 34, "ALL");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 44, "RELAYS");
      }
    if (menuData.y >= 2) {menuData.y = 0;}
  }
  
  if (menuData.page == 5) {
    display_2.clear();
    // select top center
    if ((menuData.y == 0) && (menuData.x == 1)) {
      display_2.setColor(WHITE); display_2.fillRect(display_2.getWidth()/4, 0, display_2.getWidth()/2, 14);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(2, 16, display_2.getWidth()-4, display_2.getHeight()-18);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "ENABLE");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "ALL");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, "RELAYS");
      }
    // select top left
    if ((menuData.y == 0) && (menuData.x == 0)) {
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setColor(WHITE); display_2.fillRect(2, 0, display_2.getWidth()/8, 14);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(BLACK); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(2, 16, display_2.getWidth()-4, display_2.getHeight()-18);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "ENABLE");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "ALL");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, "RELAYS");
      }
    // select top right
    if ((menuData.y == 0) && (menuData.x == 2)) {
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setColor(WHITE); display_2.fillRect(display_2.getWidth()-(display_2.getWidth()/8)-2, 0, display_2.getWidth()/8, 14);
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(2, 16, display_2.getWidth()-4, display_2.getHeight()-18);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "ENABLE");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "ALL");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, "RELAYS");
      }
    // select row 1 center
    if (menuData.y != 0) {
      display_2.setColor(WHITE); display_2.fillRect(2, 16, display_2.getWidth(), 10);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.fillRect(2, 16, display_2.getWidth()-4, display_2.getHeight()-18);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 24, "ENABLE");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 34, "ALL");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 44, "RELAYS");
      }
    if (menuData.y >= 2) {menuData.y = 0;}
  }

  if (menuData.page == 6) {
    display_2.clear();
    // select top center
    if ((menuData.y == 0) && (menuData.x == 1)) {
      display_2.setColor(WHITE); display_2.fillRect(display_2.getWidth()/4, 0, display_2.getWidth()/2, 14);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(2, 16, display_2.getWidth()-4, display_2.getHeight()-18);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "ACTIVATE");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "ALL");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, "RELAYS");
      }
    // select top left
    if ((menuData.y == 0) && (menuData.x == 0)) {
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setColor(WHITE); display_2.fillRect(2, 0, display_2.getWidth()/8, 14);
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(BLACK); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(2, 16, display_2.getWidth()-4, display_2.getHeight()-18);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "ACTIVATE");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "ALL");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, "RELAYS");
      }
    // select top right
    if ((menuData.y == 0) && (menuData.x == 2)) {
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setColor(WHITE); display_2.fillRect(display_2.getWidth()-(display_2.getWidth()/8)-2, 0, display_2.getWidth()/8, 14);
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.drawRect(2, 16, display_2.getWidth()-4, display_2.getHeight()-18);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 24, "ACTIVATE");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 34, "ALL");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 44, "RELAYS");
      }
    // select row 1 center
    if (menuData.y != 0) {
      display_2.setColor(WHITE); display_2.fillRect(2, 16, display_2.getWidth(), 10);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()/2, 0, "RELAYS");
      display_2.setTextAlignment(TEXT_ALIGN_LEFT); display_2.setColor(WHITE); display_2.drawString(2, 0, "<");
      display_2.setTextAlignment(TEXT_ALIGN_RIGHT); display_2.setColor(WHITE); display_2.drawString(display_2.getWidth()-7, 0, ">");
      display_2.setColor(WHITE); display_2.fillRect(2, 16, display_2.getWidth()-4, display_2.getHeight()-18);
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 24, "ACTIVATE");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 34, "ALL");
      display_2.setTextAlignment(TEXT_ALIGN_CENTER); display_2.setColor(BLACK); display_2.drawString(display_2.getWidth()/2, 44, "RELAYS");
      }
    if (menuData.y >= 2) {menuData.y = 0;}
  }

  display_2.display();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    DISPLAY 3

void SSD_Display_MATRIX() {
  tcaselect(3);
  display_3.setTextAlignment(TEXT_ALIGN_CENTER);
  display_3.setColor(WHITE);
  display_3.clear();
  display_3.setColor(WHITE); display_3.fillRect(display_3.getWidth()/4, 0, display_3.getWidth()/2, 14);
  display_3.setTextAlignment(TEXT_ALIGN_CENTER); display_3.setColor(BLACK); display_3.drawString(display_2.getWidth()/2, 0, "MATRIX");
  display_3.setColor(WHITE); display_3.drawRect(0, 16, display_3.getWidth(), display_3.getHeight()-16);
  // if (menuData.page != 10) {
  //   display_3.drawString(display_3.getWidth()/2, 14, "x" + String(menuData.x) + " y" + String(menuData.y) + "  s" + String(menuData.select) + " p" + String(menuData.page)); // menu x,y telemetry is here for debug menu navigation (uncomment to use)
  // }
  // else if (menuData.page == 10) {
  //   display_3.drawString(display_3.getWidth()/2, 14, "x" + String(menuData.numpad_x) + " y" + String(menuData.numpad_y) + "  s" + String(menuData.select) + " p" + String(menuData.page)); // menu x,y telemetry is here for debug menu navigation (uncomment to use)
  // }
  display_3.setColor(WHITE); display_3.drawString(display_3.getWidth()/2,18,""+String(relayData.relays_bool[0][0])+"  "+String(relayData.relays_bool[0][1])+"  "+String(relayData.relays_bool[0][2])+"  "+String(relayData.relays_bool[0][3])+"  "+String(relayData.relays_bool[0][4])+"  "+String(relayData.relays_bool[0][5])+"  "+String(relayData.relays_bool[0][6])+"  "+String(relayData.relays_bool[0][7])+"  "+String(relayData.relays_bool[0][8])+"  "+String(relayData.relays_bool[0][9]));
  display_3.setColor(WHITE); display_3.drawString(display_3.getWidth()/2,28,""+String(relayData.relays_bool[0][10])+"  "+String(relayData.relays_bool[0][11])+"  "+String(relayData.relays_bool[0][12])+"  "+String(relayData.relays_bool[0][13])+"  "+String(relayData.relays_bool[0][14])+"  "+String(relayData.relays_bool[0][15])+"  "+String(relayData.relays_bool[0][16])+"  "+String(relayData.relays_bool[0][17])+"  "+String(relayData.relays_bool[0][18])+"  "+String(relayData.relays_bool[0][19]));
  display_3.setColor(WHITE); display_3.drawString(display_3.getWidth()/2,38,""+String(relayData.relays_bool[0][20])+"  "+String(relayData.relays_bool[0][21])+"  "+String(relayData.relays_bool[0][22])+"  "+String(relayData.relays_bool[0][23])+"  "+String(relayData.relays_bool[0][24])+"  "+String(relayData.relays_bool[0][25])+"  "+String(relayData.relays_bool[0][26])+"  "+String(relayData.relays_bool[0][27])+"  "+String(relayData.relays_bool[0][28])+"  "+String(relayData.relays_bool[0][29]));
  display_3.setColor(WHITE); display_3.drawString(display_3.getWidth()/2,48,""+String(relayData.relays_bool[0][30])+"  "+String(relayData.relays_bool[0][31])+"  "+String(relayData.relays_bool[0][32])+"  "+String(relayData.relays_bool[0][33])+"  "+String(relayData.relays_bool[0][34])+"  "+String(relayData.relays_bool[0][35])+"  "+String(relayData.relays_bool[0][36])+"  "+String(relayData.relays_bool[0][37])+"  "+String(relayData.relays_bool[0][38])+"  "+String(relayData.relays_bool[0][39]));
  display_3.display();
}

void SSD_Display_MATRIX_Disabled() {
  tcaselect(3);
  display_3.setTextAlignment(TEXT_ALIGN_CENTER);
  display_3.setColor(WHITE); 
  display_3.clear();
  display_3.setColor(WHITE); display_3.fillRect(display_3.getWidth()/4, 0, display_3.getWidth()/2, 15);
  display_3.setTextAlignment(TEXT_ALIGN_CENTER); display_3.setColor(BLACK); display_3.drawString(display_3.getWidth()/2, 0, "MATRIX");
  display_3.setColor(WHITE); display_3.drawRect(0, 16, display_3.getWidth(), display_3.getHeight()-16);
  display_3.setColor(WHITE); display_3.drawString(display_3.getWidth()/2, 24, "[ DISABLED ]");
  display_3.display();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    DISPLAY 4

void SSD_Display_GPATT() {
  tcaselect(4);
  display_4.setTextAlignment(TEXT_ALIGN_CENTER);
  display_4.setColor(WHITE);
  display_4.clear();
  display_4.setColor(WHITE); display_4.fillRect(display_4.getWidth()/4, 0, display_4.getWidth()/2, 14);
  display_4.setTextAlignment(TEXT_ALIGN_CENTER); display_4.setColor(BLACK); display_4.drawString(display_4.getWidth()/2, 0, "GPATT");
  display_4.setColor(WHITE); display_4.drawRect(0, 16, display_4.getWidth(), display_4.getHeight()-16);
  display_4.setColor(WHITE); display_4.drawString(display_4.getWidth()/2, 18, "P " + String(gpattData.pitch));
  display_4.setColor(WHITE); display_4.drawString(display_4.getWidth()/2, 28, "R " + String(gpattData.roll));
  display_4.setColor(WHITE); display_4.drawString(display_4.getWidth()/2, 38, "Y " + String(gpattData.yaw));
  display_4.setColor(WHITE); display_4.drawString(display_4.getWidth()/2, 48, String("RSF " + String(gpattData.run_state_flag) + " GST " + String(gpattData.gst_data) + " RIF " + String(gpattData.run_inetial_flag)));
  display_4.display();
}

void SSD_Display_GPATT_Disabled() {
  tcaselect(4);
  display_4.setTextAlignment(TEXT_ALIGN_CENTER);
  display_4.setColor(WHITE); 
  display_4.clear();
  display_4.setColor(WHITE); display_4.fillRect(display_4.getWidth()/4, 0, display_4.getWidth()/2, 14);
  display_4.setTextAlignment(TEXT_ALIGN_CENTER); display_4.setColor(BLACK); display_4.drawString(display_4.getWidth()/2, 0, "GPATT");
  display_4.setColor(WHITE); display_4.drawRect(0, 16, display_4.getWidth(), display_4.getHeight()-16);
  display_4.setColor(WHITE); display_4.drawString(display_4.getWidth()/2, 24, "[ DISABLED ]");
  display_4.display();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    DISPLAY 5

void SSD_Display_SATCOM() {
  tcaselect(5);
  display_5.setTextAlignment(TEXT_ALIGN_CENTER);
  display_5.setColor(WHITE);
  display_5.clear();
  display_5.setColor(WHITE); display_5.fillRect(display_5.getWidth()/4, 0, display_5.getWidth()/2, 14);
  display_5.setTextAlignment(TEXT_ALIGN_CENTER); display_5.setColor(BLACK); display_5.drawString(display_5.getWidth()/2, 0, "SATCOM");
  display_5.setColor(WHITE); display_5.drawRect(0, 16, display_5.getWidth(), display_5.getHeight()-16);
  display_5.setColor(WHITE); display_5.drawString(display_5.getWidth()/2, 18, satData.sat_time_stamp_string);
  display_5.setColor(WHITE); display_5.drawString(display_5.getWidth()/2, 28, String(satData.last_sat_time_stamp_str));
  display_5.setColor(WHITE); display_5.drawString(display_5.getWidth()/2, 38, String(gnggaData.latitude_hemisphere) + " " + satData.location_latitude_gngga_str);
  display_5.setColor(WHITE); display_5.drawString(display_5.getWidth()/2, 48, String(gnggaData.longitude_hemisphere) + " " + satData.location_longitude_gngga_str);
  display_5.display();
}

void SSD_Display_SATCOM_Disabled() {
  tcaselect(5);
  display_5.setTextAlignment(TEXT_ALIGN_CENTER);
  display_5.setColor(WHITE); 
  display_5.clear();
  display_5.setColor(WHITE); display_5.fillRect(display_5.getWidth()/4, 0, display_5.getWidth()/2, 14);
  display_5.setTextAlignment(TEXT_ALIGN_CENTER); display_5.setColor(BLACK); display_5.drawString(display_5.getWidth()/2, 0, "SATCOM");
  display_5.setColor(WHITE); display_5.drawRect(0, 16, display_5.getWidth(), display_5.getHeight()-16);
  display_5.setColor(WHITE); display_5.drawString(display_5.getWidth()/2, 24, "[ DISABLED ]");
  display_5.display();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    DISPLAY 6

void SSD_Display_GNGGA() {
  display_6.setColor(WHITE);
  tcaselect(6);
  display_6.setTextAlignment(TEXT_ALIGN_CENTER);
  display_6.clear();
  display_6.setColor(WHITE); display_6.fillRect(display_6.getWidth()/4, 0, display_6.getWidth()/2, 14);
  display_6.setTextAlignment(TEXT_ALIGN_CENTER); display_6.setColor(BLACK); display_6.drawString(display_6.getWidth()/2, 0, "GNGGA");
  display_6.setColor(WHITE); display_6.drawRect(0, 16, display_6.getWidth(), display_6.getHeight()-16);
  display_6.setColor(WHITE); display_6.drawString(display_6.getWidth()/2, 15, "PF " + String(gnggaData.hdop_precision_factor) + " P " + String(gnggaData.positioning_status) + " S " + String(gnggaData.satellite_count_gngga));
  display_6.setColor(WHITE); display_6.drawString(display_6.getWidth()/2, 24, String(gnggaData.utc_time));
  display_6.setColor(WHITE); display_6.drawString(display_6.getWidth()/2, 33, String(gnggaData.latitude_hemisphere) + " " + String(gnggaData.latitude));
  display_6.setColor(WHITE); display_6.drawString(display_6.getWidth()/2, 42, String(gnggaData.longitude_hemisphere) + " " + String(gnggaData.longitude));
  display_6.setColor(WHITE); display_6.drawString(display_6.getWidth()/2, 51, "A " + String(gnggaData.altitude));
  display_6.display();
}

void SSD_Display_GNGGA_Disabled() {
  tcaselect(6);
  display_6.setTextAlignment(TEXT_ALIGN_CENTER);
  display_6.setColor(WHITE);
  display_6.clear();
  display_6.setColor(WHITE); display_6.fillRect(display_6.getWidth()/4, 0, display_6.getWidth()/2, 14);
  display_6.setTextAlignment(TEXT_ALIGN_CENTER); display_6.setColor(BLACK); display_6.drawString(display_6.getWidth()/2, 0, "GNGGA");
  display_6.setColor(WHITE); display_6.drawRect(0, 16, display_6.getWidth(), display_6.getHeight()-16);
  display_6.setColor(WHITE); display_6.drawString(display_6.getWidth()/2, 24, "[ DISABLED ]");
  display_6.display();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    DISPLAY 7

void SSD_Display_GNRMC() {
  tcaselect(7);
  display_7.setTextAlignment(TEXT_ALIGN_CENTER);
  display_7.setColor(WHITE);
  display_7.clear();
  display_7.setColor(WHITE); display_7.fillRect(display_7.getWidth()/4, 0, display_7.getWidth()/2, 14);
  display_7.setTextAlignment(TEXT_ALIGN_CENTER); display_7.setColor(BLACK); display_7.drawString(display_7.getWidth()/2, 0, "GNRMC");
  display_7.setColor(WHITE); display_7.drawRect(0, 16, display_7.getWidth(), display_7.getHeight()-16);
  display_7.setColor(WHITE); display_7.drawString(display_7.getWidth()/2, 15, "P " + String(gnrmcData.positioning_status) + " M " + String(gnrmcData.mode_indication));
  display_7.setColor(WHITE);display_7.drawString(display_7.getWidth()/2, 24, String(gnrmcData.utc_time) + " " + String(gnrmcData.utc_date));
  display_7.setColor(WHITE);display_7.drawString(display_7.getWidth()/2, 33, String(gnrmcData.latitude_hemisphere) + " " + String(gnrmcData.latitude));
  display_7.setColor(WHITE);display_7.drawString(display_7.getWidth()/2, 42, String(gnrmcData.longitude_hemisphere) + " " + String(gnrmcData.longitude));
  display_7.setColor(WHITE);display_7.drawString(display_7.getWidth()/2, 51, "H " + String(gnrmcData.ground_heading) + " S " + String(gnrmcData.ground_speed));
  display_7.display();
}

void SSD_Display_GNRMC_Disabled() {
  tcaselect(7);
  display_7.setTextAlignment(TEXT_ALIGN_CENTER);
  display_7.setColor(WHITE);
  display_7.clear();
  display_7.setColor(WHITE); display_7.fillRect(display_7.getWidth()/4, 0, display_7.getWidth()/2, 14);
  display_7.setTextAlignment(TEXT_ALIGN_CENTER); display_7.setColor(BLACK); display_7.drawString(display_7.getWidth()/2, 0, "GNRMC");
  display_7.setColor(WHITE); display_7.drawRect(0, 16, display_7.getWidth(), display_7.getHeight()-16);
  display_7.setColor(WHITE);display_7.drawString(display_7.getWidth()/2, 24, "[ DISABLED ]");
  display_7.display();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                             DISPLAYS LOADING

void SSD_Display_Loading() {
  tcaselect(2);
  display_2.setTextAlignment(TEXT_ALIGN_CENTER);
  display_2.setColor(WHITE);
  display_2.clear();
  display_2.drawString(display_2.getWidth()/2, 0, "[LOADING]");
  display_2.display();
  tcaselect(3);
  display_3.setTextAlignment(TEXT_ALIGN_CENTER);
  display_3.setColor(WHITE);
  display_3.clear();
  display_3.drawString(display_3.getWidth()/2, 0, "[LOADING]");
  display_3.display();
  tcaselect(4);
  display_4.setTextAlignment(TEXT_ALIGN_CENTER);
  display_4.setColor(WHITE);
  display_4.clear();
  display_4.drawString(display_4.getWidth()/2, 0, "[LOADING]");
  display_4.display();
  tcaselect(5);
  display_5.setTextAlignment(TEXT_ALIGN_CENTER);
  display_5.setColor(WHITE);
  display_5.clear();
  display_5.drawString(display_5.getWidth()/2, 0, "[LOADING]");
  display_5.display();
  tcaselect(6);
  display_6.setTextAlignment(TEXT_ALIGN_CENTER);
  display_6.setColor(WHITE);
  display_6.clear();
  display_6.drawString(display_6.getWidth()/2, 0, "[LOADING]");
  display_6.display();
  tcaselect(7);
  display_7.setTextAlignment(TEXT_ALIGN_CENTER);
  display_7.setColor(WHITE);
  display_7.clear();
  display_7.drawString(display_7.getWidth()/2, 0, "[LOADING]");
  display_7.display();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                         DISPLAYS: BRIGHTNESS

/*
there may be some disparity between the SSD306 panels at low brightness, possible reasons:
inconsistent current through multiplexer (sda,sdc): checked. negative.
inconsistent voltage through multiplexer (sda,sdc): checked. negative
inconsistent vcc current: checked. negative.
inconsistent vcc voltage: checked. negative.
software library: pending checks.
implementation here of software library: pending checks.
currently recommended lowest settings: 150, 16.
*/

void displayBrightness(int c, int d, int e, int f, int g, int h) {
  if (c==1) {tcaselect(2); display_2.setContrast(255, 241);} else {tcaselect(2); display_2.setContrast(150, 16);}
  if (d==1) {tcaselect(3); display_3.setContrast(255, 241);} else {tcaselect(3); display_3.setContrast(150, 16);}
  if (e==1) {tcaselect(4); display_4.setContrast(255, 241);} else {tcaselect(4); display_4.setContrast(150, 16);}
  if (f==1) {tcaselect(5); display_5.setContrast(255, 241);} else {tcaselect(5); display_5.setContrast(150, 16);}
  if (g==1) {tcaselect(6); display_6.setContrast(255, 241);} else {tcaselect(6); display_6.setContrast(150, 16);}
  if (h==1) {tcaselect(7); display_7.setContrast(255, 241);} else {tcaselect(7); display_7.setContrast(150, 16);}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                             DISPLAYS: ON/OFF

void displayOnOff(int c, int d, int e, int f, int g, int h) {
  if (c==1) {tcaselect(2); display_2.displayOn();} else {tcaselect(2); display_2.displayOff();}
  if (d==1) {tcaselect(3); display_3.displayOn();} else {tcaselect(3); display_3.displayOff();}
  if (e==1) {tcaselect(4); display_4.displayOn();} else {tcaselect(4); display_4.displayOff();}
  if (f==1) {tcaselect(5); display_5.displayOn();} else {tcaselect(5); display_5.displayOff();}
  if (g==1) {tcaselect(6); display_6.displayOn();} else {tcaselect(6); display_6.displayOff();}
  if (h==1) {tcaselect(7); display_7.displayOn();} else {tcaselect(7); display_7.displayOff();}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                    DISPLAYS: FLIP VERTICALLY

/*
may not be preferrable for SSD1306 panels that are yellow and blue (or other dual color) but may prove useful for other SSD1306 panels.
*/

void displayFlipVertically(int c, int d, int e, int f, int g, int h) {
  if (c==1) {tcaselect(2); display_2.init(); display_2.flipScreenVertically();} else {tcaselect(2); display_2.init(); display_2.normalDisplay();}
  if (d==1) {tcaselect(3); display_3.init(); display_3.flipScreenVertically();} else {tcaselect(3); display_3.init(); display_3.normalDisplay();}
  if (e==1) {tcaselect(4); display_4.init(); display_4.flipScreenVertically();} else {tcaselect(4); display_4.init(); display_4.normalDisplay();}
  if (f==1) {tcaselect(5); display_5.init(); display_5.flipScreenVertically();} else {tcaselect(5); display_5.init(); display_5.normalDisplay();}
  if (g==1) {tcaselect(6); display_6.init(); display_6.flipScreenVertically();} else {tcaselect(6); display_6.init(); display_6.normalDisplay();}
  if (h==1) {tcaselect(7); display_7.init(); display_7.flipScreenVertically();} else {tcaselect(7); display_7.init(); display_7.normalDisplay();}
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

        // ensure cleared
        memset(sdcardData.data_0, 0, 56); memset(sdcardData.data_1, 0, 56); memset(sdcardData.data_2, 0, 56);
        memset(sdcardData.data_3, 0, 56); memset(sdcardData.data_4, 0, 56); memset(sdcardData.data_5, 0, 56);
        memset(sdcardData.data_6, 0, 56);
        validData.bool_data_0 = false;
        validData.bool_data_1 = false;

        // split line on delimiter
        sdcardData.token = strtok(sdcardData.BUFFER, ",");

        // relay index
        sdcardData.token = strtok(NULL, ",");
        strcpy(sdcardData.data_0, sdcardData.token);
        if (is_all_digits(sdcardData.data_0) == true) {validData.bool_data_0 = true; Serial.println("[Ri] [PASS] " +String(sdcardData.data_0));}
        else {Serial.println("[Ri] [INVALID] " +String(sdcardData.data_0));}

        // relay function index
        sdcardData.token = strtok(NULL, ",");
        strcpy(sdcardData.data_1, sdcardData.token);
        if (is_all_digits(sdcardData.data_1) == true) {validData.bool_data_1 = true; Serial.println("[Fi] [PASS] " +String(sdcardData.data_1));}
        else {Serial.println("[Fi] [INVALID] " +String(sdcardData.data_1));}
        
        if ((validData.bool_data_0 == true) && (validData.bool_data_1 == true)) {

          // relay function name
          sdcardData.token = strtok(NULL, ",");
          strcpy(sdcardData.data_2, sdcardData.token);
          memset(relayData.relays[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)], 0, 56);
          strcpy(relayData.relays[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)], sdcardData.data_2);
          Serial.println("[Fn] [MATRIX] " +String(relayData.relays[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)]));

          // relay function data: x
          sdcardData.token = strtok(NULL, ",");
          strcpy(sdcardData.data_3, sdcardData.token);
          if (is_positive_negative_num(sdcardData.data_3) == true) {
            relayData.relays_data[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][0] = atol(sdcardData.data_3);
            Serial.println("[X]  [MATRIX] " +String(relayData.relays_data[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][0]));}
          else {Serial.println("[X] [INVALID] " + String(sdcardData.data_3));}

          // relay function data: y
          sdcardData.token = strtok(NULL, ",");
          strcpy(sdcardData.data_4, sdcardData.token);
          if (is_positive_negative_num(sdcardData.data_4) == true) {
            relayData.relays_data[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][1] = atol(sdcardData.data_4);
            Serial.println("[Y]  [MATRIX] " +String(relayData.relays_data[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][1]));}
          else {Serial.println("[Y] [INVALID] " + String(sdcardData.data_4));}
          
          // relay function data: z
          sdcardData.token = strtok(NULL, ",");
          strcpy(sdcardData.data_5, sdcardData.token);
          if (is_positive_negative_num(sdcardData.data_5) == true) {
            relayData.relays_data[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][2] = atol(sdcardData.data_5);
            Serial.println("[Z]  [MATRIX] " +String(relayData.relays_data[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][2]));}
          else {Serial.println("[Z] [INVALID] " + String(sdcardData.data_5));}
        }
      }
      else if (strncmp(sdcardData.BUFFER, "e", 1) == 0) {
        Serial.println("[checking E]");
        sdcardData.token = strtok(sdcardData.BUFFER, ",");
        sdcardData.token = strtok(NULL, ",");
        sdcardData.token = strtok(NULL, ",");
        strcpy(sdcardData.data_6, sdcardData.token);
        if (is_all_digits(sdcardData.data_6) == true) {
          relayData.relays_enable[0][atoi(sdcardData.data_0)] = atoi(sdcardData.data_6);
          Serial.println("[E]  [MATRIX] " +String(relayData.relays_enable[0][atoi(sdcardData.data_0)]));
          }
        else {Serial.println("[E]  [INVALID] " +String(sdcardData.data_6));}
      }
    }
    sdcardData.current_file.close();
    return true;
  }
  else {sdcardData.current_file.close(); return false;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                  WRITE MATRIX DATA TO SDCARD 

bool sdcard_save_matrix(char * file) {
  // sdcardData
  sdcardData.current_file = sd.open(file, FILE_WRITE);
  sdcardData.current_file.rewind();
  if (sdcardData.current_file) {
    for (int Ri = 0; Ri < relayData.MAX_RELAYS; Ri++) {
      for (int Fi = 0; Fi < relayData.MAX_RELAY_ELEMENTS; Fi++) {
        memset(sdcardData.file_data, 0 , 256);
        // tag: relay (r)
        strcat(sdcardData.file_data, sdcardData.tag_0); strcat(sdcardData.file_data, sdcardData.delim);
        // relay index
        memset(sdcardData.tmp, 0 , 256);
        sprintf(sdcardData.tmp, "%d", Ri);
        strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
        // relay function index
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
        strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
        // write line
        Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
        sdcardData.current_file.println(sdcardData.file_data);
      }
      memset(sdcardData.file_data, 0 , 256);
      // tag: enable (e)
      strcat(sdcardData.file_data, sdcardData.tag_1); strcat(sdcardData.file_data, sdcardData.delim);
      // relay index
      memset(sdcardData.tmp, 0 , 256);
      sprintf(sdcardData.tmp, "%d", Ri);
      strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
      // relay enabled 0/1
      memset(sdcardData.tmp, 0 , 256);
      itoa(relayData.relays_enable[0][Ri], sdcardData.tmp, 10);
      strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
      // write line
      Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
      sdcardData.current_file.println("");
      sdcardData.current_file.println(sdcardData.file_data);
      sdcardData.current_file.println("");
    }
    sdcardData.current_file.close();
    return true;
  }
  else {sdcardData.current_file.close(); return false;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                             MATRIX SET ENTRY

/*
                                        R F Function Name              X Y Z
example test command: $MATRIX_SET_ENTRY,0,0,SatelliteCountOver,1,0,0
example test command: $MATRIX_SET_ENTRY,0,0,SatelliteCountOver,-1,0,0
clear test command:   $MATRIX_SET_ENTRY,0,0,$NONE,0,0,0
*/

void matrix_set_entry() {
  Serial.println("[matrix_set_entry] connected");
  serial0Data.check_data_R = 0;
  memset(serial0Data.data_0, 0, 56);
  memset(serial0Data.data_1, 0, 56);
  memset(serial0Data.data_2, 0, 56);
  memset(serial0Data.data_3, 0, 56);
  memset(serial0Data.data_4, 0, 56);
  memset(serial0Data.data_5, 0, 56);
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
  }
  //                      [           RN          ][          FN            ][      VALUE        ]
  char *ptr;
  strcpy(relayData.relays[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)], serial0Data.data_2);      // set function
  relayData.relays_data[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)][0]=strtod(serial0Data.data_3, &ptr); // set function value x
  relayData.relays_data[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)][1]=strtod(serial0Data.data_4, &ptr); // set function value y
  relayData.relays_data[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)][2]=strtod(serial0Data.data_5, &ptr); // set function value z

  Serial.println("[Ri] " +String(serial0Data.data_0));
  Serial.println("[Fi] " +String(serial0Data.data_1));
  Serial.println("[Fn] " +String(relayData.relays[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)]));
  Serial.println("[X] " +String(relayData.relays_data[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)][0]));
  Serial.println("[Y] " +String(relayData.relays_data[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)][1]));
  Serial.println("[Z] " +String(relayData.relays_data[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)][2]));
}


// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                          MATRIX ENABLE ENTRY

void matrix_set_enabled(bool b) {
  Serial.println("[matrix_set_enabled] connected");
  serial0Data.check_data_R = 0;
  memset(serial0Data.data_0, 0, 56);
  serial0Data.iter_token = 0;
  serial0Data.token = strtok(serial0Data.BUFFER, ",");
  while( serial0Data.token != NULL ) {
    if      (serial0Data.iter_token == 0) {}
    else if (serial0Data.iter_token == 1) {strcpy(serial0Data.data_0, serial0Data.token);} // 0/1
    serial0Data.token = strtok(NULL, ",");
    serial0Data.iter_token++;
  }
  if (sysDebugData.serial_0_sentence == true) {
    Serial.println("[serial0Data.data_0] "         + String(serial0Data.data_0));
  }
  relayData.relays_enable[0][atoi(serial0Data.data_0)] = b; // set enable/disable
  Serial.println("[R] " + String(serial0Data.data_0) + " [E] " + String(relayData.relays_enable[0][atoi(serial0Data.data_0)]));
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                           MATRIX DISABLE ALL

// disable all matrix entries. does not directly turn relays off, instead prevents relays turning on. 
void matrix_disable_all() {for (int Ri = 0; Ri < relayData.MAX_RELAYS; Ri++) {relayData.relays_enable[0][Ri]=0;}}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                            MATRIX ENABLE ALL


// enable all matrix entries. does not directly turn relays on, instead enables relays turning on.
void matrix_enable_all() {for (int Ri = 0; Ri < relayData.MAX_RELAYS; Ri++) {relayData.relays_enable[0][Ri]=1;}}


// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                       MATRIX: ALL RELAYS OFF

// turn all relays off. recommended to first disable matrix.
void matrix_relays_disable_all() {for (int Ri = 0; Ri < relayData.MAX_RELAYS; Ri++) {relayData.relays_bool[0][Ri]=0;}}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                        MATRIX: ALL RELAYS ON

// turn all relays on. recommended to first disable matrix.
void matrix_relays_enable_all() {for (int Ri = 0; Ri < relayData.MAX_RELAYS; Ri++) {relayData.relays_bool[0][Ri]=1;}}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                   SATCOM CONVERT COORDINATES


// enable/disable coordinate conversion. performance/efficiency as required.

void satcom_convert_coordinates_on()  {satData.convert_coordinates = true;}
void satcom_convert_coordinates_off() {satData.convert_coordinates = false;}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        SETUP

void setup() {

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                               SETUP SERIAL

  Serial.begin(115200);
  Serial1.begin(115200); // ( RXD from WTGPS300P's TXD. io26 on ESP32 )

  delay(1000);

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                 SETUP WIRE

  Wire.begin();

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                              SETUP DISPLAY

  tcaselect(2);
  initDisplay2();
  display_2.flipScreenVertically();

  tcaselect(3);
  initDisplay3();
  display_3.flipScreenVertically();

  tcaselect(4);
  initDisplay4();
  display_4.flipScreenVertically();

  tcaselect(5);
  initDisplay5();
  display_5.flipScreenVertically();

  tcaselect(6);
  initDisplay6();
  display_6.flipScreenVertically();

  tcaselect(7);
  initDisplay7();
  display_7.flipScreenVertically();

  SSD_Display_Loading();

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                               SETUP SDCARD

  init_sdcard();
  sdcard_load_matrix("matrix.txt");
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                    MATRIX CHECKS: PRIMITIVES

/*
matrix switch requires all checks to return true for a relay to be active, therefore checks can be inverted as required, to return
true when otherwise a check would return false.
*/

// calculate if n0 in (+- range/2) of n1
bool in_range_check_true(double n0, double n1, double r) {
  if (n0  >=  n1 - r/2) {if (n0  <= n1 + r/2) {return true;}}
  else {return false;}
}

// calculate if n0 in (+- range/2) of n1
bool in_range_check_false(double n0, double n1, double r) {
  if (n0  >=  n1 - r/2) {if (n0  <= n1 + r/2) {return false;}}
  else {return true;}
}

bool iin_ranges_check_true(double x0, double x1, double y0, double y1, double r) {
  if (in_range_check_true(x0, x1, r) == true) {
    if (in_range_check_true(y0, y1, r) == true) {return true;} else return false;}
  else {return false;}
}

bool in_ranges_check_false(double x0, double x1, double y0, double y1, double r) {
  if (in_range_check_true(x0, x1, r) == true) {
    if (in_range_check_true(y0, y1, r) == true) {return false;} else return true;}
  else {return true;}
}

bool check_over_true(double n0, double n1) {
  if (n0 > n1) {return true;}
  else {return false;}
}

bool check_over_false(double n0, double n1) {
  if (n0 > n1) {return false;}
  else {return true;}
}

bool check_under_true(double n0, double n1) {
  if (n0 < n1) {return true;}
  else {return false;}
}

bool check_under_false(double n0, double n1) {
  if (n0 < n1) {return false;}
  else {return true;}
}

bool check_equal_true(double n0, double n1) {
  if (n0 == n1) {return true;}
  else {return false;}
}

bool check_equal_false(double n0, double n1) {
  if (n0 != n1) {return true;}
  else {return false;}
}

bool check_ge_and_le_true(double n0, double n1, double n2) {
  if ((n0 >= n1) && (n0 <= n2)) {return true;}
  else {return false;}
}

bool check_ge_and_le_false(double n0, double n1, double n2) {
  if ((n0 >= n1) && (n0 <= n2)) {return false;}
  else {return true;}
}

bool check_strncmp_true(char * c0, char * c1, int n) {
  if (strncmp(c0, c1, n) == 0) {return true;}
  else {return false;}
}

bool check_strncmp_false(char * c0, char * c1, int n) {
  if (strncmp(c0, c1, n) == 0) {return false;}
  else {return true;}
}

bool check_bool_true(bool _bool) {
  if (_bool == true) {return true;} else {return false;}
}

bool check_bool_false(bool _bool) {
  if (_bool == false) {return true;} else {return false;}
}

bool SecondsTimer(unsigned long n0, unsigned long n1, int Ri) {
  // max seconds 18446744073709551616 (584942417355.07202148 years)
  // n0: interval
  // n1: on time
  // backend interface example for on 1sec/off 1sec: $MATRIX_SET_ENTRY,0,0,SecondsTimer,1,1,0
  if ((timeData.seconds - relayData.relays_timing[0][Ri]) > n0) {relayData.relays_timing[0][Ri] = timeData.seconds; return true;}
  else if ((timeData.seconds - relayData.relays_timing[0][Ri]) < n1) {return true;}
  else {return false;}
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

    // Serial.println("[Ri] " + String(Ri) + " [E] " + String(relayData.relays_enable[0][Ri]));
    
    if (relayData.relays_enable[0][Ri] == 1) {

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
        //                                                                                                    SYSTEMS CHECKS: TIME DATA

        else if (strcmp(relayData.relays[Ri][Fi], relayData.SecondsTimer) == 0) {tmp_matrix[Fi] = SecondsTimer(relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1], Ri);}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                       SYSTEMS CHECKS: SATCOM

        // SATCOM: GNGGA
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLatGNGGAOver) == 0) {tmp_matrix[Fi] = check_over_true(satData.location_latitude_gngga, relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLatGNGGAUnder) == 0) {tmp_matrix[Fi] = check_under_true(satData.location_latitude_gngga, relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLatGNGGAEqual) == 0) {tmp_matrix[Fi] = check_equal_true(satData.location_latitude_gngga, relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLatGNGGARange) == 0) {tmp_matrix[Fi] = in_range_check_true(satData.location_latitude_gngga, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);} // is n in [2]range of [0]x (no y required)

        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLonGNGGAOver) == 0) {tmp_matrix[Fi] = check_over_true(satData.location_longitude_gngga, relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLonGNGGAUnder) == 0) {tmp_matrix[Fi] = check_under_true(satData.location_longitude_gngga, relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLonGNGGAEqual) == 0) {tmp_matrix[Fi] = check_equal_true(satData.location_longitude_gngga, relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLonGNGGARange) == 0) {tmp_matrix[Fi] = in_range_check_true(satData.location_longitude_gngga, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);} // is n in [2]range of [0]x (no y required)
        // check latitude and longitude in range: matrix {x, y, z}
        //                                                                                                                               x0                              x1                                 y0                               y1                               r   
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesGNGGARange) == 0) {tmp_matrix[Fi] = iin_ranges_check_true(satData.location_latitude_gngga, relayData.relays_data[Ri][Fi][0], satData.location_longitude_gngga, relayData.relays_data[Ri][Fi][1], relayData.relays_data[Ri][Fi][2]);}

        // SATCOM: GNRMC
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLatGNRMCOver) == 0) {tmp_matrix[Fi] = check_over_true(satData.location_latitude_gnrmc, relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLatGNRMCUnder) == 0) {tmp_matrix[Fi] = check_under_true(satData.location_latitude_gnrmc, relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLatGNRMCEqual) == 0) {tmp_matrix[Fi] = check_equal_true(satData.location_latitude_gnrmc, relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLatGNRMCRange) == 0) {tmp_matrix[Fi] = in_range_check_true(satData.location_latitude_gnrmc, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);} // is n in [2]range of [0]x (no y required)

        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLonGNRMCOver) == 0) {tmp_matrix[Fi] = check_over_true(satData.location_longitude_gnrmc, relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLonGNRMCUnder) == 0) {tmp_matrix[Fi] = check_under_true(satData.location_longitude_gnrmc, relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLonGNRMCEqual) == 0) {tmp_matrix[Fi] = check_equal_true(satData.location_longitude_gnrmc, relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.longitude_satcom_gnrmc_in_range) == 0) {tmp_matrix[Fi] = in_range_check_true(satData.location_longitude_gnrmc, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);} // is n in [2]range of [0]x (no y required)
        // check latitude and longitude in range: matrix {x, y, z}
        //                                                                                                                               x0                              x1                                 y0                               y1                               r   
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesGNGGARange) == 0) {tmp_matrix[Fi] = iin_ranges_check_true(satData.location_latitude_gnrmc, relayData.relays_data[Ri][Fi][0], satData.location_longitude_gnrmc, relayData.relays_data[Ri][Fi][1], relayData.relays_data[Ri][Fi][2]);}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                        SYSTEMS CHECKS: GNGGA

        else if (strcmp(relayData.relays[Ri][Fi], relayData.LatGNGGAOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gnggaData.latitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LatGNGGAUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gnggaData.latitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LatGNGGAEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnggaData.latitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LatGNGGARange) == 0) {tmp_matrix[Fi] = in_range_check_true(atol(gnggaData.latitude), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);} // is n in [2]range of [0]x (no y required)

        else if (strcmp(relayData.relays[Ri][Fi], relayData.LonGNGGAOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gnggaData.longitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LonGNGGAUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gnggaData.longitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LonGNGGAEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnggaData.longitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LonGNGGARange) == 0) {tmp_matrix[Fi] = in_range_check_true(atol(gnggaData.longitude), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);} // is n in [2]range of [0]x (no y required)

        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCTimeGNGGAOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gnggaData.utc_time), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCTimeGNGGAUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gnggaData.utc_time), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCTimeGNGGAEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnggaData.utc_time), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCTimeGNGGARange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gnggaData.utc_time), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.PositioningStatusGNGGA) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnggaData.positioning_status), relayData.relays_data[Ri][Fi][0]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.SatelliteCountOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gnggaData.satellite_count_gngga), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.SatelliteCountUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gnggaData.satellite_count_gngga), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.SatelliteCountEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnggaData.satellite_count_gngga), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.SatelliteCountRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gnggaData.satellite_count_gngga), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.HemisphereGNGGANorth) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnggaData.latitude_hemisphere, "N", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.HemisphereGNGGAEast) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnggaData.longitude_hemisphere, "E", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.HemisphereGNGGASouth) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnggaData.latitude_hemisphere, "S", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.HemisphereGNGGAWest) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnggaData.longitude_hemisphere, "W", 1);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.GPSPrecisionOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gnggaData.hdop_precision_factor), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GPSPrecisionUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gnggaData.hdop_precision_factor), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GPSPrecisionEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnggaData.hdop_precision_factor), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GPSPrecisionRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gnggaData.hdop_precision_factor), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AltitudeGNGGAOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gnggaData.altitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AltitudeGNGGAUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gnggaData.altitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AltitudeGNGGAEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnggaData.altitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AltitudeGNGGARange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gnggaData.altitude), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                        SYSTEMS CHECKS: GNRMC

        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCTimeGNRMCOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gnrmcData.utc_time), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCTimeGNRMCUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gnrmcData.utc_time), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCTimeGNRMCEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnrmcData.utc_time), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCTimeGNRMCRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gnrmcData.utc_time), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.LatGNRMCOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gnrmcData.latitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LatGNRMCUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gnrmcData.latitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LatGNRMCEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnrmcData.latitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LatGNRMCRange) == 0) {tmp_matrix[Fi] = in_range_check_true(atol(gnrmcData.latitude), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);} // is n in [2]range of [0]x (no y required)

        else if (strcmp(relayData.relays[Ri][Fi], relayData.LonGNRMCOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gnrmcData.longitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LonGNRMCUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gnrmcData.longitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LonGNRMCEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnrmcData.longitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LonGNRMCRange) == 0) {tmp_matrix[Fi] = in_range_check_true(atol(gnrmcData.longitude), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);} // is n in [2]range of [0]x (no y required)

        else if (strcmp(relayData.relays[Ri][Fi], relayData.HemisphereGNRMCNorth) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.latitude_hemisphere, "N", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.HemisphereGNRMCEast) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.longitude_hemisphere, "E", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.HemisphereGNRMCSouth) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.latitude_hemisphere, "S", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.HemisphereGNRMCWest) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.longitude_hemisphere, "W", 1);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.GroundSpeedGNRMCOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gnrmcData.ground_speed), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GroundSpeedGNRMCUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gnrmcData.ground_speed), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GroundSpeedGNRMCEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnrmcData.ground_speed), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GroundSpeedGNRMCRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gnrmcData.ground_speed), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.HeadingGNRMCOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gnrmcData.ground_heading), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.HeadingGNRMCUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gnrmcData.ground_heading), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.HeadingGNRMCEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnrmcData.ground_heading), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.HeadingGNRMCRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gnrmcData.ground_heading), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCDateGNRMCOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gnrmcData.utc_date), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCDateGNRMCUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gnrmcData.utc_date), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCDateGNRMCEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnrmcData.utc_date), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCDateGNRMCRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gnrmcData.utc_date), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.PositioningStatusGNRMCA) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.positioning_status, "A", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.PositioningStatusGNRMCV) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.positioning_status, "V", 1);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.ModeGNRMCA) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.mode_indication, "A", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ModeGNRMCD) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.mode_indication, "D", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ModeGNRMCN) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.mode_indication, "N", 1);}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                        SYSTEMS CHECKS: GPATT

        else if (strcmp(relayData.relays[Ri][Fi], relayData.PitchGPATTOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gpattData.pitch), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.PitchGPATTUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gpattData.pitch), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.PitchGPATTRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.pitch), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.RollGPATTOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gpattData.roll), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.RollGPATTUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gpattData.roll), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.RollGPATTEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gpattData.roll), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.RollGPATTRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.roll), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.YawGPATTOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gpattData.yaw), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.YawGPATTUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gpattData.yaw), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.YawGPATTEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gpattData.yaw), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.YawGPATTRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.yaw), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.GSTDataGPATTOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gpattData.gst_data), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GSTDataGPATTUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gpattData.gst_data), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GSTDataGPATTEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gpattData.gst_data), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GSTDataGPATTRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.gst_data), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.MileageGPATTOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gpattData.mileage), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.MileageGPATTUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gpattData.mileage), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.MileageGPATTEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gpattData.mileage), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.MileageGPATTRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.mileage), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.SpeedNumGPATTOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gpattData.speed_num), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.SpeedNumGPATTUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gpattData.speed_num), relayData.relays_data[Ri][Fi][0]);}  
        else if (strcmp(relayData.relays[Ri][Fi], relayData.SpeedNumGPATTEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gpattData.speed_num), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.SpeedNumGPATTRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.speed_num), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.LineFlagGPATTEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gpattData.line_flag), relayData.relays_data[Ri][Fi][0]);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.InertialFlagGPATTEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gpattData.run_inetial_flag), relayData.relays_data[Ri][Fi][0]);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.INSGPATTEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gpattData.ins), relayData.relays_data[Ri][Fi][0]);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.RunStateFlagGPATTEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gpattData.run_state_flag), relayData.relays_data[Ri][Fi][0]);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.StaticFlagGPATTEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gpattData.static_flag), relayData.relays_data[Ri][Fi][0]);}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                        SYSTEMS CHECKS: SPEED

        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIStateValueOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(speedData.ubi_state_value), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIStateValueUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(speedData.ubi_state_value), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIStateValueEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(speedData.ubi_state_value), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIStateValueRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(speedData.ubi_state_value), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIStateKindOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(speedData.ubi_state_kind), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIStateKindUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(speedData.ubi_state_kind), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIStateKindEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(speedData.ubi_state_kind), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIStateKindRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(speedData.ubi_state_kind), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIStateFlagOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(speedData.ubi_state_flag), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIStateFlagUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(speedData.ubi_state_flag), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIStateFlagEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(speedData.ubi_state_flag), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIStateFlagRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(speedData.ubi_state_flag), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.GyroZSPEEDOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(speedData.gyro_Z), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GyroZSPEEDUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(speedData.gyro_Z), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GyroZSPEEDEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(speedData.gyro_Z), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GyroZSPEEDRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(speedData.gyro_Z), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GyroYSPEEDOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(speedData.gyro_Y), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GyroYSPEEDUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(speedData.gyro_Y), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GyroYSPEEDEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(speedData.gyro_Y), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GyroYSPEEDRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(speedData.gyro_Y), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GyroXSPEEDOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(speedData.gyro_X), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GyroXSPEEDUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(speedData.gyro_X), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GyroXSPEEDEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(speedData.gyro_X), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GyroXSPEEDRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(speedData.gyro_X), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AccZSPEEDOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(speedData.acc_Z), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AccZSPEEDUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(speedData.acc_Z), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AccZSPEEDEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(speedData.acc_Z), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AccZSPEEDRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(speedData.acc_Z), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AccYSPEEDOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(speedData.acc_Y), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AccYSPEEDUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(speedData.acc_Y), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AccYSPEEDEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(speedData.acc_Y), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AccYSPEEDRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(speedData.acc_Y), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AccXSPEEDOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(speedData.acc_X), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AccXSPEEDUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(speedData.acc_X), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AccXSPEEDEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(speedData.acc_X), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AccXSPEEDRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(speedData.acc_X), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.StatusSPEEDOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(speedData.status), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.StatusSPEEDUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(speedData.status), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.StatusSPEEDEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(speedData.status), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.StatusSPEEDRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(speedData.status), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GroundSpeedSPEEDOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(speedData.speed), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GroundSpeedSPEEDUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(speedData.speed), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GroundSpeedSPEEDEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(speedData.speed), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GroundSpeedSPEEDRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(speedData.speed), relayData.relays_data[Ri][Fi][0],relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCTimeSPEEDOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(speedData.utc_time), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCTimeSPEEDUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(speedData.utc_time), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCTimeSPEEDEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(speedData.utc_time), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCTimeSPEEDRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(speedData.utc_time), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                        SYSTEMS CHECKS: ERROR
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ERRORGSetFlagEqual) == 0) {tmp_matrix[Fi] = check_over_true(atol(errorData.gset_flag), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ERRORSSetFlagEqual) == 0) {tmp_matrix[Fi] = check_under_true(atol(errorData.sset_flag), relayData.relays_data[Ri][Fi][0]);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.ERRORCodeFlagEqual) == 0) {tmp_matrix[Fi] = check_over_true(atol(errorData.code_flag), relayData.relays_data[Ri][Fi][0]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ERRORUTCTimeOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(errorData.utc), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ERRORUTCTimeUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(errorData.utc), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ERRORUTCTimeEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(errorData.utc), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ERRORUTCTimeRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(errorData.utc), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                        SYSTEMS CHECKS: DEBUG
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.CollisionTHeadingOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(debugData.coll_T_heading), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.CollisionTHeadingUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(debugData.coll_T_heading), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.CollisionTHeadingEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(debugData.coll_T_heading), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.CollisionTHeadingRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(debugData.coll_T_heading), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.CollisionTDataOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(debugData.coll_T_data), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.CollisionTDataUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(debugData.coll_T_data), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.CollisionTDataEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(debugData.coll_T_data), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.CollisionTDataRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(debugData.coll_T_data), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIValidEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(debugData.ubi_valid), relayData.relays_data[Ri][Fi][0]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.INSFlagOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(debugData.ins_flag), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.INSFlagUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(debugData.ins_flag), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.INSFlagEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(debugData.ins_flag), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.INSFlagRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(debugData.ins_flag), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.CarSpeedOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(debugData.car_speed), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.CarSpeedUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(debugData.car_speed), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.CarSpeedEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(debugData.car_speed), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.CarSpeedRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(debugData.car_speed), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.YawAngleOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(debugData.yaw_angle), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.YawAngleUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(debugData.yaw_angle), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.YawAngleEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(debugData.yaw_angle), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.YawAngleRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(debugData.yaw_angle), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.RollAngleOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(debugData.roll_angle), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.RollAngleUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(debugData.roll_angle), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.RollAngleEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(debugData.roll_angle), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.RollAngleRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(debugData.roll_angle), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.PitchAngleOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(debugData.pitch_angle), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.PitchAngleUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(debugData.pitch_angle), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.PitchAngleEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(debugData.pitch_angle), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.PitchAngleRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(debugData.pitch_angle), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIOnFlagOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(debugData.ubi_on_flag), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIOnFlagUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(debugData.ubi_on_flag), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIOnFlagEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(debugData.ubi_on_flag), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIOnFlagRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(debugData.ubi_on_flag), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIASetOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(debugData.ubi_a_set), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIASetUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(debugData.ubi_a_set), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIASetEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(debugData.ubi_a_set), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIASetRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(debugData.ubi_a_set), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIBSetOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(debugData.ubi_b_set), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIBSetUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(debugData.ubi_b_set), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIBSetEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(debugData.ubi_b_set), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIBSetRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(debugData.ubi_b_set), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AccXDataOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(debugData.acc_X_data), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AccXDataUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(debugData.acc_X_data), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AccXDataEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(debugData.acc_X_data), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AccXDataRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(debugData.acc_X_data), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AccYDataOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(debugData.acc_Y_data), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AccYDataUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(debugData.acc_Y_data), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AccYDataEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(debugData.acc_Y_data), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AccYDataRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(debugData.acc_Y_data), relayData.relays_data[Ri][Fi][0],relayData.relays_data[Ri][Fi][1]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GyroZDataOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(debugData.gyro_Z_data), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GyroZDataUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(debugData.gyro_Z_data), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GyroZDataEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(debugData.gyro_Z_data), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GyroZDataRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(debugData.gyro_Z_data), relayData.relays_data[Ri][Fi][0],relayData.relays_data[Ri][Fi][1]);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.AngDGetFlagEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(debugData.ang_dget_flag), relayData.relays_data[Ri][Fi][0]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.INSRunFlagEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(debugData.ins_run_flag), relayData.relays_data[Ri][Fi][0]);}
         
        else if (strcmp(relayData.relays[Ri][Fi], relayData.FixRollFlagEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(debugData.fix_roll_flag), relayData.relays_data[Ri][Fi][0]);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.FixPitchFlagEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(debugData.fix_pitch_flag), relayData.relays_data[Ri][Fi][0]);}

        else if (strcmp(relayData.relays[Ri][Fi], relayData.UBIKindFlagEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(debugData.ubi_kind_flag), relayData.relays_data[Ri][Fi][0]);}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                     SYSTEMS CHECKS: VALIDITY

        /*
        intended to conditionally switch datasets, making it possible to continue performing the same task and or other tasks instead, but with the option of
        relying on different data in the event data becomes unavailable/unreliable/etc. this can allow for fallback functions to be considered in the matrix switch.

        example: if check_bool_true(checksum)  then A using data X is active/on and B is inactive/off
                 if check_bool_false(checksum) then B using data Y is active/on and A is inactive/off
                 A and B may even be plugged into the same endpoint, and now that endpoint is on/off predicated upon different data/conditions.
        */
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gngga_valid_checksum) == 0) {tmp_matrix[Fi] = check_bool_true(gnggaData.valid_checksum);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gngga_invalid_checksum) == 0) {tmp_matrix[Fi] = check_bool_false(gnggaData.valid_checksum);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gnrmc_valid_checksum) == 0) {tmp_matrix[Fi] = check_bool_true(gnrmcData.valid_checksum);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gnrmc_invalid_checksum) == 0) {tmp_matrix[Fi] = check_bool_false(gnrmcData.valid_checksum);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gpatt_valid_checksum) == 0) {tmp_matrix[Fi] = check_bool_true(gpattData.valid_checksum);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gpatt_invalid_checksum) == 0) {tmp_matrix[Fi] = check_bool_false(gpattData.valid_checksum);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.speed_valid_checksum) == 0) {tmp_matrix[Fi] = check_bool_true(speedData.valid_checksum);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.speed_invalid_checksum) == 0) {tmp_matrix[Fi] = check_bool_false(speedData.valid_checksum);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.error_valid_checksum) == 0) {tmp_matrix[Fi] = check_bool_true(errorData.valid_checksum);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.error_invalid_checksum) == 0) {tmp_matrix[Fi] = check_bool_false(errorData.valid_checksum);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.debug_valid_checksum) == 0) {tmp_matrix[Fi] = check_bool_true(debugData.valid_checksum);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.debug_invalid_checksum) == 0) {tmp_matrix[Fi] = check_bool_false(debugData.valid_checksum);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gngga_valid_check_data) == 0) {tmp_matrix[Fi] = check_equal_true(gnggaData.check_data, 16);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gngga_invalid_check_data) == 0) {tmp_matrix[Fi] = check_equal_false(gnggaData.check_data, 16);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gnrmc_valid_check_data) == 0) {tmp_matrix[Fi] = check_equal_true(gnrmcData.check_data, 14);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gnrmc_invalid_check_data) == 0) {tmp_matrix[Fi] = check_equal_false(gnrmcData.check_data, 14);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gpatt_valid_check_data) == 0) {tmp_matrix[Fi] = check_equal_true(gpattData.check_data, 41);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.gpatt_invalid_check_data) == 0) {tmp_matrix[Fi] = check_equal_false(gpattData.check_data, 41);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.speed_valid_check_data) == 0) {tmp_matrix[Fi] = check_equal_true(speedData.check_data, 17);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.speed_invalid_check_data) == 0) {tmp_matrix[Fi] = check_equal_false(speedData.check_data, 17);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.error_valid_check_data) == 0) {tmp_matrix[Fi] = check_equal_true(errorData.check_data, 8);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.error_invalid_check_data) == 0) {tmp_matrix[Fi] = check_equal_false(errorData.check_data, 8);}
        
        else if (strcmp(relayData.relays[Ri][Fi], relayData.debug_valid_check_data) == 0) {tmp_matrix[Fi] = check_equal_true(debugData.check_data, 29);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.debug_invalid_check_data) == 0) {tmp_matrix[Fi] = check_equal_false(debugData.check_data, 29);}
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
//                                                                                                              MENU NAVIGATION

void menuDown() {menuData.y++; if (menuData.y >= menuData.menu_max_y0) {menuData.y=0;}}

void menuUp() {menuData.y--; if (menuData.y <= -1) {menuData.y=menuData.menu_max_y0-1;}}

void menuRight() {
  menuData.x++;
  if      ((menuData.y == 0) && (menuData.x == 2)) {menuData.page++; menuData.x=1; menuData.y=0; if (menuData.page >= menuData.page_max) {menuData.page=0;}}
  else if (menuData.x >= menuData.menu_max_x0) {menuData.x=0;}
}

void menuLeft() {
  menuData.x--;
  if ((menuData.y == 0) && (menuData.x == 0)) {menuData.page--; menuData.x=1; menuData.y=0; if (menuData.page <= -1) {menuData.page=menuData.page_max;}}
  else if (menuData.x <= -1) {menuData.x=menuData.menu_max_x0-1;}
}

void menuSelect() {
  // page zero only
  if (menuData.page == 0) {
    // relay enable/disable
    if ((menuData.y == 1) && (menuData.x == 1)) {if (relayData.relays_enable[0][menuData.relay_select] == 0) {relayData.relays_enable[0][menuData.relay_select] = 1;} else {relayData.relays_enable[0][menuData.relay_select] = 0;}}
    // select relay
    if ((menuData.y == 1) && (menuData.x == 0)) {menuData.page = 10; memset(menuData.input, 0, 256); menuData.numpad_key=0;}
    // select relay function
    if ((menuData.y == 1) && (menuData.x == 2)) {menuData.relay_function_select++; if (menuData.relay_function_select >= relayData.MAX_RELAY_ELEMENTS) {menuData.relay_function_select = 0;}}
    // select relay function name
    if (menuData.y == 2) {
      menuData.function_index++;
      if (menuData.function_index >= 252) {menuData.function_index=0; } memset(relayData.relays[menuData.relay_select][menuData.relay_function_select], 0, 56); strcpy(relayData.relays[menuData.relay_select][menuData.relay_function_select], relayData.function_names[menuData.function_index]);}
    // set relay function value x
    if (menuData.y == 3) {menuData.page = 10; memset(menuData.input, 0, 256); menuData.numpad_key=1;}
    // set relay function value y
    if (menuData.y == 4) {menuData.page = 10; memset(menuData.input, 0, 256); menuData.numpad_key=2;}
    // set relay function value z
    if (menuData.y == 5) {menuData.page = 10; memset(menuData.input, 0, 256); menuData.numpad_key=3;}
  }
  // page two only
  if (menuData.page == 1) {
    if (menuData.y == 1) {if (systemData.satcom_enabled == true) {systemData.satcom_enabled = false;} else {systemData.satcom_enabled = true;}}
    if (menuData.y == 2) {if (systemData.gngga_enabled == true) {systemData.gngga_enabled = false;} else {systemData.gngga_enabled = true;}}
    if (menuData.y == 3) {if (systemData.gnrmc_enabled == true) {systemData.gnrmc_enabled = false;} else {systemData.gnrmc_enabled = true;}}
    if (menuData.y == 4) {if (systemData.gpatt_enabled == true) {systemData.gpatt_enabled = false;} else {systemData.gpatt_enabled = true;}}
  }
  // special pages
  if (menuData.page == 2) {if (menuData.y != 0) {if (systemData.matrix_enabled == true) {systemData.matrix_enabled = false;} else {systemData.matrix_enabled = true;}}}
  if (menuData.page == 3) {if (menuData.y != 0) {matrix_disable_all();}}
  if (menuData.page == 4) {if (menuData.y != 0) {matrix_relays_disable_all();}}
  if (menuData.page == 5) {if (menuData.y != 0) {matrix_enable_all();}}
  if (menuData.page == 6) {if (menuData.y != 0) {matrix_relays_enable_all();}}

}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                            NUMPAD NAVIGATION

void numpadDown() {menuData.numpad_y++; if (menuData.numpad_y >= menuData.menu_numpad_max_y0) {menuData.numpad_y=0;}}
void numpadUp() {menuData.numpad_y--; if (menuData.numpad_y <= -1) {menuData.numpad_y=menuData.menu_numpad_max_y0-1;}}
void numpadRight() {menuData.numpad_x++; if (menuData.numpad_x >= menuData.menu_numpad_max_x0) {menuData.numpad_x=0;}}
void numpadLeft() {menuData.numpad_x--; if (menuData.numpad_x <= -1) {menuData.numpad_x=menuData.menu_numpad_max_x0-1;}}
void numpadSelect() {
  if ((menuData.numpad_y == 1) && (menuData.numpad_x == 0)) {strcat(menuData.input, "7");}
  if ((menuData.numpad_y == 1) && (menuData.numpad_x == 1)) {strcat(menuData.input, "8");}
  if ((menuData.numpad_y == 1) && (menuData.numpad_x == 2)) {strcat(menuData.input, "9");}
  if ((menuData.numpad_y == 2) && (menuData.numpad_x == 0)) {strcat(menuData.input, "4");}
  if ((menuData.numpad_y == 2) && (menuData.numpad_x == 1)) {strcat(menuData.input, "5");}
  if ((menuData.numpad_y == 2) && (menuData.numpad_x == 2)) {strcat(menuData.input, "6");}
  if ((menuData.numpad_y == 3) && (menuData.numpad_x == 0)) {strcat(menuData.input, "1");}
  if ((menuData.numpad_y == 3) && (menuData.numpad_x == 1)) {strcat(menuData.input, "2");}
  if ((menuData.numpad_y == 3) && (menuData.numpad_x == 2)) {strcat(menuData.input, "3");}
  if ((menuData.numpad_y == 4) && (menuData.numpad_x == 0)) {strcat(menuData.input, "0");}
  if ((menuData.numpad_y == 4) && (menuData.numpad_x == 1)) {strcat(menuData.input, ".");}
  if ((menuData.numpad_y == 4) && (menuData.numpad_x == 2)) {strcat(menuData.input, "-");}
  // remove last char
  if (((menuData.numpad_y == 5) && (menuData.numpad_x == 1)) || ((menuData.numpad_y == 5) && (menuData.numpad_x == 2))) {menuData.input[strlen(menuData.input)-1] = '\0';}
  // set current relay index
  if ((menuData.numpad_y == 5) && (menuData.numpad_x == 0) && (menuData.numpad_key==0)) {menuData.page = 0; if ((atoi(menuData.input) < relayData.MAX_RELAYS) && (atoi(menuData.input) >= 0)) {menuData.relay_select = atoi(menuData.input);}}
  // set relay function value x
  if ((menuData.numpad_y == 5) && (menuData.numpad_x == 0) && (menuData.numpad_key==1)) {menuData.page = 0; char *ptr; relayData.relays_data[menuData.relay_select][menuData.relay_function_select][0] = strtod(menuData.input, &ptr);}
  // set relay function value y
  if ((menuData.numpad_y == 5) && (menuData.numpad_x == 0) && (menuData.numpad_key==2)) {menuData.page = 0; char *ptr; relayData.relays_data[menuData.relay_select][menuData.relay_function_select][1] = strtod(menuData.input, &ptr);}
  // set relay function value z
  if ((menuData.numpad_y == 5) && (menuData.numpad_x == 0) && (menuData.numpad_key==3)) {menuData.page = 0; char *ptr; relayData.relays_data[menuData.relay_select][menuData.relay_function_select][2] = strtod(menuData.input, &ptr);}
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
    
    if (systemData.gngga_enabled == true) {
      if (strncmp(serial1Data.BUFFER, "$GNGGA", 6) == 0) {
        if ((serial1Data.nbytes == 94) || (serial1Data.nbytes == 90) ) {
          serial1Data.rcv = true;
          Serial.print(""); Serial.println(serial1Data.BUFFER);
          gnggaData.valid_checksum = validateChecksum(serial1Data.BUFFER);
          if (gnggaData.valid_checksum == true) {GNGGA();}
          else {gnggaData.bad_checksum_validity++;}
        }
      }
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                                    GNRMC

    if (systemData.gnrmc_enabled == true) {
      if (strncmp(serial1Data.BUFFER, "$GNRMC", 6) == 0) {
        if ((serial1Data.nbytes == 78) || (serial1Data.nbytes == 80)) {
          serial1Data.rcv = true;
          Serial.print(""); Serial.println(serial1Data.BUFFER);
          gnrmcData.valid_checksum = validateChecksum(serial1Data.BUFFER);
          if (gnrmcData.valid_checksum == true) {GNRMC();}
          else {gnrmcData.bad_checksum_validity++;}
        }
      }
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                                    GPATT

    if (systemData.gpatt_enabled == true) {
      if (strncmp(serial1Data.BUFFER, "$GPATT", 6) == 0) {
        if ((serial1Data.nbytes == 136) || (serial1Data.nbytes == 189)) {
          serial1Data.rcv = true;
          Serial.print(""); Serial.println(serial1Data.BUFFER);
          gpattData.valid_checksum = validateChecksum(serial1Data.BUFFER);
          if (gpattData.valid_checksum == true) {GPATT();}
          else {gpattData.bad_checksum_validity++;}
        }
      }
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                                    SPEED

    // if (systemData.speed_enabled == true) {
    //   if (strncmp(serial1Data.BUFFER, "$SPEED", 6) == 0) {
    //     serial1Data.rcv = true;
    //     Serial.print(""); Serial.println(serial1Data.BUFFER);
    //     awaiting length checks: take a ride with the laptop
    //     SPEED();
    //   }
    // }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                                    ERROR

    // if (systemData.error_enabled == true) {
    //   if (strncmp(serial1Data.BUFFER, "$ERROR", 6) == 0) {
    //     serial1Data.rcv = true;
    //     Serial.print(""); Serial.println(serial1Data.BUFFER);
    //     awaiting length checks: take a ride with the laptop
    //     ERROR();
    //   }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                                    DEBUG

    // if (systemData.debug_enabled == true) {
    //   if (strncmp(serial1Data.BUFFER, "$DEBUG", 6) == 0) {
    //     serial1Data.rcv = true;
    //     Serial.print(""); Serial.println(serial1Data.BUFFER);
    //     awaiting length checks: take a ride with the laptop
    //     DEBUG();
    //   }

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
    //                                                                                             MATRIX: ENABLE/DISABLE ENTRY

    else if (strncmp(serial0Data.BUFFER, "$MATRIX_ENABLE_ENTRY", 19) == 0) {
      matrix_set_enabled(true);
    }

    else if (strncmp(serial0Data.BUFFER, "$MATRIX_DISABLE_ENTRY", 21) == 0) {
      matrix_set_enabled(false);
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                      MATRIX: DISABLE ALL

    else if (strcmp(serial0Data.BUFFER, "$MATRIX_DISABLE_ALL") == 0) {
      matrix_disable_all();
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                       MATRIX: ENABLE ALL

    else if (strcmp(serial0Data.BUFFER, "$MATRIX_ENABLE_ALL") == 0) {
      matrix_enable_all();
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                               MATRIX: TURN ALL RELAYS ON

    else if (strcmp(serial0Data.BUFFER, "$MATRIX_RELAYS_ALL_ON") == 0) {
      matrix_relays_enable_all();
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                              MATRIX: TURN ALL RELAYS OFF

    else if (strcmp(serial0Data.BUFFER, "$MATRIX_RELAYS_ALL_OFF") == 0) {
      matrix_relays_disable_all();
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                         SDCARD: SERIAL PRINT MATRIX FILE

    else if (strcmp(serial0Data.BUFFER, "$SDCARD_READ_MATRIX") == 0) {
      sdcard_read_to_serial("matrix.txt");
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                            SDCARD: SAVE MATRIX TO SDCARD

    else if (strcmp(serial0Data.BUFFER, "$SDCARD_SAVE_MATRIX") == 0) {
      sdcard_save_matrix("matrix.txt");
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                          SDCARD: LOAD MATRIX FROM SDCARD

    else if (strcmp(serial0Data.BUFFER, "$SDCARD_LOAD_MATRIX") == 0) {
      sdcard_load_matrix("matrix.txt");
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                              SATCOM: CONVERT COORDINATES

    else if (strcmp(serial0Data.BUFFER, "$SATCOM_CONVERT_COORDINATES_ON") == 0) {
      satcom_convert_coordinates_on();
    }
    else if (strcmp(serial0Data.BUFFER, "$SATCOM_CONVERT_COORDINATES_OFF") == 0) {
      satcom_convert_coordinates_off();
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                      DISPLAY: BRIGHTNESS

    else if (strcmp(serial0Data.BUFFER, "$DISPLAY_BRIGHTNESS_MAX") == 0) {
      displayBrightness(1,1,1,1,1,1);
    }
    else if (strcmp(serial0Data.BUFFER, "$DISPLAY_BRIGHTNESS_MIN") == 0) {
      displayBrightness(0,0,0,0,0,0);
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                          DISPLAY: ON/OFF

    else if (strcmp(serial0Data.BUFFER, "$DISPLAY_ON") == 0) {
      displayOnOff(1,1,1, 1,1,1);
    }
    else if (strcmp(serial0Data.BUFFER, "$DISPLAY_OFF") == 0) {
      displayOnOff(0,0,0, 0,0,0);
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                          DISPLAY: INVERT

    else if (strcmp(serial0Data.BUFFER, "$DISPLAY_FLIP_VERTICALLY") == 0) {
      displayFlipVertically(1,1,1, 1,1,1);
    }
    else if (strcmp(serial0Data.BUFFER, "$DISPLAY_NORMAL") == 0) {
      displayFlipVertically(0,0,0, 0,0,0);
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                             SATCOM: MENU

    // any page less than 10
    if (menuData.page < 10) {
      if      (strcmp(serial0Data.BUFFER, "$DOWN") == 0) {menuDown();}
      else if (strcmp(serial0Data.BUFFER, "$UP") == 0) {menuUp();}
      else if (strcmp(serial0Data.BUFFER, "$RIGHT") == 0) {menuRight();}
      else if (strcmp(serial0Data.BUFFER, "$LEFT") == 0) {menuLeft();}
      else if (strcmp(serial0Data.BUFFER, "$SELECT") == 0) {menuSelect();}
    }
    // numpad
    else if (menuData.page == 10) {
      if (strcmp(serial0Data.BUFFER, "$DOWN") == 0) {numpadDown();}
      else if (strcmp(serial0Data.BUFFER, "$UP") == 0) {numpadUp();}
      else if (strcmp(serial0Data.BUFFER, "$RIGHT") == 0) {numpadRight();}
      else if (strcmp(serial0Data.BUFFER, "$LEFT") == 0) {numpadLeft();}
      else if (strcmp(serial0Data.BUFFER, "$SELECT") == 0) {numpadSelect();}
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

  // store current time to measure this loop time
  timeData.mainLoopTimeStart = millis();

  time_counter();

  // check serial input commands
  readRXD_0();

  // check satellite receiver
  readRXD_1();

  /*
  for performance/efficiency only do the following if data is received OR if there may be an issue receiving, this way the matrix
  switch can remain operational for data that is not received while also performing better overall. (tunable)
  */  
  if ((serial1Data.rcv == true) || (serial1Data.badrcv_i >= 10)) {
    serial1Data.badrcv_i=0;

    SSD_Display_2_Menu();

    if (systemData.satcom_enabled == true) {extrapulatedSatData(); SSD_Display_SATCOM();} else {SSD_Display_SATCOM_Disabled();}

    if (systemData.gngga_enabled == true) {SSD_Display_GNGGA();} else {SSD_Display_GNGGA_Disabled();}

    if (systemData.gnrmc_enabled == true) {SSD_Display_GNRMC();} else {SSD_Display_GNRMC_Disabled();}

    if (systemData.gpatt_enabled == true) {SSD_Display_GPATT();} else {SSD_Display_GPATT_Disabled();}

    if (systemData.matrix_enabled == true) {matrixSwitch(); SSD_Display_MATRIX();} else {SSD_Display_MATRIX_Disabled();}
    
  }
  else {serial1Data.badrcv_i++;}

  delay(1);

  // store time taken to complete
  timeData.mainLoopTimeTaken = millis() - timeData.mainLoopTimeStart;
}

// ----------------------------------------------------------------------------------------------------------------------------


