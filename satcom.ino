/*

SatCom - Written by Benjamin Jack Cullen.

For use with the WTGPS300 (chosen for its satellite compatibility) and may be highly compatible with much more than WTGPS300.

Ensure good hard connections or risk interpreting garbage.

Requries just 3 wires for WTGPS300:
  WTGPS300 TX  --> ESP32 io26 as RXD
  WTGPS300 VCC -->
  WTGPS300 GND -->
*/

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    LIBRARIES
#include <SSD1306Wire.h>   // SSD1306Wire                                https://gitlab.com/alexpr0/ssd1306wire
#include <OLEDDisplayUi.h> // ESP8266 and ESP32 OLED driver for SSD1306  https://github.com/ThingPulse/esp8266-oled-ssd1306

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                       WIRING
SSD1306Wire   display(0x3c, SDA, SCL); // let SSD1306Wire wire up our SSD1306 on the i2C bus
OLEDDisplayUi ui(&display);            // plug display into OLEDDisplayUi

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                  SERIAL DATA
struct SerialStruct {
  char BUFFER[2048];
};
SerialStruct serialData;

// ----------------------------------------------------------------------------------------------------------------------------

unsigned long iter;
unsigned long numBytes;
unsigned long ufo;
unsigned long iter_vars;
char *        token = strtok(serialData.BUFFER, ",");

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   GNGGA DATA
struct GNGGAStruct {
  char tag[56];                                       // <0> $GNGGA
  char utc_time[56];                                  // <1> UTC time, the format is hhmmss.sss
  char latitude[56];                                  // <2> Latitude, the format is  ddmm.mmmmmmm
  char latitude_hemisphere[56];                       // <3> Latitude hemisphere, N or S (north latitude or south latitude)
  char longitude[56];                                 // <4> Longitude, the format is dddmm.mmmmmmm
  char longitude_hemisphere[56];                      // <5> Longitude hemisphere, E or W (east longitude or west longitude)
  char gnss_positioning_status[56];                   /* <6> GNSS positioning status: 0 not positioned, 1 single point positioning,
                                                             2 differential GPS fixed solution, 4 fixed solution, 5 floating point
                                                             solution */
  char satellite_count[56];                           // <7> Number of satellites used
  char hddp_precision_factor[56];                     // <8> HDOP level precision factor
  char altitude[56];                                  // <9> Altitude
  char height_earth_ellipsoid_relative_to_geoid[56];  // <10> The height of the earth ellipsoid relative to the geoid
  char differential_time[56];                         // <11> Differential time
  char differential_reference_base_station_label[56]; // <12> Differential reference base station label (* Statement end marker)
  char xor_check_value[56];                           // <13> xx XOR check value of all bytes starting from $ to *
  char cr[56];                                        // <14> <CR> Carriage return, end tag
  char lf[56];                                        // <15> <LF> line feed, end tag
};
GNGGAStruct gnggaData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   GNRMC DATA
struct GNRMCStruct {
  char tag[56];                                       // <0> $GNRMC
  char utc_time[56];                                  // <1> UTC time, the format is hhmmss.sss
  char positioning_status[56];                        // <2> Positioning status, A=effective positioning, V=invalid positioning
  char latitude[56];                                  // <3> Latitude, the format is  ddmm.mmmmmmm
  char latitude_hemisphere[56];                       // <4> Latitude hemisphere, N or S (north latitude or south latitude)
  char longitude[56];                                 // <5> Longitude, the format is dddmm.mmmmmmm
  char longitude_hemisphere[56];                      // <6> Longitude hemisphere, E or W (east longitude or west longitude)
  char ground_speed[56];                              // <7> Ground speed
  char ground_heading[56];                            // <8> Ground heading (take true north as the reference datum)
  char utc_date[56];                                  // <9> UTC date, the format is ddmmyy (day, month, year)
  char magnetic_declination[56];                      // <10> Magnetic declination (000.0~180.0 degrees)
  char magnetic_declination_direction[56];            // <11> Magnetic declination direction, E (east) or W (west)
  char mode_indication[56];                           /* <12> Mode indication (A=autonomous positioning, D=differential
                                                              E=estimation, N=invalid data) */
  char xor_check_value[56];                           // <13> xx XOR check value of all bytes starting from $ to *
  char cr[56];                                        // <14> <CR> Carriage return, end tag
  char lf[56];                                        // <15> <LF> line feed, end tag
};
GNRMCStruct gnrmcData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                SSD1306 FRAME: GEIGER COUNTER

/*
this method of writing to the SSD1306 as provided in the library example, refreshes the display very satisfactorily and is far
superior to clearing parts of the screen or indeed the whole screen manually (display.cls()) prior to writing to the display.
*/

void SSD_Display(OLEDDisplay* display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->drawString(display->getWidth()/2, 0, "SATCOM");
  display->drawString(display->getWidth()/2, 14, gnggaData.satellite_count);
  display->drawString(display->getWidth()/2, 24, String(gnggaData.utc_time) + " " + String(gnrmcData.utc_date));
  display->drawString(display->getWidth()/2, 34, String(gnrmcData.latitude_hemisphere) + " " + String(gnrmcData.latitude));
  display->drawString(display->getWidth()/2, 44, String(gnrmcData.longitude_hemisphere) + " " + String(gnrmcData.longitude));
  display->drawString(display->getWidth()/2, 54, String(gnggaData.altitude) + " " + String(gnrmcData.ground_heading) + " " + String(gnrmcData.ground_speed));
}
FrameCallback frames[] = { SSD_Display }; // array keeps function pointers to all frames are the single views that slide in
int frameCount = 1;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        SETUP
void setup() {

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                               SETUP SERIAL

  Serial.begin(115200);
  Serial1.begin(115200); // ( io26 on ESP32 )

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                              SETUP DISPLAY

  display.init();
  ui.setTargetFPS(60);
  ui.disableAllIndicators();
  ui.setFrames(frames, frameCount);
  display.flipScreenVertically();
  display.setContrast(255);
  display.setFont(ArialMT_Plain_10);
  display.cls();
  display.println("starting..");
}


// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        GNGGA
void GNGGA() {
  memset(gnggaData.tag, 0, 56);
  memset(gnggaData.utc_time, 0, 56);
  memset(gnggaData.latitude, 0, 56);
  memset(gnggaData.latitude_hemisphere, 0, 56);
  memset(gnggaData.longitude, 0, 56);
  memset(gnggaData.longitude_hemisphere, 0, 56);
  memset(gnggaData.gnss_positioning_status, 0, 56);
  strcpy(gnggaData.satellite_count, "0");
  memset(gnggaData.hddp_precision_factor, 0, 56);
  memset(gnggaData.altitude, 0, 56);
  memset(gnggaData.height_earth_ellipsoid_relative_to_geoid, 0, 56);
  memset(gnggaData.differential_time, 0, 56);
  memset(gnggaData.differential_reference_base_station_label, 0, 56);
  memset(gnggaData.xor_check_value, 0, 56);
  memset(gnggaData.cr, 0, 56);

  iter_vars = 0;
  token = strtok(serialData.BUFFER, ",");
  while( token != NULL ) {
    if     (iter_vars == 0) {strcpy(gnggaData.tag, "GNGGA");}
    else if (iter_vars ==1) {if (strlen(token) <= 9) {strcpy(gnggaData.utc_time, token);}}
    else if (iter_vars ==2) {if (strlen(token) <= 17) {strcpy(gnggaData.latitude, token);}}
    else if (iter_vars ==3) {if (strlen(token) <= 1) {strcpy(gnggaData.latitude_hemisphere, token);}}
    else if (iter_vars ==4) {if (strlen(token) <= 17) {strcpy(gnggaData.longitude, token);}}
    else if (iter_vars ==5) {if (strlen(token) <= 1) {strcpy(gnggaData.longitude_hemisphere, token);}}
    else if (iter_vars ==6) {if (strlen(token) <= 1) {strcpy(gnggaData.gnss_positioning_status, token);}}
    else if (iter_vars ==7) {strcpy(gnggaData.satellite_count, token);}
    else if (iter_vars ==8) {strcpy(gnggaData.hddp_precision_factor, token);}
    else if (iter_vars ==9) {strcpy(gnggaData.altitude, token);}
    else if (iter_vars ==10) {strcpy(gnggaData.height_earth_ellipsoid_relative_to_geoid, token);}
    else if (iter_vars ==11) {strcpy(gnggaData.differential_time, token);}
    else if (iter_vars ==12) {strcpy(gnggaData.differential_reference_base_station_label, token);}                  
    else if (iter_vars ==13) {strcpy(gnggaData.xor_check_value, token);}  
    else if (iter_vars ==14) {strcpy(gnggaData.cr, token);}  
    else if (iter_vars ==15) {strcpy(gnggaData.lf, token);}
    token = strtok(NULL, ",");
    iter_vars++;
  }
}

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
  memset(gnrmcData.xor_check_value, 0, 56);
  memset(gnrmcData.cr, 0, 56);
  memset(gnrmcData.lf, 0, 56);

  iter_vars = 0;
  token = strtok(serialData.BUFFER, ",");
  while( token != NULL ) {
    if     (iter_vars == 0) {strcpy(gnrmcData.tag, "GNGGA");}
    else if (iter_vars ==1) {if (strlen(token) <= 9) {strcpy(gnggaData.utc_time, token);}}
    else if (iter_vars ==2) {strcpy(gnrmcData.positioning_status, token);}
    else if (iter_vars ==3) {if (strlen(token) <= 17) {strcpy(gnrmcData.latitude, token);}}
    else if (iter_vars ==4) {if (strlen(token) <= 1) {strcpy(gnrmcData.latitude_hemisphere, token);}}
    else if (iter_vars ==5) {if (strlen(token) <= 17) {strcpy(gnrmcData.longitude, token);}}
    else if (iter_vars ==6) {if (strlen(token) <= 1) {strcpy(gnrmcData.longitude_hemisphere, token);}}
    else if (iter_vars ==7) {strcpy(gnrmcData.ground_speed, token);}
    else if (iter_vars ==8) {strcpy(gnrmcData.ground_heading, token);}
    else if (iter_vars ==9) {if (strlen(token) <= 6) {strcpy(gnrmcData.utc_date, token);}}
    else if (iter_vars ==10) {strcpy(gnrmcData.magnetic_declination, token);}
    else if (iter_vars ==11) {strcpy(gnrmcData.magnetic_declination_direction, token);}
    else if (iter_vars ==12) {strcpy(gnrmcData.mode_indication, token);}                  
    else if (iter_vars ==13) {strcpy(gnrmcData.xor_check_value, token);}  
    else if (iter_vars ==14) {strcpy(gnrmcData.cr, token);}  
    else if (iter_vars ==15) {strcpy(gnrmcData.lf, token);}
    token = strtok(NULL, ",");
    iter_vars++;
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   READ RXD 1
void readRXD_1() {
  if (Serial1.available() > 0) {
    
    memset(serialData.BUFFER, 0, 2048);
    numBytes = (Serial1.readBytesUntil('\n', serialData.BUFFER, sizeof(serialData.BUFFER)));
    Serial.println(numBytes); // debug

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                                    GNGGA
    if (strncmp(serialData.BUFFER, "$GNGGA", 6) == 0) {
      if ((numBytes == 94) || (numBytes == 90) ) {
        Serial.print(""); Serial.println(serialData.BUFFER);
        GNGGA();
      }
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                                    GNRMC
    else if (strncmp(serialData.BUFFER, "$GNRMC", 6) == 0) {
      if ((numBytes == 78) || (numBytes == 80)) {
        Serial.print(""); Serial.println(serialData.BUFFER);
        GNRMC();
      }
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                                    DESBI
    else if (strncmp(serialData.BUFFER, "$DESBI", 6) == 0) {
      Serial.print(""); Serial.println(serialData.BUFFER);
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                                    GPATT
    else if (strncmp(serialData.BUFFER, "$GPATT", 6) == 0) {
      Serial.print(""); Serial.println(serialData.BUFFER);
    }
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    MAIN LOOP
void loop() {

  readRXD_1();

  ui.update();

  delay(1);
}
