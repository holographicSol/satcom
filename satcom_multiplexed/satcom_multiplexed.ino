/*

                                          SATCOM - Written by Benjamin Jack Cullen.


                                     Receives and Processes Transmissions from Satellites.
         Tested with WTGPS300 (WITMOTION) Satellite module this project receives data from satellites, sorts that data into
     variables that can be used later, and creates new $ sentences containing new data, calculated from the data received.


                        Converts Latitude and Longitude from $ sentences into Decimal Latitude and Longitude.
  This helps when compaing latitude and longitude with other maps, these values are then stored and placed into a new $ sentence.


                                                     Timestamps
      A datetime string is created from GNGGA and GNRMC $ sentence information, forms a part of the new $SATCOM sentence
                                                     

                                           Timestamp Zero Satellite Periods
  A Timestamp is created for keeping track of when the system last heard satellites, forms part of the new $SATCOM sentence
                        

                                                      Ranging
Specified coordinates at specified meter/mile ranges. For location pinning, guidance, tracking, motion detection, etc. these values
                                        can be added to the new $SATCOM sentence.

                                                
                                                    Serial Dump
   All extra calculated data is dumped in its own $SATCOM sentence along with the other $ sentences which are passed through
        as they would be seen as if the WTGPS300 is plugged in via USB. Dumping more (sentences/information) and dumping in
                                        a standard way and for generic use.

                                        
                                                  Compatibility
                     To Be Potentially Anything And Function As A Hardware Plugin For Other Projects
                                          Headless / Standalone / Serial.


                          Wiring for Optional Multiplexed OLED Displays (SSD1306 Monochromes)
                                    WTGPS300 TX              --> ESP32 io26 as RXD
                                    WTGPS300 VCC             --> ESP32 3.3/5v
                                    TCA9548A i2C Multiplexer --> ESP32 i2C
                                    x3 SSD1306               --> TCA9548A i2C Multiplexer


                                                SENTENCE $SATCOM

              START Tag        Last Sat Time                    ConvertedLongitude        END Tag
              |                      |                                 |                     |   
              $SATCOM,000000000000.00,000000000000.00,00.00000000000000,00.00000000000000,0,0,*Z
                     |                               |                                   | |                    
                DatetimeStamp                 ConvertedLatitude                  In/Out Range Bools


                          Create More Values For $ Ssentences And Add More Satellite Hardware.
                       Create Unique Ranging Sentences For Lists of Variable Specified Ranges.
          This sketch is a layer between a satellite module and other systems to provide a new sentence and or the above+.
    Ranging and other activities can be performed here but it may be preferrable to make satcom a pure layer between systems,
                                          depending on performance.
          If data is intened to be passed through to a more powerfull system then turning off coordinte_convert and using
          another (more powerfull) systems resources to perform the coordinate conversions may be preferrable however
                                    it can all be done here on the same system.
                  Sytem can be made more or less efficient/standalone by turning some bools on or off. 
 
*/

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    LIBRARIES

#include <Wire.h>
#include <SSD1306Wire.h>   // SSD1306Wire                                https://gitlab.com/alexpr0/ssd1306wire
#include <OLEDDisplayUi.h> // ESP8266 and ESP32 OLED driver for SSD1306  https://github.com/ThingPulse/esp8266-oled-ssd1306
#include <Timezone.h>      // Timezone                                   https://github.com/JChristensen/Timezone

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                      DEFINES

#define TCAADDR 0x70

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                       WIRING

SSD1306Wire   display(0x3c, SDA, SCL); // let SSD1306Wire wire up our SSD1306 on the i2C bus
SSD1306Wire   display2(0x3c, SDA, SCL); // let SSD1306Wire wire up our SSD1306 on the i2C bus
SSD1306Wire   display3(0x3c, SDA, SCL); // let SSD1306Wire wire up our SSD1306 on the i2C bus

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

void initDisplay() {
  display.init();
  display.flipScreenVertically();
  display.setContrast(255);
  display.setFont(ArialMT_Plain_10);
  display.cls();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                          INITIALIZE DISPLAY

void initDisplay2() {
  display2.init();
  display2.flipScreenVertically();
  display2.setContrast(255);
  display2.setFont(ArialMT_Plain_10);
  display2.cls();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                          INITIALIZE DISPLAY

void initDisplay3() {
  display3.init();
  display3.flipScreenVertically();
  display3.setContrast(255);
  display3.setFont(ArialMT_Plain_10);
  display3.cls();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   GNGGA DATA

struct GNGGAStruct {
  char tag[56];                                       // <0> $GNGGA
  char utc_time[56];                                  // <1> UTC time, the format is hhmmss.sss
  char latitude[56];                                  // <2> Latitude, the format is  ddmm.mmmmmmm
  char latitude_hemisphere[56];                       // <3> Latitude hemisphere, N or S (north latitude or south latitude)
  char longitude[56];                                 // <4> Longitude, the format is dddmm.mmmmmmm
  char longitude_hemisphere[56];                      // <5> Longitude hemisphere, E or W (east longitude or west longitude)
  char positioning_status[56];                        /* <6> GNSS positioning status: 0 not positioned, 1 single point positioning,
                                                             2 differential GPS fixed solution, 4 fixed solution, 5 floating point
                                                             solution */
  char satellite_count[56] = "0";                           // <7> Number of satellites used
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
//                                                                                                              SAT DATA STRUCT

struct SatDatatruct {

  /*
                               1 meter
  Latitude change (degrees)  = _______ x 360° = 0.00000901°
                               40075km

                               1 meter
  Longitude change (degrees) = _______ x 360° = 0.00000899°
                               40075km
  */

  unsigned long satellite_count = 0;
  char   sat_time_stamp_string[56];                               // datetime timestamp from satellite
  char   last_sat_seen_time_stamp_string[56] = "000000000000.00"; // record last time satellites were seen
  char   satDataTag[10]                      = "$SATCOM";         // satcom sentence tag

  double minutesLat;       // used for converting absolute latitude and longitude
  double minutesLong;      // used for converting absolute latitude and longitude
  double degreesLat;       // used for converting absolute latitude and longitude
  double degreesLong;      // used for converting absolute latitude and longitude
  double secondsLat;       // used for converting absolute latitude and longitude
  double secondsLong;      // used for converting absolute latitude and longitude
  double millisecondsLat;  // used for converting absolute latitude and longitude
  double millisecondsLong; // used for converting absolute latitude and longitude

  double abs_latitude_gngga_0  = 0.0; // absolute latitude from $ sentence
  double abs_longitude_gngga_0 = 0.0; // absolute longditude $ sentence
  double abs_latitude_gnrmc_0  = 0.0; // absolute latitude $ sentence
  double abs_longitude_gnrmc_0 = 0.0; // absolute longditude $ sentence

  double temp_latitude_gngga;  // degrees converted from absolute
  double temp_longitude_gngga; // degrees converted from absolute
  double temp_latitude_gnrmc;  // degrees converted from absolute
  double temp_longitude_gnrmc; // degrees converted from absolute

  double location_latitude_gngga;  // degrees converted from absolute
  double location_longitude_gngga; // degrees converted from absolute
  double location_latitude_gnrmc;  // degrees converted from absolute
  double location_longitude_gnrmc; // degrees converted from absolute
  
  char location_latitude_gngga_str[56];  // degrees converted from absolute
  char location_longitude_gngga_str[56]; // degrees converted from absolute
  char location_latitude_gnrmc_str[56];  // degrees converted from absolute
  char location_longitude_gnrmc_str[56]; // degrees converted from absolute

  double latitude_meter        = 0.00000901; // one meter (tune)
  double longitude_meter       = 0.00000899; // one meter (tune)
  double latitude_mile         = latitude_meter  * 1609.34; // one mile
  double longitude_mile        = longitude_meter * 1609.34; // one mile

  bool   area_range_enabled_0  = true; //enable/diable standalone ranging (requires coordinte_convert)
  double area_range_lat_0      = latitude_meter*1.5;  // specify latitude range
  double area_range_lon_0      = longitude_meter*1.5; // specify longitude range
  double location_range_distance_latitude[200];
  double location_range_distance_longitude[200];
  char   location_range_name[200][56];
  double location_range_latitude[200];
  double location_range_longitude[200];
  double location_range_latitude_bool[200];
  double location_range_longitude_bool[200];

  bool coordinte_convert = true; // enable/diable standalone coordinate conversions
  char coordinate_conversion_mode[10] = "GNGGA"; // choose a sentence that degrees/decimal coordinates will be created from
};
SatDatatruct satData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                        CREATE COORDINTE DATA

void calculateLocation(){

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                GNGGA COORDINATE CONVERSION

  if (satData.coordinte_convert == true) {
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
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                EXTRAPOLATE & DUMP EXTRA DATA

void extrapulatedSatData() {

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                     SATCOM SENTENCE: BEGIN

  Serial.print(satData.satDataTag + String(","));

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                   SATCOM SENTENCE: TIMESTAMP FROM SAT TIME

  memset(satData.sat_time_stamp_string, 0, 56);
  strcat(satData.sat_time_stamp_string, gnrmcData.utc_date);
  strcat(satData.sat_time_stamp_string, gnggaData.utc_time);
  Serial.print(satData.sat_time_stamp_string + String(","));

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                       SATCOM SENTENCE: LAST KNOWN DOWNLINK

  satData.satellite_count = atoi(gnggaData.satellite_count);
  if (satData.satellite_count > 0) {
    memset(satData.last_sat_seen_time_stamp_string, 0, 56);
    strcpy(satData.last_sat_seen_time_stamp_string, satData.sat_time_stamp_string);
  }
  Serial.print(String(satData.last_sat_seen_time_stamp_string) + ",");

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
    Serial.print(satData.location_latitude_gngga, 17); Serial.print(",");
    Serial.print(satData.location_longitude_gngga, 17); Serial.print(",");
  }
  else if (String(satData.coordinate_conversion_mode) == "GNRMC") {
    Serial.print(satData.location_latitude_gnrmc, 17); Serial.print(",");
    Serial.print(satData.location_longitude_gnrmc, 17); Serial.print(",");
  }

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                   SATCOM SENTENCE: RANGING
  
  // TEST RANGE: uncomment to test a range with artificial coordinates (note that this must be commented if satcom sentence in use)
  // satData.location_latitude_gngga = 40.68951821629331;
  // satData.location_longitude_gngga = -74.04483714358342;

  // coordinte_convert
  if (satData.coordinte_convert == true) {
    if (satData.area_range_enabled_0 == true) {
      for (int i = 0; i < 100; i++) {
        
        if (strlen(satData.location_range_name[i]) > 0) {

          // TEST RANGE: uncomment to view calculated target perimeter (note that this must be commented if satcom sentence in use)
          // Serial.println();
          // Serial.print("[T0] "); Serial.println(satData.location_range_name[i]);
          // Serial.print("[X0] "); Serial.println(satData.location_range_latitude[i] - satData.location_range_distance_latitude[i]/2, 17);
          // Serial.print("[X1] "); Serial.println(satData.location_range_latitude[i] + satData.location_range_distance_latitude[i]/2, 17);
          // Serial.print("[Y0] "); Serial.println(satData.location_range_longitude[i] - satData.location_range_distance_longitude[i]/2, 17);
          // Serial.print("[Y1] "); Serial.println(satData.location_range_longitude[i] + satData.location_range_distance_longitude[i]/2, 17);
          // Serial.println();

          // name into satcom sentence
          Serial.print(satData.location_range_name[i] + String(","));

          // create latitude range bool
          satData.location_range_latitude_bool[i] = false;
          if (satData.location_latitude_gngga  >=  satData.location_range_latitude[i] - satData.location_range_distance_latitude[i]/2) {
            if (satData.location_latitude_gngga  <= satData.location_range_latitude[i] + satData.location_range_distance_latitude[i]/2) {
              satData.location_range_latitude_bool[i] = true;
            }
          }
          Serial.print(String((int)satData.location_range_latitude_bool[i]) + ",");

          // create longitude range bool
          satData.location_range_longitude_bool[i] = false;
          if (satData.location_longitude_gngga >= satData.location_range_longitude[i] - satData.location_range_distance_longitude[i]/2) {
            if (satData.location_longitude_gngga  <= satData.location_range_longitude[i] + satData.location_range_distance_longitude[i]/2) {
              satData.location_range_longitude_bool[i] = true;
            }
          }
          Serial.print(String((int)satData.location_range_longitude_bool[i]) + ",");
        }
      }
    }
  }

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                       SATCOM SENTENCE: END
  Serial.println("*Z");
  }

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    DISPLAY 0

void SSD_Display_0() {
  tcaselect(6);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setColor(WHITE);
  display.clear();
  display.drawString(display.getWidth()/2, 0, "GNGGA");
  display.drawString(display.getWidth()/2, 14, "P " + String(gnggaData.positioning_status) + " S " + String(gnggaData.satellite_count));
  display.drawString(display.getWidth()/2, 24, String(gnggaData.utc_time));
  display.drawString(display.getWidth()/2, 34, String(gnggaData.latitude_hemisphere) + " " + String(gnggaData.latitude));
  display.drawString(display.getWidth()/2, 44, String(gnggaData.longitude_hemisphere) + " " + String(gnggaData.longitude));
  display.drawString(display.getWidth()/2, 54, "A " + String(gnggaData.altitude));
  display.display();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    DISPLAY 1

void SSD_Display_1() {
  tcaselect(7);
  display2.setTextAlignment(TEXT_ALIGN_CENTER);
  display2.setColor(WHITE);
  display2.clear();
  display2.drawString(display2.getWidth()/2, 0, "GNRMC");
  display2.drawString(display2.getWidth()/2, 14, "P " + String(gnrmcData.positioning_status) + " M " + String(gnrmcData.mode_indication));
  display2.drawString(display2.getWidth()/2, 24, String(gnrmcData.utc_time) + " " + String(gnrmcData.utc_date));
  display2.drawString(display2.getWidth()/2, 34, String(gnrmcData.latitude_hemisphere) + " " + String(gnrmcData.latitude));
  display2.drawString(display2.getWidth()/2, 44, String(gnrmcData.longitude_hemisphere) + " " + String(gnrmcData.longitude));
  display2.drawString(display2.getWidth()/2, 54, "H " + String(gnrmcData.ground_heading) + " S " + String(gnrmcData.ground_speed));
  display2.display();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    DISPLAY 2

void SSD_Display_2_Splash_0() {
  tcaselect(5);
  display3.setTextAlignment(TEXT_ALIGN_CENTER);
  display3.setColor(WHITE);
  display3.clear();
  display3.drawString(display3.getWidth()/2, 0, "        _,--',   _._.--._____");
  display3.drawString(display3.getWidth()/2, 10, " .--.--';_'-.', ';_      _.,-'");
  display3.drawString(display3.getWidth()/2, 20, ".'--'.  _.'    {`'-;_ .-.>.'");
  display3.drawString(display3.getWidth()/2, 30, "      '-:_      )  / `' '=.");
  display3.drawString(display3.getWidth()/2, 40, "        ) >     {_/,     /~)");
  display3.drawString(display3.getWidth()/2, 50, "snd     |/               `^ .'");
  display3.display();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    DISPLAY 2

void SSD_Display_2() {
  tcaselect(5);
  display3.setTextAlignment(TEXT_ALIGN_CENTER);
  display3.setColor(WHITE);
  display3.clear();
  display3.drawString(display3.getWidth()/2, 0, "SATCOM");
  display3.drawString(display3.getWidth()/2, 14, satData.sat_time_stamp_string);
  display3.drawString(display3.getWidth()/2, 24, String(satData.last_sat_seen_time_stamp_string));
  display3.drawString(display3.getWidth()/2, 44, String(gnggaData.latitude_hemisphere) + " " + satData.location_latitude_gngga_str);
  display3.drawString(display3.getWidth()/2, 54, String(gnggaData.longitude_hemisphere) + " " + satData.location_longitude_gngga_str);
  display3.display();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        SETUP

void setup() {

  // TEST RANGE: uncomment to test rangeing
  strcpy(satData.location_range_name[0], "location_0");
  satData.location_range_latitude[0]  =  40.68951821629331;
  satData.location_range_longitude[0] = -74.04483714358342;
  satData.location_range_distance_latitude[0] = satData.latitude_meter*1;  // specify latitude range
  satData.location_range_distance_longitude[0] = satData.longitude_meter*1; // specify longitude range
  strcpy(satData.location_range_name[1], "location_1");
  satData.location_range_latitude[1] =  40.68951821629331;
  satData.location_range_longitude[1] = -74.04483714358342;
  satData.location_range_distance_latitude[1] =  satData.latitude_meter*100;  // specify latitude range
  satData.location_range_distance_longitude[1] = satData.latitude_meter*100; // specify longitude range

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                               SETUP SERIAL

  Serial.begin(115200);
  Serial1.begin(115200); // ( io26 on ESP32 )

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                 SETUP WIRE

  Wire.begin();

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                              SETUP DISPLAY

  tcaselect(5);
  initDisplay3();
  tcaselect(6);
  initDisplay();
  tcaselect(7);
  initDisplay2();
  SSD_Display_2_Splash_0();
  delay(2000);
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
  memset(gnggaData.positioning_status, 0, 56);
  memset(gnggaData.satellite_count, 0, 56);
  memset(gnggaData.hddp_precision_factor, 0, 56);
  memset(gnggaData.altitude, 0, 56);
  memset(gnggaData.height_earth_ellipsoid_relative_to_geoid, 0, 56);
  memset(gnggaData.differential_time, 0, 56);
  memset(gnggaData.differential_reference_base_station_label, 0, 56);
  memset(gnggaData.xor_check_value, 0, 56);
  memset(gnggaData.cr, 0, 56);
  serialData.iter_token = 0;
  serialData.token = strtok(serialData.BUFFER, ",");
  while( serialData.token != NULL ) {
    if     (serialData.iter_token == 0) {strcpy(gnggaData.tag, "GNGGA");}
    else if (serialData.iter_token ==1) {if (strlen(serialData.token) <= 9) {strcpy(gnggaData.utc_time, serialData.token);}}
    else if (serialData.iter_token ==2) {if (strlen(serialData.token) <= 17) {strcpy(gnggaData.latitude, serialData.token);}}
    else if (serialData.iter_token ==3) {if (strlen(serialData.token) <= 1) {strcpy(gnggaData.latitude_hemisphere, serialData.token);}}
    else if (serialData.iter_token ==4) {if (strlen(serialData.token) <= 17) {strcpy(gnggaData.longitude, serialData.token);}}
    else if (serialData.iter_token ==5) {if (strlen(serialData.token) <= 1) {strcpy(gnggaData.longitude_hemisphere, serialData.token);}}
    else if (serialData.iter_token ==6) {if (strlen(serialData.token) <= 1) {strcpy(gnggaData.positioning_status, serialData.token);}}
    else if (serialData.iter_token ==7) {strcpy(gnggaData.satellite_count, serialData.token);}
    else if (serialData.iter_token ==8) {strcpy(gnggaData.hddp_precision_factor, serialData.token);}
    else if (serialData.iter_token ==9) {strcpy(gnggaData.altitude, serialData.token);}
    else if (serialData.iter_token ==10) {strcpy(gnggaData.height_earth_ellipsoid_relative_to_geoid, serialData.token);}
    else if (serialData.iter_token ==11) {strcpy(gnggaData.differential_time, serialData.token);}
    else if (serialData.iter_token ==12) {strcpy(gnggaData.differential_reference_base_station_label, serialData.token);}                  
    else if (serialData.iter_token ==13) {strcpy(gnggaData.xor_check_value, serialData.token);}  
    else if (serialData.iter_token ==14) {strcpy(gnggaData.cr, serialData.token);}  
    else if (serialData.iter_token ==15) {strcpy(gnggaData.lf, serialData.token);}
    serialData.token = strtok(NULL, ",");
    serialData.iter_token++;
  }
}

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
  memset(gnrmcData.xor_check_value, 0, 56);
  memset(gnrmcData.cr, 0, 56);
  memset(gnrmcData.lf, 0, 56);

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
    else if (serialData.iter_token ==12) {strncpy(gnrmcData.mode_indication, serialData.token, 1);}                  
    else if (serialData.iter_token ==13) {strcpy(gnrmcData.xor_check_value, serialData.token);}  
    else if (serialData.iter_token ==14) {strcpy(gnrmcData.cr, serialData.token);}  
    else if (serialData.iter_token ==15) {strcpy(gnrmcData.lf, serialData.token);}
    serialData.token = strtok(NULL, ",");
    serialData.iter_token++;
  }
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
    //                                                                                                                    DESBI

    else if (strncmp(serialData.BUFFER, "$DESBI", 6) == 0) {
      // Serial.print(""); Serial.println(serialData.BUFFER);
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                                    GPATT

    else if (strncmp(serialData.BUFFER, "$GPATT", 6) == 0) {
      // Serial.print(""); Serial.println(serialData.BUFFER);
    }
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    MAIN LOOP

void loop() {
  readRXD_1();
  extrapulatedSatData();
  SSD_Display_0();
  SSD_Display_1();
  SSD_Display_2();

  delay(1);
}

// ----------------------------------------------------------------------------------------------------------------------------
