/*

SatCom - Written by Benjamin Jack Cullen.

Processes and dumps data received from satellites and some data extrapulated from the data being received.

Wiring:
  WTGPS300 TX              --> ESP32 io26 as RXD
  WTGPS300 VCC             --> ESP32 3.3/5v
  TCA9548A i2C Multiplexer --> ESP32 i2C
  x3 SSD1306               --> TCA9548A i2C Multiplexer 

*/

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    LIBRARIES
#include <Wire.h>
#include <SSD1306Wire.h>   // SSD1306Wire                                https://gitlab.com/alexpr0/ssd1306wire
#include <OLEDDisplayUi.h> // ESP8266 and ESP32 OLED driver for SSD1306  https://github.com/ThingPulse/esp8266-oled-ssd1306
#include <Timezone.h>      // Timezone                                   https://github.com/JChristensen/Timezone

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
  Calculate 1 meter in longitude and latitude:

                               1 meter
  Latitude change (degrees)  = _______ x 360° = 0.00000901°

                               40075km

                               1 meter
  Longitude change (degrees) = _______ x 360° = 0.00000899°

                               40075km
  */
  unsigned long satellite_count = 0;
  char   sat_time_stamp_string[56]; // datetime timestamp from satellite
  char   last_sat_seen_time_stamp_string[56] = "000000000000.00"; // record last time satellites were seen
  char   satDataTag[10]  = "$SATCOM"; // satcom sentence tag

  double abs_latitude_gngga_0      = 0.0; // type double lat
  double abs_longitude_gngga_0     = 0.0; // type double lon
  double abs_latitude_gngga_1      = 0.0; // previous lat
  double abs_longitude_gngga_1     = 0.0; // previous lon

  double abs_latitude_gnrmc_0      = 0.0; // type double lat
  double abs_longitude_gnrmc_0     = 0.0; // type double lon
  double abs_latitude_gnrmc_1      = 0.0; // previous lat
  double abs_longitude_gnrmc_1     = 0.0; // previous lon

  double latitude_gngga_0 = 0.0;
  double longitude_gngga_0 = 0.0;
  double latitude_gngga_1 = 0.0;
  double longitude_gngga_1 = 0.0;

  double latitude_gnrmc_0 = 0.0;
  double longitude_gnrmc_0 = 0.0;
  double latitude_gnrmc_1 = 0.0;
  double longitude_gnrmc_1 = 0.0;

  double latitude_meter  = 0.00000901;
  double longitude_meter = 0.000000899;
  double latitude_mile   = latitude_meter  * 1609.34;
  double longitude_mile  = longitude_meter * 1609.34;

  bool   area_range_bool_lat_0   = false; // rangeing
  bool   area_range_bool_lon_0   = false;
  double area_range_lat_0        = latitude_meter;
  double area_range_lon_0        = longitude_meter;
  double area_range_lat_conf_0   = 0.00000000;
  double area_range_lon_conf_0   = 0.000000000;
};
SatDatatruct satData;

void calculateCurrentLocation(){

  double minutesLat; 
  double minutesLong;
  double degreesLat; 
  double degreesLong; 
  double secondsLat; 
  double secondsLong; 
  double millisecondsLat;
  double millisecondsLong; 

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                GNGGA COORDINATE CONVERSION
  double temporaryLatGNGGA = satData.abs_latitude_gngga_0;
  degreesLat = trunc(temporaryLatGNGGA/100);
  minutesLat = temporaryLatGNGGA - (degreesLat*100);
  secondsLat = (minutesLat - trunc(minutesLat)) * 60;
  millisecondsLat = (secondsLat - trunc(secondsLat)) * 1000;
  minutesLat = trunc(minutesLat);
  secondsLat = trunc(secondsLat);
  double currentDegreesLatGNGGA = degreesLat + minutesLat/60 + secondsLat/3600 + millisecondsLat/3600000;
  if (strcmp(gnggaData.latitude_hemisphere, "S") == 0) {
    currentDegreesLatGNGGA = 0-currentDegreesLatGNGGA;
  }
  // Serial.print("(GNGGA ) Degrees Latitude: "); 
  // Serial.println(currentDegreesLatGNGGA, 4);
  satData.latitude_gngga_0 = currentDegreesLatGNGGA;

  double temporaryLongGNGGA = satData.abs_longitude_gngga_0;
  degreesLong = trunc(temporaryLongGNGGA/100);
  minutesLong = temporaryLongGNGGA - (degreesLong*100);
  secondsLong = (minutesLong - trunc(minutesLong)) * 60;
  millisecondsLong = (secondsLong - trunc(secondsLong)) * 1000;
  double currentDegreesLongGNGGA = degreesLong + minutesLong/60 + secondsLong/3600 + millisecondsLong/3600000;
  if (strcmp(gnggaData.longitude_hemisphere, "W") == 0) {
    currentDegreesLongGNGGA = 0-currentDegreesLongGNGGA;
  }
  // Serial.print("(GNGGA ) Degrees Longitude: "); 
  // Serial.println(currentDegreesLongGNGGA, 4);
  satData.longitude_gngga_0 = currentDegreesLongGNGGA;

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                GNRMC COORDINATE CONVERSION
  double temporaryLatGNRMC = satData.abs_latitude_gnrmc_0;
  degreesLat = trunc(temporaryLatGNRMC/100);
  minutesLat = temporaryLatGNRMC - (degreesLat*100);
  secondsLat = (minutesLat - trunc(minutesLat)) * 60;
  millisecondsLat = (secondsLat - trunc(secondsLat)) * 1000;
  minutesLat = trunc(minutesLat);
  secondsLat = trunc(secondsLat);
  double currentDegreesLatGNRMC = degreesLat + minutesLat/60 + secondsLat/3600 + millisecondsLat/3600000;
  if (strcmp(gnggaData.latitude_hemisphere, "S") == 0) {
    currentDegreesLatGNRMC = 0-currentDegreesLatGNRMC;
  }
  // Serial.print("(GNRMC ) Degrees Latitude: "); 
  // Serial.println(currentDegreesLatGNRMC, 4);
  satData.latitude_gnrmc_0 = currentDegreesLatGNRMC;

  double temporaryLongGNRMC = satData.abs_longitude_gnrmc_0;
  degreesLong = trunc(temporaryLongGNRMC/100);
  minutesLong = temporaryLongGNRMC - (degreesLong*100);
  secondsLong = (minutesLong - trunc(minutesLong)) * 60;
  millisecondsLong = (secondsLong - trunc(secondsLong)) * 1000;
  double currentDegreesLongGNRMC = degreesLong + minutesLong/60 + secondsLong/3600 + millisecondsLong/3600000;
  if (strcmp(gnggaData.longitude_hemisphere, "W") == 0) {
    currentDegreesLongGNRMC = 0-currentDegreesLongGNRMC;
  }
  // Serial.print("(GNRMC ) Degrees Longitude: "); 
  // Serial.println(currentDegreesLongGNRMC, 4);
  satData.longitude_gnrmc_0 = currentDegreesLongGNRMC;
  // delay(500); 
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                          CREATE & DUMP EXTRAPULATED SAT DATA
void extrapulatedSatData() {

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                            SATCOM SENTENCE

  // start sentence output
  Serial.print(satData.satDataTag + String(","));

  // create datetime timestamp
  memset(satData.sat_time_stamp_string, 0, 56);
  strcat(satData.sat_time_stamp_string, gnrmcData.utc_date);
  strcat(satData.sat_time_stamp_string, gnggaData.utc_time);
  Serial.print(satData.sat_time_stamp_string + String(",")); // sentence output: Create datetime timestamp

  // create sat count int
  satData.satellite_count = atoi(gnggaData.satellite_count);
  if (satData.satellite_count > 0) {
    memset(satData.last_sat_seen_time_stamp_string, 0, 56);
    strcpy(satData.last_sat_seen_time_stamp_string, satData.sat_time_stamp_string);
  }
  Serial.print(String(satData.last_sat_seen_time_stamp_string) + ","); // sentence output: Create last seen satelits timestamp
  
  // create lat and long longs
  satData.abs_latitude_gngga_0 = atol(gnggaData.latitude);
  satData.abs_longitude_gngga_0 = atol(gnggaData.longitude);
  satData.abs_latitude_gnrmc_0 = atol(gnrmcData.latitude);
  satData.abs_longitude_gnrmc_0 = atol(gnrmcData.longitude);

  // create converted lat and long longs
  calculateCurrentLocation();
  Serial.print(String(satData.latitude_gngga_0) + ","); // sentence output: Create last seen satelits timestamp
  Serial.print(String(satData.longitude_gngga_0) + ","); // sentence output: Create last seen satelits timestamp
  Serial.print(String(satData.latitude_gnrmc_0) + ","); // sentence output: Create last seen satelits timestamp
  Serial.print(String(satData.longitude_gnrmc_0) + ","); // sentence output: Create last seen satelits timestamp

  // create ranging bools
  // Serial.println(); // debug
  // satData.latitude_gngga_0 = satData.latitude_meter-(satData.latitude_meter/2);   // debug
  // satData.longitude_gngga_0 = satData.longitude_meter-(satData.longitude_meter/2); // debug
  // Serial.print("latitude_gngga_0: "); Serial.println(satData.latitude_gngga_0, 8); // debug
  // Serial.print("calc 0    : "); Serial.println(satData.area_range_lat_conf_0 - (satData.area_range_lat_0/2), 8); // debug
  // Serial.print("calc 1    : "); Serial.println(satData.area_range_lat_conf_0 + (satData.area_range_lat_0/2), 8); // debug

  satData.area_range_bool_lat_0 = false;
  if ( ( satData.latitude_gngga_0  >= satData.area_range_lat_conf_0 - satData.area_range_lat_0/2 ) && ( satData.latitude_gngga_0  <= satData.area_range_lat_conf_0 + satData.area_range_lat_0/2) ) {
    satData.area_range_bool_lat_0 = true;
  }
  Serial.print(String(satData.area_range_bool_lat_0) + ","); // sentence output: Create range bool latitude

  // Serial.print("longitude_gngga_0: "); Serial.println(satData.longitude_gngga_0, 8); // debug
  // Serial.print("calc 0    : "); Serial.println(satData.area_range_lon_conf_0 - (satData.area_range_lon_0/2), 8); // debug
  // Serial.print("calc 1    : "); Serial.println(satData.area_range_lon_conf_0 + (satData.area_range_lon_0/2), 8); // debug

  satData.area_range_bool_lon_0 = false;
  if ( ( satData.longitude_gngga_0 >= satData.area_range_lon_conf_0 - satData.area_range_lon_0/2 ) && (satData.longitude_gngga_0  <= satData.area_range_lon_conf_0 + satData.area_range_lon_0/2) ) {
    satData.area_range_bool_lon_0 = true;
  }
  Serial.print(String(satData.area_range_bool_lon_0) + ","); // sentence output: Create range bool longitude

  // end sentence output
  Serial.println("*Z");
  // delay(1000);
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
  display3.drawString(display3.getWidth()/2, 34, " RX " + String(satData.area_range_bool_lat_0) + " RY " + String(satData.area_range_bool_lon_0));
  display3.drawString(display3.getWidth()/2, 44, String(satData.latitude_gngga_0));
  display3.drawString(display3.getWidth()/2, 54, String(satData.longitude_gngga_0));
  // display3.fillRect(0, 0, display.getWidth() - 1, 10);
  // display3.drawCircle(63, 63, 1);
  display3.display();
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

  tcaselect(5);
  initDisplay3();
  tcaselect(6);
  initDisplay();
  tcaselect(7);
  initDisplay2();
  SSD_Display_2_Splash_0();
  // delay(3000);
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
  extrapulatedSatData();
  calculateCurrentLocation();

  SSD_Display_0();
  SSD_Display_1();
  SSD_Display_2();

  delay(1);
}
