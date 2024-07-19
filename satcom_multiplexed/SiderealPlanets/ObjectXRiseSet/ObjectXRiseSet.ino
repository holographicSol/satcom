/* Sidereal Objects Library - Table Lookup object RA and DEC --> get current AZ, Alt and rise/set times using doXRiseSetTimes
*/

#include <SiderealPlanets.h>
#include <SiderealObjects.h>

// Need the following define for SAMD processors
#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

SiderealPlanets myAstro;    // for calculating azimuth and altitude
SiderealObjects myAstroObj; // for getting right ascension and declination of objects from star table

void setup() {
  Serial.begin(115200);
  delay(2000); //SAMD boards may need a long time to init SerialUSB
  myAstroObj.begin();
  Serial.println("\n\n");

  // first set time and coordinate data
  myAstro.setLatLong(0.00, 0.00;
  myAstro.setTimeZone(1);
  // myAstro.rejectDST();
  myAstro.setGMTdate(2024,7,19);
  myAstro.setLocalTime(22,40,0);
  myAstro.setGMTtime(22,40,0);

  Serial.println("------------------------------------------------------");
  Serial.println("162 9 Alpha CMa Sirius");
  myAstroObj.selectStarTable(162);
  Serial.print("Right Ascension:  "); myAstroObj.printDegMinSecs(myAstroObj.getRAdec()); Serial.println("");
  Serial.print("Declination: "); myAstroObj.printDegMinSecs(myAstroObj.getDeclinationDec()); Serial.println("");
  Serial.print("Magnitude: "); Serial.println(myAstroObj.getStarMagnitude()); Serial.println("");
  // set RA and DEC
  myAstro.setRAdec(myAstroObj.getRAdec(), myAstroObj.getDeclinationDec());
  // convert RA and DEC to AZ and Alt
  myAstro.doRAdec2AltAz();
  Serial.println("Azimuth: " + String(myAstro.getAzimuth()));
  Serial.println("Altitude: " + String(myAstro.getAltitude()));  
  // rise/set time
  myAstro.doXRiseSetTimes();
  Serial.println("Rise Time: " + String(myAstro.getRiseTime()));
  Serial.println("Set Time: " + String(myAstro.getSetTime()));

  Serial.println("------------------------------------------------------");

  Serial.println("------------------------------------------------------");
  Serial.println("457 The Owl Cluster");
  myAstroObj.selectNGCTable(457);
  Serial.print("Right Ascension: "); myAstroObj.printDegMinSecs(myAstroObj.getRAdec()); Serial.println("");
  Serial.print("Declination: "); myAstroObj.printDegMinSecs(myAstroObj.getDeclinationDec()); Serial.println("");
  // set RA and DEC
  myAstro.setRAdec(myAstroObj.getRAdec(), myAstroObj.getDeclinationDec());
  // convert RA and DEC to AZ and Alt
  myAstro.doRAdec2AltAz();
  Serial.println("Azimuth: " + String(myAstro.getAzimuth()));
  Serial.println("Altitude: " + String(myAstro.getAltitude()));  
  // rise/set time
  myAstro.doXRiseSetTimes();
  Serial.println("Rise Time: " + String(myAstro.getRiseTime()));
  Serial.println("Set Time: " + String(myAstro.getSetTime()));
  Serial.println("------------------------------------------------------");
  
  Serial.println("------------------------------------------------------");
  Serial.println("IC 4665");
  myAstroObj.selectICTable(4665);
  Serial.print("Right Ascension: "); myAstroObj.printDegMinSecs(myAstroObj.getRAdec()); Serial.println("");
  Serial.print("Declination = 05:43:00  ==> "); myAstroObj.printDegMinSecs(myAstroObj.getDeclinationDec()); Serial.println("");
  // set RA and DEC
  myAstro.setRAdec(myAstroObj.getRAdec(), myAstroObj.getDeclinationDec());
  // convert RA and DEC to AZ and Alt
  myAstro.doRAdec2AltAz();
  Serial.println("Azimuth: " + String(myAstro.getAzimuth()));
  Serial.println("Altitude: " + String(myAstro.getAltitude()));  
  // rise/set time
  myAstro.doXRiseSetTimes();
  Serial.println("Rise Time: " + String(myAstro.getRiseTime()));
  Serial.println("Set Time: " + String(myAstro.getSetTime()));
  Serial.println("------------------------------------------------------");
  
  Serial.println("------------------------------------------------------");
  Serial.println("And M13 in our table:");
  Serial.println("M13 NGC6205 Hercules Globular");
  myAstroObj.selectMessierTable(13);
  Serial.print("Right Ascension: "); myAstroObj.printDegMinSecs(myAstroObj.getRAdec()); Serial.println("");
  Serial.print("Declination: "); myAstroObj.printDegMinSecs(myAstroObj.getDeclinationDec());  Serial.println("");
  // set RA and DEC
  myAstro.setRAdec(myAstroObj.getRAdec(), myAstroObj.getDeclinationDec());
  // convert RA and DEC to AZ and Alt
  myAstro.doRAdec2AltAz();
  Serial.println("Azimuth: " + String(myAstro.getAzimuth()));
  Serial.println("Altitude: " + String(myAstro.getAltitude()));  
  // rise/set time
  myAstro.doXRiseSetTimes();
  Serial.println("Rise Time: " + String(myAstro.getRiseTime()));
  Serial.println("Set Time: " + String(myAstro.getSetTime()));
  Serial.println("------------------------------------------------------");
  
  Serial.println("------------------------------------------------------");
  Serial.println("Caldwell C13  NGC 457 The Owl Cluster");
  myAstroObj.selectCaldwellTable(13);
  Serial.print("Right Ascension: "); myAstroObj.printDegMinSecs(myAstroObj.getRAdec()); Serial.println("");
  Serial.print("Declination: "); myAstroObj.printDegMinSecs(myAstroObj.getDeclinationDec()); Serial.println("");
  // set RA and DEC
  myAstro.setRAdec(myAstroObj.getRAdec(), myAstroObj.getDeclinationDec());
  // convert RA and DEC to AZ and Alt
  myAstro.doRAdec2AltAz();
  Serial.println("Azimuth: " + String(myAstro.getAzimuth()));
  Serial.println("Altitude: " + String(myAstro.getAltitude()));  
  // rise/set time
  myAstro.doXRiseSetTimes();
  Serial.println("Rise Time: " + String(myAstro.getRiseTime()));
  Serial.println("Set Time: " + String(myAstro.getSetTime()));
  Serial.println("------------------------------------------------------");
  
  Serial.println("------------------------------------------------------");
  Serial.println("[Herschel 400 Object]");
  Serial.println("H1 NGC40 Bow-Tie Nebula");
  myAstroObj.selectHershel400Table(1);
  Serial.print("Right Ascension: "); myAstroObj.printDegMinSecs(myAstroObj.getRAdec()); Serial.println("");
  Serial.print("Declination: "); myAstroObj.printDegMinSecs(myAstroObj.getDeclinationDec());Serial.println("");
  // set RA and DEC
  myAstro.setRAdec(myAstroObj.getRAdec(), myAstroObj.getDeclinationDec());
  // convert RA and DEC to AZ and Alt
  myAstro.doRAdec2AltAz();
  Serial.println("Azimuth: " + String(myAstro.getAzimuth()));
  Serial.println("Altitude: " + String(myAstro.getAltitude()));  
  // rise/set time
  myAstro.doXRiseSetTimes();
  Serial.println("Rise Time: " + String(myAstro.getRiseTime()));
  Serial.println("Set Time: " + String(myAstro.getSetTime()));
  Serial.println("------------------------------------------------------");
  
  Serial.println("------------------------------------------------------");
  Serial.println("20001 Tr1");
  myAstroObj.selectOtherObjectsTable(20001);
  Serial.print("Right Ascension: "); myAstroObj.printDegMinSecs(myAstroObj.getRAdec()); Serial.println("");
  Serial.print("Declination: "); myAstroObj.printDegMinSecs(myAstroObj.getDeclinationDec()); Serial.println("");
  // set RA and DEC
  myAstro.setRAdec(myAstroObj.getRAdec(), myAstroObj.getDeclinationDec());
  // convert RA and DEC to AZ and Alt
  myAstro.doRAdec2AltAz();
  Serial.println("Azimuth: " + String(myAstro.getAzimuth()));
  Serial.println("Altitude: " + String(myAstro.getAltitude()));  
  // rise/set time
  myAstro.doXRiseSetTimes();
  Serial.println("Rise Time: " + String(myAstro.getRiseTime()));
  Serial.println("Set Time: " + String(myAstro.getSetTime()));
  Serial.println("------------------------------------------------------");
}

void loop() {
  while(1);
}