---

SATCOM - A powerfull general purpose satellite controlled matrix switch.

---

![plot](./resources/img_001.JPG)

---

![plot](./resources/img_000.JPG)

---




                                                                                                   GST Data: User satellite accuracy                         
                                                                           1=static 0=dynamic      |   Custom flag                                           
                                                                                  Static Flag      |   |    IMU Kind                                         
                                                                                            |      |   |    0->BIM055; 1->BMI160; 2->LSM6DS3TR-C; 3->LSM6DSOW4->ICM-40607;
                                                                                            |      |   |    5->ICM-406086->ICM-42670; 7->LSM6DSR 
                                                           1=open 0=close                   |      |   |    |   Mileage: units: km : 9999 km max
                                 Software Version   Inertial Navigation System              |      |   |    |   |       ANG_DGet_Flag
                                                |                 |  INS Channel            |      |   |    |   |       1: The Flash has an installation Angle
                            Angle Channels      |                 |  |                      |      |   |    |   |       0: The Flash has no installation Angle
         Log Header                |            |                 |  |                      |      |   |    |   |       |    
                  |                |            |                 |  |                      |      |   |    |   |       |    
                  |                |            |                 |  |                      |      |   |    |   |       |   Custom Flags Fix_Angle_Flag: F：Fix    
                  |                |            |                 |  |                      |      |   |    |   |       |    |       |   |    Extensible
                  |         _______|_______     |      Prod.ID    |  |        Custom Flag   |      |   |    |   |       |    |       |   |    |      Checksum
                  |         |      |      |     |         |       |  |            |    |    |      |   |    |   |       |    |       |   |    |      |
04:38:51.055 -> $GPATT,0.00,p,0.00,r,0.00,y,20230219,S,003E009,ID,1,INS,3335,02,0,0.00,0,7,01,1.30,0,1,D,00,7,4,0.000,0,0,00,B,0,0,0,0,7,31,0,0.000*53
                        |       |     |              |          |        |   |  |        |    |      |   |    |       |   |            |    |
                      Pitch    Roll  Yaw      Version Channel   |        |   |  | MTK Version |      |   |    |       |   |            |    |
                                                                |        |   |  |             |      |   |    |       |   |            |    ANG_Lock_Flag
                                                                |        |   |  |             |      |   |    |       |   |            |    1：Fixed setting
                                                                |        |   |  |             |      |   |    |       |   |            |    0：Self adaptive installation
                                                                |        |   |  |             |      |   |    |       |   |            Ephemeris Stored Times
                                                              ID Channel |   |  |             |      |   |    |       |   Run_Inetial_Flag: 1->4
                                                           Hardware version  |  |             |      |   |    |       Custom Flag
                                                            Run State Flag 1->3 |             |      |   |    SUBI_Car_Kind: 1: small car 2: big car
                                                                                |             |      |   Custom Flag: F:Full Update D:Full Update and Part Update
                                                                    Mis Angle Num             |      Line Flag 1：straight driving，0：curve driving
                                                                                              User Code: 1=normal user X=custom user




---
