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

Remarks : the conditions for inertial navigation to work normally:
1.GPATT protocol 12 field INS is 1
2.GPATT protocol 15 field State Flag is 03/04

If the user wants to obtain good inertia performance, such as speed, UBI alarm and other parameters, In addition to the above two results, it is recommended to wait
for inertial navigation convergence:

(1) GPATT protocol 31field Run_Inetial_Flag is 4；Table A GPATT protocol 15 field RUN_STATE_FLAG each physical meaning description:

Flag    Description 							Required Conditions
0 	Prepare initialization 						System power on
1 	Attitude initialization 					completed Vehicle Static for 5-10S
2 	Position initialization 					completed Get Position Info
3 	Get the installation angle，Enter the integrated navigation 	Driving over 5m/s for a period of time
4 	The installation Angle has been identified 			Keep driving 

Table B GPATT protocol 31 field Run_Inetial_Flag each physical meaning description:

Flag 	Description 						Required conditions
0 	Prepare initialization
1 	Inertial navigation start converged 			Copy satellite positioning only, 	Run_State_Flag=01
2	Initial convergence of inertial navigation 		Inertial navigation is training, 	Run_State_Flag=02
3 	Inertial navigation is converging 			Inertial navigation is training, 	Run_State_Flag=03
4 	Inertial navigation converges completed 		Inertial navigation completed training, Run_State_Flag=04




Monochrome OLED Displays (because they bright and perform fast):
1x SATCOM: Datetime and degrees coordinates.
1x GNGGA: UTC Time and other GNGGA information.
1x GNRMC: UTC Datetime and other GNRMC information.
1x GPATT: INS information.
1x DESBI: ? information.
1X Relays: Screen displaying relays intended turn on/off other systems.
Note: OLED displays require screensaver.

Relays:
Turn on/off other systems when flags/conditions are met. These systems could be correctional systems for example or something simpler.
Although I do not have the resources to build for example a vehicle, I would like this project to be a complete system that I only need
to update to patch and add features, everything else should be plugged in, operational and interfaceable to various degrees.
This way I have satcom for projects requiring anything from datetime from satellites and coordinates to tasks more advanced and inclusive
of the WTG300's (and other GNS modules) full range of capabilities.
Relay functions should be available to each relay.

Relay Functions:
Timeable.
Satellite count.
Coordinates.
hemispherical.
precision factor.
altitude.
differential_time
differential_reference_base_station_label
speed.
heading.
magnetic_declination.
magnetic_declination_direction.
mode_indication.
pitch.
roll
yaw
ins
Run_State_Flag.
static_flag.
user_code.
gst_data.
line_flag.
subi_car_kind.
mileage.
ang_dget_flag.
run_inetial_flag.
time_save_num.
fix_angle_flag.
ang_lock_flag.

Communications:
Serial.
Wireless.
Displayed.

Human Interface Device:
Remote: keyboard / serial / rf.
On device: small keyboard / switches.



