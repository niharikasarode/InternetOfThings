# InternetOfThings

INFANT MONITOR


This repository holds the codes for the project "Infant Monitor" that I worked on in my IoT class. 
The system was built on the Leopard Gecko Starter kit (EFM32LG990F256-STK3600) and consists of an on-board temperature sensor
that monitor the infant's body temperature and generates an alert condition (in the form of LED sttaus change) if there is an irregularity.
There is a heart-rate sensor(AD 8232) attached that records the estimate heart rate using ACMP module. Two other sensors included in this 
project are the TSL2561(Ambient Light Sensor) and MMA-8452Q (Accelerometer sensor) which monitors the infant's sleeping position, 
and alerts the user if the infant is lying on it's chest which is harmful in the long run. These two sensors communicate using I2C
with the sensor-hub.

The system communicates with an App Atmel Smart Connect using BLE from AT-SAMB11 which acts as a master for the sensor-hub 
Leopard Gecko. From the Leopard Gecko, heart rate and tempearute vlaues are sent over to AT-SAMB11 using LE_UART(low power) 
which in forwards them to the App using Bluetooth Low Energy.

The system is activated using Capacitive sense on the board that involves touch on the Capsense part. 
