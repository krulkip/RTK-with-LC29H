# RTK-with-LC29H
This is the journey i went through to understand LC29HEA.\
First step is to connect via the USB port to the LC29HEA.\
Set both the switches to USB side.
Now you can start up the QGNSS windows program. (I use v1.1).\
Try sending some of the commands and see the response in the text window.
Setting up a rover is done like so.\

$PQTMRESTOREPAR\*13 # restore PQTM params to default and reset\
$PQTMCFGRCVRMODE,W,1\*2A # set receiver to rover mode\
$PQTMSAVEPAR*5A # save PQTM params to flash\
**manually power cycle module**\
$PAIR062,2,0\*3C # turn off GSA messages\
$PAIR062,3,0\*3D # turn off GSV messages\
$PAIR062,5,0\*3B # turn off VTG messages\
$PQTMCFGNMEADP,W,3,6,3,2,3,2\*37 # set decimal precision for NMEA\
$PAIR050,200\*21 # set pos output interval to 200 ms\
$PQTMSAVEPAR\*5A # save PQTM params to flash\
**manually power cycle module**

Unfortunately the save PQTM command does not save all the parameters but more about that later.\
You should see NMEA sentences come through. If not make sure the baud rate is set at 460800 and you have the right port.\
If that works connect up an ESP32 as follows.\

![Schematic](LC29HEA_ESP32.jpg?raw=true "LC29HEA_ESP32")

Disconnect the USB from the LC29HEA and connect it to the ESP32 instead.\
Load up the Passthrough_ESP32_LC29H sketch you can find here and set the switches on the LC29HEA to UART.\
If its uploaded and running you can reconnect the QGNSS program again and you should see exactly the same as before.\
The sketch reads the serial port and passes it to the serial2 port which talks to the LC29HEA.\
You will need such a type of solution later because you want to free up the serial port for debugging while using serial2 to communicate to the LC29HEA.

If that is working you can connect the antenna to the LC29HEA and put it in a location with good sky view.\
On the QGNSS program you should now see some kind of actual position and fix in the quality indicator field in the data lock box.\
Now its time to connect to an NTRIP server where the RTCM correction signal will improve the reception.\
Find a server close to you https://map.centipede-rtk.org/index.php/view/map?repository=cent&project=centipede \
I use the centipede network because its free and there was a server only 16km away.\
This you can do in QGNSS under TOOLS : NTRIP : NTRIP Server.\
For centipede the address is crtk.net and the port is 2101 and password and username are c or centipede.\
When you click on the update sourcetable button the sourcetable is updated and you can now select your mountpoint.\
This should be close to where your rover is because if its far away the correction is not accurate.\
You can now flick the switch in QGNSS connect to host to ON.\
The NMEA data which you were seeing now get complemented with RTCM correction data.\
After a while you quality uindicator will show RTK float fix or RTK fix.\
You now have an RTK fix and the position data will be accurate.

The RTCM correction data is getting the data through wifi from the NTRIP host and the QGNSS program then sends it though your ESP32 to your rover.\
If you are allways near wifi and have your PC handy then that is OK but sometimes you will want to use other ways of getting the data to your rover.\
One other way is to use bluetooth. I have used the SW Maps iPhone app to realise this.\
First load the app onto your iPhone. Then load the UART2_BT_Bridge sketch onto your ESP32.\
This has as task to receive the RTCM correction data from your iPhone and send it through serial2 to your LC29HEA.\
It means you do not need the QGNSS program and not even your PC as there is no essential data flowing through the USB port.\
There is debug data going that way which can be very helpfull in the innitial phase but it is not needed.\
So a battery or mains supply is sufficient to power the ESP32 and LC29HEA together and you could operate in the field away from wifi.\
You could of course connect to any NTRIP host including your own.\

So how do we make our own NTRIP host or CORS Continuously Operating Reference Stations.\ 
You can use the same schematic setup as before with the ESP32 connected by 4 wires to the LC29HEA.\
This time we will setup the LC29HEA as a base station which means that it will provide RTCM data from its own observations.\
This RTCM data is then sent to the NTRIP caster where it is made available to anyone you wish. \
In fact the sketch can load the correction data to two NTRIP casters. I have use RTK2go and Onocoy.\
Onocoy is a startup company which has a decentralized platform that enables the global, community-driven distribution of high-precision GNSS (Global Navigation Satellite System) correction data. It’s built on Web3 technologies (blockchain, tokens, smart contracts) to allow anyone — individuals, businesses, or institutions—to contribute to and benefit from a global network of GNSS reference stations.\
It says you can benefit from creating your CORS station connected to Onocoy and with that it means you can earn crypto coin depending on your location and your station.\
You need to configure your LC29HEA as a base station and again RTKlibexplorer was very helpfull in giving the configuration.\

$PQTMRESTOREPAR\*13 # restore PQTM params to default and reset\
$PQTMCFGRCVRMODE,W,2\*29 # set receiver to base mode\
$PQTMSAVEPAR\*5A # save PQTM params to flash\
**manually power cycle module**
$PAIR432,1\*22 # output RTCM3 MSM7 messages\
$PAIR434,1\*24 # output RTCM3 antenna position (1005)\
$PAIR062,0,01\*0F # Enable NMEA GGA message\
$PQTMCFGSVIN,W,2,0,0,x,y,z\*3B # set base location in XYZ coords\
$PQTMSAVEPAR\*5A # save PQTM params to flash\
### manually power cycle module\

In the sketch NTRIP_DUOCaster_ESP_NOW_V035 the program generates itself the commands after the first power cycle so you need to configure just the first three commands using QGNSS program.\
The sketch has some nice features besides being able to drive two CORS stations.\
You can still type commands in the serial monitor as if you were in the QGNSS program eg $PAIR432,1\*22 would work.\
The serial monitor also lists the RTCM message numbers that are being sent and you can see also the NMEA messages.\
It also has very clear messages about what is being sent where and you can follow the connection attempts succeed or fail with a reason line.\
Finally it also sends the RTCM data via ESP-NOW to your rover. Besides wifi this is another way to transmit correction data.\
We did'nt mention ESP-NOW yet in the rover story that is something for later.\

My next step is to use the ESP-NOW correction data rather than the wifi based method.\
I also intend to use e220 LORA methodology to transmit the data.

![Schematic](RTKFix.jpg?raw=true "RTKFix")
![Schematic](centipede.jpg?raw=true "centipede")
![Schematic](LC29HEA.jpg?raw=true "LC29HEA")