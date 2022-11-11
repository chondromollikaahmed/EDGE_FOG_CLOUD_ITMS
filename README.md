# Creating a Ad-Hoc Network Using Esp32 And Esp8266
### Ad hoc networking is a type of network where individual devices (We are Using ESP32 And ESP8266) are in direct communication with one another. Without the aid of a wireless router or access point, ad hoc networks can be established between two or more wireless Nodemcu ESP8266 or ESP32 devices. The numerous ESP devices directly communicate with one another.
---



## Follow The Link For Video Details
https://drive.google.com/file/d/1A4a-G02HBzSOVacWs5yEa6z0Xr44LO9M/view?usp=drivesdk


> We will further try to add raspberry pi as a node . Make rpi compatible if it can work with espnow protocall. 
>> https://hackaday.io/project/161896-linux-espnow 



## What it can do ?
1. Send and receives data form and to the esp devices in ranges.
2. As Example a a light and a boot button of esp devices  (GPIO0) is used .
3. If a devices press the boot button it changes the states of light (if on changed state will be off and vice versa)
4. Simultaneously it sends "on"/"off" data to all the esp devices in ranges and the other devices show actions by received buffer .
5. For exchanging data between devices serial monitor is used .
> In that case we used two device with pc for serial monitor(two COM port) and another one with an 0.96" OLED display to see the recieved data .
6. Light can also be controlled by sending "on" or "off" from serial montor .
> [For Video illustration ](https://drive.google.com/file/d/1A4a-G02HBzSOVacWs5yEa6z0Xr44LO9M/view?usp=drivesdk/ "Hardware Illustration")

 > [Exchanging Data with Serial Monitor](https://raw.githubusercontent.com/chondromollikaahmed/AD-HOC-ESP32-ESP8266/master/peer_to_peer.mp4 "PeerToPeer.mp4")
## Component Used


| **Component Name**        | **Quantity** |
|---------------------------|--------------|
| NodeMCU ESP8266-12E Board | 2            |
| NodeMCU ESP32 Board       | 4            |
| 0.96" OLED DISPLAY        | 2            |
| Connecting Wires          |              |
| BreadBoard                | 6            |
| Light                     | 6            |



## Connections And Working Process 


 ![](https://raw.githubusercontent.com/chondromollikaahmed/AD-HOC-ESP32-ESP8266/master/images/peer_to_peer.png)

 ![](https://raw.githubusercontent.com/chondromollikaahmed/AD-HOC-ESP32-ESP8266/master/images/connection1.jpg)


 ![](https://raw.githubusercontent.com/chondromollikaahmed/AD-HOC-ESP32-ESP8266/master/images/connection2.jpg)


  ![](https://raw.githubusercontent.com/chondromollikaahmed/AD-HOC-ESP32-ESP8266/master/images/serial.jpg)


 




---
  # Detailed Description Will be Updated soon ***
  ---

   ## Video
  https://raw.githubusercontent.com/chondromollikaahmed/AD-HOC-ESP32-ESP8266/master/peer_to_peer.mp4