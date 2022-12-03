/*
  ESP-NOW Multi Unit Demo
  esp-now-multi.ino
  Broadcasts control messages to all devices in network
  Load script on multiple devices
*/

// Include Libraries
#ifdef ESP32
  #include <WiFi.h>
  #include <esp_now.h> 
#else
  #include <ESP8266WiFi.h>
  #include <espnow.h>
#endif



#define CHANNEL 1

// Define LED and pushbutton state booleans
bool buttonDown = false;
bool ledOn = false;

// Define LED and pushbutton pins
#define STATUS_LED 15
#define STATUS_BUTTON 0


int nodeCount=0;
String destmac[10];

void formatMacAddress(const uint8_t *macAddr, char *buffer, int maxLength)
// Formats MAC Address
{
  snprintf(buffer, maxLength, "%02x:%02x:%02x:%02x:%02x:%02x", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
}


void receiveCallback(const uint8_t *macAddr, const uint8_t *data, int dataLen)
// Called when data is received
{
  // Only allow a maximum of 250 characters in the message + a null terminating byte
  char buffer[ESP_NOW_MAX_DATA_LEN + 1];
  int msgLen = min(ESP_NOW_MAX_DATA_LEN, dataLen);
  strncpy(buffer, (const char *)data, msgLen);

  // Make sure we are null terminated
  buffer[msgLen] = 0;

  // Format the MAC address
  char macStr[18];
  formatMacAddress(macAddr, macStr, 18);

  // Send Debug log message to the serial port
  Serial.printf("Received message from: %s - %s\n", macStr, buffer);

  String msg ;
  msg=buffer;
  if (msg.indexOf("Node") == 0) {
    Serial.println("Nodes Found From Message ");

   ///  size_t ind = msg.find("Node");

//msg.erase(0,"Node".length());

   // Serial.println(msg);
    

    
    int count=0;
    String newmsg;
    newmsg =msg;
    while (newmsg.indexOf("Node")==0)
     {
      destmac[count]=newmsg.substring(newmsg.indexOf("Node")+5,22);
      Serial.println(newmsg);
      newmsg=newmsg.substring(22,newmsg.length());
      nodeCount=count+1;
      count++;
  }


    for (int i =0; i<nodeCount;i++)
    {
       Serial.println("Node ->"+destmac[i]);
    }  

    Serial.println(destmac[0]);
    Serial.println(destmac[1]);

  }



  // Check switch status
  if (strcmp("on", buffer) == 0)
  {
    ledOn = true;
  }
  else if (strcmp("off", buffer) == 0)
  {
    ledOn = false;
  }
  digitalWrite(STATUS_LED, ledOn);
}


void sentCallback(const uint8_t *macAddr, esp_now_send_status_t status)
// Called when data is sent
{
  char macStr[18];
  formatMacAddress(macAddr, macStr, 18);
  Serial.print("Last Packet Sent to: ");
  Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void broadcast(const String &message)
// Emulates a broadcast
{

  
  WiFi.mode(WIFI_STA);
  // Broadcast a message to every device in range
  uint8_t broadcastAddress[6] ;

  for(int i=0;i<nodeCount;i++)
  {

      int mac[6];
       if ( 6 == sscanf(destmac[i].c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          Serial.println("Filtering Mac Adress from String");
          for (int ii = 0; ii < 6; ++ii ) {
            broadcastAddress[ii] = (uint8_t) mac[ii];
          }
        }
    esp_now_peer_info_t peerInfo = {};
    memcpy(&peerInfo.peer_addr, broadcastAddress, 6);
    if (!esp_now_is_peer_exist(broadcastAddress))
   {
    esp_now_add_peer(&peerInfo);
    }
  // Send message
  esp_err_t result = esp_now_send(broadcastAddress, (const uint8_t *)message.c_str(), message.length());

  // Print results to serial monitor
  if (result == ESP_OK)
  {
    Serial.println("Broadcast message success");
  }
  else if (result == ESP_ERR_ESPNOW_NOT_INIT)
  {
    Serial.println("ESP-NOW not Init.");
  }
  else if (result == ESP_ERR_ESPNOW_ARG)
  {
    Serial.println("Invalid Argument");
  }
  else if (result == ESP_ERR_ESPNOW_INTERNAL)
  {
    Serial.println("Internal Error");
  }
  else if (result == ESP_ERR_ESPNOW_NO_MEM)
  {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  }
  else if (result == ESP_ERR_ESPNOW_NOT_FOUND)
  {
    Serial.println("Peer not found.");
  }
  else
  {
    Serial.println("Unknown error" +String(result) );
  }

  }
     WiFi.mode(WIFI_AP);
  // configure device AP mode
   configDeviceAP();
  // This is the mac address of the Nodes in AP Mode
}



void configDeviceAP() {
  String Prefix = "Node:";
  String Mac = WiFi.macAddress();
  String SSID = Prefix + Mac;
  String Password = "123456789";
  bool result = WiFi.softAP(SSID.c_str(), Password.c_str(), CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}



// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");

    
    esp_now_register_recv_cb(receiveCallback);
    esp_now_register_send_cb(sentCallback);
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

void setup()
{

  // Set up Serial Monitor
  Serial.begin(115200);
  delay(1000);

  
   Serial.println("This Is a Node Device");
  //Set device in AP mode to begin with
   WiFi.mode(WIFI_AP);
  // configure device AP mode
   configDeviceAP();
  // This is the mac address of the Slave in AP Mode
   Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());

 

 InitESPNow();


  // Pushbutton uses built-in pullup resistor
  pinMode(STATUS_BUTTON, INPUT_PULLUP);

  // LED Output
  pinMode(STATUS_LED, OUTPUT);
}



const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data

boolean newData = false;


void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

void showNewData() {
    if (newData == true) {
        Serial.println(receivedChars);
        newData = false;
    }
}





void loop()
{



   recvWithEndMarker();

   if (newData)
   {
    broadcast(receivedChars);
    newData = false;
   }
   


  if (digitalRead(STATUS_BUTTON))
  {
    // Detect the transition from low to high
    if (!buttonDown)
    {
      buttonDown = true;
      
      // Toggle the LED state
      ledOn = !ledOn;
      digitalWrite(STATUS_LED, ledOn);
      
      // Send a message to all devices
      if (ledOn)
      {
        broadcast("on");
      }
      else
      {
        broadcast("off");
      }
    }
    
    // Delay to avoid bouncing
    delay(500);
  }
  else
  {
    // Reset the button state
    buttonDown = false;
  }
}