
// Include Libraries
#ifdef ESP32
  #include <WiFi.h>
  #include <esp_now.h> 
#else
  #include <ESP8266WiFi.h>
  #include <espnow.h>
#endif



// code for gps module 
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

//On ESP32: GPIO-21(SDA), GPIO-22(SCL)
#define OLED_RESET -1 //Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C //See datasheet for Address
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


#define RXD2 16
#define TXD2 17
HardwareSerial neogps(1);
TinyGPSPlus gps;
TinyGPSPlus receivedGps;


#define CHANNEL 1

// Define LED and pushbutton state booleans
bool buttonDown = false;
bool ledOn = false;

// Define LED and pushbutton pins
#define STATUS_LED 15
#define STATUS_BUTTON 0


int nodeCount=0;
String destmac[10];

//fog address 
const uint8_t *fogAddr;
String geoData;


unsigned long previousMillis = 0;  // keep track of the time since the last GPS measurement
const unsigned long interval = 1000;  // time interval between GPS measurements in milliseconds


float previousSpeed =0;

float acceleration=0;

String compassData;


double minDist = 100000000.0;
int closestCar = -1,decision=-1;

// defining structure of to send gps data
typedef struct {
  String macAddr;
  float latitude;
  float longitude;
  float speed;
  double accelaration;
  String direction;
  double relativePosition;
} GPSData;

 GPSData gpsDataSend;
 GPSData gpsData;
 GPSData gpsDataArray[20];
 int numGPSData = 0;




 double RelativePositionFromFog(double mylat,double mylon) {
   float latitude=20.67890;
   float longitude=23.45678;
   double myrelativePosition = latitude-mylat ; 
   return myrelativePosition;
}

int compareGPSData(const void *a, const void *b) {
  const GPSData *gpsDataA = (const GPSData *)a;
  const GPSData *gpsDataB = (const GPSData *)b;
  
  if (gpsDataA->relativePosition < gpsDataB->relativePosition) {
    return -1;
  } else if (gpsDataA->relativePosition > gpsDataB->relativePosition) {
    return 1;
  } else {
    return 0;
  }
}


void DecideFrontBack()
{
  qsort(gpsDataArray, numGPSData, sizeof(GPSData), compareGPSData);
}


int findGPSDataIndex(String macAddress) {
  int low = 0;
  int high = numGPSData - 1;
  
  while (low <= high) {
    int mid = (low + high) / 2;
    int cmp = macAddress.compareTo(gpsDataArray[mid].macAddr);
    
    if (cmp == 0) {
      return mid;
    } else if (cmp < 0) {
      high = mid - 1;
    } else {
      low = mid + 1;
    }
  }
  // If the macAddress is not found, return -1
  return -1;
}

String overtake ="";
void DecideOvertake()
{

    int index =findGPSDataIndex(WiFi.softAPmacAddress());
    double front1stAccelaration = gpsDataArray[index-1].accelaration;
    double front2ndAcclaration = gpsDataArray[index-2].accelaration;
    if(index>1){
     double gapBetweentwoCar = haversine(gpsDataArray[index-1].latitude,gpsDataArray[index-1].longitude, gpsDataArray[index-2].latitude, gpsDataArray[index-2].longitude); 
       gapBetweentwoCar=gapBetweentwoCar*1000; //in meter

    double carLength; // need to give a cars min length in meter

    if(gapBetweentwoCar>=(carLength*1.5) && front2ndAcclaration>=front1stAccelaration){
        Serial.println("Ot");
        overtake="S";
    }
        else{
              Serial.println("no");
              overtake="N";
    }
    }
    else if(index==0){
              Serial.println("SU");
              overtake="S";
    }
    else{
              Serial.println("Ot");
              overtake="S";
    }

}

void CalculateDistanceAndPredictSpeed()
{
   
  Serial.println("Number Of GPS DATA :" + String (numGPSData));
  for (int i = 1; i < numGPSData; i++)
    {
        double dist = haversine(gps.location.lat(), gps.location.lng(), gpsDataArray[i].latitude, gpsDataArray[i].longitude);
        Serial.println("Haversine Distance Between "+ String (dist));
        if (dist < minDist)
        {
            minDist = dist;
            closestCar = i;
        }
    }

    // Print out the closest car
    Serial.print("Closest car: ");
    Serial.println(closestCar);
    double predicted_cover_dist = (/*[GPS Speed self car in km/hr]*/ gps.speed.kmph() * 2 *(1000/3600)) + (0.5 * acceleration * pow(2,2));    //#s=u*t+0.5*a*pow(t,2)
    minDist = (/*[GPS Speed front min distance car in km/hr]*/ gpsDataArray[closestCar].speed * 2 *(1000/3600)) + (0.5 * acceleration * pow(2,2));

    Serial.println("Predicted Cover Distance : " + String(predicted_cover_dist) );
    Serial.println("min Distance :"+ String( minDist));
    if(predicted_cover_dist <= minDist/2)
    {
        decision= 3;
    }
    else if(predicted_cover_dist <= minDist/2 and predicted_cover_dist > minDist/2)
    {
        decision = 2;
    }
    else if(predicted_cover_dist > minDist and predicted_cover_dist <= minDist/2)
    {
        decision=1;
    }
    else
    {
        decision=0;
    }
}


float calculateAcceleration(float timeInterval) {
  if (gps.location.isValid() == 1)
  {
  float currentSpeed = gps.speed.kmph() * 3.6;  // speed in m/s
  
  Serial.println("Speed "+ String(currentSpeed));
  Serial.println("Previous Speed "+ String(previousSpeed-currentSpeed));
  acceleration = (currentSpeed - previousSpeed) / timeInterval;
  previousSpeed = currentSpeed; 
  }
  Serial.println("Aclleartion :"+ String(acceleration));
  return acceleration;
}



String Bearing_to_Ordinal(float bearing) {
  if (bearing >= 348.75 || bearing < 11.25)  return "N";
  if (bearing >=  11.25 && bearing < 33.75)  return "NNE";
  if (bearing >=  33.75 && bearing < 56.25)  return "NE";
  if (bearing >=  56.25 && bearing < 78.75)  return "ENE";
  if (bearing >=  78.75 && bearing < 101.25) return "E";
  if (bearing >= 101.25 && bearing < 123.75) return "ESE";
  if (bearing >= 123.75 && bearing < 146.25) return "SE";
  if (bearing >= 146.25 && bearing < 168.75) return "SSE";
  if (bearing >= 168.75 && bearing < 191.25) return "S";
  if (bearing >= 191.25 && bearing < 213.75) return "SSW";
  if (bearing >= 213.75 && bearing < 236.25) return "SW";
  if (bearing >= 236.25 && bearing < 258.75) return "WSW";
  if (bearing >= 258.75 && bearing < 281.25) return "W";
  if (bearing >= 281.25 && bearing < 303.75) return "WNW";
  if (bearing >= 303.75 && bearing < 326.25) return "NW";
  if (bearing >= 326.25 && bearing < 348.75) return "NNW";
  return "?";
}


double haversine(double lat1, double lon1, double lat2, double lon2)
{
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    lat1 = lat1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;

    double a = sin(dLat/2) * sin(dLat/2) * sin(dLat/2) + sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = 6371 * c;
    return d;
}

void print_speed()
{
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
       
  if (gps.location.isValid() == 1)
  {

    compassData=Bearing_to_Ordinal(gps.course.deg());
   //String gps_speed = String(gps.speed.kmph());
    display.setTextSize(1);
    
    display.setCursor(25, 5);
    display.print("Lat: ");
    display.setCursor(50, 5);
    display.print(gps.location.lat(),6);
    Serial.println(gps.location.lat(),6);

    display.setCursor(25, 20);
    display.print("Lng: ");
    display.setCursor(50, 20);
    display.print(gps.location.lng(),6);

    display.setCursor(25, 35);
    display.print("Speed: ");
    display.setCursor(65, 35);
    display.print(gps.speed.kmph());
    
    display.setTextSize(1);
    display.setCursor(0, 50);
    display.print("OVT:");
    display.setCursor(25, 50);
    display.print(overtake+String(numGPSData));

    display.setTextSize(1);
    display.setCursor(70, 50);
    display.print("De:");
    display.setCursor(95, 50);
    //display.print(gps.altitude.meters(), 0);
    display.print(decision);

    display.display();
    
  }
  else
  {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.setTextSize(3);
    display.print("No Data");
    display.display();
  }  

}


// function to handle gps 
void getGpsData(){
  
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (neogps.available()  )
    {
      if (gps.encode(neogps.read()))
      {
        newData = true;
      }
    }
  }
  //If newData is true
  if(newData == true)
  {
    newData = false;
    Serial.println(gps.satellites.value());
     
    print_speed();
  }
  else
  {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.setTextSize(3);
    display.print("No Data");
    display.display();
  }
  
}



void formatMacAddress(const uint8_t *macAddr, char *buffer, int maxLength)
// Formats MAC Address
{
  snprintf(buffer, maxLength, "%02x:%02x:%02x:%02x:%02x:%02x", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
}





void receiveCallback(const uint8_t *macAddr, const uint8_t *data, int dataLen)
{
   // Check if incoming data is a GPSData struct
   if (dataLen == sizeof(GPSData)) {

     Serial.println(" ENtered in the GPSDATA SHell ");
    // Convert incoming data to GPSData struct
    GPSData gpsData;
    memcpy(&gpsData, data, sizeof(gpsData));

    //add value to gps data 

    for (int i = 0; i < 6; i++) {
    gpsData.macAddr.concat(String(macAddr[i], HEX)); // append each byte as a hex string to the String object
     }
    // Store GPSData struct in array
   //Condition
   //check if macaddres if not exist previously in the GPSData Array 
   //2. if in the trusted node address and also in range 
     
    // Check if GPSData struct is already in array
    bool found = false;
    for (int i = 0; i < numGPSData; i++) {
      if (gpsData.macAddr == gpsDataArray[i].macAddr) {
        found = true;
        break;
      }
    }
    // Store GPSData struct in array if not already there
    if (!found && numGPSData < 20) {
      gpsDataArray[numGPSData++] = gpsData;
    }
    // Check if each GPSData struct in array is present in destmac array
    int numDeleted = 0;
    for (int i = 0; i < numGPSData; i++) {
      bool present = false;
      for (int j = 0; j < nodeCount; j++) {
        if (gpsDataArray[i].macAddr == destmac[j]) {
          present = true;
          break;
        }
      }

      // Delete GPSData struct from array if not present in destmac array
      // if (!present) {
      //   for (int k = i - numDeleted; k < numGPSData - 1; k++) {
      //     gpsDataArray[k] = gpsDataArray[k+1];
      //   }
      //   numGPSData--;
      //   numDeleted++;
      //   i--;
      // }
   }
   }

    else
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
        //If Fog Send The Message Then It will start with Node
        if (msg.indexOf("Node") == 0) {
          fogAddr=macAddr; 
          Serial.println("Nodes Found From Message ");  
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

void broadGps()
{
  WiFi.mode(WIFI_STA);
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

    //prepare message 
     gpsDataSend.latitude = gps.location.lat(); 
     gpsDataSend.longitude = gps.location.lng();
     Serial.println("Lat  ____>" + String(gpsDataSend.latitude,6));
     Serial.println("Lon  ____>" + String(gpsDataSend.longitude,6));
     gpsDataSend.speed =gps.speed.kmph() ; 
     gpsDataSend.accelaration=acceleration;
     gpsDataSend.relativePosition=RelativePositionFromFog(gpsDataSend.latitude,gpsDataSend.longitude);
  // Send message
  esp_err_t result = esp_now_send(broadcastAddress,(const uint8_t*) &gpsDataSend, sizeof(gpsDataSend));

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

  //Begin serial communication Neo6mGPS
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);

  
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.display();
  
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

     gpsData.macAddr= WiFi.softAPmacAddress();
     gpsData.latitude = gps.location.lat(); 
     gpsData.longitude = gps.location.lng();
     Serial.println("Lat  ____>" + String(gpsData.latitude));
     Serial.println("Lon  ____>" + String(gpsData.longitude));
     gpsData.speed =gps.speed.kmph() ; 
     gpsData.accelaration=acceleration;
     gpsData.relativePosition=RelativePositionFromFog(gpsData.latitude,gpsData.longitude);
     gpsDataArray[numGPSData++] = gpsData;

  
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
   unsigned long currentMillis = millis();
 if (currentMillis - previousMillis >= interval) {
   acceleration = calculateAcceleration(1.0);
  previousMillis = currentMillis;
 }

 Serial.println("Accelaeration "+ String(acceleration));

   //recvWithEndMarker();
//   if (newData)
//   {
//    broadcast(receivedChars);
//    newData = false;
//   }

  broadGps();
  getGpsData();

  DecideFrontBack();
  DecideOvertake();

  CalculateDistanceAndPredictSpeed();
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
