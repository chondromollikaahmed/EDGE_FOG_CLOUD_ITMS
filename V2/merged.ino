
// Include Libraries
#ifdef ESP32
#include <WiFi.h>
#include <esp_now.h>
#else
#include <ESP8266WiFi.h>
#include <espnow.h>
#endif


// for mpu9250
#include <MPU9250_WE.h>
#include <Wire.h>
#define MPU9250_ADDR 0x68
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

// code for gps module
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// On ESP32: GPIO-21(SDA), GPIO-22(SCL)
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // See datasheet for Address
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

int nodeCount = 0;
String destmac[10];

// fog address
const uint8_t *fogAddr;
String geoData;

unsigned long previousMillis = 0;    // keep track of the time since the last GPS measurement
const unsigned long interval = 1000; // time interval between GPS measurements in milliseconds

float previousSpeed = 0;

float acceleration = 0;

String compassData;

double minDist = 100000000.0;
int closestCar = -1, decision = -1;

// defining structure of to send gps data
typedef struct
{
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

double RelativePositionFromFog(double mylat, double mylon)
{
  float latitude = 20.67890;
  float longitude = 23.45678;
  double myrelativePosition = latitude - mylat;
  return myrelativePosition;
}

int compareGPSData(const void *a, const void *b)
{
  const GPSData *gpsDataA = (const GPSData *)a;
  const GPSData *gpsDataB = (const GPSData *)b;

  if (gpsDataA->relativePosition < gpsDataB->relativePosition)
  {
    return -1;
  }
  else if (gpsDataA->relativePosition > gpsDataB->relativePosition)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void DecideFrontBack()
{
  qsort(gpsDataArray, numGPSData, sizeof(GPSData), compareGPSData);
}

int findGPSDataIndex(String macAddress)
{
  int low = 0;
  int high = numGPSData - 1;

  while (low <= high)
  {
    int mid = (low + high) / 2;
    int cmp = macAddress.compareTo(gpsDataArray[mid].macAddr);

    if (cmp == 0)
    {
      return mid;
    }
    else if (cmp < 0)
    {
      high = mid - 1;
    }
    else
    {
      low = mid + 1;
    }
  }
  // If the macAddress is not found, return -1
  return -1;
}

String overtake = "";
void DecideOvertake()
{

  int index = findGPSDataIndex(WiFi.softAPmacAddress());
  double front1stAccelaration = gpsDataArray[index - 1].accelaration;
  double front2ndAcclaration = gpsDataArray[index - 2].accelaration;
  if (index > 1)
  {
    double gapBetweentwoCar = haversine(gpsDataArray[index - 1].latitude, gpsDataArray[index - 1].longitude, gpsDataArray[index - 2].latitude, gpsDataArray[index - 2].longitude);
    gapBetweentwoCar = gapBetweentwoCar * 1000; // in meter

    double carLength; // need to give a cars min length in meter

    if (gapBetweentwoCar >= (carLength * 1.5) && front2ndAcclaration >= front1stAccelaration)
    {
      Serial.println("Overtake");
      overtake = "S";
    }
    else
    {
      Serial.println("Don't Overtake");
      overtake = "N";
    }
  }
  else if (index == 0)
  {
    Serial.println("Speed Up");
    overtake = "S";
  }
  else
  {
    Serial.println("Overtake");
    overtake = "S";
  }
}

void CalculateDistanceAndPredictSpeed()
{

  Serial.println("Number Of GPS DATA :" + String(numGPSData));
  /*for (int i = 1; i < numGPSData; i++)
    {
        double dist = haversine(gps.location.lat(), gps.location.lng(), gpsDataArray[i].latitude, gpsDataArray[i].longitude);
        Serial.println("Haversine Distance Between "+ String (dist));
        if (dist < minDist)
        {
            minDist = dist;
            closestCar = i;
        }
    }*/
  int index = findGPSDataIndex(WiFi.softAPmacAddress());

  // Print out the closest car
  Serial.print("Closest car: ");
  Serial.println(closestCar);
  double predicted_cover_dist = (/*[GPS Speed self car in km/hr]*/ gps.speed.kmph() * 2 * (1000 / 3600)) + (0.5 * acceleration * pow(2, 2)); // #s=u*t+0.5*a*pow(t,2)
  double front_car_predicted_cover_dist = (/*[GPS Speed front min distance car in km/hr]*/ gpsDataArray[index - 1].speed * 2 * (1000 / 3600)) + (0.5 * acceleration * pow(2, 2));
  minDist = haversine(gps.location.lat(), gps.location.lng(), gpsDataArray[index-1].latitude, gpsDataArray[index-1].longitude) + front_car_predicted_cover_dist; 
  Serial.println("Predicted Cover Distance : " + String(predicted_cover_dist));
  Serial.println("Front car Cover Distance :" + String(front_car_predicted_cover_dist));
  if (predicted_cover_dist <= minDist / 2)
  {
    decision = 3;
  }
  else if (predicted_cover_dist <= minDist and predicted_cover_dist > minDist / 2)
  {
    decision = 2;
  }
  else if (predicted_cover_dist > minDist and predicted_cover_dist <= minDist * 1.5)
  {
    decision = 1;
  }
  else
  {
    decision = 0;
  }
}

float calculateAcceleration(float timeInterval)
{
  if (gps.location.isValid() == 1)
  {
    float currentSpeed = gps.speed.kmph() / 3.6; // speed in m/s

    Serial.println("Speed " + String(currentSpeed));
    Serial.println("Previous Speed " + String(previousSpeed - currentSpeed));
    acceleration = (currentSpeed - previousSpeed) / timeInterval;
    previousSpeed = currentSpeed;
  }
  Serial.println("Aclleartion :" + String(acceleration));
  return acceleration;
}

String Bearing_to_Ordinal(float bearing)
{
  if (bearing >= 348.75 || bearing < 11.25)
    return "N";
  if (bearing >= 11.25 && bearing < 33.75)
    return "NNE";
  if (bearing >= 33.75 && bearing < 56.25)
    return "NE";
  if (bearing >= 56.25 && bearing < 78.75)
    return "ENE";
  if (bearing >= 78.75 && bearing < 101.25)
    return "E";
  if (bearing >= 101.25 && bearing < 123.75)
    return "ESE";
  if (bearing >= 123.75 && bearing < 146.25)
    return "SE";
  if (bearing >= 146.25 && bearing < 168.75)
    return "SSE";
  if (bearing >= 168.75 && bearing < 191.25)
    return "S";
  if (bearing >= 191.25 && bearing < 213.75)
    return "SSW";
  if (bearing >= 213.75 && bearing < 236.25)
    return "SW";
  if (bearing >= 236.25 && bearing < 258.75)
    return "WSW";
  if (bearing >= 258.75 && bearing < 281.25)
    return "W";
  if (bearing >= 281.25 && bearing < 303.75)
    return "WNW";
  if (bearing >= 303.75 && bearing < 326.25)
    return "NW";
  if (bearing >= 326.25 && bearing < 348.75)
    return "NNW";
  return "?";
}

double haversine(double lat1, double lon1, double lat2, double lon2)
{
  double dLat = (lat2 - lat1) * M_PI / 180.0;
  double dLon = (lon2 - lon1) * M_PI / 180.0;
  lat1 = lat1 * M_PI / 180.0;
  lat2 = lat2 * M_PI / 180.0;

  double a = sin(dLat / 2) * sin(dLat / 2) * sin(dLat / 2) + sin(dLon / 2) * sin(dLon / 2) * cos(lat1) * cos(lat2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double d = 6371 * c;
  return d;
}

void print_speed()
{
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (gps.location.isValid() == 1)
  {

    compassData = Bearing_to_Ordinal(gps.course.deg());
    // String gps_speed = String(gps.speed.kmph());
    display.setTextSize(1);

    display.setCursor(25, 5);
    display.print("Lat: ");
    display.setCursor(50, 5);
    display.print(gps.location.lat(), 6);
    Serial.println(gps.location.lat(), 6);

    display.setCursor(25, 20);
    display.print("Lng: ");
    display.setCursor(50, 20);
    display.print(gps.location.lng(), 6);

    display.setCursor(25, 35);
    display.print("Speed: ");
    display.setCursor(65, 35);
    display.print(gps.speed.kmph());

    display.setTextSize(1);
    display.setCursor(0, 50);
    display.print("OVT:");
    display.setCursor(25, 50);
    display.print(overtake + String(numGPSData));

    display.setTextSize(1);
    display.setCursor(70, 50);
    display.print("De:");
    display.setCursor(95, 50);
    // display.print(gps.altitude.meters(), 0);
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
void getGpsData()
{

  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (neogps.available())
    {
      if (gps.encode(neogps.read()))
      {
        newData = true;
      }
    }
  }
  // If newData is true
  if (newData == true)
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
  if (dataLen == sizeof(GPSData))
  {

    Serial.println(" ENtered in the GPSDATA SHell ");
    // Convert incoming data to GPSData struct
    GPSData gpsData;
    memcpy(&gpsData, data, sizeof(gpsData));

    // add value to gps data

    for (int i = 0; i < 6; i++)
    {
      gpsData.macAddr.concat(String(macAddr[i], HEX)); // append each byte as a hex string to the String object
    }
    // Store GPSData struct in array
    // Condition
    // check if macaddres if not exist previously in the GPSData Array
    // 2. if in the trusted node address and also in range

    // Check if GPSData struct is already in array
    bool found = false;
    for (int i = 0; i < numGPSData; i++)
    {
      if (gpsData.macAddr == gpsDataArray[i].macAddr)
      {
        found = true;
        break;
      }
    }
    // Store GPSData struct in array if not already there
    if (!found && numGPSData < 20)
    {
      gpsDataArray[numGPSData++] = gpsData;
    }
    // Check if each GPSData struct in array is present in destmac array
    int numDeleted = 0;
    for (int i = 0; i < numGPSData; i++)
    {
      bool present = false;
      for (int j = 0; j < nodeCount; j++)
      {
        if (gpsDataArray[i].macAddr == destmac[j])
        {
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

    String msg;
    msg = buffer;
    // If Fog Send The Message Then It will start with Node
    if (msg.indexOf("Node") == 0)
    {
      fogAddr = macAddr;
      Serial.println("Nodes Found From Message ");
      int count = 0;
      String newmsg;
      newmsg = msg;
      while (newmsg.indexOf("Node") == 0)
      {
        destmac[count] = newmsg.substring(newmsg.indexOf("Node") + 5, 22);
        Serial.println(newmsg);
        newmsg = newmsg.substring(22, newmsg.length());
        nodeCount = count + 1;
        count++;
      }
      for (int i = 0; i < nodeCount; i++)
      {
        Serial.println("Node ->" + destmac[i]);
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
  uint8_t broadcastAddress[6];
  for (int i = 0; i < nodeCount; i++)
  {
    int mac[6];
    if (6 == sscanf(destmac[i].c_str(), "%x:%x:%x:%x:%x:%x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]))
    {
      Serial.println("Filtering Mac Adress from String");
      for (int ii = 0; ii < 6; ++ii)
      {
        broadcastAddress[ii] = (uint8_t)mac[ii];
      }
    }
    esp_now_peer_info_t peerInfo = {};
    memcpy(&peerInfo.peer_addr, broadcastAddress, 6);
    if (!esp_now_is_peer_exist(broadcastAddress))
    {
      esp_now_add_peer(&peerInfo);
    }

    // prepare message
    gpsDataSend.latitude = gps.location.lat();
    gpsDataSend.longitude = gps.location.lng();
    Serial.println("Lat  ____>" + String(gpsDataSend.latitude, 6));
    Serial.println("Lon  ____>" + String(gpsDataSend.longitude, 6));
    gpsDataSend.speed = gps.speed.kmph();
    gpsDataSend.accelaration = acceleration;
    gpsDataSend.relativePosition = RelativePositionFromFog(gpsDataSend.latitude, gpsDataSend.longitude);
    // Send message
    esp_err_t result = esp_now_send(broadcastAddress, (const uint8_t *)&gpsDataSend, sizeof(gpsDataSend));

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
      Serial.println("Unknown error" + String(result));
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
  uint8_t broadcastAddress[6];

  for (int i = 0; i < nodeCount; i++)
  {

    int mac[6];
    if (6 == sscanf(destmac[i].c_str(), "%x:%x:%x:%x:%x:%x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]))
    {
      Serial.println("Filtering Mac Adress from String");
      for (int ii = 0; ii < 6; ++ii)
      {
        broadcastAddress[ii] = (uint8_t)mac[ii];
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
      Serial.println("Unknown error" + String(result));
    }
  }
  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Nodes in AP Mode
}

void configDeviceAP()
{
  String Prefix = "Node:";
  String Mac = WiFi.macAddress();
  String SSID = Prefix + Mac;
  String Password = "123456789";
  bool result = WiFi.softAP(SSID.c_str(), Password.c_str(), CHANNEL, 0);
  if (!result)
  {
    Serial.println("AP Config failed.");
  }
  else
  {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}

// Init ESP Now with fallback
void InitESPNow()
{
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK)
  {
    Serial.println("ESPNow Init Success");

    esp_now_register_recv_cb(receiveCallback);
    esp_now_register_send_cb(sentCallback);
  }
  else
  {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}


const float g = 9.81; // Acceleration due to gravity in m/s^2

// Complementary filter parameters
const float alpha = 0.9; // Weight of gyro angle estimate
const float dt = 0.01; // Time step in seconds

// Initial orientation estimates
float roll = 0.0; // Roll angle in radians
float pitch = 0.0; // Pitch angle in radians
// modify as needed 
void Printv2()
{

display.clearDisplay();
display.setTextSize(1.5);
display.setCursor(0,0);
  display.print("X: ");
  display.println(acc_z);


 display.setTextSize(1.5);
display.setCursor(0,20);
  display.print("Y: ");
  display.println(acc_y);


display.setTextSize(1.5);
display.setCursor(0,40);
  display.print("z: ");
display.println(acc_z);


display.setTextSize(1.5);
display.setCursor(60,20);
  display.print("S: ");
display.println(String(speed, 5));
display.setTextSize(1);
display.setCursor(80,30);
display.println("ms-1");


display.setTextSize(1.5);
display.setCursor(60,40);
display.print("A: ");
display.println(String(acceleration, 5));
display.setTextSize(1);
display.setCursor(80,50);
display.println("ms-2");

display.display();
}
float acc_x, acc_y,acc_z,acceleration;
void UpdateMPUdata()
{

    xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gyr = myMPU9250.getGyrValues();
  xyzFloat magValue = myMPU9250.getMagValues();
  float temp = myMPU9250.getTemperature();
  float resultantG = myMPU9250.getResultantG(gValue);

  Serial.println("Acceleration in g (x,y,z):");
  Serial.print(gValue.x);
   acc_x=gValue.x;
  Serial.print("   ");
  Serial.print(gValue.y);
    acc_y=gValue.y;
  Serial.print("   ");
  Serial.println(gValue.z);
    acc_z=gValue.z-1;
  Serial.print("Resultant g: ");
  Serial.println(resultantG);


  Serial.println("Gyroscope data in degrees/s: ");
  Serial.print(gyr.x);
 float  gx =gyr.x;
  Serial.print("   ");
  Serial.print(gyr.y);
  float   gy =gyr.y;
  Serial.print("   ");
  Serial.println(gyr.z);
  float   gz =gyr.z;


      // Compute roll and pitch angles using accelerometer and gyroscope readings
  float accel_roll = atan2(-acc_y, acc_z);
  float accel_pitch = atan2(acc_x, sqrt(acc_y * acc_y + acc_z * acc_z));
  float gyro_roll = roll + gx * dt / 131.0;
  float gyro_pitch = pitch + gy * dt / 131.0;

  // Combine accelerometer and gyroscope estimates using a complementary filter
  roll = alpha * gyro_roll + (1 - alpha) * accel_roll;
  pitch = alpha * gyro_pitch + (1 - alpha) * accel_pitch;

  // Calculate the acceleration in the direction of motion
  float cos_theta = cos(roll);
  float sin_theta = sin(roll);
  float cos_phi = cos(pitch);
  float sin_phi = sin(pitch);
  float acceleration_x = acc_x * cos_theta + acc_y * sin_phi * sin_theta + acc_z * cos_phi * sin_theta;
  float acceleration_y = acc_y * cos_phi - acc_z * sin_phi;
  float acceleration_z = -acc_x * sin_theta + acc_y * sin_phi * cos_theta + acc_z * cos_phi * cos_theta;
   acceleration = g * cos_theta * sqrt(acceleration_x * acceleration_x + acceleration_z * acceleration_z);

  // Integration
  static float speed = 0.0;
  static float last_time = millis() / 1000.0;

  float current_time = millis() / 1000.0;
  float time_diff = current_time - last_time;
 
  speed+ = acceleration * time_diff;
  last_time = current_time;

  Serial.println("Acceleration (m/s^2): " + String(acceleration, 3));
  Serial.println("Speed (m/s): " + String(speed, 3));



  Serial.println("Magnetometer Data in µTesla: ");
  Serial.print(magValue.x);
  Serial.print("   ");
  Serial.print(magValue.y);
  Serial.print("   ");
  Serial.println(magValue.z);

  Serial.print("Temperature in °C: ");
  Serial.println(temp);

  Serial.println("********************************************");

}

void setup()
{

 
  // Set up Serial Monitor
  Serial.begin(115200);


  // setting up MPU9250 

  Wire.begin();
  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }
  if(!myMPU9250.initMagnetometer()){
    Serial.println("Magnetometer does not respond");
  }
  else{
    Serial.println("Magnetometer is connected");
  }

  /* The slope of the curve of acceleration vs measured values fits quite well to the theoretical 
   * values, e.g. 16384 units/g in the +/- 2g range. But the starting point, if you position the 
   * MPU9250 flat, is not necessarily 0g/0g/1g for x/y/z. The autoOffset function measures offset 
   * values. It assumes your MPU9250 is positioned flat with its x,y-plane. The more you deviate 
   * from this, the less accurate will be your results.
   * The function also measures the offset of the gyroscope data. The gyroscope offset does not   
   * depend on the positioning.
   * This function needs to be called at the beginning since it can overwrite your settings!
   */
  Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
  delay(1000);
  myMPU9250.autoOffsets();
  Serial.println("Done!");
  
  /*  This is a more accurate method for calibration. You have to determine the minimum and maximum 
   *  raw acceleration values of the axes determined in the range +/- 2 g. 
   *  You call the function as follows: setAccOffsets(xMin,xMax,yMin,yMax,zMin,zMax);
   *  Use either autoOffset or setAccOffsets, not both.
   */
  //myMPU9250.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);

  /*  The gyroscope data is not zero, even if you don't move the MPU9250. 
   *  To start at zero, you can apply offset values. These are the gyroscope raw values you obtain
   *  using the +/- 250 degrees/s range. 
   *  Use either autoOffset or setGyrOffsets, not both.
   */
  //myMPU9250.setGyrOffsets(45.0, 145.0, -105.0);

  /*  You can enable or disable the digital low pass filter (DLPF). If you disable the DLPF, you 
   *  need to select the bandwdith, which can be either 8800 or 3600 Hz. 8800 Hz has a shorter delay,
   *  but higher noise level. If DLPF is disabled, the output rate is 32 kHz.
   *  MPU9250_BW_WO_DLPF_3600 
   *  MPU9250_BW_WO_DLPF_8800
   */
  myMPU9250.enableGyrDLPF();
  //myMPU9250.disableGyrDLPF(MPU9250_BW_WO_DLPF_8800); // bandwdith without DLPF
  
  /*  Digital Low Pass Filter for the gyroscope must be enabled to choose the level. 
   *  MPU9250_DPLF_0, MPU9250_DPLF_2, ...... MPU9250_DPLF_7 
   *  
   *  DLPF    Bandwidth [Hz]   Delay [ms]   Output Rate [kHz]
   *    0         250            0.97             8
   *    1         184            2.9              1
   *    2          92            3.9              1
   *    3          41            5.9              1
   *    4          20            9.9              1
   *    5          10           17.85             1
   *    6           5           33.48             1
   *    7        3600            0.17             8
   *    
   *    You achieve lowest noise using level 6  
   */
  myMPU9250.setGyrDLPF(MPU9250_DLPF_6);

  /*  Sample rate divider divides the output rate of the gyroscope and accelerometer.
   *  Sample rate = Internal sample rate / (1 + divider) 
   *  It can only be applied if the corresponding DLPF is enabled and 0<DLPF<7!
   *  Divider is a number 0...255
   */
  myMPU9250.setSampleRateDivider(5);

  /*  MPU9250_GYRO_RANGE_250       250 degrees per second (default)
   *  MPU9250_GYRO_RANGE_500       500 degrees per second
   *  MPU9250_GYRO_RANGE_1000     1000 degrees per second
   *  MPU9250_GYRO_RANGE_2000     2000 degrees per second
   */
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);

  /*  MPU9250_ACC_RANGE_2G      2 g   (default)
   *  MPU9250_ACC_RANGE_4G      4 g
   *  MPU9250_ACC_RANGE_8G      8 g   
   *  MPU9250_ACC_RANGE_16G    16 g
   */
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);

  /*  Enable/disable the digital low pass filter for the accelerometer 
   *  If disabled the bandwidth is 1.13 kHz, delay is 0.75 ms, output rate is 4 kHz
   */
  myMPU9250.enableAccDLPF(true);

  /*  Digital low pass filter (DLPF) for the accelerometer, if enabled 
   *  MPU9250_DPLF_0, MPU9250_DPLF_2, ...... MPU9250_DPLF_7 
   *   DLPF     Bandwidth [Hz]      Delay [ms]    Output rate [kHz]
   *     0           460               1.94           1
   *     1           184               5.80           1
   *     2            92               7.80           1
   *     3            41              11.80           1
   *     4            20              19.80           1
   *     5            10              35.70           1
   *     6             5              66.96           1
   *     7           460               1.94           1
   */
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);

  /* You can enable or disable the axes for gyroscope and/or accelerometer measurements.
   * By default all axes are enabled. Parameters are:  
   * MPU9250_ENABLE_XYZ  //all axes are enabled (default)
   * MPU9250_ENABLE_XY0  // X, Y enabled, Z disabled
   * MPU9250_ENABLE_X0Z   
   * MPU9250_ENABLE_X00
   * MPU9250_ENABLE_0YZ
   * MPU9250_ENABLE_0Y0
   * MPU9250_ENABLE_00Z
   * MPU9250_ENABLE_000  // all axes disabled
   */
  //myMPU9250.enableAccAxes(MPU9250_ENABLE_XYZ);
  //myMPU9250.enableGyrAxes(MPU9250_ENABLE_XYZ);
  
  /*
   * AK8963_PWR_DOWN       
   * AK8963_CONT_MODE_8HZ         default
   * AK8963_CONT_MODE_100HZ
   * AK8963_FUSE_ROM_ACC_MODE 
   */
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
  delay(200);

  // Begin serial communication Neo6mGPS
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.display();

  delay(1000);

  Serial.println("This Is a Node Device");
  // Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: ");
  Serial.println(WiFi.softAPmacAddress());

  InitESPNow();

  // Pushbutton uses built-in pullup resistor
  pinMode(STATUS_BUTTON, INPUT_PULLUP);

  // LED Output
  pinMode(STATUS_LED, OUTPUT);

  gpsData.macAddr = WiFi.softAPmacAddress();
  gpsData.latitude = gps.location.lat();
  gpsData.longitude = gps.location.lng();
  Serial.println("Lat  ____>" + String(gpsData.latitude));
  Serial.println("Lon  ____>" + String(gpsData.longitude));
  gpsData.speed = gps.speed.kmph();
  gpsData.accelaration = acceleration;
  gpsData.relativePosition = RelativePositionFromFog(gpsData.latitude, gpsData.longitude);
  gpsDataArray[numGPSData++] = gpsData;
}

const byte numChars = 32;
char receivedChars[numChars]; // an array to store the received data

boolean newData = false;

void recvWithEndMarker()
{
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial.available() > 0 && newData == false)
  {
    rc = Serial.read();

    if (rc != endMarker)
    {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars)
      {
        ndx = numChars - 1;
      }
    }
    else
    {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

void showNewData()
{
  if (newData == true)
  {
    Serial.println(receivedChars);
    newData = false;
  }
}

void accident_detection()
{
}

void loop()
{

    UpdateMPUdata();
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    acceleration = calculateAcceleration(1.0);
    previousMillis = currentMillis;
  }

  Serial.println("Accelaeration " + String(acceleration));

  // recvWithEndMarker();
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
