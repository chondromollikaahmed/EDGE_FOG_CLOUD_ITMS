#ifdef ESP32
  #include <WiFi.h>
#else
  #include <ESP8266WiFi.h>
#endif


#include <esp_now.h> 


#define NUMOFNODES 20
esp_now_peer_info_t nodes[NUMOFNODES] = {};
int nodeCount = 0;


#define CHANNEL 3
#define PRINTSCANRESULTS 0




// Scan for Nodes in Access Point mode
void ScanForNode() {
  int8_t scanResults = WiFi.scanNetworks();
  //reset nodes
  memset(nodes, 0, sizeof(nodes));
  nodeCount = 0;
  if (scanResults == 0) { Serial.println("No WiFi devices in AP Mode found");} 
  else
   {
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
      }
      delay(10);



      // Check if the current device starts with `Node`

      
      if (SSID.indexOf("Node") == 0) {
        // SSID of interest
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];

        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int j = 0; j < 6; ++j ) {
            nodes[nodeCount].peer_addr[j] = (uint8_t) mac[j];
          }
        }
        nodes[nodeCount].channel = CHANNEL; // pick a channel
        nodes[nodeCount].encrypt = 0; // no encryption
        nodeCount++;
      }
    }
  }

  if (nodeCount > 0) {
    Serial.print(nodeCount); Serial.println(" Nodes(s) found, processing..");
  } else {
    Serial.println("No Nodes Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}






// Check if the nodes is already paired with the master.
// If not, pair the node with master
void manageNode() {
  if (nodeCount > 0) {
    for (int i = 0; i < nodeCount; i++) {
      Serial.print("Processing: ");
      for (int ii = 0; ii < 6; ++ii ) {
        Serial.print((uint8_t) nodes[i].peer_addr[ii], HEX);
        if (ii != 5) Serial.print(":");
      }
      Serial.print(" Status: ");
      // check if the peer exists
      bool exists = esp_now_is_peer_exist(nodes[i].peer_addr);
      if (exists) {
        // Slave already paired.
        Serial.println("Already Paired");
      } else {
        // Slave not paired, attempt pair
        esp_err_t addStatus = esp_now_add_peer(&nodes[i]);
        if (addStatus == ESP_OK) {
          // Pair success
          Serial.println("Pair success");
        } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
          // How did we get so far!!
          Serial.println("ESPNOW Not Init");
        } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
          Serial.println("Add Peer - Invalid Argument");
        } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
          Serial.println("Peer list full");
        } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
          Serial.println("Out of memory");
        } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
          Serial.println("Peer Exists");
        } else {
          Serial.println("Not sure what happened");
        }
        delay(100);
      }
    }
  } else {
    // No slave found to process
    Serial.println("No Slave found to process");
  }
}








void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
