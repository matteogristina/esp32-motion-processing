#include <Arduino.h>
#include <WiFi.h>
#include <HttpClient.h>
#include <stdint.h>
#include <SparkFunLSM6DSO.h>
#include <Wire.h>

LSM6DSO myIMU; //Default constructor is I2C, addr 0x6B

// This example downloads the URL "http://arduino.cc/"

char ssid[] = "NETWORK";    // your network SSID (name) 
char pass[] = "PASSWORD"; // your network password (use for WPA, or use as key for WEP)

const char kHostname[] = "[IPADDR]";
uint16_t kPort = 5000;
const char kPath[] = "/api/detect?sensor=%.2f&timestamp=%";

// Number of milliseconds to wait without receiving any data before we give up
const int kNetworkTimeout = 30*1000;
// Number of milliseconds to wait if no data is available before trying again
const int kNetworkDelay = 1000;

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 0;

float reading;
float gZ;

int64_t get_curr_time() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  unsigned long long epoch_ms = (uint64_t)tv.tv_sec * 1000ULL + tv.tv_usec / 1000ULL;
  return epoch_ms;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  // We start by connecting to a WiFi network
  delay(1000);
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("MAC address: ");
  Serial.println(WiFi.macAddress());
  
  // calibrate time to current time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // begin circuit

  myIMU.begin();
  myIMU.initialize(BASIC_SETTINGS);

  //  circuit calibration

  gZ = 0;

  for (int i = 0; i < 2000; i++) {

    gZ += myIMU.readFloatGyroZ();

    delay(1);

  }

  gZ /= 2000;

  Serial.println("Done calibrating, device starting...");

}

void loop() {

  int err = 0;
  reading = myIMU.readFloatGyroZ();

  int64_t ustimestamp = esp_timer_get_time();
  
  WiFiClient c;
  HttpClient http(c);

  char newPath[200];
  snprintf(newPath, sizeof(newPath), "/api/detect?sensor=%.2f&timestamp=%" PRId64, reading, ustimestamp);
  Serial.println(newPath);

  err = http.get(kHostname, kPort, newPath);
  if (err == 0)
  {
    err = http.responseStatusCode();
    if (err >= 0)
    {

      err = http.skipResponseHeaders();
      if (err >= 0)
      {
        int bodyLen = http.contentLength();
        unsigned long timeoutStart = millis();
        char c;
        while ( (http.connected() || http.available()) &&
               ((millis() - timeoutStart) < kNetworkTimeout) )
        {
            if (http.available())
            {
              c = http.read();
              Serial.print(c);   
              bodyLen--;
              timeoutStart = millis();
            }
            else
            {
              delay(kNetworkDelay);
            }
        }
        Serial.println();
      }
      else
      {
        Serial.print("Failed to skip response headers: ");
        Serial.println(err);
      }
    }
    else
    {    
      Serial.print("Getting response failed: ");
      Serial.println(err);
    }
  }
  else
  {
    Serial.print("Connect failed: ");
    Serial.println(err);
  }
  http.stop();

  delay(20);
}