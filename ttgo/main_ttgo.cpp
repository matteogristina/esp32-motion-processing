#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <SparkFunLSM6DSO.h>
#include <Wire.h>
#include <deque>
#include <cstdlib>
#include <algorithm>
#include <numeric> //for peak detection algorithm
#include <WiFi.h>
#include <HTTPClient.h>


// for bluetooth initialization
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

//peak detection algorithm constants
#define BUFF_SIZE 10
#define THRESH 90
#define INFLUENCE 0.8

// threshold values for jump and step
#define JUMP_THRESH 5.0
#define STEP_THRESH 0.9

typedef long double ld;
typedef unsigned int uint;
typedef std::deque<ld>::iterator deq_iter_ld;

LSM6DSO myIMU; //Default constructor is I2C, addr 0x6B
BLECharacteristic* pCharacteristic = NULL;

double aX, aY, aZ, gX, gY, gZ;
std::deque<ld> buffer;
std::deque<ld> fil_input;
ld fil_mean;
ld fil_stdev;
int output_signal;
std::string msg = "steps: ";
std::string msg2 = " jumps: ";
std::string msg3 = "  with response time (us): ";

int stepcount = 0;
int jumpcount = 0;

/**
 * This class calculates mean and standard deviation of a subvector.
 * This is basically stats computation of a subvector of a window size qual to "lag".
 */
class VectorStats {
  public:
      /**
       * Constructor for VectorStats class.
       *
       * @param start - This is the iterator position of the start of the window,
       * @param end   - This is the iterator position of the end of the window,
       */

      VectorStats(deq_iter_ld start, deq_iter_ld end) {
        this->start = start;
        this->end = end;
        this->compute();
    }
  
      /**
       * This method calculates the mean and standard deviation using STL function.
       * This is the Two-Pass implementation of the Mean & Variance calculation.
       */
      void compute() {
          ld sum = std::accumulate(start, end, 0.0);
          uint slice_size = std::distance(start, end);
          ld mean = sum / slice_size;
          std::vector<ld> diff(slice_size);
          std::transform(start, end, diff.begin(), [mean](ld x) { return x - mean; });
          ld sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
          ld std_dev = std::sqrt(sq_sum / slice_size);
  
          this->m1 = mean;
          this->m2 = std_dev;
      }
  
      ld mean() {
          return m1;
      }
  
      ld standard_deviation() {
          return m2;
      }
  
  private:
      deq_iter_ld start;
      deq_iter_ld end;
      ld m1;
      ld m2;
  };

/**
 * @brief Constructs a status message containing the number of steps, jumps, and a timestamp.
 * 
 * @param steps The number of detected steps.
 * @param jumps The number of detected jumps.
 * @param time  The timestamp in milliseconds or microseconds (depending on system usage).
 * 
 * @return std::string A formatted message string combining the input values.
 */

std::string create_msg(int steps, int jumps, unsigned long time) {

  std::string full_msg = msg + std::to_string(stepcount) + msg2 + std::to_string(jumpcount) + msg3 + std::to_string(time);
  return full_msg;

}

void setup() {

  Serial.begin(115200);
  Wire.begin(21, 22);

  Serial.println("Starting bluetooth service...");
 
  BLEDevice::init("SDSUCS");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                                        CHARACTERISTIC_UUID,
                                        BLECharacteristic::PROPERTY_READ |
                                        BLECharacteristic::PROPERTY_WRITE
                                      );
 
  pCharacteristic->setValue("Server Example -- SDSU IOT");
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0); 
  BLEDevice::startAdvertising();

  // check for output on NRF connect

  Serial.println("Initializing I2C and calibrating gyro...");

  myIMU.begin();
  myIMU.initialize(BASIC_SETTINGS);

  // CALIBRATION

  aX = 0, aY = 0, aZ = 0, gX = 0, gY = 0, gZ = 0;

  for (int i = 0; i < 2000; i++) {

    aX += myIMU.readFloatAccelX();
    aY += myIMU.readFloatAccelY();
    aZ += myIMU.readFloatAccelZ();

    gX += myIMU.readFloatGyroX();
    gY += myIMU.readFloatGyroY();
    gZ += myIMU.readFloatGyroZ();

    delay(1);

  }

  aX /= 2000;
  aY /= 2000;
  aZ /= 2000;

  gX /= 2000;
  gY /= 2000;
  gZ /= 2000;

  for (int i = 0; i < BUFF_SIZE; i ++) {

    buffer.push_back(0.0);
    fil_input.push_back(0.0);
  }

  Serial.println("Done calibrating, device starting...");

}

void loop() {

  float reading = myIMU.readFloatGyroZ() - gZ;
  int64_t time_start = esp_timer_get_time();
  if (reading < 0) {
    reading *= -1;
  } else {
    reading = 0;
  }

  if (buffer.size() == BUFF_SIZE) {

    buffer.pop_front();
    buffer.push_back(reading);

    if (std::abs(reading - fil_mean) > THRESH * fil_stdev) {
        if (reading - fil_mean > JUMP_THRESH) {
          output_signal = 2;
        } else if (reading - fil_mean > STEP_THRESH) {
          output_signal = 1;
        }

        if (fil_input.size() >= BUFF_SIZE) {

            fil_input.pop_front();

        }

        fil_input.push_back(INFLUENCE * reading + ((1 - INFLUENCE) * fil_input[6]));

    } else {
      output_signal = 0;
        if (fil_input.size() >= BUFF_SIZE) {

            fil_input.pop_front();

        }
        fil_input.push_back(reading);
    }

  } else {
    buffer.push_back(reading);
  }

  if (output_signal) {

    if (output_signal == 2) {
      Serial.print("JUMP: ");
      jumpcount++;
    } else if (output_signal == 1) {
      Serial.print("STEP: ");
      stepcount++;
    }

    pCharacteristic->setValue(create_msg(stepcount, jumpcount, esp_timer_get_time() - time_start));
    pCharacteristic->notify();

  }

  VectorStats lag_subvector_stats(fil_input.begin(), fil_input.end());
  fil_mean = lag_subvector_stats.mean();
  fil_stdev = lag_subvector_stats.standard_deviation();


  delay(20);

}