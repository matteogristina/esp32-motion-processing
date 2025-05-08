#include "mongoose.h"
#include <stdio.h>
#include <string>
#include <vector>
#include <deque>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <numeric> //for peak detection algorithm
#include "time.h"

#define BUFF_SIZE 10
#define THRESH 3
#define INFLUENCE 0.8

#define JUMP_THRESH 2.0
#define STEP_THRESH 0.3

typedef long double ld;
typedef unsigned int uint;
typedef std::deque<ld>::iterator deq_iter_ld;

std::deque<ld> buffer;
std::deque<ld> fil_input;
ld fil_mean;
ld fil_stdev;
int output_signal;

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


int detect_step_or_jump(double reading) {

  if (reading < 0) {
    reading *= -1;
  } else {
    reading = 0;
  }

  if (buffer.size() == BUFF_SIZE) {

    buffer.pop_front();
    buffer.push_back(reading);

    printf("sens: %f, mean, %Lf\n", reading, fil_mean);

    if (std::abs(reading - fil_mean) > THRESH * fil_stdev) {
        if (reading - fil_mean > JUMP_THRESH) {
          output_signal = 2;
          printf("jump\n");
        } else if (reading - fil_mean > STEP_THRESH) {
          output_signal = 1;
          printf("step\n");
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

  VectorStats lag_subvector_stats(fil_input.begin(), fil_input.end());
  fil_mean = lag_subvector_stats.mean();
  fil_stdev = lag_subvector_stats.standard_deviation();

  return output_signal;

} 

int64_t xx_time_get_time() {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL));
}


// Connection event handler function
static void fn(struct mg_connection *c, int ev, void *ev_data) {
  if (ev == MG_EV_HTTP_MSG) {  // New HTTP request received
    struct mg_http_message *hm = (struct mg_http_message *) ev_data;  // Parsed HTTP request
    if (mg_match(hm->uri, mg_str("/api/detect"), NULL)) {              // REST API call?
      int len = 0;
      char gyro_reading[20];
      len = mg_http_get_var(&hm->query, "sensor", gyro_reading, sizeof(gyro_reading));
      if (len <= 0) {
          printf("no sensor\n");
      }

      char timestamp[100];
      len = mg_http_get_var(&hm->query, "timestamp", timestamp, sizeof(timestamp));
      if (len <= 0) {
           printf("no timestamp\n");
      }

      if (strcmp(gyro_reading, "") && strcmp(timestamp, "")) {

        //call function which returns bool with the value
        int signal = detect_step_or_jump(atof(gyro_reading));
        int64_t ttgo_timestamp = strtoll(timestamp, NULL, 10);
        int64_t cloud_timestamp = xx_time_get_time();

        //printf("ttgo: %ld cloud:  %ld\n", ttgo_timestamp, cloud_timestamp);
        cloud_timestamp -= ttgo_timestamp;
        if (signal != 0) {
          mg_http_reply(c, 200, "Content-Type: application/json\r\n", "{\"signal\": %d, \"response time (ms)\": %ld}", signal, cloud_timestamp);
        } else {
          mg_http_reply(c, 200, "Content-Type: application/json\r\n", "{\"recieved\": %s, \"response time (ms)\": %ld}", gyro_reading, cloud_timestamp);
        }
      }

    } else {
      struct mg_http_serve_opts opts = {.root_dir = "."};  // For all other URLs,
      mg_http_serve_dir(c, hm, &opts);                     // Serve static files
    }
  }
}



int main() {
  struct mg_mgr mgr;  // Mongoose event manager. Holds all connections
  mg_mgr_init(&mgr);  // Initialise event manager
  mg_http_listen(&mgr, "http://0.0.0.0:5000", fn, NULL);  // Setup listener
  for (;;) {
    mg_mgr_poll(&mgr, 1000);  // Infinite event loop
  }
  return 0;
}
