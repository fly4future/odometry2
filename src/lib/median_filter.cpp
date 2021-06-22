/* author: Daniel Hert */

/* from https://github.com/ctu-mrs/mrs_lib */

#include <lib/median_filter.h>
#include <rclcpp/rclcpp.hpp>

/* constructor //{ */

MedianFilter::MedianFilter(int buffer_size, double max_valid_value, double min_valid_value, double max_difference, rclcpp::NodeOptions options) : Node("median_filter", options) {

  is_filled = false;

  // maximum value that is considered valid
  this->max_valid_value = max_valid_value;

  // minimum value that is considered valid
  this->min_valid_value = min_valid_value;

  // if new value is more different from the filter median than this value, it is classified as invalid
  this->max_difference = max_difference;

  this->buffer_size = buffer_size;

  buffer.resize(buffer_size);
  next     = 0;
  m_median = 0;
  RCLCPP_INFO(this->get_logger(), "[MedianFilter]: initialized, buffer size: %d", buffer_size);
}

//}

/* isValid() //{ */

bool MedianFilter::isValid(double input) {

  double value = input;

  // add new value to the filter buffer
  buffer[next] = value;
  next++;

  if (next == buffer_size) {
    next      = 0;
    is_filled = true;
  }

  // check if new value is valid
  if (input > max_valid_value || input < min_valid_value) {
    return false;
  }

  int    current_value  = 0;
  int    best_med_value = std::numeric_limits<int>::max();
  int    zero_counter   = 0;
  double median         = 0;
  bool   valid          = true;

  // calculate the median from the filter buffer
  for (int i = 0; i < buffer_size; i++) {
    if (buffer[i] == 0) {
      zero_counter++;
    }
  }

  // if there is more zeroes in the buffer than half of the buffer size, the median value is automatically zero
  if (zero_counter < buffer_size / 2) {

    for (int i = 0; i < buffer_size; i++) {

      current_value = 0;

      if (buffer[i] == 0) {
        continue;
      }

      for (int j = 0; j < buffer_size; j++) {

        if (i == j) {
          continue;
        }

        if (buffer[i] > buffer[j]) {
          current_value++;
        } else if (buffer[i] < buffer[j]) {
          current_value--;
        }
      }

      if (abs(current_value) < best_med_value) {
        best_med_value = abs(current_value);
        median         = buffer[i];
      }
    }
  } else {
    median = 0;
  }

  {
    std::scoped_lock lock(mutex_median);

    m_median = median;
  }

  // if a new value is not close to the median, it is discarded
  if (fabs(value - median) > max_difference) {
    valid = false;
  } else {
    valid = true;
  }

  // check whether the output is finite
  if (std::isfinite(value)) {
    return valid;
  } else {

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000.0, "[MedianFilter]: received value is not a finite number!");
    return false;
  }
}

//}

/* isFilled() //{ */

bool MedianFilter::isFilled() {
  return is_filled;
}

//}

/* getMedian() //{ */


double MedianFilter::getMedian() {

  double median;

  {
    std::scoped_lock lock(mutex_median);

    median = m_median;
  }
  return median;
}


//}
