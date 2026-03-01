#ifndef MEDIANFILTER_H
#define MEDIANFILTER_H

#include <Arduino.h>

class MedianFilter {
  private:
    static const int MAX_SIZE = 15;

    float buffer[MAX_SIZE];
    float temp[MAX_SIZE];

    int windowSize;
    int index;
    int count;

    void sortArray(float* arr, int n) const {
      // Insertion sort (fast for small arrays)
      for (int i = 1; i < n; i++) {
        float key = arr[i];
        int j = i - 1;

        while (j >= 0 && arr[j] > key) {
          arr[j + 1] = arr[j];
          j--;
        }
        arr[j + 1] = key;
      }
    }

  public:
    // Constructor with default size
    MedianFilter(int size = 5) {
      setWindowSize(size);
    }

    void setWindowSize(int size) {
      if (size < 5) size = 5;
      if (size > MAX_SIZE) size = MAX_SIZE;

      windowSize = size;
      index = 0;
      count = 0;

      for (int i = 0; i < MAX_SIZE; i++) {
        buffer[i] = 0;
      }
    }

    void addSample(float value) {
      buffer[index] = value;
      index = (index + 1) % windowSize;

      if (count < windowSize) {
        count++;
      }
    }

    float getMedian() {
      if (count == 0) return 0;

      // Copy valid samples
      for (int i = 0; i < count; i++) {
        temp[i] = buffer[i];
      }

      sortArray(temp, count);

      if (count % 2 == 1) {
        return temp[count / 2];
      } else {
        return (temp[count / 2 - 1] + temp[count / 2]) / 2.0;
      }
    }

    void reset() {
      index = 0;
      count = 0;
      for (int i = 0; i < windowSize; i++) {
        buffer[i] = 0;
      }
    }

    int getWindowSize() const {
      return windowSize;
    }

    int getCount() const {
      return count;
    }
};

#endif