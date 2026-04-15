#ifndef ROLLING_PEAK_H
#define ROLLING_PEAK_H

#include <Arduino.h>

class RollingPeak
{
public:
    static const int MAX_WINDOW = 40;

    explicit RollingPeak(int windowSize);

    void add(float value);
    float getPeak() const;
    float addGetPeak(float value);
    bool isFull() const;
    int count() const;
    void clear();

private:
    float _buffer[MAX_WINDOW];
    int _windowSize;
    int _count;
    int _head;
};

#endif
