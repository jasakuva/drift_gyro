#include "RollingPeak.h"
#include <math.h>

RollingPeak::RollingPeak(int windowSize)
{
    if (windowSize < 1)
    {
        windowSize = 1;
    }
    if (windowSize > MAX_WINDOW)
    {
        windowSize = MAX_WINDOW;
    }

    _windowSize = windowSize;
    _count = 0;
    _head = 0;

    for (int i = 0; i < MAX_WINDOW; i++)
    {
        _buffer[i] = 0.0f;
    }
}

void RollingPeak::add(float value)
{
    _buffer[_head] = value;
    _head++;

    if (_head >= _windowSize)
    {
        _head = 0;
    }

    if (_count < _windowSize)
    {
        _count++;
    }
}

float RollingPeak::getPeak() const
{
    if (_count == 0)
    {
        return 0.0f;
    }

    float peak = _buffer[0];

    for (int i = 1; i < _count; i++)
    {
        if (fabs(_buffer[i]) > fabs(peak))
        {
            peak = _buffer[i];
        }
    }

    return peak;
}

float RollingPeak::addGetPeak(float value){
  add(value);
  return getPeak();
}

bool RollingPeak::isFull() const
{
    return (_count == _windowSize);
}

int RollingPeak::count() const
{
    return _count;
}

void RollingPeak::clear()
{
    _count = 0;
    _head = 0;

    for (int i = 0; i < MAX_WINDOW; i++)
    {
        _buffer[i] = 0.0f;
    }
}
