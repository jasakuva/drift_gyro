#pragma once
#include <stddef.h>

template <size_t N>
class myMovingAverage
{
public:
    myMovingAverage() : sum(0.0f), index(0), count(0)
    {
        for (size_t i = 0; i < N; ++i)
            buffer[i] = 0.0f;
    }

    float update(float value)
    {
        // Remove oldest value from sum
        sum -= buffer[index];

        // Insert new value
        buffer[index] = value;
        sum += value;

        // Advance circular index
        index = (index + 1) % N;

        // Track how many samples we've filled
        if (count < N)
            count++;

        return get();
    }

    float get() const
    {
        if (count == 0)
            return 0.0f;

        return sum / count;
    }

    void reset()
    {
        sum = 0.0f;
        index = 0;
        count = 0;
        for (size_t i = 0; i < N; ++i)
            buffer[i] = 0.0f;
    }

private:
    float buffer[N];
    float sum;
    size_t index;
    size_t count;
};