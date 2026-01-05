/**
 * @file filter.cpp
 * @brief Filter Library Implementation
 */

#include "filter.hpp"
#include <cmath>

namespace stampfly {

void LowPassFilter::init(float sample_freq, float cutoff_freq)
{
    if (cutoff_freq <= 0 || sample_freq <= 0) {
        alpha_ = 1.0f;
        return;
    }
    float rc = 1.0f / (2.0f * M_PI * cutoff_freq);
    float dt = 1.0f / sample_freq;
    alpha_ = dt / (rc + dt);
    initialized_ = true;
}

float LowPassFilter::apply(float input)
{
    if (!initialized_) {
        output_ = input;
        initialized_ = true;
        return output_;
    }
    output_ = output_ + alpha_ * (input - output_);
    return output_;
}

void LowPassFilter::reset()
{
    output_ = 0.0f;
    initialized_ = false;
}

}  // namespace stampfly
