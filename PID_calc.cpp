```cpp
#pragma once

#include <Arduino.h>
#include <cmath>
#include <cstddef>
#define avg_counter 3
// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------

// Clamp helper (avoids conflict with Arduino's constrain macro)
template <typename T>
inline T clampValue(T x, T lo, T hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

// Wrap angle to [-180, 180)
inline float wrapAngle180(float angle)
{
    angle = fmodf(angle + 180.0f, 360.0f);

    if (angle < 0.0f)
        angle += 360.0f;

    return angle - 180.0f;
}

// ---------------------------------------------------------------------------
// Mean helper functions
// ---------------------------------------------------------------------------

// Add one sample to an arithmetic running mean.
// Returns the updated sample count.
inline size_t addArithmeticSample(
    float value,
    float &sum,
    size_t &count,
    float &average)
{
    if (count >= avg_counter)
        return 0;

    sum += value;
    ++count;

    average = sum / static_cast<float>(count);

    return count;
}

// Add one sample to a circular running mean.
// Returns the updated sample count.
inline size_t addCircularSample(
    float angle,
    float &sinSum,
    float &cosSum,
    size_t &count,
    float &average)
{
    if (count >= avg_counter)
        return 0;

    const float radians = angle * PI / 180.0f;

    sinSum += sinf(radians);
    cosSum += cosf(radians);

    ++count;


    average = atan2f(sinSum, cosSum) * 180.0f / PI;
    average = wrapAngle180(average);
    

    return count;
}

// ---------------------------------------------------------------------------
// Control type
// ---------------------------------------------------------------------------

enum class ControlType : char {
    Standard = '0',
    Angle    = '1'
};

// ---------------------------------------------------------------------------
// PID controller
// ---------------------------------------------------------------------------

class PID {
public:
    PID(float kp, float ki, float kd,
        ControlType type = ControlType::Standard,
        float outMin = -1.0f,
        float outMax = 1.0f)
        : kp_(kp),
          ki_(ki),
          kd_(kd),
          controlType_(type),
          outMin_(outMin),
          outMax_(outMax) {}

    void setKs(float kp, float ki, float kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void setOutputLimits(float lo, float hi)
    {
        outMin_ = lo;
        outMax_ = hi;
    }

    void setSetpoint(float sp)
    {
        setpoint_ = sp;
    }

    float getSetpoint() const
    {
        return setpoint_;
    }

    ControlType getControlType() const
    {
        return controlType_;
    }

    // Reset controller memory.
    void reset(float measured)
    {
        iTerm_ = 0.0f;
        lastOutput_ = 0.0f;
        lastMeasured_ = measured;
        initialized_ = true;
    }

    // Call once per control cycle. dt is in seconds.
    float compute(float measured, float dt)
    {
        if (dt <= 0.0f)
            return lastOutput_;

        if (!initialized_)
            reset(measured);

        const float error = diff(setpoint_, measured);
        const float P = kp_ * error;

        // Integral with output-range clamping.
        iTerm_ += ki_ * error * dt;
        iTerm_ = clampValue(iTerm_, outMin_, outMax_);

        // Derivative on measurement.
        const float D =
            -kd_ * diff(measured, lastMeasured_) / dt;

        lastMeasured_ = measured;

        lastOutput_ =
            clampValue(P + iTerm_ + D, outMin_, outMax_);

        return lastOutput_;
    }

private:
    float diff(float a, float b) const
    {
        return (controlType_ == ControlType::Angle)
            ? wrapAngle180(a - b)
            : (a - b);
    }

    float kp_, ki_, kd_;
    ControlType controlType_;

    float outMin_, outMax_;

    float setpoint_ = 0.0f;
    float iTerm_ = 0.0f;
    float lastMeasured_ = 0.0f;
    float lastOutput_ = 0.0f;

    bool initialized_ = false;
};

// ---------------------------------------------------------------------------
// AutoHolder
// ---------------------------------------------------------------------------

class AutoHolder {
public:
    AutoHolder(PID &pid, float commandThreshold = 0.02f)
        : pid_(pid),
          commandThreshold_(commandThreshold) {}

    float update(float measured,
                 float commandedVelocity,
                 float dt)
    {
        // A movement command releases AutoHolder.
        if (fabsf(commandedVelocity) > commandThreshold_) {
            release();
            return 0.0f;
        }

        const unsigned long now = millis();

        // Begin collecting when the command becomes zero.
        if (!collecting_) {
            resetAccumulators();

            collecting_ = true;
            holding_ = false;
            lastSampleTime_ = now;

            // First sample is taken immediately at t = 0.
            float average = measured;
            addSample(measured, average);

            pid_.setSetpoint(average);
            pid_.reset(measured);

            return pid_.compute(measured, dt);
        }

        // Take the next sample after each 1-second interval.
        if (!holding_ &&
            static_cast<unsigned long>(now - lastSampleTime_) >= 1000UL) {

            float average = measured;
            const size_t count = addSample(measured, average);

            if (count > 0) {
                lastSampleTime_ = now;

                // Update PID target using the current mean.
                pid_.setSetpoint(average);


                if (count == avg_counter ) {
                    // The final mean has been applied.
                    holding_ = true;

                    // Prepare for the next hold cycle.
                    resetAccumulators();
                }
            }
        }

        // Keep the PID running on every control cycle.
        return pid_.compute(measured, dt);
    }

    bool isHolding() const
    {
        return holding_;
    }

    void release()
    {
        collecting_ = false;
        holding_ = false;
        lastSampleTime_ = 0;

        resetAccumulators();
    }

private:
    size_t addSample(float measured, float &average)
    {
        if (pid_.getControlType() == ControlType::Angle) {
            return addCircularSample(
                measured,
                sinSum_,
                cosSum_,
                firstAngle_,
                sampleCount_,
                average);
        }

        return addArithmeticSample(
            measured,
            sum_,
            sampleCount_,
            average);
    }

    void resetAccumulators()
    {
        sum_ = 0.0f;

        sinSum_ = 0.0f;
        cosSum_ = 0.0f;

        sampleCount_ = 0;
    }

    PID &pid_;
    float commandThreshold_;

    // Arithmetic mean state.
    float sum_ = 0.0f;

    // Circular mean state.
    float sinSum_ = 0.0f;
    float cosSum_ = 0.0f;

    // Shared sample count; only one mean type is used per PID.
    size_t sampleCount_ = 0;

    unsigned long lastSampleTime_ = 0;

    bool collecting_ = false;
    bool holding_ = false;
};
```
