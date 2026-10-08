#pragma once
#define absValue(x) ((x) < 0.0 ? -(x) : (x))
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : (x) > (hi) ? (hi) : (x))

// Wraps an angle (degrees) to the range [-180, 180]
inline double wrapAngle180(int a)
{
    while (a > 180)  a -= 360;
    while (a < -180) a += 360;
    return a;
}

// Control type configuration for PID (Standard or Angle)
enum class ControlType : char {
    Standard = '0' ,
    Angle = '1'
};

// ---------------------------------------------------------------------------
// PID controller
// ---------------------------------------------------------------------------

class PID {
public:
    PID(float kp, float ki, float kd, ControlType type = ControlType::Standard,
        float outMin = -1.0, float outMax = 1.0)
        : kp_(kp), ki_(ki), kd_(kd), controlType_(type),
          outMin_(outMin), outMax_(outMax) {}


    void setKs(float kp, float ki, float kd) { kp_ = kp; ki_ = ki; kd_ = kd; }


    void setOutputLimits(float lo, float hi)
    {
        outMin_ = lo; outMax_ = hi;
    }


    void setSetpoint(float sp) { setpoint_ = sp; }


    float getSetpoint() const  { return setpoint_; }


    // Clears the memory of the controller (call when taking over from manual)
    void reset(float measured)
    {
        iTerm_        = 0.0;     
        lastOutput_   = 0.0;
        lastMeasured_ = measured;
        initialized_ = true;
    }

    // Call once per control cycle. dt in seconds. Returns P + I + D (clamped).
    float compute(float measured, float dt)
    {
        if (dt <= 0.0) return lastOutput_;
        if (!initialized_) reset(measured);

        float error = diff(setpoint_, measured);

        float P = kp_ * error;


        iTerm_ += ki_ * error * dt;
       // iTerm_ = constrain(iTerm_, outMin_, outMax_);

        // Derivative on measurement (note the minus sign)

        float dMeasured = diff(measured, lastMeasured_) / dt;
        float D = -kd_ * dMeasured;
        lastMeasured_ = measured;

        lastOutput_ = constrain( ( P + iTerm_ + D ) , outMin_, outMax_);
        return lastOutput_;
    }

private:
    // a - b, wrapped if the control type is Angle
    float diff(float a, float b) const
    {
        return (controlType_ == ControlType::Angle) ? wrapAngle180(a - b) : (a - b);
    }

    float kp_, ki_, kd_;
    ControlType controlType_;
    float outMin_, outMax_;

    float setpoint_     = 0.0;
    float iTerm_        = 0.0;
    float lastMeasured_ = 0.0;
    float lastOutput_   = 0.0;
    bool   initialized_  = false;
};


// ---------------------------------------------------------------------------
// AutoPID
// ---------------------------------------------------------------------------

class AutoHolder {
public:
    AutoHolder(PID &pid, float settleTime = 0.3, float speedThreshold = 0.02)
        : pid_(pid), settleTime_(settleTime), speedThreshold_(speedThreshold) {}

    float update(float measured, float currentSpeed, float dt)
    {
    
        if (absValue(currentSpeed) > speedThreshold_) {
            holding_  = false;
            idleTime_ = 0.0;
            return 0.0; 
        }

        
        idleTime_ += dt;
        if (idleTime_ < settleTime_) {
            return 0.0; 
        }

        if (!holding_) {                        
            pid_.setSetpoint(measured);
            pid_.reset(measured);
            holding_ = true;
        }
        
        return pid_.compute(measured, dt);
    }

    bool isHolding() const { return holding_; }
    
    void release() { holding_ = false; idleTime_ = 0.0; }

private:
    PID   &pid_;
    float settleTime_;
    float speedThreshold_; 
    float idleTime_ = 0.0;
    bool  holding_  = false;
};

