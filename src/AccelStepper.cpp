// AccelStepper.cpp
//
// Copyright (C) 2009-2013 Mike McCauley
// $Id: AccelStepper.cpp,v 1.23 2016/08/09 00:39:10 mikem Exp $

#include "AccelStepper.h"

#if 0
// Some debugging assistance
void dump(uint8_t* p, int l)
{
    int i;

    for (i = 0; i < l; i++)
    {
	Serial.print(p[i], HEX);
	Serial.print(" ");
    }
    Serial.println("");
}
#endif

//amazing Quake invented algorithm
float sqrt2(const float x)
{
  const float xhalf = 0.5f*x;
  union // get bits for floating value
  {
    float x;
    int i;
  } u;
  u.x = x;
  u.i = SQRT_MAGIC_F - (u.i >> 1);  // gives initial guess y0
  return x*u.x*(1.5f - xhalf*u.x*u.x);// Newton step, repeating increases accuracy 
}

void AccelStepper::moveTo(long absolute)
{
    _curMode = Position;
    if (_targetPos != absolute)
    {
        _targetPos = absolute;
        computeNewSpeed();
    }
}

void AccelStepper::move(long relative)
{
    moveTo(_currentPos + relative);
}

// Implements steps according to the current step interval
// You must call this at least once per step
// returns true if a step occurred
boolean AccelStepper::runSpeed()
{
    // Dont do anything unless we actually have a step interval
    if (_stepInterval == 0)
	    return false;

    unsigned long time = micros();   
    if (time - _lastStepTime >= _stepInterval)
    {
        if (isDirForward())
        {
            // Clockwise
            if(_currentPos <  MAX_INT32_T)
                _currentPos += 1;
        }
        else
        {
            // Anticlockwise 
            if(_currentPos > -MAX_INT32_T) 
                _currentPos -= 1;
        }
        step(_currentPos);

        _lastStepTime = time; // Caution: does not account for costs in step()

	    return true;
    }
    else
    {
	    return false;
    }
}

int32_t AccelStepper::distanceToGo()
{
    if(_curMode == Position)
        return _targetPos - _currentPos;
    else //Infinite
    {
        int32_t dist = 0;
        bool increasing = isDirForward();
        if (!isRunning()) {
            increasing = _targetDir;
        }
        if (increasing) {
            dist = (maxPositionLimit - _currentPos);
        } else {
            dist = (_currentPos - minPositionLimit);
        }
        return dist;
    }
}

int32_t AccelStepper::targetPosition()
{
    return _targetPos;
}

int32_t AccelStepper::currentPosition()
{
    return _currentPos;
}

inline boolean AccelStepper::isDirForward() {
  return _direction == defaultForwardDir;
}

void AccelStepper::hardStop() {
  _stepInterval = 0; // stop;
  _speed = 0.0;
  _a_speed = 0.0;
  _targetSpeed = 0.0;
  _a_targetSpeed = 0.0;
  _final_cn = LARGE_N; // very big
  setDir(true); // sets forward
  _n = 0;
  _cn = _c0;
}

// Useful during initialisations or after initial positioning
// Sets speed to 0
void AccelStepper::setCurrentPosition(int32_t position)
{
    _targetPos = _currentPos = position;
    _n = 0;
    _stepInterval = 0;
    _speed = 0.0;
}

void AccelStepper::setInfModeMaxPosLimit(int32_t mPos) {
    if (mPos > MAX_INT32_T) {
        mPos = MAX_INT32_T;
    }
    if (mPos < _currentPos) {
        mPos = _currentPos;
    }
    if (mPos < 0) {
        mPos = 0;
    }
    maxPositionLimit = mPos;
}

int32_t AccelStepper::getInfModeMaxPosLimit() {
    return maxPositionLimit;
}

void AccelStepper::setInfModeMinPosLimit(int32_t mPos) {
    if (mPos < -MAX_INT32_T) {
        mPos = -MAX_INT32_T;
    }
    if (mPos > _currentPos) {
        mPos = _currentPos;
    }
    if (mPos > 0) {
        mPos = 0;
    }
    minPositionLimit = mPos;
}

int32_t AccelStepper::getInfModeMinPosLimit() {
    return minPositionLimit;
}

void AccelStepper::setTargetSpeed(float speed) {
    if(speed == _targetSpeed) return;
    _curMode = Infinite;
    internalSetSpeed(speed); //call computeNewSpeed at last
}

float AccelStepper::getTargetSpeed() {
    return _targetSpeed;
}

//sp is targetSpeed
//call computeNewSpeed at last
void AccelStepper::internalSetSpeed(float sp) {
  if(_curMode == Infinite) {
        //sp -> target speed -> new final_cn(final target cn) -> computeNewSpeed
        float a_sp = fabs(sp);
        if (a_sp > _maxSpeed) {
            a_sp = _maxSpeed;
        }

        // limits targetSpeed and sets final_cn === targetSpeed
        _targetSpeed = (sp >= 0) ? a_sp : -a_sp;
        _a_targetSpeed = a_sp;

        if (_a_targetSpeed < _minSpeed) {
            _a_targetSpeed = 0.0;
            _final_cn = LARGE_N; // very big
        } else {
            _final_cn = LARGE_CN_DIV / _a_targetSpeed;
        }
        _targetDir = _targetSpeed >= 0;  // the final direction we want to go in

        int32_t stepsToStop = ((int32_t)((_speed * _speed) / (2.0 * _acceleration))); // Equation 16
        
        // can be zero, will be zero if stopped
        _n = stepsToStop; // new n for acceleration

        // find the step sign to change speed
        if (_targetDir == isDirForward()) { // same direction
            setDir(_targetDir); // always
            if (_a_targetSpeed > _a_speed) {
                // n +ve
            } 
            else {
                _n = -_n; // decelerate
            }
        } 
        else {
            // change in direction
            // need to decelerate to 0 first
            _n = -_n;
            if (_n == 0) { // stopped or just about to
                // change dir to target direction
                setDir(_targetDir);
            } 
            else {
                setDir(isDirForward()); // this should be the current setting
            }
        }
        
  }
  
  computeNewSpeed();
}

void AccelStepper::computeNewSpeed()
{
    int32_t distanceTo = distanceToGo(); // +ve is clockwise from curent location
    int32_t stepsToStop = (int32_t)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
    
    if(_curMode == Infinite) {
        //distanceTo will always be positive in inf mode
        if (distanceTo == 0) {
            // hit the limit
            hardStop();
            return;
        }
        
        bool approachingLimit = false;
        // else moving in other direction
        if (stepsToStop >= distanceTo) {
            // need to slow down
            _n = -distanceTo; // may be zero
            approachingLimit = true;
        }

        if(_n == 0) {
            if(_a_targetSpeed < _minSpeed) {
                hardStop();
                return;
            }

            // else do first step
            _cn = _c0;
            setDir(_targetDir); // was at n==0 so change dir now
            if (_cn > _final_cn) { // i.e. first step slower then final
                // first step of acceleration
                // increment for next call n goes from 0 to 1
                // next call will reduce cn and increase speed towards final
                _n++; 
            } 
            else { // (final_cn > cn)
                // go straight to target speed as less the first acceleration step
                _cn = _final_cn; //maintain at target speed
                _n = LARGE_N; // this suppresses further changes
            }

            // set stepInterval and speed
            uint32_t laststepInterval = _stepInterval;
            _stepInterval = _cn;
            _a_speed = LARGE_CN_DIV / _cn;
            if (isDirForward()) {
                _speed = _a_speed;
            } else {
                _speed = -_a_speed;
            }

            if (laststepInterval == 0) {
                // was stopped so do one step NOW
                runSpeed();
            }
            return;
        }
        else {
            //_n != 0
            if (_n == LARGE_N) {
                // have reached final speed just keep going
                return;
            }
            
            // else need to adjust speed
            float deltaCn = - ((2.0 * _cn) / ((4.0 * _n) + 1));
            float final_deltaCn = _final_cn - _cn;
            float a_deltaCn = (deltaCn >= 0) ? deltaCn : -deltaCn;
            float a_final_deltaCn = (final_deltaCn >= 0) ? final_deltaCn : -final_deltaCn;
            
            if ((!approachingLimit) && (isDirForward() == _targetDir) && (a_final_deltaCn < a_deltaCn)) {
                _cn = _final_cn;
                _n = LARGE_N;
            } 
            else {
                _cn = _cn + deltaCn; // Equation 13
                if (_n < MAX_INT32_T) {
                    _n++; // for next loop
                }
            }
            _stepInterval = _cn;
            _a_speed = LARGE_CN_DIV / _cn;

            if (isDirForward()) {
                _speed = _a_speed;
            } 
            else {
                _speed = -_a_speed;
            }
        }
    }
    else { //position
        if (distanceTo == 0 && stepsToStop <= 1)
        {
            // We are at the target and its time to stop
            hardStop();
            return;
        }

        if (distanceTo > 0)
        {
            // We are anticlockwise from the target
            // Need to go clockwise from here, maybe decelerate now
            if (_n > 0)
            {
                // Currently accelerating, need to decel now? Or maybe going the wrong way?
                if ((stepsToStop >= distanceTo) || !isDirForward())
                    _n = -stepsToStop; // Start deceleration
            }
            else if (_n < 0)
            {
                // Currently decelerating, need to accel again?
                if ((stepsToStop < distanceTo) && isDirForward())
                    _n = -_n; // Start accceleration
            }
        }
        else if (distanceTo < 0)
        {
            // We are clockwise from the target
            // Need to go anticlockwise from here, maybe decelerate
            if (_n > 0)
            {
                // Currently accelerating, need to decel now? Or maybe going the wrong way?
                if ((stepsToStop >= -distanceTo) || isDirForward())
                    _n = -stepsToStop; // Start deceleration
            }
            else if (_n < 0)
            {
                // Currently decelerating, need to accel again?
                if ((stepsToStop < -distanceTo) && !isDirForward())
                    _n = -_n; // Start accceleration
            }
        }

        // Need to accelerate or decelerate
        if (_n == 0)
        {
            // First step from stopped
            _cn = _c0;
            setDir(distanceTo > 0);
        }
        else
        {
            // Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
            _cn = _cn - ((2.0 * _cn) / ((4.0 * _n) + 1)); // Equation 13
            _cn = max(_cn, _cmin); 
        }
        _n++;
        _stepInterval = _cn;
        _speed = LARGE_CN_DIV / _cn;
        if (!isDirForward()) _speed = -_speed;
    }
    
#if 0
    Serial.println(_speed);
    Serial.println(_acceleration);
    Serial.println(_cn);
    Serial.println(_c0);
    Serial.println(_n);
    Serial.println(_stepInterval);
    Serial.println(distanceTo);
    Serial.println(stepsToStop);
    Serial.println("-----");
#endif
}

// Run the motor to implement speed and acceleration in order to proceed to the target position
// You must call this at least once per step, preferably in your main loop
// If the motor is in the desired position, the cost is very small
// returns true if the motor is still running to the target position.
boolean AccelStepper::run()
{
    if (runSpeed())
	    computeNewSpeed();
    return _speed != 0.0 || distanceToGo() != 0;
}

AccelStepper::AccelStepper(uint8_t interface, uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4, bool enable)
{
    _interface = interface;
    _currentPos = 0;
    _targetPos = 0;
    _speed = 0.0;
    _maxSpeed = 1.0;
    _acceleration = 0.0;
    _sqrt_twoa = 1.0;
    _stepInterval = 0;
    _minPulseWidth = 1;
    _enablePin = 0xff;
    _lastStepTime = 0;
    _pin[0] = pin1;
    _pin[1] = pin2;
    _pin[2] = pin3;
    _pin[3] = pin4;
    _enableInverted = false;
    // NEW
    _n = 0;
    _c0 = 0.0;
    _cn = 0.0;
    _cmin = 1.0;
    _direction = DIRECTION_CCW;

    maxPositionLimit = MAX_INT32_T;
    minPositionLimit = -MAX_INT32_T;
    hardStop();
    setMaxSpeed(maxMaxSpeed); // sets cmax,cmin
    setMinSpeed(minMaxSpeed);
    
    int i;
    for (i = 0; i < 4; i++)
	_pinInverted[i] = 0;
    if (enable)
	enableOutputs();
    // Some reasonable default
    setAcceleration(1);
}

AccelStepper::AccelStepper(void (*forward)(), void (*backward)())
{
    _interface = 0;
    _currentPos = 0;
    _targetPos = 0;
    _speed = 0.0;
    _maxSpeed = 1.0;
    _acceleration = 0.0;
    _sqrt_twoa = 1.0;
    _stepInterval = 0;
    _minPulseWidth = 1;
    _enablePin = 0xff;
    _lastStepTime = 0;
    _pin[0] = 0;
    _pin[1] = 0;
    _pin[2] = 0;
    _pin[3] = 0;
    _forward = forward;
    _backward = backward;

    // NEW
    _n = 0;
    _c0 = 0.0;
    _cn = 0.0;
    _cmin = 1.0;
    _direction = DIRECTION_CCW;

    maxPositionLimit = MAX_INT32_T;
    minPositionLimit = -MAX_INT32_T;
    hardStop();
    setMaxSpeed(maxMaxSpeed); // sets cmax,cmin
    setMinSpeed(minMaxSpeed);
    
    int i;
    for (i = 0; i < 4; i++)
	_pinInverted[i] = 0;
    // Some reasonable default
    setAcceleration(1);
}

void AccelStepper::setMaxSpeed(float speed)
{
    if (speed < 0.0) speed = -speed;
    if (speed < minMaxSpeed) speed = minMaxSpeed;
    if (speed > maxMaxSpeed) speed = maxMaxSpeed;
    _maxSpeed = speed;
    if(_minSpeed > _maxSpeed) {
        //swap
        float temp = _minSpeed;
        _minSpeed = _maxSpeed;
        _maxSpeed = temp;
    }

    _cmax = LARGE_CN_DIV / _minSpeed;
    _cmin = LARGE_CN_DIV / _maxSpeed;
}

void AccelStepper::setMinSpeed(float speed)
{
    if (speed < 0.0) speed = -speed;
    if (speed < minMaxSpeed) speed = minMaxSpeed;
    if (speed > maxMaxSpeed) speed = maxMaxSpeed;
    _minSpeed = speed;
    if(_minSpeed > _maxSpeed) {
        //swap
        float temp = _minSpeed;
        _minSpeed = _maxSpeed;
        _maxSpeed = temp;
    }

    _cmax = LARGE_CN_DIV / _minSpeed;
    _cmin = LARGE_CN_DIV / _maxSpeed;
}

float AccelStepper::minSpeed()
{
    return _minSpeed;
}

float AccelStepper::maxSpeed()
{
    return _maxSpeed;
}

void AccelStepper::setAcceleration(float newA)
{
    if (newA < 0.0)
        newA = -newA;
    if (newA <= 1e-4) {
        newA = 1e-4;
    }

    if (_acceleration != newA)
    {
        // Recompute _n per Equation 17
        _n = _n * (_acceleration / newA);
        // New c0 per Equation 7, with correction per Equation 15
        _c0 = 0.676 * sqrt2(2.0 / newA) * LARGE_CN_DIV; // Equation 15
        _acceleration = newA;
        
        // recalculate n using this new accel
        internalSetSpeed(_targetSpeed); //call computeNewSpeed at last
    }
}

void AccelStepper::setSpeed(float speed)
{
    if (speed == _speed)
        return;
    speed = constrain(speed, -_maxSpeed, _maxSpeed);
    float a_sp = fabs(speed);
    if (a_sp <= _minSpeed)
	    _stepInterval = 0;
    else
    {
        _stepInterval = fabs(LARGE_CN_DIV / speed);
        setDir(speed > 0.0); //speed > 0 is forward
    }
    _speed = speed;
}

float AccelStepper::speed()
{
    return _speed;
}

// Subclasses can override
void AccelStepper::step(long step)
{
    switch (_interface)
    {
    case FUNCTION:
        step0(step);
        break;

	case DRIVER:
	    step1(step);
	    break;
    
	case FULL2WIRE:
	    step2(step);
	    break;
    
	case FULL3WIRE:
	    step3(step);
	    break;  

	case FULL4WIRE:
	    step4(step);
	    break;  

	case HALF3WIRE:
	    step6(step);
	    break;  
		
	case HALF4WIRE:
	    step8(step);
	    break;  
    }
}

// You might want to override this to implement eg serial output
// bit 0 of the mask corresponds to _pin[0]
// bit 1 of the mask corresponds to _pin[1]
// ....
void AccelStepper::setOutputPins(uint8_t mask)
{
    uint8_t numpins = 2;
    if (_interface == FULL4WIRE || _interface == HALF4WIRE)
	numpins = 4;
    else if (_interface == FULL3WIRE || _interface == HALF3WIRE)
	numpins = 3;
    uint8_t i;
    for (i = 0; i < numpins; i++)
	digitalWrite(_pin[i], (mask & (1 << i)) ? (HIGH ^ _pinInverted[i]) : (LOW ^ _pinInverted[i]));
}

// 0 pin step function (ie for functional usage)
void AccelStepper::step0(long step)
{
    (void)(step); // Unused
    if (_speed > 0)
	_forward();
    else
	_backward();
}

// 1 pin step function (ie for stepper drivers)
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step1(long step)
{
    (void)(step); // Unused

    // _pin[0] is step, _pin[1] is direction
    setOutputPins(_direction ? 0b10 : 0b00); // Set direction first else get rogue pulses
    setOutputPins(_direction ? 0b11 : 0b01); // step HIGH
    // Caution 200ns setup time 
    // Delay the minimum allowed pulse width
    delayMicroseconds(_minPulseWidth);
    setOutputPins(_direction ? 0b10 : 0b00); // step LOW
}


// 2 pin step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step2(long step)
{
    switch (step & 0x3)
    {
	case 0: /* 01 */
	    setOutputPins(0b10);
	    break;

	case 1: /* 11 */
	    setOutputPins(0b11);
	    break;

	case 2: /* 10 */
	    setOutputPins(0b01);
	    break;

	case 3: /* 00 */
	    setOutputPins(0b00);
	    break;
    }
}
// 3 pin step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step3(long step)
{
    switch (step % 3)
    {
	case 0:    // 100
	    setOutputPins(0b100);
	    break;

	case 1:    // 001
	    setOutputPins(0b001);
	    break;

	case 2:    //010
	    setOutputPins(0b010);
	    break;
	    
    }
}

// 4 pin step function for half stepper
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step4(long step)
{
    switch (step & 0x3)
    {
	case 0:    // 1010
	    setOutputPins(0b0101);
	    break;

	case 1:    // 0110
	    setOutputPins(0b0110);
	    break;

	case 2:    //0101
	    setOutputPins(0b1010);
	    break;

	case 3:    //1001
	    setOutputPins(0b1001);
	    break;
    }
}

// 3 pin half step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step6(long step)
{
    switch (step % 6)
    {
	case 0:    // 100
	    setOutputPins(0b100);
            break;
	    
        case 1:    // 101
	    setOutputPins(0b101);
            break;
	    
	case 2:    // 001
	    setOutputPins(0b001);
            break;
	    
        case 3:    // 011
	    setOutputPins(0b011);
            break;
	    
	case 4:    // 010
	    setOutputPins(0b010);
            break;
	    
	case 5:    // 011
	    setOutputPins(0b110);
            break;
	    
    }
}

// 4 pin half step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step8(long step)
{
    switch (step & 0x7)
    {
	case 0:    // 1000
	    setOutputPins(0b0001);
            break;
	    
        case 1:    // 1010
	    setOutputPins(0b0101);
            break;
	    
	case 2:    // 0010
	    setOutputPins(0b0100);
            break;
	    
        case 3:    // 0110
	    setOutputPins(0b0110);
            break;
	    
	case 4:    // 0100
	    setOutputPins(0b0010);
            break;
	    
        case 5:    //0101
	    setOutputPins(0b1010);
            break;
	    
	case 6:    // 0001
	    setOutputPins(0b1000);
            break;
	    
        case 7:    //1001
	    setOutputPins(0b1001);
            break;
    }
}
    
// Prevents power consumption on the outputs
void AccelStepper::disableOutputs()
{   
    if (! _interface) return;

    setOutputPins(0); // Handles inversion automatically
    if (_enablePin != 0xff)
    {
        pinMode(_enablePin, OUTPUT);
        digitalWrite(_enablePin, LOW ^ _enableInverted);
    }
}

void AccelStepper::enableOutputs()
{
    if (! _interface) 
	return;

    pinMode(_pin[0], OUTPUT);
    pinMode(_pin[1], OUTPUT);
    if (_interface == FULL4WIRE || _interface == HALF4WIRE)
    {
        pinMode(_pin[2], OUTPUT);
        pinMode(_pin[3], OUTPUT);
    }
    else if (_interface == FULL3WIRE || _interface == HALF3WIRE)
    {
        pinMode(_pin[2], OUTPUT);
    }

    if (_enablePin != 0xff)
    {
        pinMode(_enablePin, OUTPUT);
        digitalWrite(_enablePin, HIGH ^ _enableInverted);
    }
}

void AccelStepper::setMinPulseWidth(uint16_t minWidth)
{
    _minPulseWidth = minWidth;
}

void AccelStepper::setEnablePin(uint8_t enablePin)
{
    _enablePin = enablePin;

    // This happens after construction, so init pin now.
    if (_enablePin != 0xff)
    {
        pinMode(_enablePin, OUTPUT);
        digitalWrite(_enablePin, HIGH ^ _enableInverted);
    }
}

void AccelStepper::setPinsInverted(bool directionInvert, bool stepInvert, bool enableInvert)
{
    _pinInverted[0] = stepInvert;
    _pinInverted[1] = directionInvert;
    _enableInverted = enableInvert;
}

void AccelStepper::setPinsInverted(bool pin1Invert, bool pin2Invert, bool pin3Invert, bool pin4Invert, bool enableInvert)
{    
    _pinInverted[0] = pin1Invert;
    _pinInverted[1] = pin2Invert;
    _pinInverted[2] = pin3Invert;
    _pinInverted[3] = pin4Invert;
    _enableInverted = enableInvert;
}

/**
   setDir(bool)
   Default is HIGH for forward
*/
inline void AccelStepper::setDir(boolean flag) {
  _direction = flag;
}

// Blocks until the target position is reached and stopped
void AccelStepper::runToPosition()
{
    while (run())
	;
}

boolean AccelStepper::runSpeedToPosition()
{
    if (_targetPos == _currentPos)
	    return false;
    if (_targetPos >_currentPos)
        setDir(true);
    else
        setDir(false);
    return runSpeed();
}

// Blocks until the new target position is reached
void AccelStepper::runToNewPosition(long position)
{
    moveTo(position);
    runToPosition();
}

//slow down
void AccelStepper::stop()
{
    if (_curMode == Infinite) {
        setTargetSpeed(0);
    }
    else {
        if (_speed != 0.0)
        {    
            int32_t stepsToStop = (int32_t)((_speed * _speed) / (2.0 * _acceleration)) + 1; // Equation 16 (+integer rounding)
            if (_speed > 0)
                move(stepsToStop);
            else
                move(-stepsToStop);
        }
    }
}

bool AccelStepper::isRunning()
{
    return !(_speed == 0.0 && _targetPos == _currentPos);
}
