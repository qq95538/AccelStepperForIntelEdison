// AccelStepper.cpp
//
// Copyright (C) 2009 Mike McCauley
// $Id: AccelStepper.cpp,v 1.2 2010/10/24 07:46:18 mikem Exp mikem $

//AccelStepper for Intel Edison

#include <stdint.h>
#include <math.h>
#include <time.h>
#include "AccelStepper.hpp"


void AccelStepper::moveTo(long absolute)
{
    _targetPos = absolute;
    computeNewSpeed();
}

void AccelStepper::move(long relative)
{
    moveTo(_currentPos + relative);
}

AccelStepper::~AccelStepper()
{
}

// Implements steps according to the current speed
// You must call this at least once per step
// returns true if a step occurred
bool AccelStepper::runSpeed()
{
    //unsigned long time = millis();
	unsigned long time;
	time = clock() / (CLOCKS_PER_SEC / 1000);
    //qq95538 change get millis() time to get CPU clock ticks.
    if (time > _lastStepTime + _stepInterval)
    {
    	if (_speed > 0)
    	{
    		// Clockwise
    		_currentPos += 1;
    	}
    	else if (_speed < 0)
    	{
    		// Anticlockwise
    		_currentPos -= 1;
    	}
    	step(_currentPos & 0x3); // Bottom 2 bits (same as mod 4, but works with + and - numbers)
    	_lastStepTime = time;
    	return true;
    }
    else
    	return false;
}

long AccelStepper::distanceToGo()
{
    return _targetPos - _currentPos;
}

long AccelStepper::targetPosition()
{
    return _targetPos;
}

long AccelStepper::currentPosition()
{
    return _currentPos;
}

// Useful during initialisations or after initial positioning
void AccelStepper::setCurrentPosition(long position)
{
    _currentPos = position;
}

void AccelStepper::computeNewSpeed()
{
    setSpeed(desiredSpeed());
}

// Work out and return a new speed.
// Subclasses can override if they want
// Implement acceleration, deceleration and max speed
// Negative speed is anticlockwise
// This is called:
//  after each step
//  after user changes:
//   maxSpeed
//   acceleration
//   target position (relative or absolute)
float AccelStepper::desiredSpeed()
{
    long distanceTo = distanceToGo();

    // Max possible speed that can still decelerate in the available distance
    float requiredSpeed;
    if (distanceTo == 0)
	return 0.0; // Were there
    else if (distanceTo > 0) // Clockwise
	requiredSpeed = sqrt(2.0 * distanceTo * _acceleration);
    else  // Anticlockwise
	requiredSpeed = -sqrt(2.0 * -distanceTo * _acceleration);

    if (requiredSpeed > _speed)
    {
	// Need to accelerate in clockwise direction
	if (_speed == 0)
	    requiredSpeed = sqrt(2.0 * _acceleration);
	else
	    requiredSpeed = _speed + abs(_acceleration / _speed);
	if (requiredSpeed > _maxSpeed)
	    requiredSpeed = _maxSpeed;
    }
    else if (requiredSpeed < _speed)
    {
	// Need to accelerate in anticlockwise direction
	if (_speed == 0)
	    requiredSpeed = -sqrt(2.0 * _acceleration);
	else
	    requiredSpeed = _speed - abs(_acceleration / _speed);
	if (requiredSpeed < -_maxSpeed)
	    requiredSpeed = -_maxSpeed;
    }
//  Serial.println(requiredSpeed);
    return requiredSpeed;
}

// Run the motor to implement speed and acceleration in order to proceed to the target position
// You must call this at least once per step, preferably in your main loop
// If the motor is in the desired position, the cost is very small
// returns true if we are still running to position
bool AccelStepper::run()
{
    if (_targetPos == _currentPos)
	return false;
    
    if (runSpeed())
	computeNewSpeed();
    return true;
}

AccelStepper::AccelStepper(uint8_t pins, uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4)
{
    _pins = pins;
    _currentPos = 0;
    _targetPos = 0;
    _speed = 0.0;
    _maxSpeed = 1.0;
    _acceleration = 1.0;
    _stepInterval = 0;
    _lastStepTime = 0;
    _pin1 = new mraa::Gpio(pin1);
    _pin1->dir(mraa::DIR_OUT);
    _pin2 = new mraa::Gpio(pin2);
    _pin2->dir(mraa::DIR_OUT);
    _pin3 = new mraa::Gpio(pin3);
    _pin3->dir(mraa::DIR_OUT);
    _pin4 = new mraa::Gpio(pin4);
    _pin4->dir(mraa::DIR_OUT);
    enableOutputs();
}

AccelStepper::AccelStepper(void (*forward)(), void (*backward)())
{
    _pins = 0;
    _currentPos = 0;
    _targetPos = 0;
    _speed = 0.0;
    _maxSpeed = 1.0;
    _acceleration = 1.0;
    _stepInterval = 0;
    _lastStepTime = 0;
    _pin1 = NULL;
    _pin2 = NULL;
    _pin3 = NULL;
    _pin4 = NULL;
    _forward = forward;
    _backward = backward;
}

void AccelStepper::setMaxSpeed(float speed)
{
    _maxSpeed = speed;
    computeNewSpeed();
}

void AccelStepper::setAcceleration(float acceleration)
{
    _acceleration = acceleration;
    computeNewSpeed();
}

void AccelStepper::setSpeed(float speed)
{
    _speed = speed;
    _stepInterval = abs(1000.0 / _speed);

}

float AccelStepper::speed()
{
    return _speed;
}

// Subclasses can override
void AccelStepper::step(uint8_t step)
{
    switch (_pins)
    {
        case 0:
            step0();
            break;
	case 1:
	    step1(step);
	    break;
    
	case 2:
	    step2(step);
	    break;
    
	case 4:
	    step4(step);
	    break;  
    }
}

// 0 pin step function (ie for functional usage)
void AccelStepper::step0()
{
  if (_speed > 0) {
    _forward();
  } else {
    _backward();
  }
}

// 1 pin step function (ie for stepper drivers)
// This is passed the current step number (0 to 3)
// Subclasses can override
void AccelStepper::step1(uint8_t step)
{
    //digitalWrite(_pin2, _speed > 0); // Direction
    if (_speed > 0)
    	_pin2->write(1);
    else
    	_pin2->write(0);
	// Caution 200ns setup time
    //digitalWrite(_pin1, HIGH);
    _pin1->write(1);
    // Caution, min Step pulse width for 3967 is 1microsec
    // Delay 1microsec
    //delayMicroseconds(1);
    usleep(1000);
    //digitalWrite(_pin1, LOW);
    _pin1->write(0);
}

// 2 pin step function
// This is passed the current step number (0 to 3)
// Subclasses can override
void AccelStepper::step2(uint8_t step)
{
    switch (step)
    {
	case 0: /* 01 */
	    //digitalWrite(_pin1, LOW);
	    _pin1->write(0);
		//digitalWrite(_pin2, HIGH);
	    _pin2->write(1);
	    break;

	case 1: /* 11 */
	    //digitalWrite(_pin1, HIGH);
	    _pin1->write(1);
		//digitalWrite(_pin2, HIGH);
	    _pin2->write(1);
	    break;

	case 2: /* 10 */
	    //digitalWrite(_pin1, HIGH);
	    _pin1->write(1);
		//digitalWrite(_pin2, LOW);
	    _pin2->write(0);
	    break;

	case 3: /* 00 */
	    //digitalWrite(_pin1, LOW);
	    _pin1->write(0);
		//digitalWrite(_pin2, LOW);
	    _pin2->write(0);
	    break;
    }
}

// 4 pin step function
// This is passed the current step number (0 to 3)
// Subclasses can override
void AccelStepper::step4(uint8_t step)
{
    switch (step)
    {
	case 0:    // 1010
	    //digitalWrite(_pin1, HIGH);
	    _pin1->write(1);
		//digitalWrite(_pin2, LOW);
	    _pin2->write(0);
	    //digitalWrite(_pin3, HIGH);
	    _pin3->write(1);
	    //digitalWrite(_pin4, LOW);
	    _pin4->write(0);
	    break;

	case 1:    // 0110
	    //digitalWrite(_pin1, LOW);
	    _pin1->write(0);
		//digitalWrite(_pin2, HIGH);
	    _pin2->write(1);
	    //digitalWrite(_pin3, HIGH);
	    _pin3->write(1);
	    //digitalWrite(_pin4, LOW);
	    _pin4->write(0);
	    break;

	case 2:    //0101
	    //digitalWrite(_pin1, LOW);
	    _pin1->write(0);
		//digitalWrite(_pin2, HIGH);
	    _pin2->write(1);
	    //digitalWrite(_pin3, LOW);
	    _pin3->write(0);
	    //digitalWrite(_pin4, HIGH);
	    _pin4->write(1);
	    break;

	case 3:    //1001
	    //digitalWrite(_pin1, HIGH);
	    _pin1->write(1);
		//digitalWrite(_pin2, LOW);
	    _pin2->write(0);
		//digitalWrite(_pin3, LOW);
	    _pin3->write(0);
	    //digitalWrite(_pin4, HIGH);
	    _pin4->write(1);
	    break;
    }
}


// Prevents power consumption on the outputs
void    AccelStepper::disableOutputs()
{   
  if (! _pins) return;

    //digitalWrite(_pin1, LOW);
    _pin1->write(0);
    //digitalWrite(_pin2, LOW);
    _pin2->write(0);
    if (_pins == 4)
    {
    	//digitalWrite(_pin3, LOW);
    	_pin3->write(0);
    	//digitalWrite(_pin4, LOW);
    	_pin4->write(0);
    }
}

void    AccelStepper::enableOutputs()
{
    if (! _pins) return;

    //pinMode(_pin1, OUTPUT);
    _pin1->dir(mraa::DIR_OUT);
    //pinMode(_pin2, OUTPUT);
    _pin2->dir(mraa::DIR_OUT);
    if (_pins == 4)
    {
    	//pinMode(_pin3, OUTPUT);
    	_pin3->dir(mraa::DIR_OUT);
    	//pinMode(_pin4, OUTPUT);
    	_pin4->dir(mraa::DIR_OUT);
    }
}

// Blocks until the target position is reached
void AccelStepper::runToPosition()
{
    while (run())
	;
}

bool AccelStepper::runSpeedToPosition()
{
    return _targetPos!=_currentPos ? AccelStepper::runSpeed() : false;
}

// Blocks until the new target position is reached
void AccelStepper::runToNewPosition(long position)
{
    moveTo(position);
    runToPosition();
}


