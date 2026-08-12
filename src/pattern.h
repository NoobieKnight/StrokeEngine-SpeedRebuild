/**
 *   Patterns of the StrokeEngine
 *   A library to create a variety of stroking motions with a stepper or servo motor on an ESP32.
 *   https://github.com/theelims/StrokeEngine
 *
 * Copyright (C) 2021 theelims <elims@gmx.net>
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 *
 * Modified for use in OSSM with speed/acceleration instead of time of stroke
 * NoobieKnight
 *
 */

#pragma once

#include <Arduino.h>
#include <StrokeEngine.h>
#include <math.h>
#include "PatternMath.h"

#define DEBUG_PATTERN                 // Print some debug informations over Serial

#ifndef STRING_LEN
  #define STRING_LEN           64     // Bytes used to initialize char array. No path, topic, name, etc. should exceed this value
#endif

/**************************************************************************/
/*!
  @brief  struct to return all parameters FastAccelStepper needs to calculate
  the trapezoidal profile.
*/
/**************************************************************************/
typedef struct {
    int stroke;         //!< Absolute and properly constrainted target position of a move in steps
    int speed;          //!< Speed of a move in Steps/second
    int acceleration;   //!< Acceleration to get to speed or halt
    bool skip;          //!< no valid stroke, skip this set an query for the next --> allows pauses between strokes
} motionParameter;


/**************************************************************************/
/*!
  @class Pattern
  @brief  Base class to derive your pattern from. Offers a unified set of
          functions to store all relevant paramteres. These function can be
          overridenid necessary. Pattern should be self-containted and not
          rely on any stepper/servo related properties. Internal book keeping
          is done in steps. The translation from real word units to steps is
          provided by the StrokeEngine. Also the sanity check whether motion
          parameters are physically possible are done by the StrokeEngine.
          Imposible motion commands are clipped, cropped or adjusted while
          still having a smooth appearance.
*/
/**************************************************************************/
class Pattern {

    public:
        //! Constructor
        /*!
          @param str String containing the name of a pattern
        */
        Pattern(const char *str) { strcpy(_name, str); }

        //! Virtual destructor for proper polymorphic cleanup
        virtual ~Pattern() = default;

        //! Set the time a normal stroke should take to complete
        /*!
          @param speed time of a full stroke in [sec]
        */
        virtual void setTimeOfStroke(float speed, float speedStepsPerS) {
            _timeOfStroke = speed;
            _speedStepsPerS = speedStepsPerS;
            _calculatePatternPositions();
        }

        //! Set the maximum stroke a pattern may have
        /*!
          @param stroke stroke distance in Steps
        */
        virtual void setStroke(int stroke, float timeOfStroke) {
            _stroke = stroke;
            _calculatePatternPositions();
        }

        //! Set the maximum depth a pattern may have
        /*!
          @param stroke stroke distance in Steps
        */
        virtual void setDepth(int depth) {
            _depth = depth;
            _calculatePatternPositions();
        }

        //! Sensation is an additional parameter a pattern can take to alter its behaviour
        /*!
          @param sensation Arbitrary value from -100 to 100, with 0 beeing neutral
        */
        virtual void setSensation(float sensation, float sensationPercent) {
            _sensation = sensation;
            _sensationPercent = sensationPercent;
            _calculatePatternPositions();
        }

        //! Retrives the name of a pattern
        /*!
          @return c_string containing the name of a pattern
        */
        char *getName() { return _name; }

        //! Calculate the position of the next stroke based on the various parameters
        /*!
          @param index index of a stroke. Increments with every new stroke.
          @return Set of motion parameteres like speed, acceleration & position
        */
        virtual motionParameter nextTarget(unsigned int index) {
            _index = index;
            return _nextMove;
        }

        //! Communicates the maximum possible speed and acceleration limits of the machine to a pattern.
        /*!
          @param maxSpeed maximum speed which is possible. Higher speeds get truncated inside StrokeEngine anyway.
          @param maxAcceleration maximum possible acceleration. Get also truncated, if impossible.
          @param stepsPerMM
        */
        virtual void setSpeedLimit(unsigned int maxSpeed, unsigned int maxAcceleration, unsigned int stepsPerMM) { _maxSpeed = maxSpeed; _maxAcceleration = maxAcceleration; _stepsPerMM = stepsPerMM; }

    protected:
        int _stroke;
        int _depth;
        int _minPos; // Position where the stroke begins (_depth - _stroke)
        int _maxPos; // Position where the stroke ends (_depth)
        float _timeOfStroke;
        float _speedStepsPerS;
        float _sensation = 0.0;
        float _sensationPercent = 0.0;
        int _index = -1;
        char _name[STRING_LEN];
        motionParameter _nextMove = {0, 0, 0, false};
        int _startDelayMillis = 0;
        int _delayInMillis = 0;
        unsigned int _maxSpeed = 0;
        unsigned int _maxAcceleration = 0;
        unsigned int _stepsPerMM = 0;

        /*!
          @brief Start a delay timer which can be polled by calling _isStillDelayed().
          Uses internally the millis()-function.
        */
        void _startDelay() {
            _startDelayMillis = millis();
        }

        /*!
          @brief Update a delay timer which can be polled by calling _isStillDelayed().
          Uses internally the millis()-function.
          @param delayInMillis delay in milliseconds
        */
        void _updateDelay(int delayInMillis) {
            _delayInMillis = delayInMillis;
        }

        /*!
          @brief Poll the state of a internal timer to create pauses between strokes.
          Uses internally the millis()-function.
          @return True, if the timer is running, false if it is expired.
        */
        bool _isStillDelayed() {
            return (millis() > (_startDelayMillis + _delayInMillis)) ? false : true;
        }

        /*!
          @brief This function is called when speed, depth, stroke or sensation changes
          Calculations can be put here if there is no need for them to be recalculated every cycle
        */
        virtual void _calculatePatternPositions(){

            // Calculate the start/end of the stroke
            _minPos = _depth - _stroke;
            _maxPos = _depth;

        }

};

/**************************************************************************/
/*!
  @brief  Simple Stroke Pattern. It creates a trapezoidal stroke profile
  with 1/3 acceleration, 1/3 coasting, 1/3 deceleration. Sensation has
  no effect.
*/
/**************************************************************************/
class SimpleStroke : public Pattern {
    public:
        SimpleStroke(const char *str) : Pattern(str) {}

        motionParameter nextTarget(unsigned int index) {
            // maximum speed of the trapezoidal motion
            _nextMove.speed = _speed;

            // acceleration to meet the profile
            _nextMove.acceleration = _acceleration;

            // odd stroke is moving out
            if (index % 2) {
                _nextMove.stroke = _maxPos;

            // even stroke is moving in
            } else {
                _nextMove.stroke = _minPos;

            }

            _index = index;
            return _nextMove;
        }
        protected:
        int _acceleration;
        int _speed;

        void _calculatePatternPositions() override {
            _acceleration = min(uint(calcTrapezoidalAcceleration(_speedStepsPerS, _stroke)), _maxAcceleration);
            _speed = int(_speedStepsPerS);

            // Calculate the start/end of the stroke
            _minPos = _depth - _stroke;
            _maxPos = _depth;

        }
};

/**************************************************************************/
/*!
  @brief  Simple pattern where the sensation value can change the speed
  ratio between in and out. Sensation > 0 make the in move faster (up to 5x)
  giving a hard pounding sensation. Values < 0 make the out move going
  faster. This gives a more pleasing sensation.
*/
/**************************************************************************/
class TeasingPounding : public Pattern {
    public:
        TeasingPounding(const char *str) : Pattern(str) {}

        motionParameter nextTarget(unsigned int index) override{
            // odd stroke is moving out
            if (index % 2) {
                // maximum speed of the trapezoidal motion
                _nextMove.speed = _speedOut;

                // acceleration to meet the profile
                _nextMove.acceleration = _accOut;
                _nextMove.stroke = _minPos;
            // even stroke is moving in
            } else {
                // maximum speed of the trapezoidal motion
                _nextMove.speed = _speedIn;

                // acceleration to meet the profile
                _nextMove.acceleration = _accIn;
                _nextMove.stroke = _maxPos;
            }
            _index = index;
            return _nextMove;
        }
    protected:
        int _accIn, _accOut;
        int _speedIn, _speedOut;
        void _calculatePatternPositions() override{

            // Calculate the start/end of the stroke
            _minPos = _depth - _stroke;
            _maxPos = _depth;

            if (_sensation > 0.0) {
                _speedIn = _speedStepsPerS;
                _speedOut = int(_speedStepsPerS / fscale(0.0, 100.0, 1.0, 5.0, abs(_sensation), 0.0));
            } else {
                _speedOut = _speedStepsPerS;
                _speedIn = int(_speedStepsPerS / fscale(0.0, 100.0, 1.0, 5.0, abs(_sensation), 0.0));
            }

            // Calculate the acceleration for the in and out stroke
            _accIn = min(uint(calcTrapezoidalAcceleration(_speedIn, _stroke)), _maxAcceleration);
            _accOut = min(uint(calcTrapezoidalAcceleration(_speedOut, _stroke)), _maxAcceleration);

#ifdef DEBUG_PATTERN
            if (_speedOut > 0 && _speedIn > 0 && _accOut > 0 && _accIn > 0) {
                Serial.println("SpeedOfInStroke: " + String(_speedIn));
                Serial.println("SpeedOfOutStroke: " + String(_speedOut));
            }
#endif
        }
};


/**************************************************************************/
/*!
  @brief  Robot Stroke Pattern. Sensation controls the acceleration of the
  stroke. Positive value increase acceleration until it is a constant speed
  motion (feels robotic). Negative reduces acceleration into a triangle profile.
*/
/**************************************************************************/
class RoboStroke : public Pattern {
    public:
        RoboStroke(const char *str) : Pattern(str) {}

        void setSensation(float sensation, float sensationPercent) override {
            _sensation = sensation;
            _sensationPercent = sensationPercent;

            // Calculate the acceleration based on the sensation value. Positive values increase acceleration, negative values decrease it.
            _acceleration = fscale(-100.0, 100.0, 5000, _maxAcceleration, _sensation, 0.0);

#ifdef DEBUG_PATTERN
            Serial.println("Acceleration:" + String(_acceleration));
#endif

        }

        void setTimeOfStroke(float speed, float speedStepsPerS) override {
            _timeOfStroke = speed;
            _speedStepsPerS = speedStepsPerS;

            _speed = int(_speedStepsPerS);
        }

        motionParameter nextTarget(unsigned int index) override {
            // maximum speed of the trapezoidal motion
            _nextMove.speed = _speed;

            // acceleration to meet the profile
            _nextMove.acceleration = _acceleration;

            // odd stroke is moving out
            if (index % 2) {
                _nextMove.stroke = _minPos;

            // even stroke is moving in
            } else {
                _nextMove.stroke = _maxPos;
            }

            _index = index;
            return _nextMove;
        }
    protected:
        int _acceleration;
        int _speed;

};

/**************************************************************************/
/*!
  @brief  Like Teasing or Pounding, but every second stroke is only half the
  depth. The sensation value can change the speed ratio between in and out.
  Sensation > 0 make the in move faster (up to 5x) giving a hard pounding
  sensation. Values < 0 make the out move going faster. This gives a more
  pleasing sensation. The top speed of the stroke remains the same for
  all strokes, even half ones.
*/
/**************************************************************************/
class HalfnHalf : public Pattern {
    public:
        HalfnHalf(const char *str) : Pattern(str) {}

        motionParameter nextTarget(unsigned int index) override {
            // check if this is the very first
            if (index == 0) {
              //pattern started for the very fist time, so we start gentle with a half move
              _half = true;
            }

            // odd stroke is moving out
            if (index % 2) {
                // maximum speed of the trapezoidal motion
                _nextMove.speed = _speedOut;

                // acceleration to meet the profile
                _nextMove.acceleration = _accOut;
                _nextMove.stroke = _minPos;
            // even stroke is moving in
            } else {
                // maximum speed of the trapezoidal motion
                _nextMove.speed = _speedIn;

                // acceleration to meet the profile
                _nextMove.acceleration = _accIn;
            if (_half == true) {
                // half the stroke length
                _nextMove.stroke = _maxPos - _halfStroke;
                _half = false;
            } else {
                _nextMove.stroke = _maxPos;
                _half = true;
            }}

            _index = index;
            return _nextMove;
        }

        protected:
        int _accIn, _accOut;
        int _speedIn, _speedOut;
        int _halfStroke;
        bool _half = true;

        void _calculatePatternPositions() override{

            // Calculate the start/end of the stroke
            _minPos = _depth - _stroke;
            _maxPos = _depth;
            _halfStroke = _stroke / 2;

            if (_sensation > 0.0) {
                _speedIn = _speedStepsPerS;
                _speedOut = int(_speedStepsPerS / fscale(0.0, 100.0, 1.0, 5.0, abs(_sensation), 0.0));
            } else {
                _speedOut = _speedStepsPerS;
                _speedIn = int(_speedStepsPerS / fscale(0.0, 100.0, 1.0, 5.0, abs(_sensation), 0.0));
            }

            // Calculate the acceleration for the in and out stroke
            _accIn = min(uint(calcTrapezoidalAcceleration(_speedIn, _stroke)), _maxAcceleration);
            _accOut = min(uint(calcTrapezoidalAcceleration(_speedOut, _stroke)), _maxAcceleration);

#ifdef DEBUG_PATTERN
            if (_speedOut > 0 && _speedIn > 0 && _accOut > 0 && _accIn > 0) {
                Serial.println("SpeedOfInStroke: " + String(_speedIn));
                Serial.println("SpeedOfOutStroke: " + String(_speedOut));
            }
#endif
        }
};

/**************************************************************************/
/*!
  @brief  The insertion depth ramps up gradually with each stroke until it
  reaches its maximum. It then resets and restars. Sensations controls how
  many strokes there are in a ramp.
*/
/**************************************************************************/
class Deeper : public Pattern {
    public:
        Deeper(const char *str) : Pattern(str) {}

        motionParameter nextTarget(unsigned int index) override {

            // The pattern recycles so we use modulo to get a cycling index.
            // Factor 2 because index increments with each full stroke twice
            // add 1 because modulo = 0 is index = 1
            int cycleIndex = (index / 2) % _countStrokesForRamp + 1;

            // Amplitude is slope * cycleIndex
            int amplitude = _slope * cycleIndex;
#ifdef DEBUG_PATTERN
            Serial.println("amplitude: " + String(amplitude)
                         + " cycleIndex: " + String(cycleIndex));
#endif

            // maximum speed of the trapezoidal motion
            _nextMove.speed = int(_speedStepsPerS);

            // acceleration to meet the profile
            _nextMove.acceleration = min(uint(calcTrapezoidalAcceleration(_speedStepsPerS, _stroke)), _maxAcceleration);

            // odd stroke is moving out
            if (index % 2) {
                _nextMove.stroke = _minPos;

            // even stroke is moving in
            } else {
                _nextMove.stroke = min(_minPos + amplitude, _maxPos);
            }

            _index = index;
            return _nextMove;
        }

    protected:
        int _countStrokesForRamp = 2;
        int _slope = 0;

        void _calculatePatternPositions() override{

            // Calculate the start/end of the stroke
            _minPos = _depth - _stroke;
            _maxPos = _depth;

            // maps sensation to useful values [2,22] with 12 beeing neutral
            if (_sensation < 0) {
                _countStrokesForRamp = map(_sensation, -100, 0, 2, 11);
            } else {
                _countStrokesForRamp = map(_sensation, 0, 100, 11, 32);
            }

#ifdef DEBUG_PATTERN
            Serial.println("_countStrokesForRamp: " + String(_countStrokesForRamp));
#endif

            // How many steps is each stroke advancing
            _slope = _stroke / _countStrokesForRamp;


        }
};

/**************************************************************************/
/*!
  @brief  Pauses between a series of strokes.
  The number of strokes ramps from 1 stroke to 5 strokes and back. Sensation
  changes the length of the pauses between stroke series.
*/
/**************************************************************************/
class StopNGo : public Pattern {
    public:
        StopNGo(const char *str) : Pattern(str) {}

        void setSensation(float sensation, float sensationPercent) override {
            _sensation = sensation;

            // maps sensation to a delay from 100ms to 10 sec
            _updateDelay(map(sensation, -100, 100, 100, 10000));
        }


        motionParameter nextTarget(unsigned int index) override {
            // maximum speed of the trapezoidal motion
            _nextMove.speed = int(_speedStepsPerS);

            // acceleration to meet the profile
            _nextMove.acceleration = acceleration;

            // adds a delay between each stroke
            if (_isStillDelayed() == false) {

                // odd stroke is moving out
                if (index % 2) {
                    _nextMove.stroke = _minPos;

                    if (_strokeIndex >= _strokeSeriesIndex) {
                        // Reset stroke index to 1
                        _strokeIndex = 0;

                        if (_strokeSeriesIndex >= _numberOfStrokes) {
                            // change count direction once we reached the maximum number of strokes
                            _countStrokesUp = false;
                        } else if (_strokeSeriesIndex <= 1) {
                            // change count direction once we reached one stroke counting down
                            _countStrokesUp = true;
                        }

                        // increment or decrement strokes counter
                        if (_countStrokesUp == true) {
                            _strokeSeriesIndex++;
                        } else {
                            _strokeSeriesIndex--;
                        }

                        // start delay after having moved out
                        _startDelay();
                    }


                // even stroke is moving in
                } else {
                    _nextMove.stroke = _maxPos;
                    // Increment stroke index by one
                    _strokeIndex++;
                }
                _nextMove.skip = false;
            } else {
                _nextMove.skip = true;
            }

            _index = index;

            return _nextMove;
        }

    protected:
        const int _numberOfStrokes = 5;

        int _strokeSeriesIndex = 1;
        int _strokeIndex = 0;
        bool _countStrokesUp = true;
        int acceleration;

        void _calculatePatternPositions() override{
            // Calculate the start/end of the stroke
            _minPos = _depth - _stroke;
            _maxPos = _depth;

            // Calculate the acceleration for the in and out stroke
            acceleration = min(uint(calcTrapezoidalAcceleration(_speedStepsPerS, _stroke)), _maxAcceleration);
        }
};

/**************************************************************************/
/*!
  @brief  Sensation reduces the effective stroke length while keeping the
  stroke speed constant to the full stroke. This creates interesting
  vibrational pattern at higher sensation values. With positive sensation the
  strokes will wander towards the front, with negative values towards the back.
*/
/**************************************************************************/
class Insist : public Pattern {
    public:
        Insist(const char *str) : Pattern(str) {}

        void setSensation(float sensation, float sensationPercent) override {
            _sensation = sensation;

            // Create a stroke fraction from 0.0 to 1.0 based on the sensation value. This will be used to reduce the effective stroke length.
            _strokeFraction = constrain(abs(sensationPercent), 0.001, 1.0);

            // Determine if the stroke is in front or back based on the sign of the sensation value.
            _strokeInFront = (sensation > 0) ? true : false;

            _calculatePatternPositions();
        }

        motionParameter nextTarget(unsigned int index) override {

            // acceleration & speed to meet the profile
            _nextMove.acceleration = _acceleration;
            _nextMove.speed = _speedStepsPerS;

            // odd stroke is moving out
            if (index % 2) {
                _nextMove.stroke = _minPos;

            // even stroke is moving in
            } else {
                _nextMove.stroke = _maxPos;
            }

            _index = index;
            return _nextMove;
        }

    protected:
        int _speed = 0;
        int _acceleration = 0;
        int _realStroke = 0;
        float _strokeFraction = 1.0;
        bool _strokeInFront = false;
        void _calculatePatternPositions() override{

            // Calculate fractional stroke length
            _realStroke = int(_stroke - _stroke * _strokeFraction);

            // Calculate the max/min position in steps
            if (_strokeInFront) {
                _maxPos = _depth;
                _minPos = _maxPos - _realStroke;
            } else {
                _minPos = _depth - _stroke;
                _maxPos = _minPos + _realStroke;
            }

            // Acceleration to hold 1/3 profile with fractional strokes
            _acceleration = constrain(int(calcTrapezoidalAcceleration(_speedStepsPerS, _realStroke)), 0, _maxAcceleration);

        }

};

/**************************************************************************/
/*!
  @brief  Sensation modifies speed of the last part of the thrust in (Positive sensation)
  or first part of the thrust out (Negative sensation)
*/
/**************************************************************************/
class SensualDistance : public Pattern {
  public:
    SensualDistance(const char *str) : Pattern(str) {}

    motionParameter nextTarget(unsigned int index) override {
        int phase = index % 3; // Number of moves in a cycle
                // Set speed and acceleration
                _nextMove.speed = _speedFast;
                _nextMove.acceleration = _accelFast;

        switch (phase) {
            case 0:
                // Set Stroke length
                // Should thrust slow down near the end?
                if (_useSlowIn){
                    _nextMove.stroke = _maxPos - _distFast;
                }else{
                    _nextMove.stroke = _maxPos;
                }
                break;
            case 1:
                // Set Stroke length
                // Should thrust move slow in the start of the thrust back?
                if (_useSlowIn){
                    _nextMove.stroke = _maxPos;
                } else if (_useSlowOut){
                    _nextMove.stroke = _maxPos - _distFast;
                } else {
                    break;
                }
                // Set speed and acceleration
                _nextMove.speed = _speedSlow;
                _nextMove.acceleration = _accelSlow;
                break;
            case 2:
                // Thrust all the way back fast
                _nextMove.stroke = _minPos;
                break;
        }

        // Go to next move
        _index = index;
        return _nextMove;
    }

  protected:
    const float _slowSpeedFactor = 0.25; // How much slower the slow part is compared to the fast part
    const float _slowMaxDistanceFactor = 0.5; // How much of the stroke will be allowed to be slow. 1 = 100% of the stroke, 0.5 = 50% of the stroke, 0.25 = 25% of the stroke

    int _distFast;
    int _speedFast, _speedSlow;
    int _accelFast, _accelSlow;
    int _maxPos, _minPos;
    bool _useSlowIn, _useSlowOut;

    void _calculatePatternPositions() override {
        float desiredAccelMMperS2_fast = 10000.0f; // Hardcoded acceleration fast setpoint
        float desiredAccelMMperS2_slow = 500.0f; // Hardcoded acceleration slow setpoint
        float maxSpeedMMperS_slow = 200.0f; // Hardcoded speed setpoint

        // Calculate the maximum speed and acceleration in steps
        unsigned int desiredAccelStepsperS2_fast = (unsigned int)(desiredAccelMMperS2_fast * _stepsPerMM);
        unsigned int desiredAccelStepsperS2_slow= (unsigned int)(desiredAccelMMperS2_slow * _stepsPerMM);
        unsigned int safeMaxAccel = (_maxAcceleration > 0) ? _maxAcceleration : desiredAccelStepsperS2_slow;

        // Logic check to determine if we should use slow in or slow out based on sensation
        _useSlowIn = _sensation > 0;
        _useSlowOut = _sensation < 0;

        // Calculate the max/min position in steps
        _maxPos = _depth;
        _minPos = _maxPos - _stroke;

        // Calulate the speed for the fast and the slow part
        _speedFast = _speedStepsPerS;
        _speedSlow = int(min(float(_speedFast * _slowSpeedFactor), float(maxSpeedMMperS_slow * _stepsPerMM)));

        // Calculate acceleration for the fast and slow part, constrained to the safe max acceleration
        _accelFast = int(constrain(desiredAccelStepsperS2_fast, (1), safeMaxAccel));
        _accelSlow = int(constrain(desiredAccelStepsperS2_slow, (1), min(desiredAccelStepsperS2_fast,safeMaxAccel)));


        // Calculate the distance of the fast part of the stroke based on the sensation and the _slowMaxDistanceFactor
        _distFast = int((_stroke * _slowMaxDistanceFactor) * abs(_sensationPercent));

    }
};
