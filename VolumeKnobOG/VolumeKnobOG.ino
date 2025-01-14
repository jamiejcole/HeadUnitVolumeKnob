#define PAUL_ENCODER 0
#define ROTARY_ENCODER 1

#define ENCODER ROTARY_ENCODER

#include "Arduino.h"
#if ENCODER == ROTARY_ENCODER
#include <RotaryEncoder.h>
#else
#include <Encoder.h>
#endif
#include <Stopwatch.h>

// ------------------------------------------------------------------------------------
// CONSTANTS
// ------------------------------------------------------------------------------------
// Uncomment this line to see debugging info
#define SERIAL_DEBUG

// Set this to non-zero to have an initial start value for testing
#define INITIAL_START 0

// Define these as the physical pins that the encoder is attached to.
const uint8_t ENCODER_PIN_A = 8;
const uint8_t ENCODER_PIN_B = 9;
// This pin is actually NC for our case but it is required to pass to the library.
const uint8_t ENCODER_BUTTON = 7;

// Define these as the digital output pins in which the two N-channel MOSFETS connect.
const uint8_t MOSFET_DECREASING = 4;
const uint8_t MOSFET_INCREASING = 5;

// Define the test pin. Setting this pin to GND will put the device in test mode which
// sends the signal for the extended hold time, allowing you to configure the device via
// the android SwC app.
const uint8_t TEST_PIN = 11;

// Length of time in milliseconds to hold the moseft open simulating a human touch.
//
// If this value is too short, the head unit will not register a simulated button press
// by an external user. If the value is too long, this program could miss increments if
// the person is spinning the knob rapidly.
const uint32_t MOSFET_HOLD_MS = 80;
const uint32_t MOSFET_HOLD_LOW_MS = 20;

// Length of time in milliseconds to wait before polling the encoder to see if the
// state changed.
const uint32_t ENCODER_READ_MS = 1;

// Length in time in milliseconds that MOSFET is held when in test mode.
const uint32_t MOSFET_HOLD_EXTENDED_MS = 4000;
// ------------------------------------------------------------------------------------


// ------------------------------------------------------------------------------------
// Enums
// ------------------------------------------------------------------------------------
enum InternalState : uint8_t
{
    Off,
    Decreasing,
    Increasing,
    HoldLow,
};
// ------------------------------------------------------------------------------------


// ------------------------------------------------------------------------------------
// Variables
// ------------------------------------------------------------------------------------
#if ENCODER == ROTARY_ENCODER
// Encoder Variable; this does the heavy lifting to read the encoder values and update positions.
RotaryEncoder volumeKnob(ENCODER_PIN_A, ENCODER_PIN_B, RotaryEncoder::LatchMode::FOUR3);
#else
Encoder volumeKnob(ENCODER_PIN_A, ENCODER_PIN_B);
#endif
// Stopwatch variables to trigger turning off the MOSFETs and checking the encoder
Stopwatch encoderStopwatch;
Stopwatch mosfetStopwatch;
Stopwatch holdLowStopwatch;

long previousPosition;

// Keeps track of when the MOSFETS need to be shut off.
InternalState state = InternalState::Off;

uint32_t lengthToHold;

long count = 0;
// ------------------------------------------------------------------------------------


// ------------------------------------------------------------------------------------
// Arduino Methods
// ------------------------------------------------------------------------------------
void setup()
{
    #ifdef SERIAL_DEBUG
    {
        Serial.begin(9600);
    }
    #endif // SERIAL_DEBUG

    pinMode(MOSFET_DECREASING, OUTPUT);
    pinMode(MOSFET_INCREASING, OUTPUT);
    pinMode(TEST_PIN, INPUT_PULLUP);
    digitalWrite(MOSFET_DECREASING, LOW);
    digitalWrite(MOSFET_INCREASING, LOW);

    encoderStopwatch.Start();
    mosfetStopwatch.Start();

    lengthToHold = MOSFET_HOLD_MS;

    #if INITIAL_START != 0
    {
        count = 0;
        state = InternalState::Off;
    }
    #endif  // INITIAL_START != 0
}

void loop()
{
    volumeKnob.tick();
    encoderStopwatch.Update();

    if (state == InternalState::Decreasing || state == InternalState::Increasing)
    {
        mosfetStopwatch.Update();
    }
    else
    {
        mosfetStopwatch.Reset();
    }

    if (state == InternalState::HoldLow)
    {
        holdLowStopwatch.Update();
    }
    else
    {
        holdLowStopwatch.Reset();
    }

    // if (encoderStopwatch.HasElapsed(ENCODER_READ_MS))
    {
        CheckForRotaryChange();
        encoderStopwatch.Reset();
    }

    if ((state == InternalState::Increasing || state == InternalState::Decreasing)
         && mosfetStopwatch.HasElapsed(lengthToHold))
    {
        HandleTurnOffMosfets();
        mosfetStopwatch.Reset();
        uint32_t testPinVal = digitalRead(TEST_PIN);
        lengthToHold = testPinVal == LOW ? MOSFET_HOLD_EXTENDED_MS : MOSFET_HOLD_MS;
    }

    if (state == InternalState::HoldLow && holdLowStopwatch.HasElapsed(MOSFET_HOLD_LOW_MS))
    {
        state = count == 0
            ? InternalState::Off
            : count > 0
                ? InternalState::Increasing
                : InternalState::Decreasing;
        
        if (state != InternalState::Off)
        {
            uint8_t pin = state == InternalState::Decreasing ? MOSFET_DECREASING : MOSFET_INCREASING;
            digitalWrite(pin, HIGH);
        }
    }
}
// ------------------------------------------------------------------------------------


// ------------------------------------------------------------------------------------
// Private methods
// ------------------------------------------------------------------------------------
void CheckForRotaryChange()
{
    int newPos = volumeKnob.getPosition();
    int delta = abs(newPos - previousPosition);

    #ifdef SERIAL_DEBUG
    if (delta != 0)
    {
        Serial.print("delta: ");
        Serial.println(delta);
    }

    #endif // SERIAL_DEBUG
    if (newPos > previousPosition)
    {
        HandleKnobChange(delta, InternalState::Decreasing);
    }
    else if (newPos < previousPosition)
    {
        HandleKnobChange(delta, InternalState::Increasing);
    }

    previousPosition = newPos;
}

void HandleKnobChange(int delta, InternalState newState)
{
    #ifdef SERIAL_DEBUG
    {
        Serial.print("Handling ");
        Serial.println(newState == InternalState::Decreasing ? "Decreasing" : "Increasing");
    }
    #endif // SERIAL_DEBUG

    if (count == 0)
    {
        uint8_t pin = newState == InternalState::Decreasing ? MOSFET_DECREASING : MOSFET_INCREASING;
        digitalWrite(pin, HIGH);
    }

    state = newState;
    count += state == InternalState::Increasing ? delta : -delta;
}

void HandleTurnOffMosfets()
{
    #ifdef SERIAL_DEBUG
    {
        Serial.println("Turning off mosfets");
    }
    #endif // SERIAL_DEBUG

    count += state == InternalState::Increasing ? -1 : 1;
    state = InternalState::HoldLow;
    digitalWrite(MOSFET_DECREASING, LOW);
    digitalWrite(MOSFET_INCREASING, LOW);
}
// ------------------------------------------------------------------------------------