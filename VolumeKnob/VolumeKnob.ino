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
// #define SERIAL_DEBUG

// Set this to non-zero to have an initial start value for testing
#define INITIAL_START 0

// Define these as the physical pins that the encoder is attached to.
const uint8_t ENCODER_PIN_A = 8;
const uint8_t ENCODER_PIN_B = 9;
// This pin is actually NC for our case but it is required to pass to the library.
const uint8_t ENCODER_BUTTON = 12;
const uint8_t ENCODER_PUSH_BUTTON = 7;

const uint8_t PUSH_BUTTON = 6;
const uint8_t PUSH_BUTTON_PWR = 10;

// Define these as the digital output pins in which the two N-channel MOSFETS connect.
const uint8_t MOSFET_DECREASING = 4;
const uint8_t MOSFET_INCREASING = 5;
const uint8_t MOSFET_ENC_BUT = 2;
const uint8_t MOSFET_PUSH_BUT = 3;

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

// Handling debounce issues
const uint32_t debounceDelay = 300;
uint32_t lastDebounceTimeEnc = 0;
uint32_t lastDebounceTimePush = 0;

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
    EncButtonPressing,
    PushButtonPressing
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
    pinMode(MOSFET_ENC_BUT, OUTPUT);
    pinMode(MOSFET_PUSH_BUT, OUTPUT);

    pinMode(TEST_PIN, INPUT_PULLUP);
    pinMode(PUSH_BUTTON, INPUT_PULLUP); // adding push button as an input
    pinMode(ENCODER_PUSH_BUTTON, INPUT_PULLUP); // adding encoder push button as input

    pinMode(PUSH_BUTTON_PWR, OUTPUT); // for providing power to LED
    digitalWrite(PUSH_BUTTON_PWR, HIGH); // for providing power to LED

    digitalWrite(MOSFET_DECREASING, LOW);
    digitalWrite(MOSFET_INCREASING, LOW);
    digitalWrite(MOSFET_ENC_BUT, LOW);
    digitalWrite(MOSFET_PUSH_BUT, LOW);
    

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

  if (state == InternalState::Decreasing || state == InternalState::Increasing ||
      state == InternalState::EncButtonPressing || state == InternalState::PushButtonPressing)
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

  // This calls checkforrotarychange, which listens to knob, then if change is made, calls handleKnobChange
  // handleKnobChange then sets the mosfet pin high, changes state to increase/decrease (1 or 2), and +/-'s 1 from count.
  CheckForRotaryChange();
  encoderStopwatch.Reset();

  // If for example, turning knob to the right, we then return here with a state of 1 (decreasing) and count of -1

  if ((state == InternalState::Increasing || state == InternalState::Decreasing ||
        state == InternalState::EncButtonPressing || state == InternalState::PushButtonPressing)
      && mosfetStopwatch.HasElapsed(lengthToHold))
  {
    // This code only runs if our state is 1 or 2 (increase/decrease) and the mosfetStopwatch has elapsed, meaning its time to turn off the monsfet.
    HandleTurnOffEncMosfets();
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
void checkForButtonChange()
{
  if (digitalRead(ENCODER_PUSH_BUTTON) == LOW)
  {
    uint32_t currentMillis = millis();
    if ((currentMillis - lastDebounceTimeEnc) > debounceDelay && state != InternalState::EncButtonPressing)
    {
      lastDebounceTimeEnc = currentMillis; // Update debounce time
      HandleButtonPress(InternalState::EncButtonPressing);
    }
  }
  else if (digitalRead(PUSH_BUTTON) == LOW)
  {
    uint32_t currentMillis = millis();
    if ((currentMillis - lastDebounceTimeEnc) > debounceDelay && state != InternalState::PushButtonPressing)
    {
      lastDebounceTimeEnc = currentMillis; // Update debounce time
      HandleButtonPress(InternalState::PushButtonPressing);
    }
  }
}

void CheckForRotaryChange()
{
  int newPos = volumeKnob.getPosition();
  int delta = abs(newPos - previousPosition);
  

  #ifdef SERIAL_DEBUG
  if (delta != 0)
  {
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

  checkForButtonChange();
}

void HandleButtonPress(InternalState newState)
{
  if (count == 0)
  {
    uint8_t pin = newState == InternalState::EncButtonPressing ? MOSFET_ENC_BUT : MOSFET_PUSH_BUT;
    digitalWrite(pin, HIGH);
  }

  state = newState;
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

  if (state == InternalState::Increasing) {
    count += delta;
  } else {
    count += -delta;
  }
}

void HandleTurnOffEncMosfets()
{
  #ifdef SERIAL_DEBUG
  {
    Serial.println("###");
    Serial.println("Turning off ENCODER mosfets");
    Serial.print("state, count: "); Serial.print(state); Serial.print(", "); Serial.println(count);
  }
  #endif // SERIAL_DEBUG
  // This function will always set state to holdLow and write all mosfets to low.

  // If current state is increasing, we -1 to count. If state is decreasing, we +1 to count. 
  if (state == InternalState::Increasing)
  {
    count += -1;
  }
  else { if (state == InternalState::Decreasing || state == InternalState::Off || state == InternalState::HoldLow)
  {
    count += 1;
  }}
  // count += state == InternalState::Increasing ? -1 : 1;
  state = InternalState::HoldLow;

  digitalWrite(MOSFET_DECREASING, LOW);
  digitalWrite(MOSFET_INCREASING, LOW);
  digitalWrite(MOSFET_ENC_BUT, LOW);
  digitalWrite(MOSFET_PUSH_BUT, LOW);
}


// ------------------------------------------------------------------------------------
