#include <vector>

#include "Arduino.h"
#include "C610Bus.h"

long last_command = 0;  // To keep track of when we last commanded the motors
C610Bus<CAN2> bus;      // Initialize the Teensy's CAN bus to talk to the motors

const int LOOP_DELAY_MILLIS = 5;  // Wait for 0.005s between motor updates.

const float m1_offset = 0.0;
const float m2_offset = 0.0;

// Step 5. Implement your own PD controller here.
float pd_control(float pos,
                 float vel,
                 float target,
                 float Kp,
                 float Kd) {
  return (-Kp * (pos - target)) + (-Kd * vel);
}

/*
Logs info for IDs from lower to upper, inclusive
*/
void logInfo(int lower, int upper) {
  for (int i = lower; i <= upper; i++) {
    Serial.print("\tm");
    Serial.print(i);
    Serial.print("_pos: ");
    Serial.print(bus.Get(i).Position());
    Serial.print("\tm");
    Serial.print(i);
    Serial.print("_vel: ");
    Serial.print(bus.Get(i).Velocity());
  }
}

void sanitize_current_command(float &command,
                              float pos,
                              float vel,
                              float max_current = 2000,
                              float max_pos = 3.141,
                              float max_vel = 30,
                              float reduction_factor = 0.1) {
  /* Sanitize current command to make it safer.

  Clips current command between bounds. Reduces command if actuator outside of position or velocity bounds.
  Max current defaults to 1000mA. Max position defaults to +-180degs. Max velocity defaults to +-5rotations/s.
  */
  command = command > max_current ? max_current : command;
  command = command < -max_current ? -max_current : command;
  if (pos > max_pos || pos < -max_pos) {
    Serial.println("ERROR: Actuator position outside of allowed bounds.");
  }
  if (vel > max_vel || vel < -max_vel) {
    command = 0;
    Serial.println("ERROR: Actuactor velocity outside of allowed bounds. Setting torque to 0.");
  }
}

/*
Gets the sanitized current for a given motor ID
*/
float get_current(int id, float target, float Kp = 2000.0, float Kd = 100.0) {
  float current = (-Kp * (bus.Get(id).Position() - target)) + (-Kd * bus.Get(id).Velocity());
  sanitize_current_command(current, bus.Get(id).Position(), bus.Get(id).Velocity());
  return current;
}

// This code waits for the user to type s before executing code.
void setup() {
  // Remove all characters that might have been stored up in the serial input buffer prior to running this program
  while (Serial.available()) {
    Serial.read();
  }
  long last_print = millis();
  while (true) {
    char c = Serial.read();
    if (c == 's') {
      Serial.println("Starting code.");
      /* TODO Instantiate ActuatorController for each controller and push_back to vector */
      break;
    }
    if (millis() - last_print > 2000) {
      Serial.println("Press s to start.");
      last_print = millis();
    }
  }
}
void loop() {
  bus.PollCAN();  // Check for messages from the motors.

  long now = millis();

  if (Serial.available()) {
    if (Serial.read() == 's') {
      bus.CommandTorques(0, 0, 0, 0, C610Subbus::kIDZeroToThree);
      bus.CommandTorques(0, 0, 0, 0, C610Subbus::kIDFourToSeven);  // Safety first kids!
      Serial.println("Stopping.");
      while (true) {
      }
    }
  }

  if (now - last_command >= LOOP_DELAY_MILLIS) {
    // logInfo(1, 6);

    float current3 = get_current(3, bus.Get(0).Position());
    Serial.print(bus.Get(1).Position());
    // Serial.print(current4);
    Serial.print("\t");
    float current4 = get_current(4, bus.Get(1).Position());
    Serial.print(bus.Get(2).Position());
    // Serial.print(current5);
    Serial.print("\t");
    float current5 = get_current(5, bus.Get(2).Position());
    Serial.print(bus.Get(3).Position());
    // Serial.print(current6);

    // Only call CommandTorques once per loop! Calling it multiple times will override the last command.
    bus.CommandTorques(0, 0, 0, current3, C610Subbus::kIDZeroToThree);
    // Once you motors with ID=4 to 7, use this command
    bus.CommandTorques(current4, current5, 0, 0, C610Subbus::kIDFourToSeven);

    last_command = now;
    Serial.println();
  }
}