#include "Arduino.h"
#include "C610Bus.h"

#include <vector>

long last_command = 0; // To keep track of when we last commanded the motors
C610Bus<CAN2> bus;     // Initialize the Teensy's CAN bus to talk to the motors

const int LOOP_DELAY_MILLIS = 5; // Wait for 0.005s between motor updates.

const float m1_offset = 0.0;
const float m2_offset = 0.0;

const std::vector<ActuatorController> controllers;

class ActuatorController {
  public:
    float position, velocity, current;

    ActuatorController (float position = 0.0, float velocity = 0.0, float current = 0.0) {
      this->position = position;
      this->velocity = velocity;
      this->current = current;
    }
};

// Step 5. Implement your own PD controller here.
float pd_control(float pos,
                 float vel,
                 float target,
                 float Kp,
                 float Kd)
{
  return (-Kp * (pos - target)) + (-Kd * vel);
}

void sanitize_current_command(float &command,
                              float pos,
                              float vel,
                              float max_current = 2000,
                              float max_pos = 3.141,
                              float max_vel = 30,
                              float reduction_factor = 0.1)
{
  /* Sanitize current command to make it safer.

  Clips current command between bounds. Reduces command if actuator outside of position or velocity bounds.
  Max current defaults to 1000mA. Max position defaults to +-180degs. Max velocity defaults to +-5rotations/s.
  */
  command = command > max_current ? max_current : command;
  command = command < -max_current ? -max_current : command;
  if (pos > max_pos || pos < -max_pos)
  {
    Serial.println("ERROR: Actuator position outside of allowed bounds.");
  }
  if (vel > max_vel || vel < -max_vel)
  {
    command = 0;
    Serial.println("ERROR: Actuactor velocity outside of allowed bounds. Setting torque to 0.");
  }
}

// This code waits for the user to type s before executing code.
void setup()
{
  // Remove all characters that might have been stored up in the serial input buffer prior to running this program
  while (Serial.available()) {
    Serial.read();
  }
  long last_print = millis();
  while (true)
  {
    char c = Serial.read();
    if (c == 's')
    {
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
void loop()
{
  bus.PollCAN(); // Check for messages from the motors.

  long now = millis();

  if (Serial.available())
  {
    if (Serial.read() == 's')
    {
      bus.CommandTorques(0, 0, 0, 0, C610Subbus::kIDZeroToThree);
      Serial.println("Stopping.");
      while (true)
      {
      }
    }
  }

  if (now - last_command >= LOOP_DELAY_MILLIS)
  {
    // TODO Actuator ID indices start at 1, not 0
    float m1_pos = bus.Get(1).Position(); // Get the shaft position of motor 0 in radians.
    float m1_vel = bus.Get(1).Velocity(); // Get the shaft velocity of motor 0 in radians/sec.
    float m1_current = 0.0;
    Serial.print("m1_pos: ");
    Serial.print(m1_pos);
    Serial.print("\tm1_vel: ");
    Serial.print(m1_vel);

    float m2_pos = bus.Get(2).Position(); // Get the shaft position of motor 0 in radians.
    float m2_vel = bus.Get(2).Velocity(); // Get the shaft velocity of motor 0 in radians/sec.
    float m2_current = 0.0;

    float m3_pos = bus.Get(3).Position(); // Get the shaft position of motor 0 in radians.
    float m3_vel = bus.Get(3).Velocity(); // Get the shaft velocity of motor 0 in radians/sec.
    float m3_current = 0.0;

    float m4_pos = bus.Get(4).Position(); // Get the shaft position of motor 0 in radians.
    float m4_vel = bus.Get(4).Velocity(); // Get the shaft velocity of motor 0 in radians/sec.
    float m4_current = 0.0;

    float m5_pos = bus.Get(5).Position(); // Get the shaft position of motor 0 in radians.
    float m5_vel = bus.Get(5).Velocity(); // Get the shaft velocity of motor 0 in radians/sec.
    float m5_current = 0.0;

    float m6_pos = bus.Get(6).Position(); // Get the shaft position of motor 0 in radians.
    float m6_vel = bus.Get(6).Velocity(); // Get the shaft velocity of motor 0 in radians/sec.
    float m6_current = 0.0;


    // Step 8. Change the target position to something periodic
    // float time = millis() / 1000.0; // millis() returns the time in milliseconds since start of program

    // Step 5. Your PD controller is run here.
    float Kp = 1000.0;
    float Kd = 0;
    float target_position = 0.0; // modify in step 8
    m1_current = pd_control(m1_pos, m1_vel, target_position, Kp, Kd);

    // Step 4. Uncomment for bang-bang control. Comment out again before Step 5.
    // if(m0_pos < 0) {
    //   m0_current = 800;
    // } else {
    //   m0_current = -800;
    // }

    // Step 10. Program periodic motion for all three motors.

    // Step 9. Program PID control for the two other motors.
    // Right bottom, middle, top, respectively
    float m1_pos = bus.Get(1).Position();
    float m1_vel = bus.Get(1).Velocity();
    float m2_pos = bus.Get(2).Position();
    float m2_vel = bus.Get(2).Velocity();
    float m3_pos = bus.Get(3).Position();
    float m3_vel = bus.Get(3).Velocity();
    Serial.print("\tm1_pos: ");
    Serial.print(m1_pos);
    Serial.print("\tm1_vel: ");
    Serial.print(m1_vel);
    Serial.print("\tm2_pos: ");
    Serial.print(m2_pos);
    Serial.print("\tm2_vel: ");
    Serial.print(m2_vel);
    Serial.print("\tm3_pos: ");
    Serial.print(m3_pos);
    Serial.print("\tm3_vel: ");
    Serial.print(m3_vel);

    // Left bottom, middle, top, respectively
    float m4_pos = bus.Get(4).Position();
    float m4_vel = bus.Get(4).Velocity();
    float m5_pos = bus.Get(5).Position();
    float m5_vel = bus.Get(5).Velocity();
    float m6_pos = bus.Get(6).Position();
    float m6_vel = bus.Get(6).Velocity();
    Serial.print("\tm4_pos: ");
    Serial.print(m4_pos);
    Serial.print("\tm4_vel: ");
    Serial.print(m4_vel);
    Serial.print("\tm5_pos: ");
    Serial.print(m5_pos);
    Serial.print("\tm5_vel: ");
    Serial.print(m5_vel);
    Serial.print("\tm6_pos: ");
    Serial.print(m6_pos);
    Serial.print("\tm6_vel: ");
    Serial.print(m6_vel);
    // m1_current = YOUR PID CODE
    // m2_current = YOUR PID CODE

    // Sanitizes your computed current commands to make the robot safer.
    sanitize_current_command(m1_current, m1_pos, m1_vel);
    sanitize_current_command(m2_current, m2_pos, m2_vel);
    sanitize_current_command(m3_current, m3_pos, m3_vel);
    
    sanitize_current_command(m4_current, m4_pos, m4_vel);
    sanitize_current_command(m5_current, m5_pos, m5_vel);
    sanitize_current_command(m6_current, m6_pos, m6_vel);

    // Only call CommandTorques once per loop! Calling it multiple times will override the last command.
    bus.CommandTorques(m1_current, m1_current, m1_current, 0, C610Subbus::kIDZeroToThree);
    // Once you motors with ID=4 to 7, use this command
    // bus.CommandTorques(0, 0, 0, 0, C610Subbus::kIDFourToSeven);

    last_command = now;
    Serial.println();
  }
}