/**
   spider_robot
*/

// includes.
#include <Servo.h>
#include <FlexiTimer2.h>

// constants.
const int THIGH = 0;
const int SHIN = 1;
const int SHOULDER = 2;
const int FORWARD = 0;
const int BACKWARD = 1;
const int LEFT = 0;
const int RIGHT = 1;

// initialize servo object.
Servo g_servo[4][3];

// set servo pins; thigh, shin and shoulder.
const int g_servo_pin[4][3] = { {2, 3, 4}, {5, 6, 7}, {8, 9, 10}, {11, 12, 13} };

// robot properties.
const float g_thigh_length = 55;
const float g_shin_length = 77.5;
const float g_shoulder_length = 28;
const float g_shoulder_default_angle = 90.00;
const float g_body_clearance = 30;
const float g_extent = 77.5;
const float g_leg_clearance = 10;
const int g_horizontal_location[4] = { RIGHT, RIGHT, LEFT, LEFT };
const int g_speed = 5;

// initial values.
const float g_initial_servo_angle[4][3] = { {34.92, 49.23, 82.65}, {34.92, 49.23, 97.35}, {40.45, 62.46, 57.17}, {40.45, 62.46, 122.83} };
//const float g_initial_thigh_angle = 84.55;
//const float g_initial_shin_angle = 78.00;
//const float g_initial_alpha_reference = 78.00;
//const float g_initial_beta_reference = 25.97;


// servo angles; current and target; thigh, shin and shoulder.
float g_servo_angle_current[4][3];
float g_servo_angle_target[4][3];

// servo increments.
float g_servo_increment[4][3];

// leg positions.
float g_leg_position[4];

/**
   Setup.
*/
void setup()
{
  // start servo controller service.
  FlexiTimer2::set(20, servo_service);
  FlexiTimer2::start();

  // attach_servos,
  attach_servos();

  // initialize legs.
  initialize();
}

/**
   Initialize legs.
*/
void initialize()
{
  // initialize leg positions.
  g_leg_position[0] = -10;
  g_leg_position[1] = 10;
  g_leg_position[2] = 50;
  g_leg_position[3] = -50;

  // go through legs.
  for (int leg_index = 0; leg_index < 4; leg_index++)
  {
    // initialize leg.
    set_servo_values(leg_index, g_initial_servo_angle[leg_index][THIGH], g_initial_servo_angle[leg_index][SHIN], calculate_shoulder_angle(leg_index, g_leg_position[leg_index]));
  }

  // wait until all legs has reached target.
  wait_all_reach();
}

/**
   Attach servos.
*/
void attach_servos(void)
{
  // go through legs.
  for (int leg_index = 0; leg_index < 4; leg_index++)
  {
    // go though servos.
    for (int servo_index = 0; servo_index < 3; servo_index++)
    {
      // attach servo.
      g_servo[leg_index][servo_index].attach(g_servo_pin[leg_index][servo_index]);
      delay(100);
    }
  }
}

/**
   Detach servos.
*/
void detach_servos(void)
{
  // go through legs.
  for (int leg_index = 0; leg_index < 4; leg_index++)
  {
    // go though servos.
    for (int servo_index = 0; servo_index < 3; servo_index++)
    {
      // detach servo.
      g_servo[leg_index][servo_index].detach();
      delay(100);
    }
  }
}

/**
   Loop.
*/
void loop()
{
  // walk.
  walk();
}

/**
   Walk.
*/
void walk()
{
  // lift leg 0.
  lift_leg(0);

  // wait until leg has reached target.
  wait_reach(0);

  // move leg 0 forward 120 units.
  move_leg(0, FORWARD, 120);

  // wait until leg has reached target.
  wait_reach(0);

  // move body forward 60 units.
  move_body(FORWARD, 60);

  // wait until all legs have reached target.
  wait_all_reach();

  // move leg 3 forward 120 units.
  move_leg(3, FORWARD, 120);

  // wait until leg has reached target.
  wait_reach(3);

  // move leg 2 forward 120 units.
  move_leg(2, FORWARD, 120);

  // wait until leg has reached target.
  wait_reach(2);

  // move body forward 60 units.
  move_body(FORWARD, 60);

  // wait until all legs have reached target.
  wait_all_reach();

  // move leg 1 forward 120 units.
  move_leg(1, FORWARD, 120);

  // wait until leg has reached target.
  wait_reach(1);
}

/**
   Lift leg.
*/
void lift_leg(int leg)
{
  // set servo increments; lift leg.
  set_servo_increments(leg, g_servo_angle_current[leg][THIGH] - g_leg_clearance, g_servo_angle_current[leg][SHIN], g_servo_angle_current[leg][SHOULDER]);

  // set servo values; lift leg.
  set_servo_values(leg, g_servo_angle_current[leg][THIGH] - g_leg_clearance, g_servo_angle_current[leg][SHIN], g_servo_angle_current[leg][SHOULDER]);
}

/**
   Move leg.
*/
void move_leg(int leg, int direction, float distance)
{
  // variables.
  float new_leg_position;
  float thigh_angle_target;
  float shin_angle_target;
  float shoulder_angle_target;

  // direction is forward.
  if (direction == FORWARD)
  {
    // increase leg position with distance.
    new_leg_position = g_leg_position[leg] + distance;
  }
  // direction is backward.
  else
  {
    // decrease leg position with distance.
    new_leg_position = g_leg_position[leg] - distance;
  }

  // calculate shin angle target.
  shin_angle_target = calculate_shin_angle(new_leg_position);

  // calculate thigh angle target.
  thigh_angle_target = calculate_thigh_angle(leg, shin_angle_target);

  // calculate shoulder angle target.
  shoulder_angle_target = calculate_shoulder_angle(leg, new_leg_position);

  // set servo increments.
  set_servo_increments(leg, thigh_angle_target, shin_angle_target, shoulder_angle_target);

  // set servo values.
  set_servo_values(leg, thigh_angle_target, shin_angle_target, shoulder_angle_target);

  // update leg position.
  g_leg_position[leg] = new_leg_position;
}

/**
   Move body.
*/
void move_body(int direction, float distance)
{
  // variables.
  int leg_direction;

  // body move direction is forward.
  if (direction == FORWARD)
  {
    // set leg move direction to backward.
    leg_direction = BACKWARD;
  }
  // body move direction is backward.
  else
  {
    // set leg move direction to forward.
    leg_direction = FORWARD;
  }

  // go through legs.
  for (int leg_index = 0; leg_index < 4; leg_index++)
  {
    move_leg(leg_index, leg_direction, distance);
  }
}

/**
   Calculate shin angle.

   Shin angle is calculated based on leg length, thigh length and shin length.
*/
float calculate_shin_angle(float destination)
{
  // variables.
  float leg_distance;
  float leg_length;
  float shin_angle;

  // calculate leg distance; substract shoulder length.
  leg_distance = sqrt(pow(g_extent, 2) + pow(abs(destination), 2));
  leg_distance = leg_distance - g_shoulder_length;

  // calculate leg length.
  leg_length = sqrt(pow(g_body_clearance, 2) + pow(leg_distance, 2));

  // calculate shin angle.
  shin_angle = calculate_angle(leg_length, g_shin_length, g_thigh_length);

  // return shin angle.
  return shin_angle;
}

/**
   Calculate thigh angle.

   Thigh angle is changed relatively to the the change in the shin angle.
*/
float calculate_thigh_angle(int leg, float shin_angle_target)
{
  // variables.
  float thigh_angle;

  // calculate thigh angle; add difference in shin angle.
  thigh_angle = g_servo_angle_current[leg][THIGH] + (g_servo_angle_current[leg][SHIN] - shin_angle_target);

  // return thigh angle.
  return thigh_angle;
}

/**
   Calculate shoulder angle.
*/
float calculate_shoulder_angle(int leg, float destination)
{
  // variables.
  float degrees_angle;
  float shoulder_angle;

  // calculate angle.
  degrees_angle = radians_to_degrees(atan(abs(destination) / g_extent));

  // left leg.
  if (g_horizontal_location[leg] == 0)
  {
    // positive destination.
    if (destination >= 0)
    {
      // set shoulder angle.
      shoulder_angle = g_shoulder_default_angle - degrees_angle;
    }
    // negative destination.
    else
    {
      // set shoulder angle.
      shoulder_angle = g_shoulder_default_angle + degrees_angle;
    }
  }
  // right leg.
  else
  {
    // positive destination.
    if (destination >= 0)
    {
      // set shoulder angle.
      shoulder_angle = g_shoulder_default_angle + degrees_angle;
    }
    // negative destination.
    else
    {
      // set shoulder angle.
      shoulder_angle = g_shoulder_default_angle - degrees_angle;
    }
  }

  // return shoulder angle.
  return shoulder_angle;
}

/**
   Set servo increments.
*/
void set_servo_increments(int leg, float thigh_angle_target, float shin_angle_target, float shoulder_angle_target)
{
  // variables.
  float thigh_movement;
  float shin_movement;
  float shoulder_movement;

  // calculate servo movements.
  thigh_movement = thigh_angle_target - g_servo_angle_current[leg][THIGH];
  shin_movement = shin_angle_target - g_servo_angle_current[leg][SHIN];
  shoulder_movement = shoulder_angle_target - g_servo_angle_current[leg][SHOULDER];

  // calculate servo increments.
  g_servo_increment[leg][THIGH] = thigh_movement / g_speed;
  g_servo_increment[leg][SHIN] = shin_movement / g_speed;
  g_servo_increment[leg][SHOULDER] = shoulder_movement / g_speed;
}

/**
   Set servo values.
*/
void set_servo_values(int leg, float thigh_angle_target, float shin_angle_target, float shoulder_angle_target)
{
  // set servo values.
  g_servo_angle_target[leg][THIGH] = thigh_angle_target;
  g_servo_angle_target[leg][SHIN] = shin_angle_target;
  g_servo_angle_target[leg][SHOULDER] = shoulder_angle_target;
}

/**
   Servo service.
*/
void servo_service()
{
  // go through legs.
  for (int leg_index = 0; leg_index < 4; leg_index++)
  {
    // go through servos.
    for (int servo_index = 0; servo_index < 3; servo_index++)
    {
      // servo has not reached the target.
      if (abs(g_servo_angle_target[leg_index][servo_index] - g_servo_angle_current[leg_index][servo_index]) >= abs(g_servo_increment[leg_index][servo_index]))
      {
        // increase/decrease current value with servo increment.
        g_servo_angle_current[leg_index][servo_index] = g_servo_angle_current[leg_index][servo_index] + g_servo_increment[leg_index][servo_index];
      }
      // servo has reach the target.
      else
      {
        // set target value to current value.
        g_servo_angle_current[leg_index][servo_index] = g_servo_angle_target[leg_index][servo_index];
      }

      // write values to servo.
      write_values_to_servo(leg_index, g_servo_angle_current[leg_index][THIGH], g_servo_angle_current[leg_index][THIGH], g_servo_angle_current[leg_index][THIGH]);
    }
  }
}

/**
   Write values to servo.
*/
void write_values_to_servo(int leg, float thigh_angle, float shin_angle, float shoulder_angle)
{
  // write values to servo.
  g_servo[leg][THIGH].write(thigh_angle);
  g_servo[leg][SHIN].write(shin_angle);
  g_servo[leg][SHOULDER].write(shoulder_angle);
}

/**
   Wait until leg has reached target.
*/
void wait_reach(int leg)
{
  // repeat until all servos in leg have reached targets.
  while (true)
  {
    if (g_servo_angle_current[leg][THIGH] == g_servo_angle_target[leg][THIGH] &&
        g_servo_angle_current[leg][SHIN] == g_servo_angle_target[leg][SHIN] &&
        g_servo_angle_current[leg][SHOULDER] == g_servo_angle_target[leg][SHOULDER] )
    {
      // break the loop.
      break;
    }
  }
}

/**
   Wait until all legs have reached target.
*/
void wait_all_reach()
{
  // go through all legs.
  for (int leg_index = 0; leg_index < 4; leg_index++)
  {
    // wait until leg has reached target.
    wait_reach(leg_index);
  }
}

/**
   Calculate angle based on three sides of a triangle.
*/
float calculate_angle(float side_a, float side_b, float side_c)
{
  // calculate and return angle.
  return acos((pow(side_b, 2) + pow(side_c, 2) - pow(side_a, 2)) / (2 * side_b * side_c));
}

/**
   Convert radians to degrees.
*/
float radians_to_degrees(float radians)
{
  // calculate and return degrees.
  return radians * 57296 / 1000;
}

/**
   Convert degrees to radians.
*/
float degrees_to_radians(float degrees)
{
  // calculate and return radians.
  return degrees * 1000 / 57296;
}
