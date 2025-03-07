#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "pros/vision.hpp"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;
const double red[2] = {3.0,12.0};
const double blue[2] = {214.0, 224.0};

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  // https://ez-robotics.github.io/EZ-Template/tutorials/tuning_constants
  
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions
  
  /*chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         
  chassis.pid_heading_constants_set(11, 0.0, 20);        
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           
  chassis.pid_odom_angular_constants_set(0.05, 0.0, 0.25);   
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  
  */

  // Exit conditions
  // https://ez-robotics.github.io/EZ-Template/tutorials/tuning_exit_conditions
  chassis.pid_turn_exit_condition_set(90_ms, 2_deg, 250_ms, 5_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 2_deg, 250_ms, 5_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 2_deg, 250_ms, 5_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  //chassis.pid_odom_turn_exit_condition_set(90_ms, 1_deg, 250_ms, 3_deg, 500_ms, 750_ms);
  //chassis.pid_odom_drive_exit_condition_set(90_ms, 0.5_in, 250_ms, 1.5_in, 500_ms, 750_ms); // Adjusted values

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  // https://ez-robotics.github.io/EZ-Template/tutorials/slew_constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  // chassis.odom_turn_bias_set(0.9);
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there

}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.
  chassis.pid_odom_set(48_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-24_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-24_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 30 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .

void joyce_com_blue_left(pros::Motor intakeMotorTop, ez::Piston clamp_piston, pros::Optical optical_sensor, pros::Task intake_control_task, pros::Motor liftMotor){
  intake_control_task.suspend();
  intakeMotorTop.move_velocity(0);
  clamp_piston.button_toggle(0);
  chassis.pid_odom_set(-24_in, 60,true);
  chassis.pid_wait();
  clamp_piston.button_toggle(1);
  pros::delay(400);  
  clamp_piston.button_toggle(0);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait(); 
  intake_control_task.resume();
  chassis.pid_odom_set(18_in, DRIVE_SPEED,true);
  chassis.pid_wait();   
  pros::delay(1200);   
  chassis.pid_turn_set(-63_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set(46_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  pros::delay(1700);
  chassis.pid_odom_set(18_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  pros::delay(1400);    
  chassis.pid_odom_set(-10_in, DRIVE_SPEED,true);
  chassis.pid_wait(); 
  liftMotor.move_velocity(70);
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  liftMotor.move_velocity(0);
  chassis.pid_odom_set(8_in, DRIVE_SPEED,true);
  chassis.pid_wait_quick_chain();
  intake_control_task.suspend();  
  chassis.pid_odom_set(6_in, 50,true);
  chassis.pid_wait();  
}

void joyce_com_blue_right(pros::Motor intakeMotorTop, ez::Piston clamp_piston, pros::Optical optical_sensor, pros::Task intake_control_task, pros::Motor liftMotor){
  intake_control_task.suspend();
  intakeMotorTop.move_velocity(0);
  clamp_piston.button_toggle(0);
  chassis.pid_odom_set(-24_in,60,true);
  chassis.pid_wait();
  clamp_piston.button_toggle(1);
  pros::delay(400);  
  clamp_piston.button_toggle(0);
  chassis.pid_turn_set(-136_deg, TURN_SPEED);
  chassis.pid_wait(); 
  intake_control_task.resume();
  chassis.pid_odom_set(16_in, DRIVE_SPEED,true);
  chassis.pid_wait();  
  chassis.pid_turn_set(-91_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(19_in, DRIVE_SPEED,true);
  chassis.pid_wait();   
  chassis.pid_swing_set(ez::LEFT_SWING, -135_deg, SWING_SPEED, 45);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, -90_deg, SWING_SPEED, 45);
  chassis.pid_wait();
  chassis.pid_odom_set(27_in, DRIVE_SPEED,true);
  chassis.pid_wait();   
  liftMotor.move_velocity(70);
  chassis.pid_turn_set(-257_deg, TURN_SPEED);
  chassis.pid_wait(); 
  liftMotor.move_velocity(0); 
  chassis.pid_odom_set(31_in, 80,true);
  chassis.pid_wait(); 
  intake_control_task.suspend(); 
}

void joyce_com_red_right(pros::Motor intakeMotorTop, ez::Piston clamp_piston, pros::Optical optical_sensor, pros::Task intake_control_task, pros::Motor liftMotor){
  intake_control_task.suspend();
  clamp_piston.button_toggle(0);
  intakeMotorTop.move_velocity(0);
  chassis.pid_odom_set(-24_in, 60,true);
  chassis.pid_wait();
  clamp_piston.button_toggle(1);
  pros::delay(400);  
  clamp_piston.button_toggle(0);
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait(); 
  intake_control_task.resume();
  chassis.pid_odom_set(16_in, DRIVE_SPEED,true);
  chassis.pid_wait(); 
  pros::delay(1200);    
  chassis.pid_turn_set(63_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set(43_in, 80,true);
  chassis.pid_wait();
  pros::delay(1700); 
  chassis.pid_odom_set(14_in, DRIVE_SPEED,true);
  chassis.pid_wait(); 
  pros::delay(1400); 
  chassis.pid_odom_set(-14_in, DRIVE_SPEED,true);
  chassis.pid_wait(); 
  liftMotor.move_velocity(70);
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();  
  liftMotor.move_velocity(0);
  chassis.pid_odom_set(6_in, DRIVE_SPEED,true);
  chassis.pid_wait_quick_chain();
  intake_control_task.suspend();
  chassis.pid_odom_set(5_in, 50,true);
  chassis.pid_wait(); 
/*  intake_control_task.suspend();
  clamp_piston.button_toggle(0);
  intakeMotorTop.move_velocity(0);
  chassis.pid_odom_set(28_in, 60,true);
  chassis.pid_wait();
  chassis.pid_swing_set(ez::LEFT_SWING, -90_deg, SWING_SPEED, 45);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-15_in, 70,true);
  chassis.pid_wait();
  clamp_piston.button_toggle(1);
  pros::delay(400);  
  clamp_piston.button_toggle(0);
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait(); 
  intake_control_task.resume();
  chassis.pid_odom_set(18_in, DRIVE_SPEED,true);
  chassis.pid_wait();    
  chassis.pid_turn_set(330_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set(30_in, DRIVE_SPEED,true);
  chassis.pid_wait_quick_chain();
  clamp_piston.button_toggle(1);
  pros::delay(200); 
  clamp_piston.button_toggle(0);
  chassis.pid_odom_set(15_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  pros::delay(800);    
  intake_control_task.suspend();  
  intakeMotorTop.move_velocity(0);  
  chassis.pid_odom_set(15_in, 80,true);
  chassis.pid_wait(); 
  int timer_start = pros::millis();
  while (optical_sensor.get_hue() < red[0] || optical_sensor.get_hue() > red[1]) {
    intakeMotorTop.move_velocity(-400); 
    if (pros::millis() - timer_start > 1500) 
      break;
  }
  intakeMotorTop.move_velocity(0); 
  chassis.pid_odom_set(-14_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set(-17_in, 60,true);
  chassis.pid_wait(); 
  intakeMotorTop.move_velocity(-400);     
  pros::delay(800); 
  intakeMotorTop.move_velocity(0);  
  chassis.pid_odom_set(30_in, DRIVE_SPEED,true);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(10_in, 60,true);
  chassis.pid_wait();*/
}

void joyce_com_red_left(pros::Motor intakeMotorTop, ez::Piston clamp_piston, pros::Optical optical_sensor, pros::Task intake_control_task, pros::Motor liftMotor){
  intake_control_task.suspend();
  intakeMotorTop.move_velocity(0);
  clamp_piston.button_toggle(0);
  chassis.pid_odom_set(-23_in,60,true);
  chassis.pid_wait();
  clamp_piston.button_toggle(1);
  pros::delay(400);  
  clamp_piston.button_toggle(0);
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait(); 
  intake_control_task.resume();
  chassis.pid_odom_set(15_in, DRIVE_SPEED,true);
  chassis.pid_wait();   
  chassis.pid_turn_set(88_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(12_in, DRIVE_SPEED,true);
  chassis.pid_wait();   
  chassis.pid_swing_set(ez::RIGHT_SWING, 135_deg, SWING_SPEED, 45);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::LEFT_SWING, 94_deg, SWING_SPEED, 45);
  chassis.pid_wait();
  chassis.pid_odom_set(22_in, DRIVE_SPEED,true);
  chassis.pid_wait();   
  liftMotor.move_velocity(70);
  chassis.pid_turn_set(257_deg, TURN_SPEED);
  chassis.pid_wait(); 
  liftMotor.move_velocity(0); 
  chassis.pid_odom_set(24_in, 80,true);
  chassis.pid_wait(); 
  intake_control_task.suspend(); 
}

void joyce_com_skill(pros::Motor intakeMotorTop, ez::Piston clamp_piston){

  intakeMotorTop.move_velocity(-400);
  pros::delay(800);  
  intakeMotorTop.move_velocity(0);
  chassis.pid_odom_set(18_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(-28_in, 70,true);
  chassis.pid_wait();
  clamp_piston.button_toggle(0);
  clamp_piston.button_toggle(1);
  pros::delay(200);  
  clamp_piston.button_toggle(0);
  chassis.pid_odom_set(4_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  intakeMotorTop.move_velocity(-400);
  chassis.pid_odom_set(16_in, DRIVE_SPEED,true);
  chassis.pid_wait(); 
  pros::delay(400); 
  chassis.pid_turn_set(56_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(43_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(207_deg, TURN_SPEED);
  chassis.pid_wait();  
  chassis.pid_odom_set(30_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set(40_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_odom_set(-20_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set(15_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_odom_set(-15_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set(-34_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  clamp_piston.button_toggle(1);
  pros::delay(200);  
  clamp_piston.button_toggle(0);

  intakeMotorTop.move_velocity(0);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(-72_in, 70,true);
  chassis.pid_wait(); 
  clamp_piston.button_toggle(1);
  pros::delay(200);  
  clamp_piston.button_toggle(0);
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait(); 
  intakeMotorTop.move_velocity(-400);
  chassis.pid_odom_set(24_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(-56_deg, TURN_SPEED);
  chassis.pid_wait();     
  chassis.pid_odom_set(43_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();   
  chassis.pid_odom_set(-12_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(50_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_odom_set(-12_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(12_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(63_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(-24_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  clamp_piston.button_toggle(1);
  pros::delay(200);  
  clamp_piston.button_toggle(0); 
  intakeMotorTop.move_velocity(0);

  chassis.pid_odom_set(10_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(72_in, DRIVE_SPEED,true);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, SWING_SPEED, 45);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(48_in, DRIVE_SPEED,true);
  chassis.pid_wait(); 
  chassis.pid_turn_set(225_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(-10_in, 70,true);
  chassis.pid_wait();
  clamp_piston.button_toggle(1);
  pros::delay(200);  
  clamp_piston.button_toggle(0); 
  intakeMotorTop.move_velocity(-400);
  chassis.pid_odom_set(20_in, DRIVE_SPEED,true);
  chassis.pid_wait(); 
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(20_in, DRIVE_SPEED,true);
  chassis.pid_wait(); 
  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(20_in, DRIVE_SPEED,true);
  chassis.pid_wait(); 
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(20_in, DRIVE_SPEED,true);
  chassis.pid_wait(); 
  chassis.pid_turn_set(203_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(-50_in, DRIVE_SPEED,true);
  chassis.pid_wait(); 
  clamp_piston.button_toggle(1);
  pros::delay(200);  
  clamp_piston.button_toggle(0); 
  
  intakeMotorTop.move_velocity(0);
  chassis.pid_odom_set(24_in, DRIVE_SPEED,true);
  chassis.pid_wait(); 
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(48_in, 125,true);
  chassis.pid_wait(); 
  chassis.pid_turn_set(-70_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(72_in, 125,true);
  chassis.pid_wait(); 
}

void joyce_com_skill_stuck(pros::Motor intakeMotorTop, ez::Piston clamp_piston, pros::Task intake_control_task, pros::Motor liftMotor){

  intake_control_task.suspend();
  intakeMotorTop.move_velocity(-400);
  pros::delay(600);  
  intakeMotorTop.move_velocity(0);
  clamp_piston.button_toggle(0);
  chassis.pid_odom_set(12_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(-27_in, 70,true);
  chassis.pid_wait();
  clamp_piston.button_toggle(1);
  pros::delay(100);  
  clamp_piston.button_toggle(0);
  chassis.pid_odom_set(4_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  intakeMotorTop.move_velocity(-400);
  intake_control_task.resume();
  chassis.pid_odom_set(22_in, DRIVE_SPEED,true);
  chassis.pid_wait(); 
  chassis.pid_turn_set(59_deg, TURN_SPEED);
  chassis.pid_wait();
  liftMotor.tare_position();
  intake_control_task.suspend();   
	liftMotor.move_absolute(55, 120); //ready position to shoot step 1
  pros::delay(260); 
  liftMotor.move_absolute(75, 70);  //ready position to shoot step 2
  pros::delay(180); 
  intakeMotorTop.move_velocity(-400);
  chassis.pid_odom_set(37.5_in, DRIVE_SPEED,true);
  chassis.pid_wait();
/*  int timer_start = pros::millis();
  while (intakeMotorTop.get_actual_velocity()!=0) {
    intakeMotorTop.move_velocity(-400); 
    if (pros::millis() - timer_start > 800) 
      break;
  }
  intakeMotorTop.move_velocity(0);*/
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(3_in, 70,true);
  chassis.pid_wait();
  intakeMotorTop.move(1200);
  pros::delay(80);        
  intakeMotorTop.move_velocity(0);   
  liftMotor.move_velocity(127); 
  pros::delay(630);     
  liftMotor.move_velocity(0);
  chassis.pid_odom_set(-12_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  intakeMotorTop.move_velocity(-400);
  intake_control_task.resume();
  liftMotor.move_velocity(-50);
  chassis.pid_turn_set(179_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(23_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  liftMotor.move_velocity(0); 
  pros::delay(360); 
  chassis.pid_odom_set(22_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  pros::delay(360);
  chassis.pid_odom_set(14_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  pros::delay(310); 
  chassis.pid_odom_set(-21_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait(); 
  //liftMotor.move_velocity(0); 
  chassis.pid_odom_set(16_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  pros::delay(220); 
  chassis.pid_odom_set(-18_in, DRIVE_SPEED,true);
  chassis.pid_wait(); 
  chassis.pid_turn_set(-28_deg, TURN_SPEED);
  chassis.pid_wait();
  //liftMotor.move_velocity(-50);
  chassis.pid_odom_set(-33_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  //liftMotor.move_velocity(0);
  clamp_piston.button_toggle(1);
  pros::delay(100); 
  clamp_piston.button_toggle(0);
  intake_control_task.suspend();
  intakeMotorTop.move_velocity(200);
  pros::delay(170);  
  intakeMotorTop.move_velocity(0);
  clamp_piston.button_toggle(0);

  chassis.pid_odom_set(11_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(88_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(-62_in, DRIVE_SPEED,true);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-8_in, 60,true);
  chassis.pid_wait(); 
  clamp_piston.button_toggle(1);
  pros::delay(100);  
  clamp_piston.button_toggle(0);
  pros::delay(80);  
  intakeMotorTop.move_velocity(-400);
  intake_control_task.resume();
  //chassis.pid_odom_set(2_in, 90,true);
  //chassis.pid_wait();   
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(19_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(-60_deg, TURN_SPEED);
  chassis.pid_wait();  
  chassis.pid_odom_set(35_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  pros::delay(400); 
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();  
  chassis.pid_odom_set(-6_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set(17_in, 100,true);
  chassis.pid_wait();
  pros::delay(360); 
  chassis.pid_odom_set(22_in, 100,true);
  chassis.pid_wait();
  pros::delay(360); 
  chassis.pid_odom_set(13_in, 100,true);
  chassis.pid_wait(); 
  pros::delay(360); 
  chassis.pid_odom_set(-26_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  pros::delay(300); 
  chassis.pid_turn_set(-145_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set(16_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  pros::delay(300); 
  chassis.pid_odom_set(-12_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(23_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(-31_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  clamp_piston.button_toggle(1);
  intake_control_task.suspend();
  intakeMotorTop.move_velocity(200);
  pros::delay(310);  
  intakeMotorTop.move_velocity(0);
  clamp_piston.button_toggle(0);
  chassis.pid_odom_set(30_in, DRIVE_SPEED,true);
  chassis.pid_wait(); 
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  intakeMotorTop.move_velocity(-400);
  intake_control_task.resume();
  chassis.pid_odom_set(48_in, DRIVE_SPEED,true);
  chassis.pid_wait();  
  intake_control_task.suspend();
  intakeMotorTop.move_velocity(0);
  chassis.pid_turn_set(-117_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(-41_in, DRIVE_SPEED,true);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-12_in, 65,true);
  chassis.pid_wait();   
  clamp_piston.button_toggle(1);
  pros::delay(180);  
  clamp_piston.button_toggle(0); 
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait(); 
  intakeMotorTop.move_velocity(-400);
  intake_control_task.resume();
  chassis.pid_odom_set(34_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  pros::delay(160);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(26_in, DRIVE_SPEED,true);
  chassis.pid_wait(); 
  pros::delay(100); 
  chassis.pid_turn_set(-64_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  clamp_piston.button_toggle(1);
  intake_control_task.suspend();
  intakeMotorTop.move_velocity(0);
  clamp_piston.button_toggle(0);
  chassis.pid_odom_set(120_in, 127,true);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-30_in, 127,true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(75_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(81_in, 127,true);
  chassis.pid_wait_quick_chain();
  intakeMotorTop.move_velocity(200);
  chassis.pid_odom_set(-50_in, 127,true);
  chassis.pid_wait();  
  /*chassis.pid_odom_set(11_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  liftMotor.move_velocity(0);
  chassis.pid_turn_set(88_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(-65_in, DRIVE_SPEED,true);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-7_in, 60,true);
  chassis.pid_wait(); 
  clamp_piston.button_toggle(1);
  pros::delay(200);  
  clamp_piston.button_toggle(0);
  intakeMotorTop.move_velocity(-400);
  intake_control_task.resume();
  chassis.pid_odom_set(2_in, 90,true);
  chassis.pid_wait();   
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(20_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(-59_deg, TURN_SPEED);
  chassis.pid_wait(); 
  liftMotor.tare_position(); 
  intake_control_task.suspend();   
  liftMotor.tare_position();
  liftMotor.move_absolute(70, 125); 
  chassis.pid_odom_set(20_in, DRIVE_SPEED,true);
  chassis.pid_wait_quick_chain();
  intakeMotorTop.move_velocity(-400);
  chassis.pid_odom_set(16_in, 80,true);
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(3_in, 60,true);
  chassis.pid_wait();
  intakeMotorTop.move(1200);
  pros::delay(90);        
  intakeMotorTop.move_velocity(0);   
  liftMotor.move_velocity(105); 
  pros::delay(600);     
  liftMotor.move_velocity(0);
  chassis.pid_odom_set(-13_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  intakeMotorTop.move_velocity(-400);
  intake_control_task.resume();
  chassis.pid_turn_set(182_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(22_in, 100,true);
  chassis.pid_wait();
  pros::delay(400); 
  chassis.pid_odom_set(21_in, 100,true);
  chassis.pid_wait();
  pros::delay(400); 
  chassis.pid_odom_set(12_in, 100,true);
  chassis.pid_wait(); 
  pros::delay(300); 
*/

  /*chassis.pid_turn_set(203_deg, TURN_SPEED);
  chassis.pid_wait();  
  chassis.pid_odom_set(23_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  pros::delay(400); 
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set(22_in, 80,true);
  chassis.pid_wait();
  pros::delay(500);
  chassis.pid_odom_set(13_in, 80,true);
  chassis.pid_wait();
  pros::delay(500);
  chassis.pid_odom_set(-23_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(138_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set(16_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  pros::delay(500); 
  chassis.pid_odom_set(-18_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(-27_deg, TURN_SPEED);
  chassis.pid_wait();  
  chassis.pid_odom_set(-32_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  clamp_piston.button_toggle(1);
  intake_control_task.suspend();
  intakeMotorTop.move_velocity(200);
  pros::delay(400);  
  intakeMotorTop.move_velocity(0);
  clamp_piston.button_toggle(0);

  chassis.pid_odom_set(11_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(88_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(-65_in, DRIVE_SPEED,true);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-7_in, 70,true);
  chassis.pid_wait(); 
  clamp_piston.button_toggle(1);
  pros::delay(200);  
  clamp_piston.button_toggle(0);
  intakeMotorTop.move_velocity(-400);
  intake_control_task.resume();
  chassis.pid_odom_set(2_in, 90,true);
  chassis.pid_wait();   
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(20_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(-59_deg, TURN_SPEED);
  chassis.pid_wait();  
  chassis.pid_odom_set(35_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  pros::delay(700); 
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();   
  chassis.pid_odom_set(-6_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(16_in, 100,true);
  chassis.pid_wait();
  pros::delay(500); 
  chassis.pid_odom_set(22_in, 100,true);
  chassis.pid_wait();
  pros::delay(600); 
  chassis.pid_odom_set(13_in, 100,true);
  chassis.pid_wait(); 
  pros::delay(400); 
  chassis.pid_odom_set(-24_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  pros::delay(400); 
  chassis.pid_turn_set(-145_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set(16_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  pros::delay(400); 
  chassis.pid_odom_set(-12_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(23_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(-31_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  clamp_piston.button_toggle(1);
  intake_control_task.suspend();
  intakeMotorTop.move_velocity(200);
  pros::delay(400);  
  intakeMotorTop.move_velocity(0);
  clamp_piston.button_toggle(0);
  chassis.pid_odom_set(30_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait(); 
  intake_control_task.resume();
  chassis.pid_odom_set(48_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  pros::delay(100);  
  intake_control_task.suspend();
  chassis.pid_turn_set(-116_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(-54_in, 80,true);
  chassis.pid_wait(); 
  clamp_piston.button_toggle(1);
  pros::delay(200);  
  clamp_piston.button_toggle(0); 
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait(); 
  intake_control_task.resume();
  chassis.pid_odom_set(35_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  pros::delay(200);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set(26_in, DRIVE_SPEED,true);
  chassis.pid_wait(); 
  pros::delay(200); 
  chassis.pid_turn_set(-63_deg, TURN_SPEED);
  chassis.pid_wait();
  intake_control_task.suspend();
  clamp_piston.button_toggle(1);
  clamp_piston.button_toggle(0);
  chassis.pid_odom_set(122_in, 122,true);
  chassis.pid_wait();
  chassis.pid_odom_set(-40_in, DRIVE_SPEED,true);
  chassis.pid_wait();
  chassis.pid_turn_set(73_deg, TURN_SPEED);
  chassis.pid_wait(); 
  chassis.pid_odom_set(106_in, 122,true);
  chassis.pid_wait();*/
}

void odom_drive_example2() {
    
    chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);

    // Stricter exit conditions
    //chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 0.5_in, 500_ms, 750_ms);

    // Perform motion
    
    chassis.pid_odom_set(48_in, DRIVE_SPEED, true); 
    chassis.pid_wait(); 
    chassis.pid_odom_set(-24_in, DRIVE_SPEED, true); 
    chassis.pid_wait();
    chassis.pid_odom_set(-24_in, DRIVE_SPEED, true); 
    chassis.pid_wait();
    //chassis.pid_turn_set(90_deg, TURN_SPEED);
    //chassis.pid_wait();
    //chassis.pid_odom_set(24_in, DRIVE_SPEED, true); 
    chassis.pid_wait(); 

    /*
    left_motor_1.move(100);  // Set left motor to full power
    right_motor_1.move(100); // Set right motor to full power
    pros::delay(2000);       // Run for 2 seconds
    left_motor_1.move(0);    // Stop left motor
    right_motor_1.move(0);   // Stop right motor
    */
/*
chassis.pid_odom_set({{{12_in, 20_in}, fwd, DRIVE_SPEED}}, true);
//chassis.pid_wait_quick_chain();
chassis.pid_wait();
//pros::delay(1000);  
chassis.pid_odom_set({{{0_in, 30_in}, fwd, DRIVE_SPEED}}, true);
//chassis.pid_wait_quick_chain();
chassis.pid_wait();
//pros::delay(1000);  
chassis.pid_odom_set({{{0_in, 50_in}, fwd, DRIVE_SPEED}}, true);
//chassis.pid_wait_quick_chain();
chassis.pid_wait();
//pros::delay(1000);  
chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
chassis.pid_wait(); 

*/
}



