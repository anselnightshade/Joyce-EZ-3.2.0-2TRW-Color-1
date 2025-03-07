#include "main.h"
#include "autons.hpp"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////
// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing! //the first motor is also the motor on top of the stacked
    {-5,6,-4},  // Left Chassis Ports (negative port will reverse it!) //originally -5, 6, -4
    {3,7,-17},  // Right Chassis Ports (negative port will reverse it!) //originally 3, 7, -17
    20,      // IMU Port //originally 21
    3.25,   // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450, 1.0);  // Wheel RPM = cartridge * (motor gear / wheel gear) //originally 257

// Are you using tracking wheels?  Comment out which ones you're using here!
//  `2.75` is the wheel diameter
//  `4.0` is the distance from the center of the wheel to the center of the robot
// ez::tracking_wheel right_tracker({-'A', -'B'}, 2.75, 4.0);  // ADI Encoders
// ez::tracking_wheel left_tracker(1, {'C', 'D'}, 2.75, 4.0);  // ADI Encoders plugged into a Smart port
// ez::tracking_wheel horiz_tracker(1, 2.75, 4.0);             // Rotation sensors

ez::tracking_wheel horiz_tracker({'A', 'B'}, 2.75, 0.5);

// Motors
//pros::Motor intakeMotorBottom(-11);
pros::Motor intakeMotor(8); 
pros::Motor liftMotor(11);

pros::adi::Potentiometer potentiometer('G', pros::E_ADI_POT_EDR);

// Pneumatics
ez::Piston clamp_piston('E');

//pros::Vision vision_sensor(12);
pros::Optical optical_sensor(9);

int stuck_timer = 0; // Timer to track how long the motor has been stuck
int color_desired = 1; //0 blue, 1 red
int intake_auto_on = 0; //0 off, 1 on

void intake_color_check(pros::Motor intakeMotorTop, pros::Optical& opticalSensor, double reject_hue_low, double reject_hue_high) {
    // Get the hue value detected by the Optical Sensor
    double detected_hue = opticalSensor.get_hue();

    // Check if the detected hue is outside the desired range
    if (detected_hue > reject_hue_low && detected_hue < reject_hue_high) {
        intakeMotorTop.move_velocity(-400); 
        pros::delay(270);  
        intakeMotorTop.move_velocity(0);    
        pros::delay(700);                
    } else {
      intakeMotorTop.move_velocity(-400); 
    }

}
void intake_stuck_check(pros::Motor intakeMotorTop, int reverse_delay_ms, int stuck_delay_ms) {

    double velocity = intakeMotorTop.get_actual_velocity();

    if (fabs(velocity) < 5 ) { 
        stuck_timer += 20; 
        if (stuck_timer >= stuck_delay_ms) {
            // Reverse motors after the stuck delay
            intakeMotorTop.move_velocity(400);  
            pros::delay(reverse_delay_ms);

            // Reset stuck timer and resume original direction
            stuck_timer = 0;
            intakeMotorTop.move_velocity(-400);  
          }
        } else {
            intakeMotorTop.move_velocity(-400);  
    }
}
//blue desired
void intake_blue_task() {
    optical_sensor.set_led_pwm(100); 
    intakeMotor.move_velocity(-400); 
    while (true) { 
        intake_stuck_check(intakeMotor, 500, 1000);
        pros::delay(ez::util::DELAY_TIME);
        intake_color_check(intakeMotor, optical_sensor, 4,11);//reject red
        pros::delay(ez::util::DELAY_TIME);
    }
}
//red desired
void intake_red_task() {
    optical_sensor.set_led_pwm(100); 
    intakeMotor.move_velocity(-400); 
    while (true) { 
        intake_stuck_check(intakeMotor, 500, 1000);
        pros::delay(ez::util::DELAY_TIME);
        intake_color_check(intakeMotor, optical_sensor, 215.0,223.0);//reject blue
        pros::delay(ez::util::DELAY_TIME);
    }
}
void intake_stuck_task() {
    intakeMotor.move_velocity(-400); 
    while (true) { 
        intake_stuck_check( intakeMotor, 90, 2500);
        pros::delay(ez::util::DELAY_TIME);
    }
}
void log_motor_data_to_screen(const std::vector<pros::Motor>& motors, int line){
  int port1 = motors[0].get_port();
  int port2 = motors[1].get_port();
  int port3 = motors[2].get_port();
  double position1 = motors[0].get_position();
  double position2 = motors[1].get_position();
  double position3 = motors[2].get_position();

  pros::lcd::print(line, "%d: %.2f | %d: %.2f | %d: %.2f", port1, position1, port2, position2, port3, position3); 
}
void log_1motor_data_to_screen(pros::Motor motor, int line){
  int port1 = motor.get_port();
  double position1 = motor.get_position();

  pros::lcd::print(line, "%d: %.2f ", port1, position1); 
}
void log_angle_data_to_screen(pros::adi::Potentiometer p, int line){
  double angle = p.get_angle();
  pros::lcd::print(line, "%.2f ", angle); 
}
const double kP = 0.5;  // Proportional gain
const double kI = 0.01; // Integral gain
const double kD = 0.1;  // Derivative gain

// PID state structure
struct PIDState {
    double error = 0;
    double last_error = 0;
    double integral = 0;
    bool is_complete = false;
};

// PID function to move the motor to a target position based on potentiometer angle
bool move_to_target(PIDState& pid, double target_angle, pros::Motor& motor) {
    if (pid.is_complete) return true;

    // Use the potentiometer's angle as the current position
    double current_angle = potentiometer.get_angle();

    // Calculate error
    pid.error = target_angle - current_angle;

    // Calculate integral (accumulated error)
    pid.integral += pid.error;

    // Anti-windup: Reset integral for small errors
    if (fabs(pid.error) < 1) pid.integral = 0;

    // Calculate derivative (rate of error change)
    double derivative = pid.error - pid.last_error;

    // Compute motor output
    double output = (kP * pid.error) + (kI * pid.integral) + (kD * derivative);

    // Limit output to motor's range (-127 to 127)
    output = std::max(-90.0, std::min(90.0, output));

    // Move the motor
    motor.move(output);

    // Update last error
    pid.last_error = pid.error;

    // Mark as complete if within threshold
    if (fabs(pid.error) < 2) { // Threshold value
        motor.move(0); // Stop motor
        pid.is_complete = true;
        return true;
    }

    return false;
}
void ready_arm_task() {
    PIDState return_pid;

    // Define original position (angle in degrees)
    double original_angle = 10.0; // Adjust based on your potentiometer's initial value

    // Move to the original position
    while (!move_to_target(return_pid, original_angle, liftMotor)) {
        pros::delay(10); // Allow other tasks to run
    }
}
// Task to lift the arm to the target position
void lift_arm_task() {
    PIDState return_pid;

    // Define original position (angle in degrees)
    double shoot_angle = 100.0; // Adjust based on your potentiometer's initial value

    // Move to the original position
    while (!move_to_target(return_pid, shoot_angle, liftMotor)) {
        pros::delay(10); // Allow other tasks to run
    }
}

// Task to return the arm to the original position
void return_arm_task() {
    PIDState lift_pid;

    // Define target position for lifting (angle in degrees)
    double lift_target_angle = 0.0; // Adjust based on your potentiometer's range

    // Move to the lift target
    while (!move_to_target(lift_pid, lift_target_angle, liftMotor)) {
        pros::delay(10); // Allow other tasks to run
    }
}

void log_motor_data_to_sd(const std::vector<pros::Motor>& motors, const std::string& filename) {
    // Create or open the file on the SD card
    std::string filepath = "/usd/" + filename; // Path for the SD card file
    FILE* file = fopen(filepath.c_str(), "a"); // Open in append mode

    if (file == nullptr) {
        pros::lcd::print(0, "Failed to open file: %s", filepath.c_str());
        return; // Exit if the file cannot be opened
    }

    // Write a header if the file is empty
    fseek(file, 0, SEEK_END);
    if (ftell(file) == 0) {
        fprintf(file, "Timestamp (ms),Motor Port,Position,Velocity,Current,Temperature\n");
    }

    // Get the current timestamp
    int timestamp = pros::millis();

    // Loop through each motor and log its data
    for (const auto& motor : motors) {
        int port = motor.get_port();
        double position = motor.get_position();
        double velocity = motor.get_actual_velocity();
        double current = motor.get_current_draw();
        double temperature = motor.get_temperature();

        // Write motor data to the file
        fprintf(file, "%d,%d,%.2f,%.2f,%.2f,%.2f\n", timestamp, port, position, velocity, current, temperature);
    }

    // Close the file
    fclose(file);

    pros::lcd::print(7, "Logged motor data to: %s", filepath.c_str());
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Are you using tracking wheels?  Comment out which ones you're using here!
  // chassis.odom_tracker_right_set(&right_tracker);
  // chassis.odom_tracker_left_set(&left_tracker);
  // chassis.odom_tracker_back_set(&horiz_tracker);  // Replace `back` to `front` if your tracker is in the front!
  chassis.odom_tracker_back_set(&horiz_tracker);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example2},
      {"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
      {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
      {"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
      {"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
      {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", odom_boomerang_injected_pure_pursuit_example},
      {"Drive\n\nDrive forward and come back", drive_example},
      {"Turn\n\nTurn 3 times.", turn_example},
      {"Drive and Turn\n\nDrive forward, turn, come back", drive_and_turn},
      {"Drive and Turn\n\nSlow down during drive", wait_until_change_speed},
      {"Swing Turn\n\nSwing in an 'S' curve", swing_example},
      {"Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining},
      {"Combine all 3 movements", combining_movements},
      {"Interference\n\nAfter driving forward, robot performs differently if interfered or not", interfered_example},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");

  liftMotor.tare_position();
  liftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency


  pros::Task intake_red_autotask(intake_red_task);
  pros::Task intake_blue_autotask(intake_blue_task);
  pros::Task intake_stuck_autotask(intake_stuck_task);
  intake_red_autotask.suspend();
  intake_blue_autotask.suspend();
  intake_stuck_autotask.suspend();
  intake_auto_on = 0;//off 
  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */
  //ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
  //joyce_com_blue_left(intakeMotor, clamp_piston, optical_sensor, intake_blue_autotask, liftMotor); //blue_positive 2
  //joyce_com_red_right(intakeMotor, clamp_piston, optical_sensor, intake_red_autotask, liftMotor); //red_positive 3
  //joyce_com_red_left(intakeMotor, clamp_piston, optical_sensor, intake_red_autotask, liftMotor); //red_negative 4
  //joyce_com_blue_right(intakeMotor, clamp_piston, optical_sensor, intake_blue_autotask, liftMotor); //blue_negative 5
  //No stuck test joyce_com_skill(intakeMotor, clamp_piston);
  joyce_com_skill_stuck(intakeMotor, clamp_piston, intake_stuck_autotask, liftMotor);//skills 6
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}

pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }
    
    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  // This is preference to what you like to drive on
  //chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  pros::motor_brake_mode_e_t driver_preference_brake = MOTOR_BRAKE_COAST;
  bool intakeIn = false;
  bool waspressed = false;
  int arm_stage = 0;

  chassis.drive_brake_set(driver_preference_brake);

  pros::Task intake_red_autotask(intake_red_task);
  pros::Task intake_blue_autotask(intake_blue_task);
  pros::Task intake_stuck_autotask(intake_stuck_task);
  intake_red_autotask.suspend();
  intake_blue_autotask.suspend();
  intake_stuck_autotask.suspend();
  intake_auto_on = 0;//off 
  liftMotor.tare_position();
  liftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    // ez_template_extras();
    //pros::lcd::print(5, "left_raw: %d", chassis.left_motors.front().get_position()); 
    //pros::lcd::print(6, "righ_raw: %d", chassis.right_motors.front().get_position()); 
    //pros::lcd::print(5, "l: %d %d %d", chassis.drive_sensor_left_raw()); 
    //pros::lcd::print(6, "r: %d", chassis.drive_sensor_left_raw()); 
    //for (size_t i = 0; i < chassis.left_motors.size(); ++i) {
    //  pros::lcd::print(i+5, "%d %d, %d %d", chassis.left_motors[i].is_reversed(),chassis.left_motors[i].get_position(), 
    //  chassis.right_motors[i].is_reversed(), chassis.right_motors[i].get_position());
    //}
    // chassis.opcontrol_tank();  // Tank control
    // chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
     chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    // . . .
    // Initiate intake motors
    // . . .
    // bool intakeIn = false;
    // bool intakeOut = false;
    // if(master.get_digital(DIGITAL_R1)) {
    //   //intakeMotorBottom.move_velocity(-180);
    //   //intakeMotorTop.move_velocity(-180);
    //   intakeIn = !intakeIn;
    // } else if(master.get_digital(DIGITAL_R2)) {
    //   intakeMotorBottom.move_velocity(180);
    //   intakeMotorTop.move_velocity(180);
    // } else {
    //   //intakeMotorBottom.move_velocity(0);
    //   //intakeMotorTop.move_velocity(0);
    //   intakeIn = intakeIn;
    // }

  //   if(master.get_digital(DIGITAL_R2)) {
  //     intakeMotorBottom.move_velocity(180);
  //     intakeMotorTop.move_velocity(180);
  //     //intakeIn = false;
  //     //intakeOut = true;
  //   } else if(master.get_digital(DIGITAL_R1) && waspressed==false) {
  //     //intakeMotorBottom.move_velocity(-180);
  //     //intakeMotorTop.move_velocity(-180);
  //     intakeIn = !intakeIn;
  //     //intakeOut = false;
  //   } 
  //  waspressed = master.get_digital(DIGITAL_R1);

  //   if(!(master.get_digital(DIGITAL_R2)) && intakeIn == true) {
  //     intakeMotorBottom.move_velocity(-180);
  //     intakeMotorTop.move_velocity(-180);
  //     //intakeIn = !intakeIn;
  //   } else if(!(master.get_digital(DIGITAL_R2)) && intakeIn == false) {
  //     intakeMotorBottom.move_velocity(0);
  //     intakeMotorTop.move_velocity(0);
  //     //intakeOut = true;
  //   }//else if(intakeIn == false && intakeOut == false){
  //   //   intakeMotorBottom.move_velocity(0);
  //   //   intakeMotorTop.move_velocity(0);
  //   // }

  //   //string intake = '';
    
  //   if(intakeIn == true){
  //     pros::lcd::print(1,"true");
  //   } else{
  //     pros::lcd::print(1,"false");
  //   }
    if (master.get_digital(DIGITAL_Y)) {
      //pros::Task front_arm_autotask(return_arm_task);
	    //liftMotor.move_absolute(0, 70); 
      //pros::delay(600);
      //liftMotor.tare_position();
      //arm_stage = 0;
      //liftMotor.tare_position();
      liftMotor.move_velocity(-70); 
      pros::delay(600);   
      liftMotor.move_velocity(0);
    }
    if (master.get_digital(DIGITAL_X)) {
      //pros::Task ready_arm_autotask(ready_arm_task);
	    //liftMotor.move_velocity(80);
      //pros::delay(100); 
      //liftMotor.move_velocity(40);
      //pros::delay(100);       
      //liftMotor.move_velocity(0);    
	    liftMotor.move_absolute(50, 120); 
      pros::delay(300); 
      liftMotor.move_absolute(75, 80);  
      pros::delay(200); 
    }
    if (master.get_digital(DIGITAL_A) ) {
      intakeMotor.move(1200);
      pros::delay(90);        
      intakeMotor.move_velocity(0);      
	    liftMotor.move_velocity(70); 
      pros::delay(600);     
      //liftMotor.move_velocity(-70); 
      //pros::delay(600);   
      //liftMotor.move_velocity(0);
      //pros::delay(200);    
      //liftMotor.tare_position();
    } 
    if (master.get_digital(DIGITAL_LEFT) && master.get_digital(DIGITAL_Y)) {
      color_desired = 0;//blue
      intake_auto_on = 0;
      intake_blue_autotask.suspend();
      intake_red_autotask.suspend();
      intake_stuck_autotask.suspend();
    }
    if (master.get_digital(DIGITAL_RIGHT) && master.get_digital(DIGITAL_A)) {
      color_desired = 1;//red
      intake_auto_on = 0;
      intake_blue_autotask.suspend();
      intake_red_autotask.suspend();
      intake_stuck_autotask.suspend();
    }
    
    if (intake_auto_on == 0 ) {
      if(master.get_digital(DIGITAL_L1)) {
        liftMotor.move_velocity(70);
      } else if(master.get_digital(DIGITAL_L2)) {
        liftMotor.move_velocity(-70);
      } else {
        liftMotor.move_velocity(0);
      }
    }
    if(master.get_digital(DIGITAL_UP)){
      if (color_desired == 0)
        intake_blue_autotask.resume();
      else
        intake_red_autotask.resume();
      intake_auto_on = 1;
    }
    if(master.get_digital(DIGITAL_DOWN)){
      intake_blue_autotask.suspend();
      intake_red_autotask.suspend();
      intake_stuck_autotask.suspend();
      intake_auto_on = 0;
    }
    if (intake_auto_on == 0) {
      if(master.get_digital(DIGITAL_R1)) {
        //intakeMotor.move(-1200);
        intakeMotor.move_velocity(-127);
      } else if(master.get_digital(DIGITAL_R2)) {
        //intakeMotor.move(1200);
        intakeMotor.move_velocity(127);
      } else {
        //intakeMotor.move(0);
        intakeMotor.move_velocity(0);
      }
    }
    // Clamp piston toggle
    clamp_piston.button_toggle(master.get_digital(DIGITAL_B));
    //pros::lcd::print(5, "Pressing B: %d", master.get_digital(DIGITAL_B));
    //pros::lcd::print(5, "%f, %f, %f", chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get());
    if(color_desired == 0)
      pros::lcd::print(5, "Blue");
    else
      pros::lcd::print(5, "Red");

    //log_motor_data_to_screen(chassis.left_motors,6);
    //log_motor_data_to_screen(chassis.right_motors, 7);
    log_1motor_data_to_screen(liftMotor,7);
    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}


