#pragma once

void default_constants();

void drive_example();
void turn_example();
void drive_and_turn();
void wait_until_change_speed();
void swing_example();
void motion_chaining();
void combining_movements();
void interfered_example();
void odom_drive_example();
void odom_pure_pursuit_example();
void odom_pure_pursuit_wait_until_example();
void odom_boomerang_example();
void odom_boomerang_injected_pure_pursuit_example();
void measure_offsets();

void joyce_com_blue_left(pros::Motor intakeMotorTop, Piston clamp_piston, pros::Optical optical_sensor, pros::Task intake_control_task, pros::Motor liftMotor);
void joyce_com_blue_right(pros::Motor intakeMotorTop, ez::Piston clamp_piston, pros::Optical optical_sensor, pros::Task intake_control_task, pros::Motor liftMotor);
void joyce_com_red_right(pros::Motor intakeMotorTop, ez::Piston clamp_piston, pros::Optical optical_sensor, pros::Task intake_control_task, pros::Motor liftMotor);
void joyce_com_red_left(pros::Motor intakeMotorTop, ez::Piston clamp_piston, pros::Optical optical_sensor, pros::Task intake_control_task, pros::Motor liftMotor);
void joyce_com_skill(pros::Motor intakeMotorTop, ez::Piston clamp_piston);
void joyce_com_skill_stuck(pros::Motor intakeMotorTop, ez::Piston clamp_piston, pros::Task intake_control_task, pros::Motor liftMotor);
void odom_drive_example2();