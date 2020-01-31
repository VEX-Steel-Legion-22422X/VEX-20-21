#include "main.h"
#include "pros/apix.h"
#include "utility.hpp"
#include <algorithm>

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

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
	pros::Motor left_drive1(19);
	pros::Motor left_drive2(12);
	pros::Motor right_drive1(20);
	pros::Motor right_drive2(11);
	pros::Motor left_intake(1);
	left_intake.set_brake_mode(MOTOR_BRAKE_BRAKE);
	pros::Motor right_intake(2);
	right_intake.set_brake_mode(MOTOR_BRAKE_BRAKE);
	pros::Motor tray(5);
	tray.set_brake_mode(MOTOR_BRAKE_BRAKE);
	tray.tare_position();
	pros::Motor lift(6);
	lift.set_brake_mode(MOTOR_BRAKE_BRAKE);
	pros::ADIUltrasonic rear_ultrasonic(1, 2);
	pros::ADIDigitalIn front_limitSwitch(8);

	left_drive1 = 35;
	left_drive2 = 35;
	right_drive1 = -35;
	right_drive2 = -35;
	left_intake = 100;
	right_intake = -100;

	pros::delay(5000);

	left_drive1 = -75;
	left_drive2 = -75;
	right_drive1 = 75;
	right_drive2 = 75;

	pros::delay(2000);

	left_drive1 = 75;
	left_drive2 = 75;
	right_drive1 = 75;
	right_drive2 = 75;
	left_intake = 0;
	right_intake = 0;

	pros::delay(900);

	left_drive1 = 35;
	left_drive2 = 35;
	right_drive1 = -30;
	right_drive2 = -30;

	while(front_limitSwitch.get_value() == 0){
		pros::delay(25);
	}

	left_drive1 = 0;
	left_drive2 = 0;
	right_drive1 = 0;
	right_drive2 = 0;
	left_intake = -50;
	right_intake = 50;

	pros::delay(300);

	left_intake = 0;
	right_intake = 0;

	while(tray.get_raw_position(NULL) < 3450){
		int speed = scale(tray.get_raw_position(NULL), 1200, 3400, 128, 30);
		limitMotor(tray, trim(tray.get_raw_position(NULL) > 1200 ? speed : 128, 30, 128), 0, 3500);
		pros::delay(25);
	}

	tray = 0;
	left_drive1 = -50;
	left_drive2 = -50;
	right_drive1 = 50;
	right_drive2 = 50;
	left_intake = -80;
	right_intake = 80;

	pros::delay(1000);

	left_drive1 = 0;
	left_drive2 = 0;
	right_drive1 = 0;
	right_drive2 = 0;
	left_intake = 0;
	right_intake = 0;

	while(tray.get_raw_position(NULL) > 0){
		limitMotor(tray, -80, -50, 3500);
		pros::delay(25);
	}

	tray = 0;
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor left_drive1(19);
	left_drive1.set_gearing(MOTOR_GEARSET_18);
	pros::Motor left_drive2(12);
	left_drive2.set_gearing(MOTOR_GEARSET_18);
	pros::Motor right_drive1(20);
	right_drive1.set_gearing(MOTOR_GEARSET_18);
	pros::Motor right_drive2(11);
	right_drive2.set_gearing(MOTOR_GEARSET_18);
	pros::Motor left_intake(1);
	left_intake.set_brake_mode(MOTOR_BRAKE_BRAKE);
	pros::Motor right_intake(2);
	right_intake.set_brake_mode(MOTOR_BRAKE_BRAKE);
	pros::Motor tray(5);
	tray.set_brake_mode(MOTOR_BRAKE_BRAKE);
	tray.tare_position();
	pros::Motor lift(6);
	lift.set_brake_mode(MOTOR_BRAKE_BRAKE);

	pros::ADIUltrasonic rear_ultrasonic(1, 2);
	pros::ADIDigitalIn front_limitSwitch(8);

	int left = 0;
	int right = 0;
	bool tank = true;

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

		pros::lcd::print(2, "%d", tray.get_raw_position(NULL));
		pros::lcd::print(3, "%d", lift.get_raw_position(NULL));
		pros::lcd::print(5, "%d", rear_ultrasonic.get_value());


		if(master.get_digital(DIGITAL_X)){
			tank = false;
		}else if(master.get_digital(DIGITAL_B)){
			tank = true;
		}

		if(tank){
			int leftTarget = deadband(master.get_analog(ANALOG_LEFT_Y), 5);
			int rightTarget = deadband(master.get_analog(ANALOG_RIGHT_Y), 5);
			if(master.get_digital(DIGITAL_A)){
				left = leftTarget;
				right = rightTarget;
			}else{
				left = limitAcceleration(left, leftTarget, 4, 5);
				right = limitAcceleration(right, rightTarget, 4, 5);
			}
		}else{
			int direction = deadband(master.get_analog(ANALOG_LEFT_X), 5);
			int speed = deadband(master.get_analog(ANALOG_RIGHT_Y), 5);
			if(master.get_digital(DIGITAL_A)){
				left = calcLeftDrive(speed, direction);
				right = calcRightDrive(speed, direction);
			}else{
				left = limitAcceleration(left, calcLeftDrive(speed, direction), 4, 5);
				right = limitAcceleration(right, calcRightDrive(speed, direction), 4, 5);
			}
		}

		left_drive1 = left;
		left_drive2 = left;
		right_drive1 = -right;
		right_drive2 = -right;

		if(master.get_digital(DIGITAL_L1)){
			left_intake = 100;
			right_intake = -100;
		}else if(master.get_digital(DIGITAL_L2)){
			left_intake = -100;
			right_intake = 100;
		}else{
			left_intake = 0;
			right_intake = 0;
		}

		if(master.get_digital(DIGITAL_R1)){
			lift = -100; //max encoder 2500
		}else if(master.get_digital(DIGITAL_R2)){
			lift = 50;
		}else{
			lift = 0;
		}

		if(master.get_digital(DIGITAL_UP)){
			int speed = scale(tray.get_raw_position(NULL), 1200, 3400, 128, 30);
			limitMotor(tray, trim(tray.get_raw_position(NULL) > 1200 ? speed : 128, 30, 128), 0, 3450);
			pros::lcd::print(3, "%d", speed);
		}else if(master.get_digital(DIGITAL_DOWN)){
			limitMotor(tray, -100, 0, 3450);
		}else{
			int trayLowerLimit = min(max(-lift.get_raw_position(NULL), 0), 1200);
			int trayUpperLimit = 4000;
			if(trayLowerLimit > 400){
				trayUpperLimit = trayLowerLimit + 200;
			}
			forceLimitMotor(tray, 0, 80, trayLowerLimit, trayUpperLimit);
			pros::lcd::print(4, "%d, %d", trayLowerLimit, trayUpperLimit);
		}

		pros::delay(20);
	}
}
