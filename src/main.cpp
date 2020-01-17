#include "main.h"
#include "pros/apix.h"
#include "utility.hpp"

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

	left_drive1 = 25;
	left_drive2 = 25;
	right_drive1 = -25;
	right_drive2 = -25;
	left_intake = 100;
	right_intake = -100;

	pros::delay(8000);

	left_drive1 = 0;
	left_drive2 = 0;
	right_drive1 = 0;
	right_drive2 = 0;
	left_intake = 0;
	right_intake = 0;
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

	int left = 0;
	int right = 0;
	bool tank = true;

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

		pros::lcd::print(2, "%d", tray.get_raw_position(NULL));
		pros::lcd::print(3, "%d", lift.get_raw_position(NULL));


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
				left = limitAcceleration(left, leftTarget, 3, 5);
				right = limitAcceleration(right, rightTarget, 3, 5);
			}
		}else{
			int direction = deadband(master.get_analog(ANALOG_LEFT_X), 5);
			int speed = deadband(master.get_analog(ANALOG_RIGHT_Y), 5);
			if(master.get_digital(DIGITAL_A)){
				left = calcLeftDrive(speed, direction);
				right = calcRightDrive(speed, direction);
			}else{
				left = limitAcceleration(left, calcLeftDrive(speed, direction), 3, 5);
				right = limitAcceleration(right, calcRightDrive(speed, direction), 3, 5);
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

		if(master.get_digital(DIGITAL_UP)){
			lift = -100;
		}else if(master.get_digital(DIGITAL_DOWN)){
			lift = 50;
		}else{
			lift = 0;
		}

		if(master.get_digital(DIGITAL_R1)){
			int speed = scale(tray.get_raw_position(NULL), 1200, 3400, 128, 30);
			limitMotor(tray, trim(tray.get_raw_position(NULL) > 1200 ? speed : 128, 30, 128), 0, 3500);
			pros::lcd::print(3, "%d", speed);
		}else if(master.get_digital(DIGITAL_R2)){
			limitMotor(tray, -80, 0, 3500);
		}else{
			tray = 0;
		}

		pros::delay(20);
	}
}
