#include "main.h"
#include "selection.h"
#include "robot.hpp"

Robot robot(5, 8, 5);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	robot.initialize();
	selectorInit();
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

	robot.lift = -127;
	pros::delay(1500);
	robot.lift = 100;

	robot.setIntakeSpeed(127);
	robot.drive(2, 45);
	robot.lift = 0;

	pros::delay(400);

	robot.turn(-57);
	robot.drive(1.6, -100);
	robot.turn(0);
	robot.drive(2.5, 55);
	robot.turn(155);

	robot.setIntakeSpeed(-10);
	robot.tray = 30;

	robot.drive(2, 80);

	robot.setIntakeSpeed(0);

	while(robot.tray.get_raw_position(NULL) < 3300){
		int speed = scale(robot.tray.get_raw_position(NULL), 1200, 3400, 128, 11);
		robot.limitMotor(robot.tray, trim(robot.tray.get_raw_position(NULL) > 1200 ? speed : 128, 30, 128), 0, 3400);
		pros::delay(25);
	}

	robot.tray = -20;
	robot.drive(1, -50);

/*
	robot.turn(0, 50);
	robot.drive(2.75, 70);

	robot.turn(140, 50);
	robot.drive(3, 70);
*/

	/*
	if(abs(autonSelection) == 3){
		return;
	}

	robot.setDriveSpeed(40);
	robot.setIntakeSpeed(127);

	pros::delay(4300);

	robot.setDriveSpeed(-75);

	while(robot.rear_ultrasonic.get_value() > 220){
		pros::delay(10);
	}

	if(autonSelection < 0){
		robot.setDriveSpeed(75, -75);
	}else if(autonSelection > 0){
		robot.setDriveSpeed(-75, 75);
	}
	robot.setIntakeSpeed(0);

	pros::delay(850);

	robot.setDriveSpeed(35);

	pros::delay(1000);

	robot.setDriveSpeed(0);
	robot.setIntakeSpeed(-50);

	pros::delay(300);

	robot.setIntakeSpeed(0);

	while(robot.tray.get_raw_position(NULL) < 3450){
		int speed = scale(robot.tray.get_raw_position(NULL), 1200, 3400, 128, 30);
		robot.limitMotor(robot.tray, trim(robot.tray.get_raw_position(NULL) > 1200 ? speed : 128, 30, 128), 0, 3500);
		pros::delay(25);
	}

	robot.tray = 0;
	robot.setDriveSpeed(-50);

	pros::delay(800);

	robot.setDriveSpeed(0);

	while(robot.tray.get_raw_position(NULL) > 0){
		robot.limitMotor(robot.tray, -100, -50, 3500);
		pros::delay(25);
	}

	robot.tray = 0;
	*/
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
	pros::lcd::initialize();

	bool tank{false};
	int lastLimit = 0;
	int liftState = 0;

	while (true) {
		pros::lcd::print(1, "%f", robot.left_drive1.get_position());
		pros::lcd::print(2, "%d", robot.tray.get_raw_position(NULL));
		pros::lcd::print(3, "%d", robot.lift.get_raw_position(NULL));
		pros::lcd::print(5, "%d", robot.rear_ultrasonic.get_value());
		pros::lcd::print(6, "%f", robot.imu.get_rotation());

		if(robot.front_limitswitch.get_value() == 1 && lastLimit == 0){
			master.rumble("-");
			lastLimit = 1;
		}else if(robot.front_limitswitch.get_value() == 0){
			lastLimit = 0;
		}

		if(master.get_digital(DIGITAL_X)){
			tank = false;
		}else if(master.get_digital(DIGITAL_B)){
			tank = true;
		}

		if(tank){
			robot.tankDrive(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y), false);
		}else{
			robot.arcadeDrive(master.get_analog(ANALOG_RIGHT_Y), master.get_analog(ANALOG_LEFT_X), false);
		}

		if(master.get_digital(DIGITAL_L1)){
			robot.setIntakeSpeed(127);
		}else if(master.get_digital(DIGITAL_L2)){
			robot.setIntakeSpeed(-100);
		}else{
			robot.setIntakeSpeed(0);
		}

		if(master.get_digital(DIGITAL_R2)) liftState = 0;
		if(master.get_digital(DIGITAL_R1)) liftState = 1;
		if(master.get_digital(DIGITAL_A)) liftState = 2;
		if(liftState == 0){
			robot.forceLimitMotor(robot.lift, 0, 127, 0, 100);
		}else if(liftState == 1){
			robot.forceLimitMotor(robot.lift, 0, 127, -1800, -1700);
		}else if(liftState == 2){
			robot.forceLimitMotor(robot.lift, 0, 127, -2500, -2400);
		}

		if(master.get_digital(DIGITAL_RIGHT)){
			liftState = 3;
			robot.lift = -120; //max encoder 2500
		}else if(master.get_digital(DIGITAL_LEFT)){
			liftState = 3;
			robot.lift = 120;
		}else if(liftState == 3){
			robot.lift = 0;
		}

		if(master.get_digital(DIGITAL_UP)){
			int speed = scale(robot.tray.get_raw_position(NULL), 1200, 3400, 128, 11);
			robot.limitMotor(robot.tray, trim(robot.tray.get_raw_position(NULL) > 1200 ? speed : 128, 30, 128), 0, 3400);
			pros::lcd::print(3, "%d", speed);
		}else if(master.get_digital(DIGITAL_DOWN)){
			robot.limitMotor(robot.tray, -100, 0, 3450);
		}else{
			int trayLowerLimit = min(max(-robot.lift.get_raw_position(NULL), 0), 1200) - 50;
			int trayUpperLimit = 4000;
			if(trayLowerLimit > 400){
				trayUpperLimit = trayLowerLimit + 100;
			}
			robot.forceLimitMotor(robot.tray, 0, 100, trayLowerLimit, trayUpperLimit);
			pros::lcd::print(4, "%d, %d", trayLowerLimit, trayUpperLimit);
		}

		pros::delay(25);
	}
}
