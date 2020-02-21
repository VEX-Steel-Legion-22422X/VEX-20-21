#include "main.h"
#include "pros/apix.h"
#include "utility.hpp"
#include "selection.h"
#include <algorithm>

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
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
std::shared_ptr<ChassisController> robot =
 	ChassisControllerBuilder()
	.withMotors({19, 12}, {-20, -11})
	.withDimensions(AbstractMotor::gearset::green, {{3.25_in, 10_in}, imev5GreenTPR})
	.build();

std::shared_ptr<AsyncMotionProfileController> profileController =
	AsyncMotionProfileControllerBuilder()
	.withLimits({1.0, 2.0, 10.0})
	.withOutput(robot)
	.buildMotionProfileController();

void autonomous() {

	if(abs(autonSelection) == 3){
		return;
	}

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

	left_drive1 = 40;
	left_drive2 = 40;
	right_drive1 = -40;
	right_drive2 = -40;
	left_intake = 127;
	right_intake = -127;

	pros::delay(4300);

	left_drive1 = -75;
	left_drive2 = -75;
	right_drive1 = 75;
	right_drive2 = 75;

	while(rear_ultrasonic.get_value() > 220){
		pros::delay(10);
	}

	if(autonSelection < 0){
		left_drive1 = 75;
		left_drive2 = 75;
		right_drive1 = 75;
		right_drive2 = 75;
	}else if(autonSelection > 0){
		left_drive1 = -75;
		left_drive2 = -75;
		right_drive1 = -75;
		right_drive2 = -75;
	}
	left_intake = 0;
	right_intake = 0;

	pros::delay(850);

	left_drive1 = 35;
	left_drive2 = 35;
	right_drive1 = -35;
	right_drive2 = -35;

	pros::delay(1000);

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

	pros::delay(800);

	left_drive1 = 0;
	left_drive2 = 0;
	right_drive1 = 0;
	right_drive2 = 0;

	while(tray.get_raw_position(NULL) > 0){
		limitMotor(tray, -100, -50, 3500);
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
const int maxAccel = 5;
const int maxDecel = 5;
const int stickDeadband = 5;

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
	pros::Imu imu(3);
	imu.reset();
	while(imu.is_calibrating()){
		pros::delay(10);
	}

	int left = 0;
	int right = 0;
	bool tank{false};
	int lastLimit = 0;
	int liftState = 0;

	pros::lcd::initialize();
	while (true) {
		pros::lcd::print(2, "%d", tray.get_raw_position(NULL));
		pros::lcd::print(3, "%d", lift.get_raw_position(NULL));
		pros::lcd::print(5, "%d", rear_ultrasonic.get_value());
		pros::lcd::print(6, "%d", imu.get_rotation());

		if(front_limitSwitch.get_value() == 1 && lastLimit == 0){
			master.rumble("-");
			lastLimit = 1;
		}else if(front_limitSwitch.get_value() == 0){
			lastLimit = 0;
		}


		if(master.get_digital(DIGITAL_X)){
			tank = false;
		}else if(master.get_digital(DIGITAL_B)){
			tank = true;
		}

		if(tank){
			int leftTarget = deadband(master.get_analog(ANALOG_LEFT_Y), stickDeadband);
			int rightTarget = deadband(master.get_analog(ANALOG_RIGHT_Y), stickDeadband);
			if(master.get_digital(DIGITAL_A)){
				left = leftTarget;
				right = rightTarget;
			}else{
				left = limitAcceleration(left, leftTarget, maxAccel, maxDecel);
				right = limitAcceleration(right, rightTarget, maxAccel, maxDecel);
			}
		}else{
			int direction = deadband(master.get_analog(ANALOG_LEFT_X), stickDeadband);
			int speed = deadband(master.get_analog(ANALOG_RIGHT_Y), stickDeadband);
			if(master.get_digital(DIGITAL_A)){
				left = calcLeftDrive(speed, direction);
				right = calcRightDrive(speed, direction);
			}else{
				left = limitAcceleration(left, calcLeftDrive(speed, direction), maxAccel, maxDecel);
				right = limitAcceleration(right, calcRightDrive(speed, direction), maxAccel, maxDecel);
			}
		}

		left_drive1 = left;
		left_drive2 = left;
		right_drive1 = -right;
		right_drive2 = -right;

		if(master.get_digital(DIGITAL_L1)){
			left_intake = 127;
			right_intake = -127;
		}else if(master.get_digital(DIGITAL_L2)){
			left_intake = -100;
			right_intake = 100;
		}else{
			left_intake = 0;
			right_intake = 0;
		}

		if(master.get_digital(DIGITAL_R2)) liftState = 0;
		if(master.get_digital(DIGITAL_R1)) liftState = 1;
		if(master.get_digital(DIGITAL_A)) liftState = 2;
		if(liftState == 0){
			forceLimitMotor(lift, 0, 127, 0, 100);
		}else if(liftState == 1){
			forceLimitMotor(lift, 0, 127, -1800, -1700);
		}else if(liftState == 2){
			forceLimitMotor(lift, 0, 127, -2500, -2400);
		}

		if(master.get_digital(DIGITAL_RIGHT)){
			liftState = 3;
			lift = -120; //max encoder 2500
		}else if(master.get_digital(DIGITAL_LEFT)){
			liftState = 3;
			lift = 120;
		}else if(liftState == 3){
			lift = 0;
		}

		if(master.get_digital(DIGITAL_UP)){
			int speed = scale(tray.get_raw_position(NULL), 1200, 3400, 128, 30);
			limitMotor(tray, trim(tray.get_raw_position(NULL) > 1200 ? speed : 128, 30, 128), 0, 3450);
			pros::lcd::print(3, "%d", speed);
		}else if(master.get_digital(DIGITAL_DOWN)){
			limitMotor(tray, -100, 0, 3450);
		}else{
			int trayLowerLimit = min(max(-lift.get_raw_position(NULL), 0), 1200) - 50;
			int trayUpperLimit = 4000;
			if(trayLowerLimit > 400){
				trayUpperLimit = trayLowerLimit + 100;
			}
			forceLimitMotor(tray, 0, 100, trayLowerLimit, trayUpperLimit);
			pros::lcd::print(4, "%d, %d", trayLowerLimit, trayUpperLimit);
		}

		pros::delay(20);
	}
}
