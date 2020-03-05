#include "main.h"
#include "utility.hpp"

class Robot{
    public:
        pros::Controller controller;

        pros::Motor left_drive1;
        pros::Motor left_drive2;
        pros::Motor right_drive1;
        pros::Motor right_drive2;
        pros::Motor left_intake;
        pros::Motor right_intake;
        pros::Motor tray;
        pros::Motor lift;

        pros::ADIUltrasonic rear_ultrasonic;
    	pros::ADIDigitalIn front_limitswitch;
    	pros::Imu imu;

        Robot(int, int, int);
        void initialize();
        void arcadeDrive(int, int, bool);
        void tankDrive(int, int, bool);
        void setDriveSpeed(int);
        void setDriveSpeed(int, int);
        void setIntakeSpeed(int);
        int deadband(int, int);
        int cubifySpeed(int);
        int limitAcceleration(int, int, int, int);
        void limitMotor(pros::Motor, int, int, int);
        void forceLimitMotor(pros::Motor, int, int, int, int);
        void driveForward(int, int);

    private:
        int leftSpeed;
        int rightSpeed;
        int maxAccel;
        int maxDecel;
        int joyDeadband;
};

Robot::Robot(int maxAcceleration, int maxDeceleration, int joystickDeadband)
    :controller(pros::E_CONTROLLER_MASTER),
    left_drive1(19, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES),
    left_drive2(12, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES),
    right_drive1(20, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES),
    right_drive2(11, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES),
    left_intake(1, MOTOR_GEARSET_18, false),
    right_intake(2, MOTOR_GEARSET_18, true),
    tray(5),
    lift(6),
    rear_ultrasonic(1, 2),
    front_limitswitch(8),
    imu(4)
{
    left_intake.set_brake_mode(MOTOR_BRAKE_BRAKE);
    right_intake.set_brake_mode(MOTOR_BRAKE_BRAKE);
    tray.set_brake_mode(MOTOR_BRAKE_BRAKE);
    lift.set_brake_mode(MOTOR_BRAKE_BRAKE);

    left_drive1.tare_position();
    left_drive2.tare_position();
    right_drive1.tare_position();
    right_drive2.tare_position();
    left_intake.tare_position();
    right_intake.tare_position();
    tray.tare_position();
    lift.tare_position();

    leftSpeed = 0;
    leftSpeed = 0;
    maxAccel = maxAcceleration;
    maxDecel = maxDeceleration;
    joyDeadband = joystickDeadband;
}

void Robot::initialize(){
    imu.reset();
	while(imu.is_calibrating()){
		pros::delay(10);
	}
}

void Robot::arcadeDrive(int speed, int direction, bool noLimit){
    direction = deadband(direction, joyDeadband);
    speed = deadband(speed, joyDeadband);
    int left = cubifySpeed(speed + direction * 0.85);
    int right = cubifySpeed(speed - direction * 0.85);
    if(noLimit){
        leftSpeed = left;
        rightSpeed = right;
    }else{
        leftSpeed = limitAcceleration(leftSpeed, left, maxAccel, maxDecel);
        rightSpeed = limitAcceleration(rightSpeed, right, maxAccel, maxDecel);
    }
    setDriveSpeed(leftSpeed, rightSpeed);
}

void Robot::tankDrive(int left, int right, bool noLimit){
    int leftTarget = deadband(left, joyDeadband);
    int rightTarget = deadband(right, joyDeadband);
    if(noLimit){
        leftSpeed = leftTarget;
        rightSpeed = rightTarget;
    }else{
        leftSpeed = limitAcceleration(leftSpeed, leftTarget, maxAccel, maxDecel);
        rightSpeed = limitAcceleration(rightSpeed, rightTarget, maxAccel, maxDecel);
    }
    setDriveSpeed(leftSpeed, rightSpeed);
}

int Robot::cubifySpeed(int val){
    return scale(pow(scale(trim(val, -128, 128), -128, 128, -1, 1), 3), -1, 1, -128, 128);
}

void Robot::setDriveSpeed(int speed){
    left_drive1 = speed;
    left_drive2 = speed;
    right_drive1 = speed;
    right_drive2 = speed;
}

void Robot::setDriveSpeed(int left, int right){
    left_drive1 = left;
    left_drive2 = left;
    right_drive1 = right;
    right_drive2 = right;
}

void Robot::setIntakeSpeed(int speed){
    left_intake = speed;
    right_intake = speed;
}

int Robot::deadband(int val, int limit){
    int upper = limit;
    int lower = -limit;
    if(val >= limit || val <= -limit){
        return val + (val >= limit ? -limit : limit) * (128/(128-limit));
    }else{
        return 0;
    }
}

int Robot::limitAcceleration(int speed, int target, int accelLimit, int deccelLimit){
    if(abs(target - speed) < accelLimit){
        return target;
    }

    if(target < speed && speed > 0){
        if(abs(target - speed) > deccelLimit){
            return speed - deccelLimit;
        }else{
            return target;
        }
    }else if(target < speed){
        return speed - accelLimit;
    }else if(target > speed && speed < 0){
        if(abs(target - speed) > deccelLimit){
            return speed + deccelLimit;
        }else{
            return target;
        }
    }else if(target > speed){
        return speed + accelLimit;
    }

    return 0;
}

void Robot::limitMotor(pros::Motor mtr, int speed, int lower, int upper){
    if((mtr.get_raw_position(NULL) <= lower && speed < 0) || (mtr.get_raw_position(NULL) >= upper && speed > 0)){
        mtr = 0;
    }else{
        mtr = speed;
    }
}

void Robot::forceLimitMotor(pros::Motor mtr, int speed, int correctionSpeed, int lower, int upper){
    if(mtr.get_raw_position(NULL) <= lower && speed <= 0){
        mtr = correctionSpeed;
    }else if(mtr.get_raw_position(NULL) >= upper && speed >= 0){
        mtr = -correctionSpeed;
    }else{
        mtr = speed;
    }
}

void Robot::driveForward(int distance, int speed){
    left_drive1.tare_position();
    left_drive2.tare_position();
    right_drive1.tare_position();
    right_drive2.tare_position();

    double heading = imu.get_rotation();
    int traveled = 0;
    while(abs(distance) > abs(traveled)){
        traveled = (left_drive1.get_raw_position(NULL) + left_drive2.get_raw_position(NULL) +
                    right_drive1.get_raw_position(NULL) + right_drive1.get_raw_position(NULL))/4;
        //double error = heading - imu.get_rotation();
        setDriveSpeed(speed, speed);
        pros::delay(25);
    }

    setDriveSpeed(0);
}
