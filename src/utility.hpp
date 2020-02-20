#include "math.h"
#include "pros/motors.hpp"

float scale(float val, float lowerrange, float upperrange, float lowerscale, float upperscale){
    float slope = (upperscale - lowerscale)/(upperrange - lowerrange);
    return val * slope + (lowerscale - slope * lowerrange);
}

int trim(int val, int lower, int upper){
    return (val > upper ? upper : (val < lower ? lower : val));
}

int max(int a, int b){
    return (a > b ? a : b);
}

int min(int a, int b){
    return (a > b ? b : a);
}

int cubifySpeed(int val){
    return scale(pow(scale(trim(val, -128, 128), -128, 128, -1, 1), 3), -1, 1, -128, 128);
}

int calcLeftDrive(int y, int x){
    return cubifySpeed(y + (x * 0.85));
}

int calcRightDrive(int y, int x){
    return cubifySpeed(y - (x * 0.85));
}

int deadband(int val, int limit){
    int upper = limit;
    int lower = -limit;
    if(val >= limit || val <= -limit){
        return val + (val >= limit ? -limit : limit) * (128/(128-limit));
    }else{
        return 0;
    }
}

int limitAcceleration(int speed, int target, int accelLimit, int deccelLimit){
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

void limitMotor(pros::Motor mtr, int speed, int lower, int upper){
    if((mtr.get_raw_position(NULL) <= lower && speed < 0) || (mtr.get_raw_position(NULL) >= upper && speed > 0)){
        mtr = 0;
    }else{
        mtr = speed;
    }
}

void forceLimitMotor(pros::Motor mtr, int speed, int correctionSpeed, int lower, int upper){
    if(mtr.get_raw_position(NULL) <= lower && speed <= 0){
        mtr = correctionSpeed;
    }else if(mtr.get_raw_position(NULL) >= upper && speed >= 0){
        mtr = -correctionSpeed;
    }else{
        mtr = speed;
    }
}

double maxVelocity = 200;
double circumference = 3.25 * 3.14159;
int ticksPerRevolution = 900;
int ticksToAccelerate = 900;
int profileMotion(int currentTicks, int distance){
    currentTicks = max(currentTicks, 0);
    double revolutions = distance/circumference;
    double ticks = ticksPerRevolution * revolutions;
    double speed = 0;
    if(ticks > ticksToAccelerate * 2){
        if(currentTicks < ticksToAccelerate){
            speed = sqrt(currentTicks/ticksToAccelerate) * maxVelocity + 5;
        }else if(currentTicks < ticks - ticksToAccelerate){
            speed = maxVelocity;
        }else if(currentTicks < ticks){
            speed = sqrt((ticks - currentTicks)/ticksToAccelerate) * maxVelocity;
        }
    }else{
        if(currentTicks < ticks/2){
            speed = sqrt(currentTicks/ticksToAccelerate) * maxVelocity + 5;
        }else if(currentTicks < ticks){
            speed = sqrt((ticks - currentTicks)/ticksToAccelerate) * maxVelocity;
        }
    }
    return (int)speed;
}

void initProfile(pros::Motor motor, double distance){
    double revolutions = distance/circumference;
    double ticks = ticksPerRevolution * revolutions;
    motor.tare_position();
    motor.move_relative(ticks, 0);
}
