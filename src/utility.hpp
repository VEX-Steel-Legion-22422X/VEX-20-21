#include "math.h"
#include "pros/motors.hpp"

float scale(float val, float lowerrange, float upperrange, float lowerscale, float upperscale){
    float slope = (upperscale - lowerscale)/(upperrange - lowerrange);
    return val * slope + (lowerscale - slope * lowerrange);
}

int trim(int val, int lower, int upper){
    return (val > upper ? upper : (val < lower ? lower : val));
}

int cubifySpeed(int val){
    return scale(pow(scale(trim(val, -128, 128), -128, 128, -1, 1), 3), -1, 1, -128, 128);
}

int calcLeftDrive(int y, int x){
    return cubifySpeed(y + (x * 0.8));
}

int calcRightDrive(int y, int x){
    return cubifySpeed(y - (x * 0.8));
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
