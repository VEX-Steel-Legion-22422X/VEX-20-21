#include <math.h>
float PI = 3.141592653589793238;

float scale(float val, float lowerrange, float upperrange, float lowerscale, float upperscale){
    float slope = (upperscale - lowerscale)/(upperrange - lowerrange);
    return val * slope + (lowerscale - slope * lowerrange);
}

//TODO: may want to add generic types through function templates

int trim(int val, int lower, int upper){
    return (val > upper ? upper : (val < lower ? lower : val));
}

int max(int a, int b){
    return (a > b ? a : b);
}

int min(int a, int b){
    return (a > b ? b : a);
}

float sinusoidal_s_curve(float  width, float height, bool isReversed, float x){
    bool reverse = isReversed ? 1 : -1;
    return (reverse * cos(M_PI * x / width) / 2 + 0.5) * height;
}
