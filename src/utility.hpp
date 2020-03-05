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
