#ifndef MATHS_H
#define MATHS_H

float map(float value, float in_min, float in_max, float out_min, float out_max){
  float d_in = in_max - in_min;
  float d_out = out_max - out_min;
  float scale_multiplier = d_out/d_in;

  float output = (value - in_min)*scale_multiplier;
  output = output + out_min;
  return output;
}

float constrain(float value, float min, float max){
  if(value > max)
    value = max;
  if(value < min)
    value = min;

  return value;
}

float absf(float value){
  if(value > 0)
    return value;
  else
    return -value;
}

#endif // MATHS_H
