#ifndef DEFS
#define DEFS 

#include <BasicLinearAlgebra.h>
using namespace BLA;

// helpful commands
#define NOP __asm__("nop");  // 0.0108 us (at pico base frequency)

/**
 * The signum function sgn(x) = -1 if x < 0, else 1
 */
template <typename T>
inline int sgn(T x){
  return x < 0 ? -1 : 1;
}

inline float max(float a, float b){
  return (a < b ? b : a);
}

// helpful constants
#define PI_F 3.14159265359f
#define TO_DEG 57.29578f
#define TO_RAD 0.01745329251f

// helpful types
using Vector3 = Matrix<3>;
using Vector4 = Matrix<4>;
using Matrix3 = Matrix<3, 3>;

const Vector4 zero_4vector = {0,0,0,0};
const Vector3 zero_3vector = {0,0,0};

// Flight controller-used structures definitions

typedef struct Control{
  float roll = 0;
  float pitch = 0;
  float yaw = 0;
  float throttle = 0;
  uint32_t motors_on = 0;
} Control;

typedef struct State{
  float roll = 0;
  float pitch = 0;
  float yaw = 0;
  float alt = 0;
} State;

typedef struct Measurements{
  Vector3 acc_vec;
  Vector3 mag_vec;
  Vector3 gyro_vec;
  float altitude;
  float battery = 12;
  float integration_period;
} Measurements;   // struct to hold all measured values

struct PID_config{
  float P;
  float I;
  float D;
  float sat;
  float LPc;

  void set(float newP, float newI, float newD, float newsat, float newLPc){
    P = newP;
    I = newI; 
    D = newD;
    sat = newsat;
    LPc = newLPc;
  }
};

#endif