#ifndef CONFIG__H
#define CONFIG__H

// Period Control
#define EKF_PER_   10
#define PRT_PER_   100
// EKF
//#define USE_EKF_
// Radio
#define RD_RNG_    1000.0f
#define RD_THR_    0.0f
#define RD_AIL_    -500.0f
#define RD_RUD_    -500.0f
#define RD_ELV_    -500.0f
// Servo
#define PIN_THR_   3
#define PIN_AIL_L_ 5
#define PIN_AIL_R_ 6
#define PIN_RUD_   10
#define PIN_ELV_   9
// IMU filter
#define AX_COF_  2.0f
#define AY_COF_  2.0f
#define AZ_COF_  2.0f
// Controller
#define ROLL_COF_  1.0f
#define PITCH_COF_ 1.0f
#define ROLL_KP_   60.0f
#define ROLL_KI_   2.0f
#define ROLL_KD_   2.0f
#define PITCH_KP_  60.0f
#define PITCH_KI_  2.0f
#define PITCH_KD_  2.0f
#define PID_ILM_   5.0f
// Motion Constraints
#define L_AIL_AVG_ 90.0f
#define L_AIL_RNG_ 25.0f
#define R_AIL_AVG_ 90.0f
#define R_AIL_RNG_ 25.0f
#define RUD_AVG_   90.0f
#define RUD_RNG_   35.0f
#define ELV_AVG_   90.0f
#define ELV_RNG_   35.0f
#define FLP_RNG_   20.0f
// Flight Kinematic Constraints
#define ROLL_RNG_  0.785 // 45 degree
#define PITCH_RNG_ 0.785 // 45 degree
// Print
#define PRT_PER
//#define PRT_RADIO
//#define PRT_IMU
#define PRT_ACC
//#define PRT_GYR
//#define PRT_RPY
//#define PRT_CONTROL

#endif
