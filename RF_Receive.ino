#include <MPU6050_tockn.h>
#include <quaternion_6axis.h>
#include <ekf7.h>
#include <Wire.h>
#include <SPI.h>
#include "RF24.h"
#include <Servo.h>
#include "rc_filter.h"
#include "pid.h"
#include "config.h"

#define DEBUG_FLG 1
#define DEBUG_PRINT(x){if(DEBUG_FLG){Serial.print(x);}}

// Period Control
unsigned int cnt = 0, cnt_max = 1e4;
FTYPE last_print = 0;
// Radio
RF24 rf24(7, 8); // CE腳, CSN腳
const byte addr[] = "1Node";
const byte pipe = 1;  // 指定通道編號
struct MyData {
  int throttle = 0;
  int aileron  = 0;
  int rudder   = 0;
  int elevator = 0;
  int flap     = 0;
  int fine     = 0;
};
MyData msg;
// Servo
Servo mythr, myailL, myailR, myrud, myelv;
int avgailL = L_AIL_AVG_;
int rngailL = L_AIL_RNG_;
int avgailR = R_AIL_AVG_;
int rngailR = R_AIL_RNG_;
int avgrud  = RUD_AVG_;
int rngrud  = RUD_RNG_;
int avgelv  = ELV_AVG_;
int rngelv  = ELV_RNG_;
int angflap = FLP_RNG_;
// IMU
MPU6050 mpu6050(Wire);
// Kalman Filter
FTYPE last_time = 0;
// Controller
// set up lpf
FT pitch_filter, roll_filter;
// set up pid
FT pitch_pid, roll_pid;
// filter and controller space
DTYPE pitch_filter_last_input[RC_MEMORY_SIZE];
DTYPE pitch_filter_last_output[RC_MEMORY_SIZE];
DTYPE pitch_filter_param[RC_PARAM_SIZE];
DTYPE roll_filter_last_input[RC_MEMORY_SIZE];
DTYPE roll_filter_last_output[RC_MEMORY_SIZE];
DTYPE roll_filter_param[RC_PARAM_SIZE];
DTYPE pitch_pid_last_input[RC_MEMORY_SIZE];
DTYPE pitch_pid_last_output[RC_MEMORY_SIZE];
DTYPE pitch_pid_param[PID_PARAM_SIZE];
DTYPE roll_pid_last_input[RC_MEMORY_SIZE];
DTYPE roll_pid_last_output[RC_MEMORY_SIZE];
DTYPE roll_pid_param[PID_PARAM_SIZE];

void setup() {
  // Serial Port
  Serial.begin(9600);
  Serial.println("=== Start ===");

  // Radio
  rf24.begin();
  rf24.setChannel(83);  // 設定頻道編號
  rf24.setPALevel(RF24_PA_MIN);
  rf24.setDataRate(RF24_250KBPS);
  rf24.openReadingPipe(pipe, addr);  // 開啟通道和位址
  rf24.startListening();  // 開始監聽無線廣播
  DEBUG_PRINT("nRF24L01 ready!")

  // Servo
  // define servo pins
  mythr.attach(PIN_THR_);
  myailL.attach(PIN_AIL_L_);
  myailR.attach(PIN_AIL_R_);
  myrud.attach(PIN_RUD_);
  myelv.attach(PIN_ELV_);

  // IMU
  Wire.begin();
  mpu6050.begin();
  //mpu6050.calcGyroOffsets(true);

  // EKF Test
  ekf7_init();
  quaternion_init_QE6();
  ekf7_set_Propagation_Noise(1e-1, 1e-1, 1e-1, 1e-3, 1e-3, 1e-3, 1e-3);
  ekf7_set_Observation_Noise(1e-3, 1e-3, 1e-3, 1e-3);
  set_reference_QE6(0, 0, 1);

  // Controller Setup
  // Pitch filter
  pitch_filter.size = RC_MEMORY_SIZE;
  pitch_filter.last_pos = 0;
  pitch_filter.last_time = 0.0f;
  pitch_filter.current_input = 0;
  pitch_filter.current_output = 0;
  pitch_filter.last_input = pitch_filter_last_input;
  pitch_filter.last_output = pitch_filter_last_output;
  pitch_filter.param = pitch_filter_param;
  pitch_filter.param[RC_CUTOFF] = PITCH_COF_;
  // initialize
  for (int i = 0; i < pitch_filter.size; i++) {
    pitch_filter_last_input[i] = 0;
    pitch_filter_last_output[i] = 0;
  }
  // Roll filter
  roll_filter.size = RC_MEMORY_SIZE;
  roll_filter.last_pos = 0;
  roll_filter.last_time = 0.0f;
  roll_filter.current_input = 0;
  roll_filter.current_output = 0;
  roll_filter.last_input = roll_filter_last_input;
  roll_filter.last_output = roll_filter_last_output;
  roll_filter.param = roll_filter_param;
  roll_filter.param[RC_CUTOFF] = ROLL_COF_;
  // initialize
  for (int i = 0; i < roll_filter.size; i++) {
    roll_filter_last_input[i] = 0;
    roll_filter_last_output[i] = 0;
  }
  // Pitch PID Controller
  pitch_pid.size = RC_MEMORY_SIZE;
  pitch_pid.last_pos = 0;
  pitch_pid.last_time = 0.0f;
  pitch_pid.current_input = 0;
  pitch_pid.current_output = 0;
  pitch_pid.last_input = pitch_pid_last_input;
  pitch_pid.last_output = pitch_pid_last_output;
  pitch_pid.param = pitch_pid_param;
  pitch_pid.param[PID_KP] = PITCH_KP_;
  pitch_pid.param[PID_KI] = PITCH_KI_;
  pitch_pid.param[PID_KD] = PITCH_KD_;
  pitch_pid.param[PID_Ilimit] = PID_ILM_;
  pitch_pid.param[PID_SIdt] = 0.0f;
  for (int i = 0; i < pitch_pid.size; i++) {
    pitch_pid_last_input[i] = 0;
    pitch_pid_last_output[i] = 0;
  }
  // Roll PID Controller
  roll_pid.size = RC_MEMORY_SIZE;
  roll_pid.last_pos = 0;
  roll_pid.last_time = 0.0f;
  roll_pid.current_input = 0;
  roll_pid.current_output = 0;
  roll_pid.last_input = roll_pid_last_input;
  roll_pid.last_output = roll_pid_last_output;
  roll_pid.param = roll_pid_param;
  roll_pid.param[PID_KP] = ROLL_KP_;
  roll_pid.param[PID_KI] = ROLL_KI_;
  roll_pid.param[PID_KD] = ROLL_KD_;
  roll_pid.param[PID_Ilimit] = PID_ILM_;
  roll_pid.param[PID_SIdt] = 0.0f;
  for (int i = 0; i < roll_pid.size; i++) {
    roll_pid_last_input[i] = 0;
    roll_pid_last_output[i] = 0;
  }

}

void loop() {
  // Period Control
  cnt++;
  if (cnt > cnt_max) {
    cnt = 0;
  }

  // Radio
  if (rf24.available(&pipe)) {
    rf24.read(&msg, sizeof(msg));
    /*
    */
  }

  // Take IMU Measurement
  mpu6050.update();
  FTYPE ax = mpu6050.getAccX();
  FTYPE ay = mpu6050.getAccY();
  FTYPE az = mpu6050.getAccZ();
  FTYPE norma = sqrt(pow(ax, 2) + pow(ay, 2) + pow(az, 2));
  FTYPE gx = mpu6050.getGyroX();
  FTYPE gy = mpu6050.getGyroY();
  FTYPE gz = mpu6050.getGyroZ();

  // Take dt
  FTYPE now = (FTYPE) millis() / 1000;
  FTYPE dt = now - last_time;
  last_time = now;

  // Attitude Estimation
  // Set Measurement
  set_measurement_QE6(ax / norma, ay / norma, az / norma);
  // Update
  quaternion_update_QE6();
  // Get qe6 quaternion
  FTYPE qw_, qx_, qy_, qz_;
  get_qk_QE6(&qw_, &qx_, &qy_, &qz_);
  // Get qe6 rpy
  FTYPE _r, _p, _y;
  get_rpy_QE6(&_r, &_p, &_y);

  FTYPE bx = 0, by = 0, bz = 0;
  FTYPE _er = 0, _ep = 0, _ey = 0;
  // Bias estimation for EKF
  if ((cnt % EKF_PER_) == 0) {
    // Predict Step For EKF
    ekf7_predict(dt, gx, gy, gz, EKF7_COV_UPDATE);
    // Update
    ekf7_update(qw_, qx_, qy_, qz_);
    // Get Filtered Value
    ekf7_get_gyroscope_bias(&bx, &by, &bz);
    // Get ekf rpy
    ekf7_get_rpy(&_er, &_ep, &_ey);
  }

  // Controller
  // Roll command
  DTYPE p = 0, i = 0, d = 0;
  DTYPE rolcmd = ((FTYPE) msg.aileron * ROLL_RNG_) / RD_RNG_; // desired roll angle
  roll_pid.current_input = _r;
  pid_update(now, rolcmd, &roll_pid, PID_NOT_QUEUE, &roll_filter);
  rolcmd = roll_pid.current_output;
  // Pitch command
  DTYPE pthcmd = ((FTYPE) msg.elevator * PITCH_RNG_) / RD_RNG_;
  pitch_pid.current_input = _p;
  pid_update(now, pthcmd, &pitch_pid, PID_NOT_QUEUE, &pitch_filter);
  pthcmd = pitch_pid.current_output;

  // Actuation
  // Set Servo Output
  int _mthr = 1000 + msg.throttle;
  int _mail_l = avgailL + rolcmd - (angflap * msg.flap);
  int _mail_r = avgailR - rolcmd - (angflap * msg.flap);
  int _melv = avgelv + pthcmd;
  int _mrud = map(msg.rudder, 500, -500, avgrud - rngrud, avgrud + rngrud);
  mythr.writeMicroseconds (_mthr);
  myailL.write(_mail_l);
  myailR.write(_mail_r);
  myrud.write(_melv);
  myelv.write(_mrud);

  // Debug Print
  if ((cnt % PRT_PER_) == 0) {
    // if(false){
    Serial.println(now - last_print);
#ifdef PRT_RADIO
    DEBUG_PRINT("[Radio] Throttle: ")
    DEBUG_PRINT(msg.throttle) // 顯示訊息內容
    DEBUG_PRINT(" ,Aileron: ")
    DEBUG_PRINT(msg.aileron)
    DEBUG_PRINT(" ,Rudder: ")
    DEBUG_PRINT(msg.rudder)
    DEBUG_PRINT(" ,Elevator: ")
    DEBUG_PRINT(msg.elevator)
    DEBUG_PRINT(" ,Flap: ")
    DEBUG_PRINT(msg.flap)
    DEBUG_PRINT(" ,Fine: ")
    DEBUG_PRINT(msg.fine)
    DEBUG_PRINT("\r\n")
#endif
#ifdef PRT_IMU
    DEBUG_PRINT("[IMU] Gyro (");
    DEBUG_PRINT(gx);
    DEBUG_PRINT(",");
    DEBUG_PRINT(gy);
    DEBUG_PRINT(",");
    DEBUG_PRINT(gz);
    DEBUG_PRINT("), Substract Bias (");
    DEBUG_PRINT(gx - bx);
    DEBUG_PRINT(",");
    DEBUG_PRINT(gy - by);
    DEBUG_PRINT(",");
    DEBUG_PRINT(gz - bz);
    DEBUG_PRINT(")\r\n")
#endif
#ifdef PRT_CONTROL
    DEBUG_PRINT("[Command] Throttle: ");
    DEBUG_PRINT(_mthr);
    DEBUG_PRINT(", Aileron L: ");
    DEBUG_PRINT(_mail_l);
    DEBUG_PRINT(", Aileron R: ");
    DEBUG_PRINT(_mail_r);
    DEBUG_PRINT(", Elevator: ");
    DEBUG_PRINT(_melv);
    DEBUG_PRINT(", Rudder: ");
    DEBUG_PRINT(_mrud);
    DEBUG_PRINT("\r\n")
#endif
    last_print = now;
  }
}