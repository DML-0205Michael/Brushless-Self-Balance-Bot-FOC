////////////////////////////////////////////////////////////// IMU //////////////////////////////////////////////////////////////
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

Adafruit_MPU6050 mpu;

// accel
float ax_float, ay_float, az_float;
float ax_error, ay_error, az_error; // error of linear acceleration

// gyro
float wx, wy, wz; // rad/s
float wx_error, wy_error, wz_error;// rad/s
Kalman kalmanY; // Create the Kalman instances
double p_kf=0;
uint32_t timer_kf;
////////////////////////////////////////////////////////////// IMU //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// Encoder //////////////////////////////////////////////////////////////
#include "AS5600.h"

AS5600 as5600_0(&Wire);
AS5600 as5600_1(&Wire1);

int current_angle_raw,previous_angle_raw;
float enc_1_position, enc_2_speed, previous_speed,total_angle;
float alpha_speed=0.7;
////////////////////////////////////////////////////////////// Encoder //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// Motor //////////////////////////////////////////////////////////////
#define PWM_res 11
#define PWM_freq 30000
#define motor_pwm_max 2048

#define PWM_A_pin 16
#define PWM_A_CH 0

#define PWM_B_pin 17
#define PWM_B_CH 1

#define PWM_C_pin 18
#define PWM_C_CH 2

// #define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
float voltage_limit=7.6;
float voltage_power_supply=8.1;
float voltage_upper_bound=voltage_power_supply/2+voltage_limit;
float voltage_lower_bound=voltage_power_supply/2-voltage_limit;
float shaft_angle=0, open_loop_timestamp=0;
float zero_electric_angle=0, Ualpha, Ubeta=0, Ua=0, Ub=0, Uc=0, dc_a=0, dc_b=0, dc_c=0;

#define   EN_GATE 4
#define   M_PWM 26 
#define   M_OC 27
#define   OC_ADJ 14
////////////////////////////////////////////////////////////// Motor //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// Control //////////////////////////////////////////////////////////////
float KP_pos=-0.350;
float KI_pos=-0.000150;
float KD_pos=-1.8;

float target_pos=0,previous_target_pos, err_sum_pos,pre_err; 
float PI_pos_output;

unsigned long loop_start_time=0, pre_time;
const int loop_time=4000; // micro seconds
const int control_loop_freq=250;
////////////////////////////////////////////////////////////// Control //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// Iq  //////////////////////////////////////////////////////////////
#define resistance 0.01 // ohms
#define Ia_pin 36
#define Ib_pin 39
#define Ic_pin 34

int Ia_err, Ib_err;
float pre_Iq;
float alpha_Iq=0.8;
////////////////////////////////////////////////////////////// Iq  //////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(460800);

  AS5600_setup();
  mpu6050_setup();
  
  motor_setup();

  Iq_setup();

  pinMode(2,OUTPUT);
  digitalWrite(2,HIGH);delay(100);digitalWrite(2,LOW);delay(100);
  digitalWrite(2,HIGH);delay(100);digitalWrite(2,LOW);delay(100);
  digitalWrite(2,HIGH);delay(100);digitalWrite(2,LOW);delay(100);
  
}

void loop() {
  read_bluetooth_data();
  
  loop_time_holder();

  // read_encoder_position();
  control_loop();

  motor_output_loop();

  Serial.println();
}

////////////////////////////////////////////////////////////// Iq  //////////////////////////////////////////////////////////////
void Iq_setup(){
  pinMode(Ia_pin, INPUT);
  pinMode(Ib_pin, INPUT);
  pinMode(Ic_pin, INPUT);

  Iq_calibrate();
}

float read_Iq(){
  float Ia=-(analogRead(Ia_pin)-Ia_err);
  float Ib=(analogRead(Ib_pin)-Ib_err);
  float Iq;
  // Serial.print("Ia:");
  Serial.print(Ia); Serial.print(" ");
  // Serial.print("Ib:");
  Serial.print(Ib); Serial.print(" ");
  // int Ic=analogRead(Ic_pin);
  // Ic=analogRead(Ic_pin)*
  // actual_Ic(in Amps)*R*chip_gain/3.3=x=analogRead()
  // actual_Ic(in Amps)=analogRead()/R/chip_gain*3.3=analogRead()*A_constant

  float I_alpha=Ia;
  float I_beta=sqrt(3)*(Ia+2*Ib)/3;

  float angle_el=_electrical_angle();

  float ct=cos(angle_el);
  float st=sin(angle_el);

  Iq=I_beta*ct-I_alpha*st;

  pre_Iq=Iq;
 
  float Iq_filter=pre_Iq*alpha_Iq+(1.f-alpha_Iq)*Iq;

  return Iq_filter;
}

void Iq_calibrate(){
  step3_PWM_output(0, 0, 0);
  delay(3000);
  digitalWrite(2,HIGH);
  int num_of_loop=50; // number of sample
  int i=0;
  Ia_err=0; Ib_err=0;
  while (i < num_of_loop) {
    Ia_err=Ia_err+analogRead(Ia_pin)-2048;
    Ib_err=Ib_err+analogRead(Ib_pin)-2048;
    i++;
  }
  Ia_err=Ia_err/num_of_loop+2048; // unit: raw data
  Ib_err=Ib_err/num_of_loop+2048;
  digitalWrite(2,LOW);
  Serial.print("Ia_err: "); Serial.println(Ia_err);
  Serial.print("Ib_err: "); Serial.println(Ib_err);
  delay(2000);
}
////////////////////////////////////////////////////////////// Iq  //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// Encoder //////////////////////////////////////////////////////////////
void AS5600_setup(){
  Wire.begin(21, 22);    // SDA=21, SCL=22
  Wire1.begin(19, 23);   // SDA=19, SCL=23

  if (!as5600_0.begin()) {
    Serial.println("No 1 AS5600 Found");
    while (1);
  }
  if (!as5600_1.begin()) {
    Serial.println("No 2 AS5600 Found");
    while (1);
  }
  current_angle_raw=as5600_0.readAngle();
  previous_angle_raw=as5600_0.readAngle();
  enc_1_position=0;
  
}

void read_encoder_position(){
  current_angle_raw=as5600_0.readAngle(); // 0~4096
  int dv=current_angle_raw-previous_angle_raw;
  if (dv<-1000){ // ex: dv=100-4000=-3900
    dv+=4096; // dv=100+(4096-4000)
  } else if (dv>1000) { // dv=4000-100
    dv-=4096; // dv=-((4096-4000)+100)=-(4096-4000+100)=-4096+4000-100
  }
  
  enc_1_position=enc_1_position+float(dv)/4096*2*M_PI;// rad/s
  previous_angle_raw=current_angle_raw;
  
}

////////////////////////////////////////////////////////////// Encoder //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// IMU  //////////////////////////////////////////////////////////////
void mpu6050_setup(){ 
  Serial.flush();
  delay(2500);
  mpu6050_start();

  delay(2000);

  read_accel_gyro_raw();
  ax_float-=ax_error;
  ay_float-=ay_error;
  az_float-=az_error;

  double pitch=atan(-1*ax_float/sqrt(sq(ay_float)+sq(az_float)));

  kalmanY.setAngle(pitch); // Set starting angle

  timer_kf = micros();
}

void mpu6050_start(){
  // Try to initialize!
  Serial.println("MPU6050 Start");delay(1000);
  // DO NOT USE: if (!mpu.begin()) { 
  //        USE: ↓↓↓↓↓↓↓↓↓↓↓↓
  if (!mpu.begin(0x68,&Wire)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}

void read_accel_gyro_raw(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  ax_float=a.acceleration.x;
  ay_float=a.acceleration.y;
  az_float=a.acceleration.z;

  wx=g.gyro.x;
  wy=g.gyro.y;
  wz=g.gyro.z;
}

void read_mpu6050_angle_loop(){
  read_accel_gyro_raw();
  ax_float-=ax_error;
  ay_float-=ay_error;
  az_float-=az_error;
  wx-=wx_error;
  wy-=wy_error;
  wz-=wz_error;

  double dt = (double)(micros() - timer_kf) / 1000000; // Calculate delta time
  timer_kf = micros();
  
  double pitch=atan(ay_float/sqrt(sq(ax_float)+sq(az_float)))*180/M_PI;
  p_kf = kalmanY.getAngle(pitch, wx*180/M_PI, dt); // pitch
  // Serial.print(-50); // To freeze the lower limit
  // Serial.print(" ");
  // Serial.print(50); // To freeze the upper limit
  // Serial.print(" ");

  // Serial.print("Pitch: ");
  // Serial.print(p_kf); Serial.print("\t");
  Serial.print(" p_kf:");Serial.print(p_kf); Serial.print("\t");
}
////////////////////////////////////////////////////////////// IMU //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// Control //////////////////////////////////////////////////////////////
void control_loop(){
  // Serial.print("act:"); 
  // Serial.print(enc_1_position); Serial.print("\t");
  // Serial.print("tar:"); 
  // Serial.print(target_pos); Serial.print("\t");
  // Serial.print("Uq:");Serial.print(PI_pos_output); // Serial.print();

  Serial.print(-4100); // To freeze the lower limit
  Serial.print(" ");

  Serial.print(4100); // To freeze the upper limit
  Serial.print(" ");



  read_encoder_position();
  position_PI(target_pos,enc_1_position);

  float Iq=read_Iq();
  // Serial.print("Iq:"); 
  Serial.print(Iq); Serial.print("\t");
}

void position_PI(float target, float actual){
  // float speed=v1*(1-alpha_speed)+previous_speed*alpha_speed;
  // previous_speed=speed;
  // Serial.print("wheel:");Serial.print(speed); Serial.print(" ");
  // pre_speed=speed;
  float err=target-actual;
  // Serial.print("Speed err:");Serial.print(err); Serial.print("\t");
  // if (abs(err)<7){
  err_sum_pos+=err; // Serial.print("err sum:");Serial.print(err_sum_pos); Serial.print("\t");
  err_sum_pos=constrain(err_sum_pos,-20000,20000);
  // } else err_sum_pos=0;
  if (previous_target_pos != target) err_sum_pos=0;
  // if (previous_target_pos*target <= 0 ) err_sum_pos=0;
  previous_target_pos=target;

  // Serial.print("err sum:");Serial.print(err_sum_pos); Serial.print("\t");
  float KP_out,KI_out,KD_out;
  KP_out=KP_pos*err;
  KI_out=KI_pos*err_sum_pos;
  KD_out=KD_pos*(err-pre_err);
  PI_pos_output=constrain(KP_out+KI_out+KD_out,-3,3);

  pre_err=err;

  // Serial.print("PI_output:"); Serial.print(PI_pos_output); Serial.print("\t");
}
////////////////////////////////////////////////////////////// Control //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// Motor //////////////////////////////////////////////////////////////
void motor_setup(){
  // DRV8302 specific code
  pinMode(EN_GATE,OUTPUT);
  digitalWrite(EN_GATE,HIGH);
  // M_OC  - enable over-current protection
  pinMode(M_OC,OUTPUT);
  digitalWrite(M_OC,LOW);
  // M_PWM  - enable 3pwm mode
  pinMode(M_PWM,OUTPUT);
  digitalWrite(M_PWM,HIGH);
  // OD_ADJ - set the maximum over-current limit possible
  // Better option would be to use voltage divisor to set exact value
  pinMode(OC_ADJ,OUTPUT);
  digitalWrite(OC_ADJ,HIGH);

  ledcAttachPin(PWM_A_pin,PWM_A_CH);
  ledcSetup(PWM_A_CH,PWM_freq,PWM_res);

  ledcAttachPin(PWM_B_pin,PWM_B_CH);
  ledcSetup(PWM_B_CH,PWM_freq,PWM_res);

  ledcAttachPin(PWM_C_pin,PWM_C_CH);
  ledcSetup(PWM_C_CH,PWM_freq,PWM_res);

  // step2_set_phase_voltage(2, 0, _3PI_2);
  Serial.println("zero ele angle START");
  step3_PWM_output(0.5, 0, 0);
  digitalWrite(2,HIGH);
  delay(3000);
  zero_electric_angle=_electrical_angle();
  digitalWrite(2,LOW);
  step3_PWM_output(0, 0, 0);
  Serial.print("zero ele angle: "); Serial.println(zero_electric_angle);
  delay(2000);
  // step2_set_phase_voltage(0, 0, _3PI_2);

}

void motor_output_loop(){
  // shaft_angle=float(as5600_0.readAngle())/4096*2*M_PI;
  float ele_ang=_electrical_angle();
  step2_set_phase_voltage(PI_pos_output, 0, ele_ang);
}

void step3_PWM_output(float Ua, float Ub, float Uc){
  // set limit
  Ua=constrain(Ua, 0.0f, voltage_limit);
  Ub=constrain(Ub, 0.0f, voltage_limit);
  Uc=constrain(Uc, 0.0f, voltage_limit);

  // find duty cycle
  dc_a=constrain(Ua/voltage_power_supply, 0.0f, 1.0f);
  dc_b=constrain(Ub/voltage_power_supply, 0.0f, 1.0f);
  dc_c=constrain(Uc/voltage_power_supply, 0.0f, 1.0f);

  ledcWrite(PWM_A_CH, dc_a*motor_pwm_max);
  ledcWrite(PWM_B_CH, dc_b*motor_pwm_max);
  ledcWrite(PWM_C_CH, dc_c*motor_pwm_max);
}

void step2_set_phase_voltage(float Uq, float Ud, float angle_el){
  angle_el = _normalize_angle(angle_el);

  Ualpha=-Uq*sin(angle_el); 
  Ubeta=Uq*cos(angle_el); 
  //                                    add offset of sine wave
  Ua=Ualpha+                            voltage_power_supply/2; 
  Ub=(sqrt(3)*Ubeta-Ualpha)/2          +voltage_power_supply/2;
  Uc=(-Ualpha-sqrt(3)*Ubeta)/2         +voltage_power_supply/2; 

  step3_PWM_output(Ua, Ub, Uc);
}

float _normalize_angle(float angle){
  float a=fmod(angle, 2*M_PI);
  return a>= 0 ? a : (a+2*M_PI);
}

int PP=7, DIR=-1;
float _electrical_angle(){
  return _normalize_angle(float(DIR*PP*as5600_0.readAngle())/4096*2*M_PI-zero_electric_angle);
}
////////////////////////////////////////////////////////////// Motor //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// BLUETOOTH //////////////////////////////////////////////////////////////
void read_bluetooth_data(){
  // fromt serial monitor
  if (Serial.available()){
    char data_in=Serial.read();  //Get next character
    if(data_in=='A'){KP_pos=float(Serial.parseInt())/1000;}// KI_pos=KP_pos/200;}
    if(data_in=='B'){KI_pos=float(Serial.parseInt())/1000000;}
    if(data_in=='C'){KD_pos=float(Serial.parseInt())/1000;}
    if(data_in=='T'){target_pos=float(Serial.parseInt())/100;}
    if(data_in=='a'){alpha_speed=float(Serial.parseInt())/100;}
  }
}
////////////////////////////////////////////////////////////// BLUETOOTH //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// Time //////////////////////////////////////////////////////////////
void loop_time_holder(){
  if(micros() - loop_start_time > (loop_time+50)) digitalWrite(2, HIGH);
  else digitalWrite(2, LOW);

  // unsigned long while_start_time=micros();
  while ((micros()- loop_start_time)<loop_time){} 
  // unsigned long while_end_time=micros();
  // Serial.print("while time:");Serial.print(while_end_time-while_start_time);Serial.print("  ");
  // pre_time=loop_start_time;
  loop_start_time = micros(); 
  // int dt=loop_start_time-pre_time;
  // Serial.print("dt:"); Serial.print(dt); Serial.print("\t");


}
////////////////////////////////////////////////////////////// Time //////////////////////////////////////////////////////////////