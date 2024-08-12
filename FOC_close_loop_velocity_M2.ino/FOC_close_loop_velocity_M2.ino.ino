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
float enc_1_speed, enc_2_speed, previous_speed,total_angle;
float alpha=0.7;
////////////////////////////////////////////////////////////// Encoder //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// Motor //////////////////////////////////////////////////////////////
#define PWM_res 11
#define PWM_freq 30000
#define motor_pwm_max 2048

#define PWM_D_pin 32
#define PWM_D_CH 0

#define PWM_E_pin 33
#define PWM_E_CH 1

#define PWM_F_pin 25
#define PWM_F_CH 2

// #define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
float voltage_limit=7.6;
float voltage_power_supply=8.1;
float voltage_upper_bound=voltage_power_supply/2+voltage_limit;
float voltage_lower_bound=voltage_power_supply/2-voltage_limit;
float shaft_angle=0, open_loop_timestamp=0;
float zero_electric_angle=0, Ualpha, Ubeta=0, Ua=0, Ub=0, Uc=0, dc_a=0, dc_b=0, dc_c=0;
#define _3PI_2 4.71238898038f // =3*M_PI/2

#define   EN_GATE 14
#define   M_PWM 4
#define   M_OC 15
#define   OC_ADJ_2 13
////////////////////////////////////////////////////////////// Motor //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// Control //////////////////////////////////////////////////////////////
float KP_speed=0.030;
float KI_speed=0.003200;
float KD_speed=0;

float target_speed=0,previous_target_speed, err_sum_speed, pre_err; 
float PI_speed_output;

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
  pinMode(2,OUTPUT);

  AS5600_setup();
  mpu6050_setup();

  motor_setup();

  Iq_setup();

  digitalWrite(2,HIGH);delay(100);digitalWrite(2,LOW);delay(100);
  digitalWrite(2,HIGH);delay(100);digitalWrite(2,LOW);delay(100);
  digitalWrite(2,HIGH);delay(100);digitalWrite(2,LOW);delay(100);

}

void loop() {
  read_bluetooth_data();
  
  loop_time_holder();

  // read_encoder_speed();
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
  float Ib=-(analogRead(Ib_pin)-Ib_err);
  float Iq;
  // Serial.print("Ia:"); Serial.print(Ia); Serial.print(" ");
  // Serial.print("Ib:"); Serial.print(Ib); Serial.print(" ");
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
  current_angle_raw=as5600_1.readAngle();
  previous_angle_raw=as5600_1.readAngle();
  
}

void read_encoder_speed(){
  current_angle_raw=as5600_1.readAngle(); // 0~4096
  int dv=current_angle_raw-previous_angle_raw;
  if (dv<-1000){ // ex: dv=100-4000=-3900
    dv+=4096; // dv=100+(4096-4000)
  } else if (dv>1000) { // dv=4000-100
    dv-=4096; // dv=-((4096-4000)+100)=-(4096-4000+100)=-4096+4000-100
  }
  
  enc_1_speed=float(dv)/4096*2*M_PI*control_loop_freq;// rad/s
  // total_angle=total_angle+float(dv)/4096*2*M_PI;
  // Serial.print("tot ang:");Serial.print(total_angle);Serial.print("\t");
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
  // Serial.print("act:"); Serial.print(enc_1_speed); Serial.print("\t");
  // Serial.print("tar:"); Serial.print(target_speed); Serial.print("\t");
  // Serial.print("Uq:");Serial.print(PI_speed_output); // Serial.print();
  // Serial.print(-4100); // To freeze the lower limit
  // Serial.print(" ");
  // Serial.print(4100); // To freeze the upper limit
  // Serial.print(" ");



  read_encoder_speed();
  speed_PI(target_speed,enc_1_speed);

  float Iq=read_Iq();
  // Serial.print("Iq:"); Serial.print(Iq); Serial.print("\t");
}

void speed_PI(float target, float v1){
  // v1/v2: wheel 1/2 ang speed; target:target ang speed
  float speed=v1*(1-alpha)+previous_speed*alpha;
  previous_speed=speed;
  // Serial.print("wheel:");Serial.print(speed); Serial.print(" ");
  // pre_speed=speed;
  float err=target-speed;
  // Serial.print("Speed err:");Serial.print(err); Serial.print("\t");
  // if (abs(err)<7){
  err_sum_speed+=err; // Serial.print("err sum:");Serial.print(err_sum_speed); Serial.print("\t");
  err_sum_speed=constrain(err_sum_speed,-20000,20000);
  // } else err_sum_speed=0;
  // if (previous_target_speed != target) err_sum_speed=0;
  if (previous_target_speed*target <= 0 ) err_sum_speed=0;
  previous_target_speed=target;

  // Serial.print("err sum:");Serial.print(err_sum_speed); Serial.print("\t");
  float KP_out,KI_out,KD_out;
  KP_out=KP_speed*err;
  KI_out=KI_speed*err_sum_speed;
  KD_out=KD_speed*(err-pre_err);
  PI_speed_output=KP_out+KI_out; //+KD_out;
  // PI_speed_output=constrain(PI_speed_output,-3,3);
  // speed_PI_output=KP_speed*err+KI_speed*err_sum_speed+KD_speed*(err-pre_err_speed);
  pre_err=err;

  // Serial.print("PI_output:"); Serial.print(PI_speed_output); Serial.print("\t");
  // Serial.print("KP_out:");Serial.print(KP_out,5); Serial.print("\t");
  // Serial.print("KI_out:");Serial.print(KI_out,5); Serial.print("\t");
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
  pinMode(OC_ADJ_2,OUTPUT);
  digitalWrite(OC_ADJ_2,HIGH);

  ledcAttachPin(PWM_D_pin,PWM_D_CH);
  ledcSetup(PWM_D_CH,PWM_freq,PWM_res);

  ledcAttachPin(PWM_E_pin,PWM_E_CH);
  ledcSetup(PWM_E_CH,PWM_freq,PWM_res);

  ledcAttachPin(PWM_F_pin,PWM_F_CH);
  ledcSetup(PWM_F_CH,PWM_freq,PWM_res);

  // step2_set_phase_voltage(2, 0, _3PI_2);
  Serial.println("zero ele angle START");
  step3_PWM_output(2, 0, 0);
  digitalWrite(2,HIGH);
  delay(3000);
  zero_electric_angle=_electrical_angle();
  digitalWrite(2,LOW);
  step3_PWM_output(0, 0, 0);
  Serial.println("zero ele angle: "); Serial.println(zero_electric_angle);
  delay(5000);
  // step2_set_phase_voltage(0, 0, _3PI_2);

}

void motor_output_loop(){
  shaft_angle=float(as5600_1.readAngle())/4096*2*M_PI;
  // Serial.print("shaft ang:"); Serial.print(shaft_angle); Serial.print("\t");
  float ele_ang=_electrical_angle();
  // Serial.print("ele ang:"); Serial.print(ele_ang); Serial.print("\t"); // =_normalize_angle(float(DIR*PP*as5600_1.readAngle())/4096*2*M_PI)
  step2_set_phase_voltage(PI_speed_output, 0, ele_ang);
  // step1_velocity_open_loop(15);
}

void step3_PWM_output(float Ua, float Ub, float Uc){
  // set limit
  Ua=constrain(Ua, 0.0f, voltage_limit);
  Ub=constrain(Ub, 0.0f, voltage_limit);
  Uc=constrain(Uc, 0.0f, voltage_limit);

  // Ua=constrain(Ua, voltage_lower_bound, voltage_upper_bound);
  // Ub=constrain(Ub, voltage_lower_bound, voltage_upper_bound);
  // Uc=constrain(Uc, voltage_lower_bound, voltage_upper_bound);

  // Serial.print("A-B:"); Serial.print(Ua-Ub); Serial.print("\t");
  // Serial.print("B-C:"); Serial.print(Ub-Uc); Serial.print("\t");
  // Serial.print("A-C:"); Serial.print(Ua-Uc); Serial.print("\t");

  // find duty cycle
  dc_a=constrain(Ua/voltage_power_supply, 0.0f, 1.0f);
  dc_b=constrain(Ub/voltage_power_supply, 0.0f, 1.0f);
  dc_c=constrain(Uc/voltage_power_supply, 0.0f, 1.0f);

  Serial.print("dc_a:"); Serial.print(dc_a); Serial.print("\t");
  Serial.print("dc_b:"); Serial.print(dc_b); Serial.print("\t");
  Serial.print("dc_c:"); Serial.print(dc_c); Serial.print("\t");

  ledcWrite(PWM_D_CH, dc_a*motor_pwm_max);
  ledcWrite(PWM_D_CH, dc_b*motor_pwm_max);
  ledcWrite(PWM_F_CH, dc_c*motor_pwm_max);
}

void step2_set_phase_voltage(float Uq, float Ud, float angle_el){
  angle_el = _normalize_angle(angle_el);
  // Serial.print("ele ang:"); Serial.print(angle_el); Serial.print("\t"); // =ele_ang

  Ualpha=-Uq*sin(angle_el); // Uq=2, Ualpha=-0.1643
  Ubeta=Uq*cos(angle_el); // 1.99323
  //                                    add offset of sine wave
  Ua=Ualpha+                            voltage_power_supply/2; // 3.8357
  Ub=(sqrt(3)*Ubeta-Ualpha)/2          +voltage_power_supply/2; // 5.644
  Uc=(-Ualpha-sqrt(3)*Ubeta)/2         +voltage_power_supply/2; // 2.3559
  // Serial.print("Ualpha:"); Serial.print(Ualpha); Serial.print("\t");
  // Serial.print("Ubeta:"); Serial.print(Ubeta); Serial.print("\t");

  // Serial.print("Ua:"); Serial.print(Ua); Serial.print("\t");
  // Serial.print("Ub:"); Serial.print(Ub); Serial.print("\t");
  // Serial.print("Uc:"); Serial.print(Uc); Serial.print("\t");
  step3_PWM_output(Ua, Ub, Uc);
}

// float step1_velocity_open_loop(float target_velocity){
//   unsigned long current_time=micros();
//   float Ts=(current_time-open_loop_timestamp)*1e-6;
//   if (Ts <- 0 || Ts>0.5f) Ts=1e-3;

//   shaft_angle=_normalize_angle(shaft_angle+target_velocity*Ts);
//   Serial.print("shaft ang:"); Serial.print(shaft_angle); Serial.print("\t");
//   float Uq=voltage_limit;

//   step2_set_phase_voltage(Uq, 0, _electrical_angle(shaft_angle, 7));

//   open_loop_timestamp=current_time;
  
//   return Uq;

// }

float _normalize_angle(float angle){
  float a=fmod(angle, 2*M_PI);
  return a>= 0 ? a : (a+2*M_PI);
}

int PP=7, DIR=1;
float _electrical_angle(){
  // Serial.print("zero ang:"); Serial.print(zero_electric_angle); Serial.print("\t");
  // int temp1=DIR*PP*as5600_1.readAngle();
  // float temp2=1.f/4096.f*2.f*M_PI;
  // float temp3=float(temp1)*temp2;
  // Serial.print("temp1:"); Serial.print(temp1); Serial.print("\t");
  // Serial.print("temp2:"); Serial.print(temp2); Serial.print("\t");
  // return _normalize_angle(temp3);
  return _normalize_angle(float(DIR*PP*as5600_1.readAngle())/4096*2*M_PI-zero_electric_angle);
}
////////////////////////////////////////////////////////////// Motor //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// BLUETOOTH //////////////////////////////////////////////////////////////
void read_bluetooth_data(){
  // fromt serial monitor
  if (Serial.available()){
    char data_in=Serial.read();  //Get next character
    if(data_in=='A'){KP_speed=float(Serial.parseInt())/1000;}// KI_speed=KP_speed/200;}
    if(data_in=='B'){KI_speed=float(Serial.parseInt())/1000000;}
    if(data_in=='C'){KD_speed=float(Serial.parseInt())/1000;}
    if(data_in=='T'){target_speed=float(Serial.parseInt());}
    if(data_in=='R'){ESP.restart();}
    if(data_in=='a'){alpha=float(Serial.parseInt())/100;}
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