////////////////////////////////////////////////////////////// IMU //////////////////////////////////////////////////////////////
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

Adafruit_MPU6050 mpu;

#include <EEPROM.h>
#define EEPROM_size 28 // ax, ay, az error, gx,gy,gz error

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
////////////////////////////////////////////////////////////// ENCODER //////////////////////////////////////////////////////////////
#include "AS5600.h"

AS5600 as5600_0(&Wire);
AS5600 as5600_1(&Wire1);

int current_angle_raw_1,previous_angle_raw_1;
int current_angle_raw_2,previous_angle_raw_2;
float enc_speed, enc_pos, previous_speed, total_angle;
float alpha=0.7;
////////////////////////////////////////////////////////////// ENCODER //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// BLUETOOTH //////////////////////////////////////////////////////////////
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
int update_interval=200; // time interval in ms for updating panel indicators
unsigned long last_time=0; // time of last update
char data_in; // data received from serial link
////////////////////////////////////////////////////////////// BLUETOOTH //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// CONTROL //////////////////////////////////////////////////////////////
float KP_stand=-0.030; // -0.050*0.6;
float KD_stand=0.380; // 0.450*0.6; 

float KP_speed=-0.9;
float KI_speed=KP_speed/200;

float KP_pos=0.000500;
float KI_pos=0.005;
float KD_pos=-0.120;
bool pos_en=0;

float KP_yaw=0.001; 
float KD_yaw=-0.1; 
float med_angle=-0.8;

float target_speed=0,previous_target_speed, err_sum_speed, target_yaw_speed=0; 

float current_pos, previous_target_pos, err_sum_pos,err_pre_pos;
float target_speed_BT, target_yaw_speed_BT;


unsigned long loop_start_time=0, pre_time;
const int loop_time=8000; // micro seconds
const int control_loop_freq=125;
////////////////////////////////////////////////////////////// CONTROL //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// Voltage monitor //////////////////////////////////////////////////////////////
#define Vb_pin 27
#define voltage_constant_a 0.009306248
#define voltage_constant_b 1.683
// float Vb; =voltage_power_supply
// void read_voltage_loop(){Vb=voltage_constant*analogRead(Vb_pin);} // volts
////////////////////////////////////////////////////////////// Voltage monitor //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// MOTOR //////////////////////////////////////////////////////////////
#define PWM_res 11
#define PWM_freq 30000
#define motor_pwm_max 2048

// M1
#define PWM_A_pin 16
#define PWM_A_CH 0

#define PWM_B_pin 17
#define PWM_B_CH 1

#define PWM_C_pin 18
#define PWM_C_CH 2

// M2
#define PWM_D_pin 32
#define PWM_D_CH 3

#define PWM_E_pin 33
#define PWM_E_CH 4

#define PWM_F_pin 25
#define PWM_F_CH 5

// #define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
float voltage_limit=7;
float voltage_power_supply=12;
// float voltage_upper_bound=voltage_power_supply/2+voltage_limit;
// float voltage_lower_bound=voltage_power_supply/2-voltage_limit;
float shaft_angle=0, open_loop_timestamp=0;
float zero_electric_angle_1=0, zero_electric_angle_2=0;
// float Ualpha, Ubeta=0, Ua=0, Ub=0, Uc=0; // dc_a=0, dc_b=0, dc_c=0;
float M1_speed, M2_speed;

#define   EN_GATE 14
#define   M_PWM 4
#define   M_OC 15
#define   OC_ADJ_1 26
#define   OC_ADJ_2 13
////////////////////////////////////////////////////////////// MOTOR //////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(460800);
  
  build_panel(); // BT setup

  AS5600_setup();
  mpu6050_setup();

  motor_setup();

  digitalWrite(2,HIGH); delay(100); digitalWrite(2,LOW); delay(100);
  digitalWrite(2,HIGH); delay(100); digitalWrite(2,LOW); delay(100);
  digitalWrite(2,HIGH); delay(100); digitalWrite(2,LOW); delay(100);
  digitalWrite(2,HIGH); delay(100); digitalWrite(2,LOW); delay(100);
  Serial.print("Set up ends");

}

void loop() {
  read_bluetooth_data();

  send_bluetooth_data();

  loop_time_holder();
  
  control_loop();
  motor_output_loop();

  Serial.println();
}

////////////////////////////////////////////// IMU //////////////////////////////////////////////
void mpu6050_setup(){ 
  Serial.flush();
  pinMode(2,OUTPUT);
  delay(2500);
  // Serial.println("wait for serial port input");
  // delay(5000); // wait for serial port input
  mpu6050_start();

  EEPROM.begin(EEPROM_size);

  // IMU_calibration(); 

  // if (digitalRead(KEY1)==LOW){ // if pushed
  //   digitalWrite(2,HIGH); delay(100); digitalWrite(2,LOW); delay(100);
  //   digitalWrite(2,HIGH); delay(100); digitalWrite(2,LOW); delay(100);
  //   delay(5000);
  //   IMU_calibration(); 
  // } else {
    digitalWrite(2,HIGH); delay(100); digitalWrite(2,LOW); delay(100);
    digitalWrite(2,HIGH); delay(100); digitalWrite(2,LOW); delay(100);
    digitalWrite(2,HIGH); delay(100); digitalWrite(2,LOW); delay(100);
    digitalWrite(2,HIGH); delay(100); digitalWrite(2,LOW); delay(100);
    // EEPROM.get(0, ax_error);
    // EEPROM.get(4, ay_error);
    // EEPROM.get(8, az_error);
    // EEPROM.get(12, wx_error);
    // EEPROM.get(16, wy_error);
    // EEPROM.get(20, wz_error);
    // EEPROM.get(24, med_angle);
  // }
  
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

void IMU_calibration() {
  digitalWrite(2,HIGH); 
  Serial.println("Start IMU calibration. ");
  // acceleration error
  int num_of_loop=1000; // number of sample
  int i=0;
  while (i < num_of_loop) {
    read_accel_gyro_raw();
    ax_error=ax_error+ax_float; // unit: m/s^2
    ay_error=ay_error+ay_float;
    az_error=az_error+az_float;
    i++;
  }
  ax_error=ax_error/num_of_loop; // unit: raw data
  ay_error=ay_error/num_of_loop;
  az_error=(az_error/num_of_loop-9.81); // should measure 1g when stationary. chip face is -z
  EEPROM.put(0, ax_error);
  EEPROM.put(4, ay_error);
  EEPROM.put(8, az_error);
  
  // angular velocity error
  i=0;
  while (i < num_of_loop) {
    read_accel_gyro_raw();
    wx_error=wx_error+wx; // unit: rad/sec
    wy_error=wy_error+wy;
    wz_error=wz_error+wz;
    i++;
  }
  wx_error=wx_error/num_of_loop; // rad/sec
  wy_error=wy_error/num_of_loop;
  wz_error=wz_error/num_of_loop;
  EEPROM.put(12, wx_error);
  EEPROM.put(16, wy_error);
  EEPROM.put(20, wz_error);
  EEPROM.commit();
  delay(500);
  Serial.println("EEPROM wrote");
  delay(2000);

  Serial.println("Minus these errors when converting from int16_t to float. ");
  Serial.println("Acceleration error (m/s^2): ");
  Serial.print("ax_error: ");
  Serial.println(ax_error);
  Serial.print("ay_error: ");
  Serial.println(ay_error);
  Serial.print("az_error: ");
  Serial.println(az_error);
  Serial.println();

  Serial.println("Angular velocity error (rad/s): ");
  Serial.print("wx_error: ");
  Serial.println(wx_error);
  Serial.print("wy_error: ");
  Serial.println(wy_error);
  Serial.print("wz_error: ");
  Serial.println(wz_error);

  Serial.println("Accel and gyro Calibration finished.");
  Serial.println();
  digitalWrite(2,LOW); delay(100);
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
  Serial.print("p_kf:");Serial.print(p_kf); // Serial.print("\t");
}
////////////////////////////////////////////// IMU //////////////////////////////////////////////
////////////////////////////////////////////// ENCODER //////////////////////////////////////////////
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
  current_angle_raw_1=as5600_0.readAngle();
  previous_angle_raw_1=as5600_0.readAngle();
  
  current_angle_raw_2=as5600_1.readAngle();
  previous_angle_raw_2=as5600_1.readAngle();

  current_pos=0;
}

void read_encoder_speed(){
  current_angle_raw_1=as5600_0.readAngle(); // 0~4096
  int dv_1=current_angle_raw_1-previous_angle_raw_1;
  if (dv_1<-1000){ // ex: dv_1=100-4000=-3900
    dv_1+=4096; // dv_1=100+(4096-4000)
  } else if (dv_1>1000) { // dv_1=4000-100
    dv_1-=4096; // dv_1=-((4096-4000)+100)=-(4096-4000+100)=-4096+4000-100
  }

  current_angle_raw_2=as5600_1.readAngle(); // 0~4096
  int dv_2=current_angle_raw_2-previous_angle_raw_2;
  if (dv_2<-1000){ 
    dv_2+=4096; 
  } else if (dv_2>1000) { 
    dv_2-=4096; 
  }
  
  // need to reverse dv_2, speed=dv_1 + -dv_2
  enc_speed=float(dv_1-dv_2)/4096*2*M_PI*control_loop_freq;// rad/s
  enc_pos+=enc_speed;

  previous_angle_raw_1=current_angle_raw_1;
  previous_angle_raw_2=current_angle_raw_2;
}
////////////////////////////////////////////// ENCODER //////////////////////////////////////////////
////////////////////////////////////////////// CONTROL //////////////////////////////////////////////
void control_loop(){
  // Serial.print("TS:");Serial.print(target_speed); Serial.print("\t");
  // Serial.print("KP:");Serial.print(KP_speed,3); Serial.print("\t");
  // Serial.print("KI:");Serial.print(KI_speed,7); Serial.print("\t");
  float speed_PI_target=0;

  read_encoder_speed();
  if (target_speed_BT==0 && pos_en){
    speed_PI_target=pos_PID(current_pos,enc_pos);
  } else {
    enc_pos=0;
    speed_PI_target=target_speed_BT;
  }
  Serial.print("enc_pos:");Serial.print(enc_pos); Serial.print("\t");
  Serial.print("en pos:");Serial.print(pos_en); Serial.print("\t");
  float speed_PI_output=speed_PI(speed_PI_target, enc_speed);
  // previous_target_speed=target_speed;
  // Serial.print("Speed PI output:");Serial.print(speed_PI_output); Serial.print("\t");
  // Serial.print("KP:");Serial.print(KP_stand); Serial.print("\t");
  // Serial.print("KD:");Serial.print(KD_stand); Serial.print("\t");
  read_mpu6050_angle_loop();
  // Serial.print("MED:");Serial.print(med_angle); Serial.print("\t");
  // Serial.print("PI+MED:");Serial.print(speed_PI_output+med_angle); Serial.print("\t");
  float stand_PD_output=stand_PD(speed_PI_output+med_angle, p_kf, wx); 
    
  // Serial.print("TY:");Serial.print(target_yaw_speed); Serial.print("\t");
  // Serial.print("KP:");Serial.print(KP_yaw); Serial.print("\t");
  // Serial.print("KD:");Serial.print(KD_yaw); Serial.print("\t");
  float yaw_PD_output=yaw_PD(target_yaw_speed_BT,wz);

  if (abs(p_kf)<50){
  M1_speed=stand_PD_output-yaw_PD_output;
  M2_speed=stand_PD_output+yaw_PD_output;

  // M1_speed=stand_PD_output+speed_PI_output-yaw_PD_output;
  // M2_speed=stand_PD_output+speed_PI_output+yaw_PD_output;
  // 临时调试,旋转方向
  // M1_speed=speed_PI_output-yaw_PD_output;
  // M2_speed=speed_PI_output+yaw_PD_output;

  M1_speed=constrain(M1_speed, -2,2);
  M2_speed=constrain(M2_speed, -2,2);
  } else if (abs(p_kf)>=50){
    M1_speed=0;
    M2_speed=0;
    err_sum_speed=0;
    err_sum_pos=0;
    enc_pos=0;
  }
  Serial.print("M1:");Serial.print(M1_speed); Serial.print("\t");
  Serial.print("M2:");Serial.print(M2_speed); Serial.print("\t");
}

float stand_PD(float target, float actual, float ang_speed){
  float err=target-actual;

  float KP_out, KD_out;
  KP_out=KP_stand*err;
  KD_out=KD_stand*ang_speed;

  float output=KP_out+KD_out;

  return output; 
  // stand_PD_output=KP_stand*err+KD_stand*ang_speed+KI_stand*err_stand_sum*KI_flag;
  // Serial.print("KP_out:");Serial.print(KP_out); Serial.print("\t");
  // Serial.print("KI_out:");Serial.print(KI_out); Serial.print("\t");
  // Serial.print("KD_out:");Serial.print(KD_out); Serial.print("\t");
}

// float pre_speed=0;
float speed_PI(float target, float actual_speed){
  float filtered_speed=actual_speed*(1.f-alpha)+previous_speed*alpha;
  previous_speed=filtered_speed;
  // Serial.print("wheel:");Serial.print(filtered_speed); Serial.print(" ");
  // pre_speed=speed;
  float err=target-filtered_speed;
  // Serial.print("Speed err:");Serial.print(err); Serial.print("\t");
  // if (abs(err)<7){
  err_sum_speed+=err; // Serial.print("err sum:");Serial.print(err_sum_speed); Serial.print("\t");
  err_sum_speed=constrain(err_sum_speed,-20000,20000);
  // } else err_sum_speed=0;
  // if (previous_target_speed != target) err_sum_speed=0;
  if (previous_target_speed*target <= 0 ) err_sum_speed=0;
  previous_target_speed=target;

  // Serial.print("err sum:");Serial.print(err_sum_speed); Serial.print("\t");
  float KP_out,KI_out;
  KP_out=KP_speed*err;
  KI_out=KI_speed*err_sum_speed;
  float output=KP_out+KI_out; //+KD_out;

  // Serial.print("err:");Serial.print(err); Serial.print("\t");
  // Serial.print("tar:");Serial.print(target); Serial.print("\t");
  // Serial.print("act:");Serial.print(actual_speed); Serial.print("\t");
  // Serial.print("KP_out:");Serial.print(KP_out); Serial.print("\t");
  // Serial.print("KI_out:");Serial.print(KI_out); Serial.print("\t");
  // Serial.print("sp PI output:");Serial.print(output); Serial.print("\t");
  return output;

}

float pos_PID(float target, float actual){
  float err=target-actual;

  err_sum_pos+=err;
  err_sum_pos=constrain(err_sum_pos,-20000,20000);

  if (previous_target_pos*target <= 0 ) err_sum_pos=0;
  previous_target_pos=target;

  // Serial.print("err sum:");Serial.print(err_sum_speed); Serial.print("\t");
  float KP_out,KI_out,KD_out;
  KP_out=KP_pos*err;
  KI_out=KI_pos*err_sum_pos;
  KD_out=KD_pos*(err-err_pre_pos);
  float output=KP_out+KI_out+KD_out;
  err_pre_pos=err;

  // Serial.print("err:");Serial.print(err); Serial.print("\t");
  // Serial.print("tar:");Serial.print(target); Serial.print("\t");
  // Serial.print("act:");Serial.print(actual_speed); Serial.print("\t");
  // Serial.print("KP_out:");Serial.print(KP_out); Serial.print("\t");
  // Serial.print("KI_out:");Serial.print(KI_out); Serial.print("\t");
  // Serial.print("output:");Serial.print(output); Serial.print("\t");
  return output;

}

float yaw_PD(float target, float ang_speed){
  float output;
  if (target==0){
    output=KD_yaw*ang_speed;
  } else {
    output=KP_yaw*target;
  }
  return output;
}
////////////////////////////////////////////// CONTROL //////////////////////////////////////////////
////////////////////////////////////////////////////////////// BLUETOOTH //////////////////////////////////////////////////////////////
void read_bluetooth_data(){
  // fromt serial monitor
  if (Serial.available()){
    data_in=Serial.read();  //Get next character
    if(data_in=='A'){KP_stand=float(Serial.parseInt())/1000;}
    if(data_in=='B'){KD_stand=float(Serial.parseInt())/1000;}
    if(data_in=='C'){KP_speed=float(Serial.parseInt())/1000;KI_speed=KP_speed/200;}
    if(data_in=='D'){KI_speed=float(Serial.parseInt())/1000000;}
    if(data_in=='E'){KP_yaw=float(Serial.parseInt())/1000;}
    if(data_in=='F'){KD_yaw=float(Serial.parseInt())/1000;}
    if(data_in=='G'){KP_pos=float(Serial.parseInt())/1000000;KI_pos=KP_pos/200;}
    if(data_in=='H'){KI_pos=float(Serial.parseInt())/1000;}
    if(data_in=='I'){KD_pos=float(Serial.parseInt())/1000;}
    if(data_in=='N'){med_angle=float(Serial.parseInt())/100;}
    if(data_in=='a'){alpha=float(Serial.parseInt())/100;}
    if(data_in=='R'){ESP.restart();}
  }

  // From bluetooth
  if (SerialBT.available()){
    data_in=SerialBT.read();  //Get next character
    
    // from terminal
    if(data_in=='A'){KP_stand=float(SerialBT.parseInt());}
    else if(data_in=='B'){KD_stand=float(SerialBT.parseInt());}
    else if(data_in=='C'){KP_speed=float(SerialBT.parseInt())/1000;KI_speed=KP_speed/200;}
    else if(data_in=='D'){KI_speed=float(SerialBT.parseInt())/1000000;}
    else if(data_in=='E'){KP_yaw=float(SerialBT.parseInt());}
    else if(data_in=='F'){KD_yaw=float(SerialBT.parseInt());}
    else if(data_in=='N'){med_angle=p_kf;EEPROM.put(24, med_angle);EEPROM.commit();}

    // from others
    else if(data_in=='M'){ESP.restart();} //Button restart ESP32
    else if(data_in=='L'){pos_en=!pos_en;} 

    if(data_in=='P'){ // right joy stick
      while(true){
        if (SerialBT.available()){
          data_in=SerialBT.read();  //Get next character
          if(data_in=='Y') target_speed_BT=-float(SerialBT.parseInt())*0.1; // encoder speed
          if(data_in=='P') break; // End character
        }
      }
    }

    if(data_in=='W'){ // left joy stick
      while(true){
        if (SerialBT.available()){
          data_in=SerialBT.read();  //Get next character
          if(data_in=='X') target_yaw_speed_BT=-SerialBT.parseInt();
          if(data_in=='W') break; // End character
        }
      }
    }

  }
}

void send_bluetooth_data(){
  unsigned long t=millis();
  if ((t-last_time)>update_interval){
    String temp;
    last_time=t;
    // voltage monitor
    float Vb=voltage_constant_a*analogRead(Vb_pin)+voltage_constant_b;
    // Serial.print("Vb: "); Serial.print(Vb); Serial.print("\t");
    SerialBT.print("*V"+String(Vb)+"*");


    // SerialBT.print("*M"+temp+"*");
    // Serial.print("enc_1+2===:");Serial.print(enc_1_speed+enc_2_speed); Serial.print(" ");
    // // Serial.print("enc_2_speed:");Serial.print(enc_2_speed); Serial.print(" ");
    // Serial.print("a:");Serial.print(a); Serial.print(" ");
    // Serial.print("previous_speed:");Serial.print(previous_speed); Serial.print(" ");
    // Serial.print("speed_temp:");Serial.print(speed_temp); Serial.print(" ");
    int state=0;
    if (pos_en) state=1;
    else state=0;
    SerialBT.print("*M"+String(state)+"*"); 
    SerialBT.print("*I"+String(med_angle)+"*");
    SerialBT.print("*G"+String(p_kf)+"*");
    // SerialBT.print("*F"+String(distance)+"*");
  }
}

void build_panel(){
  SerialBT.begin(460800);
  SerialBT.println("*.kwl");
  SerialBT.println("clear_panel()");
  SerialBT.println("set_grid_size(12,9)");
  SerialBT.println("add_text_box(0,1,3,C,M2,245,240,245,)");
  SerialBT.println("add_text_box(6,1,3,C,Ctrl Mode,245,240,245,)");
  SerialBT.println("add_text_box(3,1,3,C,Vb,245,240,245,)");
  SerialBT.println("add_text_box(9,1,3,C,M1,245,240,245,)");
  SerialBT.println("add_text_box(0,2,2,L,0,245,240,245,G)");
  SerialBT.println("add_text_box(10,2,2,L,0,245,240,245,F)");
  SerialBT.println("add_text_box(0,0,3,C,,245,240,245,I)");
  SerialBT.println("add_text_box(9,0,3,C,,245,240,245,H)");
  SerialBT.println("add_text_box(3,0,3,C,,245,240,245,V)");
  SerialBT.println("add_text_box(6,0,3,C,BT,245,240,245,M)");
  SerialBT.println("add_button(7,6,1,M,)");
  SerialBT.println("add_slider(1,9,3,0,100,0,A,A,0)");
  SerialBT.println("add_slider(0,4,4,0,100,15,S,,0)");
  SerialBT.println("add_slider(1,8,2,30,200,30,L,,0)");
  SerialBT.println("add_slider(8,8,2,30,200,164,R,,0)");
  SerialBT.println("add_free_pad(8,5,-255,255,0,0,P,P)");
  SerialBT.println("add_free_pad(1,5,-255,255,0,0,W,W)");
  SerialBT.println("add_send_box(4,2,5,S50,,)");
  SerialBT.println("set_panel_notes(-,,,)");
  SerialBT.println("run()");
  SerialBT.println("*");
}
////////////////////////////////////////////////////////////// BLUETOOTH //////////////////////////////////////////////////////////////
////////////////////////////////////////////// MOTOR OUTPUT //////////////////////////////////////////////
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
  pinMode(OC_ADJ_1,OUTPUT);
  digitalWrite(OC_ADJ_1,HIGH);
  pinMode(OC_ADJ_2,OUTPUT);
  digitalWrite(OC_ADJ_2,HIGH);

  ledcAttachPin(PWM_A_pin,PWM_A_CH);
  ledcSetup(PWM_A_CH,PWM_freq,PWM_res);

  ledcAttachPin(PWM_B_pin,PWM_B_CH);
  ledcSetup(PWM_B_CH,PWM_freq,PWM_res);

  ledcAttachPin(PWM_C_pin,PWM_C_CH);
  ledcSetup(PWM_C_CH,PWM_freq,PWM_res);

  ledcAttachPin(PWM_D_pin,PWM_D_CH);
  ledcSetup(PWM_D_CH,PWM_freq,PWM_res);

  ledcAttachPin(PWM_E_pin,PWM_E_CH);
  ledcSetup(PWM_E_CH,PWM_freq,PWM_res);

  ledcAttachPin(PWM_F_pin,PWM_F_CH);
  ledcSetup(PWM_F_CH,PWM_freq,PWM_res);

  // M1
  Serial.println("zero ele angle START");
  step3_PWM_output(1, 0, 0,1);
  digitalWrite(2,HIGH);
  delay(2000);
  zero_electric_angle_1=_electrical_angle(1);
  digitalWrite(2,LOW);
  step3_PWM_output(0, 0, 0,1);
  // Serial.println("zero ele angle: "); Serial.println(zero_electric_angle_1);

  // M2
  step3_PWM_output(1, 0, 0,2);
  digitalWrite(2,HIGH);
  delay(2000);
  zero_electric_angle_2=_electrical_angle(2);
  digitalWrite(2,LOW);
  step3_PWM_output(0, 0, 0,2);

  delay(5000);
}

void motor_output_loop(){
  // shaft_angle=float(as5600_0.readAngle())/4096*2*M_PI;
  // Serial.print("shaft ang:"); Serial.print(shaft_angle); Serial.print("\t");
  float ele_ang_1=_electrical_angle(1);
  float ele_ang_2=_electrical_angle(2);
  step2_set_phase_voltage(M1_speed, ele_ang_1, 1);
  step2_set_phase_voltage(M2_speed, ele_ang_2, 2);
}

void step3_PWM_output(float Ua, float Ub, float Uc, int motor){
  // set limit
  Ua=constrain(Ua, 0.0f, voltage_limit);
  Ub=constrain(Ub, 0.0f, voltage_limit);
  Uc=constrain(Uc, 0.0f, voltage_limit);

  // find duty cycle
  float dc_a=constrain(Ua/voltage_power_supply, 0.0f, 1.0f);
  float dc_b=constrain(Ub/voltage_power_supply, 0.0f, 1.0f);
  float dc_c=constrain(Uc/voltage_power_supply, 0.0f, 1.0f);

  if (motor==1){
    ledcWrite(PWM_A_CH, dc_a*motor_pwm_max);
    ledcWrite(PWM_B_CH, dc_b*motor_pwm_max);
    ledcWrite(PWM_C_CH, dc_c*motor_pwm_max);
  } else if (motor==2){
    ledcWrite(PWM_D_CH, dc_a*motor_pwm_max);
    ledcWrite(PWM_E_CH, dc_b*motor_pwm_max);
    ledcWrite(PWM_F_CH, dc_c*motor_pwm_max);
  }
}

void step2_set_phase_voltage(float Uq, float angle_el, int motor){
  angle_el = _normalize_angle(angle_el);

  float Ualpha=-Uq*sin(angle_el); 
  float Ubeta=Uq*cos(angle_el); 
  //                                    add offset of sine wave
  float Ua=Ualpha+                            voltage_power_supply/2; 
  float Ub=(sqrt(3)*Ubeta-Ualpha)/2          +voltage_power_supply/2; 
  float Uc=(-Ualpha-sqrt(3)*Ubeta)/2         +voltage_power_supply/2; 

  step3_PWM_output(Ua, Ub, Uc, motor);
}

float _normalize_angle(float angle){
  float a=fmod(angle, 2*M_PI);
  return a>= 0 ? a : (a+2*M_PI);
}

int PP=7, DIR_1=-1, DIR_2=1;
float _electrical_angle(int motor){
  if (motor==1){
    return _normalize_angle(float(DIR_1*PP*as5600_0.readAngle())/4096*2*M_PI-zero_electric_angle_1);
  } else if (motor==2){
    return _normalize_angle(float(DIR_2*PP*as5600_1.readAngle())/4096*2*M_PI-zero_electric_angle_2);
  }
}
////////////////////////////////////////////// MOTOR OUTPUT //////////////////////////////////////////////
////////////////////////////////////////////// TIME //////////////////////////////////////////////
// unsigned long pre_time;
void loop_time_holder(){
  if(micros() - loop_start_time > (loop_time+50)) digitalWrite(2, HIGH);
  else digitalWrite(2, LOW);

  // unsigned long while_start_time=micros();
  while ((micros()- loop_start_time)<loop_time){} 
  // unsigned long while_end_time=micros();
  // Serial.print("while time:");Serial.print(while_end_time-while_start_time);Serial.print("  ");
  
  pre_time=loop_start_time;
  loop_start_time = micros(); 
  int dt=loop_start_time-pre_time;
  //Serial.print("dt:"); Serial.print(dt); Serial.print("\t");

}

////////////////////////////////////////////// TIME //////////////////////////////////////////////