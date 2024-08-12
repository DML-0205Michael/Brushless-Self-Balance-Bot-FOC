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
#include <SimpleFOC.h>

MagneticSensorI2C sensor0 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);


void setup() {
  Serial.begin(460800);
  Serial.println();
  Serial.println("Serial start"); delay(200);
  
  // DO NOT PUT MPU6050 SETUP HERE: mpu6050_setup();

  _delay(750);

  // I2Cone.begin(19,18, 400000);   //SDA0,SCL0
  // I2Ctwo.begin(23,5, 400000);

  // OK
  // I2Cone.begin(21,22, 400000);   //SDA0,SCL0
  // I2Ctwo.begin(18,23, 400000); // SDA=18=S1, SCL=23=S2

  I2Cone.begin(21,22, 400000);   //SDA0,SCL0
  I2Ctwo.begin(19,23, 400000); // SDA=19, SCL=23

  // OK
  // I2Cone.begin(21,22, 400000);   //SDA0,SCL0
  // I2Ctwo.begin(18,13, 400000); // SDA=18=S1, SCL=21=S4/Vb

  // NOT OK: USE GPIO2=S3

  sensor0.init(&I2Cone);
  sensor1.init(&I2Ctwo);

  // PUT MPU6050 SETUP HERE, AFTER AS5600
  mpu6050_setup();

  Serial.println("set up ends."); delay(50);
  
}

void loop() {
  //loop_time_holder();
  read_mpu6050_angle_loop();

  sensor0.update(); // 如果simplefoc库版本为2.20及以上，取消这两行的注释
  sensor1.update();
  //_delay(200);
  Serial.print(sensor0.getAngle()); 
  Serial.print(" - "); 
  Serial.print(sensor1.getAngle());
  Serial.println();
}


unsigned long loop_start_time=0;
const int loop_time=10000; // micro seconds
void loop_time_holder(){
  // if(micros() - loop_start_time > (loop_time+50)) digitalWrite(2, HIGH);
  // else digitalWrite(2, LOW);

  // unsigned long while_start_time=micros();
  while ((micros()- loop_start_time)<loop_time){} 
  // unsigned long while_end_time=micros();
  // Serial.print("while time:");Serial.print(while_end_time-while_start_time);Serial.print("  ");
  loop_start_time = micros(); 
}
////////////////////////////////////////////// IMU //////////////////////////////////////////////
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
  if (!mpu.begin(0x68,&I2Cone)) {
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
  Serial.print("p_kf:");Serial.print(p_kf); Serial.print("\t");
}
////////////////////////////////////////////// IMU //////////////////////////////////////////////