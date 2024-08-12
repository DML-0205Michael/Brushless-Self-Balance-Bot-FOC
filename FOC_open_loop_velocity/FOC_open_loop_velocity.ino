#define PWM_res 12
#define PWM_freq 10000

#define PWM_A_pin 16
#define PWM_A_CH 0

#define PWM_B_pin 17
#define PWM_B_CH 1

#define PWM_C_pin 18
#define PWM_C_CH 2

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
float voltage_limit=7; // must lower than power supply
float voltage_power_supply=8;
float shaft_angle=0, open_loop_timestamp=0;
float zero_electric_angle=0, Ualpha, Ubeta=0, Ua=0, Ub=0, Uc=0, dc_a=0, dc_b=0, dc_c=0;

#define   EN_GATE 4
#define   M_PWM 26 
#define   M_OC 27
#define   OC_ADJ 14

float _normalize_angle(float angle){
  float a=fmod(angle, 2*M_PI);
  return a>= 0 ? a : (a+2*M_PI);
}

float _electrical_angle(float shaft_angle, int pole_pairs){
  return (shaft_angle*pole_pairs);
}

void setup() {
  Serial.begin(460800);

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

  pinMode(2,OUTPUT);
  digitalWrite(2,HIGH);delay(100);digitalWrite(2,LOW);delay(100);
  digitalWrite(2,HIGH);delay(100);digitalWrite(2,LOW);delay(100);
  digitalWrite(2,HIGH);delay(100);digitalWrite(2,LOW);delay(100);

}

void loop() {
  step1_velocity_open_loop(25);

}

void step3_PWM_output(float Ua, float Ub, float Uc){
  // set limit
  Ua=_constrain(Ua, 0.0f, voltage_limit);
  Ub=_constrain(Ub, 0.0f, voltage_limit);
  Uc=_constrain(Uc, 0.0f, voltage_limit);

  // find duty cycle
  dc_a=_constrain(Ua/voltage_power_supply, 0.0f, 1.0f);
  dc_b=_constrain(Ub/voltage_power_supply, 0.0f, 1.0f);
  dc_c=_constrain(Uc/voltage_power_supply, 0.0f, 1.0f);

  ledcWrite(PWM_A_CH, dc_a*4096);
  ledcWrite(PWM_B_CH, dc_b*4096);
  ledcWrite(PWM_C_CH, dc_c*4096);
}

void step2_set_phase_voltage(float Uq, float Ud, float angle_el){
  angle_el = _normalize_angle(angle_el+zero_electric_angle);

  Ualpha=-Uq*sin(angle_el);
  Ubeta=Uq*cos(angle_el);
  //                                    add offset of sine wave
  Ua=Ualpha+                            voltage_power_supply/2;
  Ub=(sqrt(3)*Ubeta-Ualpha)/2          +voltage_power_supply/2;
  Uc=(-Ualpha-sqrt(3)*Ubeta)/2         +voltage_power_supply/2;

  step3_PWM_output(Ua, Ub, Uc);
}

float step1_velocity_open_loop(float target_velocity){
  unsigned long current_time=micros();
  float Ts=(current_time-open_loop_timestamp)*1e-6;
  if (Ts <- 0 || Ts>0.5f) Ts=1e-3;

  shaft_angle=_normalize_angle(shaft_angle+target_velocity*Ts);

  float Uq=voltage_limit/3;

  step2_set_phase_voltage(Uq, 0, _electrical_angle(shaft_angle, 7));

  open_loop_timestamp=current_time;
  
  return Uq;

}