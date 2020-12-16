//PD제어
#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9  
#define PIN_SERVO 10 
#define PIN_IR A0  
#define EMA_ALPHA 0.35     // EMA 필터 값을 결정하는 ALPHA 값. 작성자가 생각하는 최적값임.
// Framework setting
#define _DIST_TARGET 255  
#define _DIST_MIN 68
#define _DIST_MAX 410
#define DELAY_MICROS  1500
// Distance sensor
 

// Servo range
#define _DUTY_MIN 700 
#define _DUTY_NEU 1480    
#define _DUTY_MAX 2300 

// Servo speed control
#define _SERVO_ANGLE 30     
#define _SERVO_SPEED 800       

// Event periods
#define _INTERVAL_DIST 30 
#define _INTERVAL_SERVO 20 
#define _INTERVAL_SERIAL 100 

// PID parameters
#define _KD 120
#define _KP 1.5
#define _KI 0.001
//////////////////////
// global variables //
//////////////////////
float dist_min, dist_max, dist_raw ;
// Servo instance
Servo myservo; 

// Distance sensor
float ema_dist=0;            // EMA 필터에 사용할 변수
float dist_target; // location to send the ball
float dist_ema;
float samples_num = 3;
// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist;
bool event_servo, event_serial;

// Servo speed control
float duty_chg_per_interval; 
float duty_target, duty_curr;
// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;
const float coE[] = {0.0000022, -0.0024557, 1.6055455, 10.8782584};
void setup() {
// initialize GPIO pins for LED and attach servo 
 pinMode(PIN_LED,OUTPUT);
 digitalWrite(PIN_LED, 1);
 myservo.attach(PIN_SERVO); 

  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;
// initialize global variables
  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX; 
  duty_curr = _DUTY_NEU;
  dist_target = 255;
// move servo to neutral position
 myservo.writeMicroseconds(_DUTY_NEU); // [3228]

// initialize serial port
Serial.begin(57600); 

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / 180) * ((float)_INTERVAL_SERVO / 1000); 
}
  
void loop() {
/////////////////////
// Event generator // 
/////////////////////
  if (millis() >= last_sampling_time_dist + _INTERVAL_DIST) event_dist = true;
  if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO) event_servo = true;
  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL) event_serial = true;

////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
      event_dist = true; 
  // get a distance reading from the distance sensor
  
   dist_raw = ir_distance_filter_true();

  // PID control logic
    error_curr = _DIST_TARGET - dist_raw;
    pterm = _KP * error_curr;
    dterm =  _KD * (error_curr - error_prev);
    iterm += _KI * error_curr;
    control = dterm + pterm + iterm;

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control; 
    error_prev = error_curr;
  // [3133] keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;
    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN; 

    error_prev = error_curr;
    last_sampling_time_dist += millis(); 
  }
  
  if(event_servo) {
    event_servo = false; 
    if(duty_target > duty_curr) {
    duty_curr += duty_chg_per_interval;
    if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
    duty_curr -= duty_chg_per_interval;
    if(duty_curr < duty_target) duty_curr = duty_target;
    }
//     update servo position
    myservo.writeMicroseconds(duty_curr);
    last_sampling_time_servo = millis(); 

  }
  
  if(event_serial) {
    event_serial = false; 
    Serial.print("IR:");
   Serial.print(dist_raw);
   Serial.print(",T:");
   Serial.print(dist_target);
   Serial.print(",P:");
   Serial.print(map(pterm,-1000,1000,510,610));
   Serial.print(",D:");
   Serial.print(map(dterm,-1000,1000,510,610));
   Serial.print(",I:");
   Serial.print(map(iterm,-1000,1000,510,610));
   Serial.print(",DTT:");
   Serial.print(map(duty_target,1000,2000,410,510));
   Serial.print(",DTC:");
   Serial.print(map(duty_curr,1000,2000,410,510));
   Serial.println(",-G:245,+G:265,m:0,M:800");
    last_sampling_time_serial = millis(); 

  }
}

float ir_distance(void){ // return value unit: mm 
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}
float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}
// 박우혁 학생의 적외선 노이즈 필터 사용
float filtered_ir_distance(void){ // 아래로 떨어지는 형태의 스파이크를 제거 후, 위로 치솟는 스파이크를 제거하고 EMA필터를 적용함.
  // under_noise_filter를 통과한 값을 upper_nosie_filter에 넣어 최종 값이 나옴.
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  ema_dist = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*ema_dist;
  return ema_dist;
}
float ir_distance_filter_true(void) {
  float x = filtered_ir_distance();
  dist_raw = coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
  return dist_raw;

}
