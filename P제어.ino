#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9  
#define PIN_SERVO 10 
#define PIN_IR A0  
// Framework setting
#define _DIST_TARGET 255 
#define _DIST_MIN 68
#define _DIST_MAX 410

// Distance sensor
#define _DIST_ALPHA 0.35   

// Servo range
#define _DUTY_MIN 1160  
#define _DUTY_NEU 1480        
#define _DUTY_MAX 1800  

// Servo speed control
#define _SERVO_ANGLE 30        
#define _SERVO_SPEED 800       

// Event periods
#define _INTERVAL_DIST 30 
#define _INTERVAL_SERVO 20 
#define _INTERVAL_SERIAL 100 

// PID parameters
#define _KP 1.7

//////////////////////
// global variables //
//////////////////////
float dist_min, dist_max, dist_raw ; // [3228]
// Servo instance
Servo myservo; 

// Distance sensor
float dist_target; // location to send the ball
float dist_ema;

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
 // [3228]
  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;

// initialize global variables
  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX; 
  duty_curr = _DUTY_NEU;

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
    pterm = error_curr;
    control = _KP * pterm;

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control; 

  // [3133] keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;
    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN; 
  
    last_sampling_time_dist += _INTERVAL_DIST; 
  }
  
  if(event_servo) {
    event_servo = true; 
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
    last_sampling_time_servo += _INTERVAL_SERVO; 

  }
  
  if(event_serial) {
    event_serial = true; 
    Serial.print("dist_ir:");
    Serial.println(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
    last_sampling_time_serial += _INTERVAL_SERIAL; 

  }
}

float ir_distance(void){ // return value unit: mm 
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

float ir_distance_filtered(void) {
  static float val = _DIST_TARGET;
  static float dist_ema = 0;
  float raw = ir_distance();
  if (raw >= _DIST_MIN && raw <= _DIST_MAX)
      val = raw;
  dist_ema = (1.0 - _DIST_ALPHA) * val + _DIST_ALPHA * dist_ema;
  return dist_ema;
}


float ir_distance_filter_true(void) {
  float x = ir_distance_filtered();
  dist_raw = coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
  return dist_raw;

}
