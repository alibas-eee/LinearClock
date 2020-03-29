
#include <Wire.h> // must be included here so that Arduino library object file references work
#include <RtcDS1307.h>
RtcDS1307<TwoWire> Rtc(Wire);
RtcDateTime now;

#define PIN_ENCODER_0A 2
#define PIN_ENCODER_1A 3

#define PIN_BTN_HOUR    12
#define PIN_BTN_MIN     A3

#define PIN_LED_R       9
#define PIN_LED_G       10
#define PIN_LED_B       11

#define PIN_LIMIT_UPPER_A 5
#define PIN_LIMIT_UPPER_B 4

#define PIN_LIMIT_LOWER_A 7
#define PIN_LIMIT_LOWER_B 6

#define PIN_MOTOR_HOUR_A  A0
#define PIN_MOTOR_HOUR_B  13

#define PIN_MOTOR_MIN_A   A1  
#define PIN_MOTOR_MIN_B   A2 

//ERROR
#define NO_ERROR      		0
#define ERROR_LIMIT_M       1
#define ERROR_LIMIT_H       2
#define ERROR_MOTOR_M_UP    3
#define ERROR_MOTOR_M_DOWN  4
#define ERROR_MOTOR_H_UP    5
#define ERROR_MOTOR_H_DOWN  6

#define DIR_STOP      0
#define DIR_UP        1
#define DIR_DOWN      2

#define ENCODER_MAX_LIMIT_H 11700
#define ENCODER_MAX_H 10800

#define ENCODER_MAX_LIMIT_M 11700
#define ENCODER_MAX_M 10800

#define ENCODER_H_STEP ENCODER_MAX_H/12
#define ENCODER_M_STEP ENCODER_MAX_M/60
#define OFFSET_M	0
#define OFFSET_H	-(ENCODER_H_STEP/2)

#define TOLERANCE     10

int pos_min = 0;
int pos_hour = 0;

struct S_LIMIT {
  byte U_A: 1;
  byte U_B: 1;
  byte L_A: 1;
  byte L_B: 1;
  byte btn_min: 1;
  byte btn_hour: 1;
  byte cal_min: 1;
  byte cal_hour: 1;
};

union U_LIMIT
{
  S_LIMIT bits;
  byte val;
};

struct S_ERROR {
  byte error;
  byte LastLimit;
  int  LastMPos;
  int  LastHPos;
};

U_LIMIT u_limit;
S_LIMIT s_limit;
S_ERROR s_error;
byte m_motor_state = DIR_STOP;
byte h_motor_state = DIR_STOP;

byte on_off;
byte M = 2, H = 2;

void CheckError(S_LIMIT limit, S_ERROR error) {

  if (pos_min < -ENCODER_MAX_LIMIT_M || pos_min > ENCODER_MAX_LIMIT_M + (ENCODER_MAX_LIMIT_M / 10) ) {
    s_error.error = ERROR_LIMIT_M;// limit error
  }
  else if (pos_hour < -ENCODER_MAX_LIMIT_M  || pos_hour > ENCODER_MAX_LIMIT_M + (ENCODER_MAX_LIMIT_M / 10) ) {
    s_error.error = ERROR_LIMIT_H;// limit error
  }
  /*else if(m_motor_state == DIR_UP && pos_min == s_error.LastMPos)
    {
  	s_error.error = ERROR_MOTOR_M_UP;// motor or encoder error
    }
    else if(m_motor_state == DIR_DOWN && pos_min == s_error.LastMPos)
    {
  	s_error.error = ERROR_MOTOR_M_DOWN;// motor or encoder error
    }
    else if(h_motor_state == DIR_UP && pos_hour == s_error.LastHPos)
    {
  	s_error.error = ERROR_MOTOR_H_UP;// motor or encoder error
    }
    else if(h_motor_state == DIR_DOWN && pos_hour == s_error.LastHPos)
    {
  	s_error.error = ERROR_MOTOR_H_DOWN;// motor or encoder error
    }*/


  s_error.LastMPos = pos_min;
  s_error.LastHPos = pos_hour;
  s_error.LastLimit = u_limit.val;
}


void ISR_ENCODER_0() {
  if (m_motor_state == DIR_UP) {
    pos_min ++;
  }
  else if (m_motor_state == DIR_DOWN) {
    pos_min --;
  }
  else { pos_min ++;}
}
void ISR_ENCODER_1() {
  if (h_motor_state == DIR_UP) {
    pos_hour ++;
  }
  else if (h_motor_state == DIR_DOWN) {
    pos_hour --;
  }
  else {pos_hour ++; }
}


void CheckLimit();
void MotorMinute();
void MotorHour();

void setup() {
  Serial.begin(9600);
  pinMode(PIN_ENCODER_0A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_1A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_0A), ISR_ENCODER_0, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_1A), ISR_ENCODER_1, RISING);
  pinMode(PIN_BTN_HOUR, 0);
  pinMode(PIN_BTN_MIN, 0);

  pinMode(PIN_ENCODER_0A, 0);
  pinMode(PIN_ENCODER_1A, 0);

  pinMode(PIN_LIMIT_UPPER_A, 0);
  pinMode(PIN_LIMIT_UPPER_B, 0);
  pinMode(PIN_LIMIT_LOWER_A, 0);
  pinMode(PIN_LIMIT_LOWER_B, 0);

  pinMode(PIN_MOTOR_HOUR_A, 1);
  pinMode(PIN_MOTOR_HOUR_B, 1);
  pinMode(PIN_MOTOR_MIN_A, 1);
  pinMode(PIN_MOTOR_MIN_B, 1);

  pinMode(PIN_LED_R, 1);
  pinMode(PIN_LED_G, 1);
  pinMode(PIN_LED_B, 1);
  delay(1000);
  Rtc.Begin();
  if (!Rtc.GetIsRunning()) {
    Serial.println("RTC was not actively running, starting now");
    Rtc.SetIsRunning(true);
  }
}

void ReadClock()
{
  now = Rtc.GetDateTime();
  H = (now.Hour() % 12) + 1;
  M = (now.Minute() % 60) + 1;
}

void loop() {
  ReadClock();
  CheckLimit(u_limit.bits);
  if (s_error.error > NO_ERROR)
  {
    MotorHour(DIR_STOP);
    MotorMinute(DIR_STOP);
    Led(0, 255, 0);
  }
  else {    
    ControlMotorMinute(u_limit.bits);
    ControlMotorHour(u_limit.bits);
    Led(H, M,u_limit.bits);
  }
  delay(10);
  SetClock(u_limit.bits);
  CheckError(u_limit.bits, s_error);
  print();
}

void print()
{
  Serial.print(u_limit.val, BIN);

  Serial.print(" E=");
  Serial.print(s_error.error);

  Serial.print(" H=");
  Serial.print(H);
  Serial.print(" M=");
  Serial.print(M);

  Serial.print(" EH=");
  Serial.print(pos_hour);
  Serial.print(" EM=");
  Serial.println(pos_min);


}


void Led(byte r, byte g, byte b ) {
  analogWrite(PIN_LED_R, r);
  analogWrite(PIN_LED_G, g);
  analogWrite(PIN_LED_B, b);
}

void Led(byte h, byte m, S_LIMIT &limit) {

  byte led_R = (1200 - m * h) / 4.7;
  byte led_B = m * h / 4.7 ;
  byte led_G = 0;
  if(limit.cal_hour== 0 ||  limit.cal_min == 0)
  {
	led_G = 255;  
  }
  
  

  analogWrite(PIN_LED_R, led_R);
  analogWrite(PIN_LED_G, led_G);
  analogWrite(PIN_LED_B, led_B);
}

void ControlMotorHour(S_LIMIT &limit) {
  int target_pos;
  target_pos = ENCODER_H_STEP * H  + OFFSET_H;
  if ( m_motor_state == DIR_STOP)
  {
    if (limit.cal_hour == 0) {
      MotorHour(DIR_DOWN);
      if (limit.L_A == 1) {
        limit.cal_hour = 1;
        pos_hour = 0;
      }
    }
    else {
      if ((target_pos - TOLERANCE) > pos_hour) {
		if(limit.U_A == 1)
		{
			MotorHour(DIR_STOP);
		}
		else
		{
			MotorHour(DIR_UP);
		}
      }
	  else if((target_pos + 2*ENCODER_H_STEP) < pos_hour)// position invalid
	  {
		limit.cal_hour = 0; 
	  }
      else {
        MotorHour(DIR_STOP);
      }
    }
  }
  else {
    MotorHour(DIR_STOP);
  }

}

void ControlMotorMinute(S_LIMIT &limit) {
  int target_pos;
  target_pos = ENCODER_M_STEP * M + OFFSET_M;

  if (limit.cal_min == 0) {
    MotorMinute(DIR_DOWN);
    if (limit.L_B == 1) {
      limit.cal_min = 1;
      pos_min = 0;
    }
  }
  else {
    if ((target_pos - TOLERANCE) > pos_min) {
	  if(limit.U_B == 1)
		{
			MotorMinute(DIR_STOP);
		}
		else
		{
			MotorMinute(DIR_UP);
		}
    }
	else if((target_pos + 2*ENCODER_M_STEP) < pos_min) //pos invalid
	  {
		limit.cal_min = 0; 
	  }
    else {
      MotorMinute(DIR_STOP);
    }
  }

}


void SetClock(S_LIMIT limit) {
  byte set = 0;
  if(limit.btn_min == 1 && limit.btn_hour == 1)
  {
	  delay(500);
	  set = 1;
	  H = 11;
	  M = 59;
  }
  else if (limit.btn_hour == 1) {
    //H++;
    H %= 12;
    delay(1000);
    set = 1;
    M--;
  }
  else if (limit.btn_min == 1 ) {
    //M++;
    M %= 60;
    delay(500);
    set = 1;
    H--;
  }

  if ( set == 1)
  {
    Rtc.SetDateTime(RtcDateTime(2020, 1, 1, H, M, 0));
  }
}



void MotorHour(byte dir) {

  if (dir == DIR_UP  ) {
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_1A), ISR_ENCODER_1, RISING);
	h_motor_state = DIR_UP;
    digitalWrite(PIN_MOTOR_HOUR_A, 0);
    digitalWrite(PIN_MOTOR_HOUR_B, 1);	
  }
  else if (dir == DIR_DOWN ) {
    h_motor_state = DIR_DOWN;
	attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_1A), ISR_ENCODER_1, RISING);	
    digitalWrite(PIN_MOTOR_HOUR_A, 1);
    digitalWrite(PIN_MOTOR_HOUR_B, 0);
  }
  else {
    h_motor_state = DIR_STOP;
    digitalWrite(PIN_MOTOR_HOUR_A, 0);
    digitalWrite(PIN_MOTOR_HOUR_B, 0);
	detachInterrupt(digitalPinToInterrupt(PIN_ENCODER_1A));
  }


}

void MotorMinute(byte dir) {
  if (dir == DIR_UP) {
    //attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_0A), ISR_ENCODER_0, RISING);
	m_motor_state = DIR_UP;
    digitalWrite(PIN_MOTOR_MIN_A, 0);
    digitalWrite(PIN_MOTOR_MIN_B, 1);	
  }
  else if (dir == DIR_DOWN) {
    //attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_0A), ISR_ENCODER_0, RISING);
	m_motor_state = DIR_DOWN;
    digitalWrite(PIN_MOTOR_MIN_A, 1);
    digitalWrite(PIN_MOTOR_MIN_B, 0);	
  }
  else {
    m_motor_state = DIR_STOP;
    digitalWrite(PIN_MOTOR_MIN_A, 0);
    digitalWrite(PIN_MOTOR_MIN_B, 0);
	//detachInterrupt(digitalPinToInterrupt(PIN_ENCODER_0A));
  }
}



void CheckLimit(S_LIMIT &limit) {
  if ( digitalRead(PIN_LIMIT_UPPER_A) == 1) {
    limit.U_A = 1;
  } else {
    limit.U_A = 0;
  }

  if ( digitalRead(PIN_LIMIT_UPPER_B) == 1) {
    limit.U_B = 1;
  } else {
    limit.U_B = 0;
  }

  if ( digitalRead(PIN_LIMIT_LOWER_A) == 1) {
    limit.L_A = 1;
  } else {
    limit.L_A = 0;
  }

  if ( digitalRead(PIN_LIMIT_LOWER_B) == 1) {
    limit.L_B = 1;
  } else {
    limit.L_B = 0;
  }

  if ( digitalRead(PIN_BTN_HOUR) == 1) {
    limit.btn_hour = 0;
  } else {
    limit.btn_hour = 1;
  }

  if ( digitalRead(PIN_BTN_MIN) == 1) {
    limit.btn_min = 0;
  } else {
    limit.btn_min = 1;
  }
}
