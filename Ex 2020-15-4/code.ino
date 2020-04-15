class SimplePID {
private:
  double prevErr = 0.0;
  double integral = 0.0;
public:
  double Kp, Ki, Kd;
  
  SimplePID(double Kp, double Ki, double Kd) {
  	this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
  }
  
  double feedbackViaErr(double error, double dt) {
    this->integral += error * dt;
    double derivative = (error - prevErr) / dt;
    prevErr = error;
    
    return Kp * error + Ki * integral + Kd * derivative;
  }
  
  double feedback(double mesured, double setpoint, double dt) {
  	double error = setpoint - mesured;
    return this->feedbackViaErr(error, dt);
  }
  
  void reset() {
  	this->prevErr = 0.0;
    this->integral = 0.0;
  }
};

class DcMotor {
private:
  int enableA, enableB, speedPin;
  int rotation = 0;
public:
  DcMotor(int enableA, int enableB, int speedPin) {
  	this->enableA = enableA;
    this->enableB = enableB;
    this->speedPin = speedPin;
    this->rotation = 0;
    
    pinMode(enableA, OUTPUT);
    pinMode(enableB, OUTPUT);
    pinMode(speedPin, OUTPUT);
    
    digitalWrite(this->enableA, HIGH);
    digitalWrite(this->enableB, LOW);
  }
  
  void setSpeedPWD(int pwd) {
   	
    int new_rotation;
    if(pwd > 0) {new_rotation = 1;}
    else if(pwd < 0) {new_rotation = -1;}
    else {new_rotation = 0;}
    
    if(this->rotation != new_rotation) {
      if(pwd > 0) {
        digitalWrite(this->enableA, HIGH);
        digitalWrite(this->enableB, LOW);
      }
      else if(pwd < 0) {
        digitalWrite(this->enableA, LOW);
        digitalWrite(this->enableB, HIGH);
      }
      else if(pwd == 0) {
        digitalWrite(this->enableA, HIGH);
        digitalWrite(this->enableB, HIGH);
      }
      
      this->rotation = new_rotation;
    }
    
    
    analogWrite(this->speedPin, abs(pwd));
  }
};

class AutoTimer {
private:
  void (*callback)();
  long period;
  long periodBegin;
public:
  AutoTimer(long period, void (*callback)()) {
  	this->period = period;
    this->callback = callback;
    this->periodBegin = 0;
  }
  
  void begin() {
    this->periodBegin = millis();
  }
  
  void tryRun() {
    long currentTime = millis();
    long deltaTime = currentTime - this->periodBegin;
    if(deltaTime >= this->period) {
      this->callback();
      this->periodBegin = currentTime;
    }
  }
};

const int BTN_1_PIN = 8;
const int BTN_2_PIN = 12;
const int LED_PIN = 13;

const int encoderPinA = 2;
const int encoderPinB = 4;

volatile long position = 0;
int target = 0;

const int BTN_ROT_SPEED = 200;

DcMotor motor(10, 9, 11);

SimplePID pid(0.17, 0.2, 0.0);


AutoTimer motorTimer(100, pidUpdate);
AutoTimer ledTimer(500, ledUpdate);
AutoTimer btnTimer(500, btnUpdate);


void setup()
{
  pinMode(BTN_1_PIN, INPUT_PULLUP);
  pinMode(BTN_2_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  
  
  Serial.begin(9600);
  noInterrupts();
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderCount, RISING);
  interrupts();
  
  
  motor.setSpeedPWD(1);
  
  motorTimer.begin();
  ledTimer.begin();
  btnTimer.begin();
}

void encoderCount() {
	position += digitalRead(encoderPinB)? 1 : -1;
}

long timer;


void pidUpdate() {
  double dt = (double)(millis() - timer) / 1000;
  timer = millis();
  
  
  double speed = (double)(position)/dt;
  position = 0;
  
  double speedD = pid.feedback(
    speed, 
    target,
    dt
  );
  int pwd = constrain((int)(round(speedD)), -255, 255);
  
  if(pwd != 0)
  {
  	motor.setSpeedPWD(pwd);
  }
  else {
  	motor.setSpeedPWD(1);
  }
    
  Serial.print(pwd);
  Serial.print(", ");
  Serial.print(speed);
  Serial.print(", ");
  Serial.print(target);
  Serial.println();
}

void ledUpdate() {
	PORTB = PORTB ^ 0b00100000;
}

void btnUpdate() {
  if(!digitalRead(BTN_1_PIN)) {
  	target = BTN_ROT_SPEED;
  	pid.reset();
  }
  else if(!digitalRead(BTN_2_PIN)) {
  	target = -BTN_ROT_SPEED;
 	pid.reset();
  }
}

void loop()
{
  motorTimer.tryRun();
  ledTimer.tryRun();
  btnTimer.tryRun();
  delay(100);
}

void serialEvent() {
  if(Serial.read() != 's') {return;}
  float val = Serial.parseFloat();
  if(Serial.read() != 'n') {return;}
  if(Serial.available()) {
  	Serial.read();
  }
  
  target = constrain(val, -100.0, 100.0) * 15.0;
  
  pid.reset();
  
  Serial.println(target);
  
}