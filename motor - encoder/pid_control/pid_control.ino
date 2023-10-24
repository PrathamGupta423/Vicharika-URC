// REFRENCE CODE (not to be uploaded to arduino)



// orginal code from tutorial (part -4)

#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM 5
#define IN2 6
#define IN1 7

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/


long prevT = 0;
float eprev = 0;
float eintegral = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  
  Serial.println("target pos");
}

void loop() {

  // set target position
  //int target = 1200;
  int target = 250*sin(prevT/1e6);

  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
  
  // error
  int e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);


  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}














// edited code from tutorial (part -4)


// Include the ATOMIC_BLOCK macro from the util library
#include <util/atomic.h>

// Define constants for the encoder pins, PWM pin, and motor direction pins
#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM 5
#define IN2 6
#define IN1 7

// Declare a volatile integer variable to store the encoder position
volatile int posi = 0;

// Declare variables to store the previous time, previous error, and integral error
long prevT = 0;
float eprev = 0;
float eintegral = 0;

// Setup function - runs once when the Arduino is powered on or reset
/**
 * Initializes the serial communication, encoder pins, PWM pin, and motor direction pins.
 * Attaches interrupts to the encoder pins to call the readEncoder function.
 */
void setup() {
  Serial.begin(9600);

  // Set the encoder pins as inputs
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);



  // Set the PWM pin and motor direction pins as outputs
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Attach an interrupt to the encoder B pin to call the readEncoder function
  attachInterrupt(digitalPinToInterrupt(ENCB), readEncoder, RISING);
}

// Loop function - runs repeatedly as long as the Arduino is powered on
void loop() {
  
  // set target position
  //int target = 1200;
  int target = 250*sin(prevT/1e6);

  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;
  
  // Get the current time
  long currT = millis();

  // Calculate the elapsed time since the last loop iteration
  float dt = ((float) (currT - prevT))/( 1.0e6 );


  // // Read the current encoder position
  // int pos = posi;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }

  // Calculate the error between the current position and the desired position
  float error = pos - target;

  // Calculate the derivative error
  float ederiv = (error - eprev) / dt;

  // Calculate the integral error
  eintegral += error * dt;

  // Limit the integral error to prevent windup
  if (eintegral > 1000) {
    eintegral = 1000;
  } else if (eintegral < -1000) {
    eintegral = -1000;
  }

  // Calculate the PID output
  // float output = 0.5 * error + 0.2 * ederiv + 0.1 * eintegral;
  float output = kp*error + kd*ederiv + ki*eintegral

  // Limit the output to the range of -255 to 255
  if (output > 255) {
    output = 255;
  } else if (output < -255) {
    output = -255;
  }

  // Set the motor direction and PWM value based on the PID output
  if (output > 0) {
    setMotor(1, output, PWM, IN1, IN2);
  } else if (output < 0) {
    setMotor(-1, -output, PWM, IN1, IN2);
  } else {
    setMotor(0, 0, PWM, IN1, IN2);
  }
  
  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();

  // Store the current time and error for the next loop iteration
  prevT = currT;
  eprev = error;
}

// Function to set the motor direction and PWM value
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

// Interrupt service routine to read the encoder position
void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    posi++;
  } else {
    posi--;
  }
}








class Motor {
  private:
    int encA;
    int encB;
    int pwm;
    int in1;
    int in2;
    volatile int posi = 0;
    long prevT = 0;
    float eprev = 0;
    float eintegral = 0;

  public:
    Motor(int encA_pin, int encB_pin, int pwm_pin, int in1_pin, int in2_pin) {
      encA = encA_pin;
      encB = encB_pin;
      pwm = pwm_pin;
      in1 = in1_pin;
      in2 = in2_pin;
      pinMode(encA, INPUT);
      pinMode(encB, INPUT);
      pinMode(pwm, OUTPUT);
      pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);
      attachInterrupt(digitalPinToInterrupt(encB), Motor::readEncoder, RISING);
    }

    void setMotor(int dir, int pwmVal) {
      analogWrite(pwm, pwmVal);
      if (dir == 1) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
      } else if (dir == -1) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
      } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
      }
    }

    static void readEncoder() {
      int b = digitalRead(encB);
      if (b > 0) {
        posi++;
      } else {
        posi--;
      }
    }





class Motor {
  private:
    int encA;
    int encB;
    int pwm;
    int in1;
    int in2;
    volatile int posi = 0;
    long prevT = 0;
    float eprev = 0;
    float eintegral = 0;

  public:
    Motor(int encA_pin, int encB_pin, int pwm_pin, int in1_pin, int in2_pin) {
      encA = encA_pin;
      encB = encB_pin;
      pwm = pwm_pin;
      in1 = in1_pin;
      in2 = in2_pin;
      pinMode(encA, INPUT);
      pinMode(encB, INPUT);
      pinMode(pwm, OUTPUT);
      pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);
      attachInterrupt(digitalPinToInterrupt(encB), Motor::readEncoder, RISING);
    }

    void setMotor(int dir, int pwmVal) {
      analogWrite(pwm, pwmVal);
      if (dir == 1) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
      } else if (dir == -1) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
      } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
      }
    }

    static void readEncoder() {
      int b = digitalRead(encB);
      if (b > 0) {
        posi++;
      } else {
        posi--;
      }
    }

    void moveToPos(int targetPos, float kp, float ki, float kd) {
      long currT = millis();
      long dt = currT - prevT;
      float error = targetPos - posi;
      float derivative = (error - eprev) / dt;
      eintegral += error * dt;
      float output = kp * error + ki * eintegral + kd * derivative;
      setMotor(output > 0 ? 1 : -1, abs(output));
      eprev = error;
      prevT = currT;

      // Wait until the motor reaches the target position
      while (abs(targetPos - posi) > 1) {
        // Read encoder position and update error, derivative, and integral
        currT = millis();
        dt = currT - prevT;
        error = targetPos - posi;
        derivative = (error - eprev) / dt;
        eintegral += error * dt;
        output = kp * error + ki * eintegral + kd * derivative;
        setMotor(output > 0 ? 1 : -1, abs(output));
        eprev = error;
        prevT = currT;
      }
    }
};


