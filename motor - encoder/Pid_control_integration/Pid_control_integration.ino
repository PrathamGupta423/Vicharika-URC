#include <util/atomic.h> // For the ATOMIC_BLOCK macro

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
      float dt = ((float) (currT - prevT))/( 1.0e6 ); // Convert to seconds
      float error = targetPos - posi;
      float derivative = (error - eprev) / dt;
      eintegral += error * dt;
      float output = kp * error + ki * eintegral + kd * derivative;
      
      if (output > 255) {
        output = 255;
      } else if (output < -255) {
        output = -255;
      }
      
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

void setup{
    
}