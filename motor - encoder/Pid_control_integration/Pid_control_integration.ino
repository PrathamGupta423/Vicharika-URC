#include <ros.h>
#include <std_msgs/Int32.h>
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

ros::NodeHandle nh;

void motormoveCallback(const std_msgs::String& msg) {
    
    // standardise message as : "<motor_id>*<target position>*<kp>*<ki>*<kd>"
    
    // Extract the target position from the message
    std::string message = msg.data;
    
    // Extract the target position from the message
    int id_delim = message.find("*");
    Motor motor_id = (message.substr(0, id_delim));

    // Extract the target position from the message
    int pos_delim = message.find("*", id_delim + 1);
    float targetPos = std::stof(message.substr(0, pos_delim));

    // Extract the PID constants from the message
    int kp_delim = message.find("*", pos_delim + 1);
    float kp = std::stof(message.substr(pos_delim + 1, kp_delim - pos_delim - 1));

    int ki_delim = message.find("*", kp_delim + 1);
    float ki = std::stof(message.substr(kp_delim + 1, ki_delim - kp_delim - 1));

    float kd = std::stof(message.substr(ki_delim + 1));

    // Move the motor to the target position using PID control
    // Motor motor(2, 3, 5, 6, 7); // Replace with your own pin numbers
    motor_id.moveToPos(targetPos, kp, ki, kd);
}
ros::Subscriber<std_msgs::String> motormoveSub("motormove", &motormoveCallback);

void motorSetupCallback( const std_msgs::String& msg) {
    // standardise message as : "<motor_id>*<encA>*<encB>*<pwm>*<in1>*<in2>"
    
    
    std::string message = msg.data;
    
    // Extract the motor_id from the message
    int id_delim = message.find("*");
    Motor motor_id = (message.substr(0, id_delim));

    // Extract the encA from the message
    int encA_delim = message.find("*", id_delim + 1);
    int encA = std::stoi(message.substr(id_delim + 1, encA_delim - id_delim - 1));

    // Extract the encB from the message
    int encB_delim = message.find("*", encA_delim + 1);
    int encB = std::stoi(message.substr(encA_delim + 1, encB_delim - encA_delim - 1));

    // Extract the pwm from the message
    int pwm_delim = message.find("*", encB_delim + 1);
    int pwm = std::stoi(message.substr(encB_delim + 1, pwm_delim - encB_delim - 1));

    // Extract the in1 from the message
    int in1_delim = message.find("*", pwm_delim + 1);
    int in1 = std::stoi(message.substr(pwm_delim + 1, in1_delim - pwm_delim - 1));

    // Extract the in2 from the message
    int in2_delim = message.find("*", in1_delim + 1);
    int in2 = std::stoi(message.substr(in1_delim + 1, in2_delim - in1_delim - 1));

    // Create the motor object
    motor_id = Motor(encA, encB, pwm, in1, in2);


}


ros::Subscriber<std_msgs::String> motorSetupSub("motor_setup", &motorSetupCallback);





void setup() {
    ros::NodeHandle nh("Robotic_Arm"); // join an existing node with the name "~"
    nh.subscribe(motormoveSub);
    nh.subscribe(motorSetupSub);
}

void loop() {
    nh.spinOnce();
}

