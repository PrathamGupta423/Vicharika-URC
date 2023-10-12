import RPi.GPIO as GPIO
import time

class StepperMotor:
    def __init__(self, step_pin, direction_pin, steps_per_revolution):
        self.step_pin = step_pin
        self.direction_pin = direction_pin
        self.steps_per_revolution = steps_per_revolution

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.direction_pin, GPIO.OUT)
        
        self.current_step = 0

    def set_direction(self, direction):
        GPIO.output(self.direction_pin, direction)

    def step(self, steps, delay=0.001):
        for _ in range(abs(steps)):
            if steps > 0:
                self.current_step = (self.current_step + 1) % self.steps_per_revolution
            else:
                self.current_step = (self.current_step - 1) % self.steps_per_revolution

            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(delay)

    def move_degrees(self, degrees, delay=0.001):
        steps = int((degrees / 360) * self.steps_per_revolution)
        self.step(steps, delay)

    def cleanup(self):
        GPIO.cleanup()

if __name__ == "__main__":
    # Define your stepper motor's GPIO pins and steps per revolution
    step_pin = 17
    direction_pin = 18
    steps_per_revolution = 200  # Change this to your motor's specifications

    motor = StepperMotor(step_pin, direction_pin, steps_per_revolution)
    
    try:
        while True:
            motor.set_direction(GPIO.HIGH)  # Set the motor direction (CW)
            motor.move_degrees(360, delay=0.001)  # Rotate 360 degrees
            time.sleep(1)

            motor.set_direction(GPIO.LOW)  # Set the motor direction (CCW)
            motor.move_degrees(360, delay=0.001)  # Rotate 360 degrees
            time.sleep(1)
    except KeyboardInterrupt:
        motor.cleanup()


if __name__ == "__main__":
    # Define your stepper motor's GPIO pins and steps per revolution
    step_pin = 17
    direction_pin = 18
    steps_per_revolution = 200  # Change this to your motor's specifications

    motor = StepperMotor(step_pin, direction_pin, steps_per_revolution)
    
    try:
        while True:
            motor.set_direction(GPIO.HIGH)  # Set the motor direction (CW)
            motor.move_degrees(360, delay=0.001)  # Rotate 360 degrees
            time.sleep(1)

            motor.set_direction(GPIO.LOW)  # Set the motor direction (CCW)
            motor.move_degrees(360, delay=0.001)  # Rotate 360 degrees
            time.sleep(1)
    except KeyboardInterrupt:
        motor.cleanup()
