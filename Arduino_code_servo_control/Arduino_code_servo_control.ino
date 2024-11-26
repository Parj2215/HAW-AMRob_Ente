#include <Servo.h>

// Pin definitions
#define ESC_PIN 6
#define SERVO_PIN 3
#define BUZZER_PIN 5

// ESC control values (microseconds)
#define ESC_FORWARD_FULL 2000
#define ESC_BACKWARD_FULL 1000
#define ESC_STOP 1500

// Servo angle limits
#define SERVO_ANGLE_MAX 180
#define SERVO_ANGLE_MID 90
#define SERVO_ANGLE_MIN 0

// Delays (milliseconds)
#define DEBOUNCE_DELAY 15
#define DELAY_1S 1000
#define DELAY_2S 2000

// Global variables
int pulseWidth = 1500;
int servoAngle = 90;
int command = 0;
int pos = SERVO_ANGLE_MID;
unsigned long buzzerTime = 0;
bool buzzerOn = false;

Servo myESC;
Servo myServo;

void setup() {
  Serial.begin(9600);
  attachPins();
  pinMode(BUZZER_PIN, OUTPUT);
  caliberation();
  myServo.write(SERVO_ANGLE_MID);  // Set servo to mid position
  delay(DEBOUNCE_DELAY);
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming message
    String message = Serial.readStringUntil('\n');
    int separatorIndex1 = message.indexOf(',');
    int separatorIndex2 = message.lastIndexOf(',');
    
    // Extract the horizontal angle, vertical PWM, and buzzer command
    int horizontal_angle = message.substring(0, separatorIndex1).toInt();
    int vertical_pwm = message.substring(separatorIndex1 + 1, separatorIndex2).toInt();
    int buzzerCommand = message.substring(separatorIndex2 + 1).toInt();

    // Round vertical_pwm to the nearest tens
    if (vertical_pwm > 1450 && vertical_pwm < 1550) {
      vertical_pwm = 1500;  // Set value to 1500 if within the specified range
    } else {
      vertical_pwm = ((vertical_pwm + 5) / 10) * 10;  // Round to nearest tens
    }
    
    // Validate the horizontal angle and vertical PWM
    if ((horizontal_angle >= SERVO_ANGLE_MIN) && (horizontal_angle <= SERVO_ANGLE_MAX)) {
      if ((vertical_pwm >= ESC_BACKWARD_FULL) && (vertical_pwm <= ESC_FORWARD_FULL)) {
        setServoAngle(horizontal_angle);  // Update servo position
        setMotorPWM(vertical_pwm);        // Update motor speed
      }
    }

    // Handle buzzer based on command (1 or 0)
    if (buzzerCommand == 1) {
      buzzerTime = millis();  // Record the time when buzzer should turn on
      buzzerOn = true;
    }
  }
  // Control buzzer timing without blocking
  controlBuzzer();
}

void attachPins() {
  myESC.attach(ESC_PIN);
  myServo.attach(SERVO_PIN);
}

void caliberation() {
  // Arming the ESC
  myESC.writeMicroseconds(1500);
  delay(2000);
  Serial.println("ESC Armed");

  // Calibrate ESC throttle range
  myESC.writeMicroseconds(1500);  // Neutral throttle
  delay(1000);
  myESC.writeMicroseconds(2000);  // Full throttle forward
  delay(2000);
  myESC.writeMicroseconds(1000);  // Full throttle reverse
  delay(2000);
  myESC.writeMicroseconds(1500);  // Return to neutral throttle
  delay(1000);

  Serial.println("Calibration complete");
}

void controlBuzzer() {
  // Turn on buzzer for 10 seconds if buzzerOn is true
  if (buzzerOn && millis() - buzzerTime < 10000) {
    digitalWrite(BUZZER_PIN, HIGH);  // Turn on the buzzer
  } else {
    digitalWrite(BUZZER_PIN, LOW);  // Turn off the buzzer
    buzzerOn = false;  // Reset buzzer control
  }
}

void setServoAngle(int angle) {
  // Move the servo to the specified angle smoothly
  if (angle > pos) {
    for (; pos <= angle; pos++) {
      myServo.write(pos);
      delay(DEBOUNCE_DELAY);
    }
  } else {
    for (; pos >= angle; pos--) {
      myServo.write(pos);
      delay(DEBOUNCE_DELAY);
    }
  }
}

void setMotorPWM(int pwm) {
  // Update ESC PWM signal
  myESC.writeMicroseconds(pwm);
  delay(DEBOUNCE_DELAY);
}
