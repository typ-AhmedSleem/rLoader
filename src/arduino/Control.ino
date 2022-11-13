// Includes
#include "AFMotor.h"
#include "Servo.h"
// Hardware components pins
#define DPIN_CONTROL_MODE 0
#define DPIN_LFS_1 1
#define DPIN_LFS_2 2
#define PIN_MOTOR_SERVO 3 // INFO: PWM pin 3
#define DPIN_SOUND 4
#define APIN_SPEED A0
#define APIN_ANGLE A1
// Statuses
const int STATUS_STOPPED = 0;
const int STATUS_RUNNING = 1;
// Control modes
const int CONTROL_MODE_MANUAL = 0;
const int CONTROL_MODE_LINE_FOLLOWING = 1;
// Statics
const int NO_SPEED = 0;
const int MAX_SPEED = 255;
const int ROTATING_SPEED = 250;
const int DEFAULT_SPEED = MAX_SPEED;
const int ANGLE_HARD_LEFT = 0;
const int ANGLE_CENTER = 128;
const int ANGLE_HARD_RIGHT = 255;
// Runtime
int currentSpeed = NO_SPEED;
int currentAngle = ANGLE_CENTER;
int status = STATUS_STOPPED;
int controlMode = CONTROL_MODE_LINE_FOLLOWING;
int servoAngle = 90;
bool servoToRight = true;
bool isRediatorEnabled = false;

// Hardware
Servo cameraServo;
AF_DCMotor leftFrontMotor(1);
AF_DCMotor rightFrontMotor(2);
AF_DCMotor leftRearMotor(3);
AF_DCMotor rightRearMotor(4);

// Functions
void driveMotor(AF_DCMotor motor, int speed, int direction);
void driveMotor(AF_DCMotor motor, int speed, int direction)
{
    if (speed > 255)
    {
        speed = 255;
    }
    else if (speed <= 0)
    {
        speed = 0;
    }
    // Apply speed and direction to motors
    motor.setSpeed(speed);
    motor.run(direction);
}

void setup()
{
    // Reset all runtime
    status = STATUS_STOPPED;                   // INFO: Set status to STOPPED.
    currentSpeed = NO_SPEED;                   // INFO: Reset the speed.
    currentAngle = ANGLE_CENTER;               // INFO: Center the wheels.
    controlMode = CONTROL_MODE_LINE_FOLLOWING; // INFO: Set status to STOPPED.
    servoAngle = 90;                           // INFO: Center the servo.
    servoToRight = true;                       // INFO: Set the direction of servo rotation to right.
    // Setup IO pins
    pinMode(DPIN_CONTROL_MODE, INPUT); // INFO: Control mode input from RPi
    pinMode(DPIN_LFS_1, INPUT);        // INFO: LineFollowingSesnor 1
    pinMode(DPIN_LFS_2, INPUT);        // INFO: LineFollowingSesnor 2
    pinMode(DPIN_SOUND, INPUT);        // INFO: DigitalSoundSensor
    pinMode(APIN_SPEED, INPUT);        // INFO: Speed input analog pin from RPi
    pinMode(APIN_ANGLE, INPUT);        // INFO: Direction angle input analog pin from RPi
    // Attach the servo
    cameraServo.attach(PIN_MOTOR_SERVO); // INFO: Attach the camera servo to PWM pin 3.
}

void loop()
{
    // Change control mode if change requested by host
    int requestedControlMode = digitalRead(DPIN_CONTROL_MODE);
    bool controlModeChanged = (requestedControlMode != controlMode);
    if (controlModeChanged)
    {
        controlMode = requestedControlMode;
    }
    // INFO: Manual control mode
    if (controlMode == CONTROL_MODE_MANUAL)
    {
        // Read speed and angle from analog pins A0 & A1
        int requestedSpeed = analogRead(APIN_SPEED);
        int requestedAngle = analogRead(APIN_ANGLE);
        // Map speed and direction values to be btw (0 & 255)
        currentSpeed = map(requestedSpeed, 0, 1023, 0, 255);
        currentAngle = map(requestedAngle, 0, 1023, 0, 255);
        // Move the robot using the speed and angle
        if (currentAngle > ANGLE_CENTER && currentAngle <= ANGLE_HARD_RIGHT)
        {
            // Rotate to right by given angle
            driveMotor(leftFrontMotor, ROTATING_SPEED, FORWARD);
            driveMotor(rightFrontMotor, ROTATING_SPEED, BACKWARD);
            driveMotor(leftRearMotor, ROTATING_SPEED, FORWARD);
            driveMotor(rightRearMotor, ROTATING_SPEED, BACKWARD);
        }
        else if (currentAngle >= ANGLE_HARD_LEFT && currentAngle < ANGLE_CENTER)
        {
            // Rotate to left by given angle
            driveMotor(leftFrontMotor, ROTATING_SPEED, BACKWARD);
            driveMotor(rightFrontMotor, ROTATING_SPEED, FORWARD);
            driveMotor(leftRearMotor, ROTATING_SPEED, BACKWARD);
            driveMotor(rightRearMotor, ROTATING_SPEED, FORWARD);
        }
        else
        {
            // Move Forward or Backward according to Rediator enabled or not
            int direction = isRediatorEnabled ? BACKWARD : FORWARD;
            driveMotor(leftFrontMotor, currentSpeed, direction);
            driveMotor(rightFrontMotor, currentSpeed, direction);
            driveMotor(leftRearMotor, currentSpeed, direction);
            driveMotor(rightRearMotor, currentSpeed, direction);
        }
    }
    // INFO: LineFollowing Mode
    else
    {
        // Make speed constant for LineFollowing mode
        currentSpeed = DEFAULT_SPEED;
        // Read analog reading from LineFollowingSensor
        bool isLeftLFS_Black = digitalRead(DPIN_LFS_1) == HIGH;
        bool isRightLFS_Black = digitalRead(DPIN_LFS_2) == HIGH;
        // Determine what to do according to left and
        if (!isLeftLFS_Black and isRightLFS_Black)
        {
            // Rotate to right by given angle
            driveMotor(leftFrontMotor, ROTATING_SPEED, FORWARD);
            driveMotor(rightFrontMotor, ROTATING_SPEED, BACKWARD);
            driveMotor(leftRearMotor, ROTATING_SPEED, FORWARD);
            driveMotor(rightRearMotor, ROTATING_SPEED, BACKWARD);
        }
        else if (isLeftLFS_Black and !isRightLFS_Black)
        {
            // Rotate to left by given angle
            driveMotor(leftFrontMotor, ROTATING_SPEED, BACKWARD);
            driveMotor(rightFrontMotor, ROTATING_SPEED, FORWARD);
            driveMotor(leftRearMotor, ROTATING_SPEED, BACKWARD);
            driveMotor(rightRearMotor, ROTATING_SPEED, FORWARD);
        }
        else
        {
            // Move forward
            driveMotor(leftFrontMotor, DEFAULT_SPEED, FORWARD);
            driveMotor(rightFrontMotor, DEFAULT_SPEED, FORWARD);
            driveMotor(leftRearMotor, DEFAULT_SPEED, FORWARD);
            driveMotor(rightRearMotor, DEFAULT_SPEED, FORWARD);
        }
    }
    // Rotate the camera servo
    if (servoAngle <= 0)
        servoAngle = 0; // INFO: Servo has reached the most left end.
    else if (servoAngle >= 180)
        servoAngle = 180; // INFO: Servo has reached the most right end.
    else
    {
        if (servoToRight)
            servoAngle += 1; // INFO: Increase angle by 1 degree.
        else
            servoAngle -= 1; // INFO: Decrease angle by 1 degree.
    }
    cameraServo.write(servoAngle); // INFO: Write the new angle to the servo
    // Change robot status if needed
    status = currentSpeed == NO_SPEED ? STATUS_STOPPED : STATUS_RUNNING;
    // Delay for 50 ms
    delay(50);
}
