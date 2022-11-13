#include <AFMotor.h>

#define NO_SPEED 0
#define DEFAULT_SPEED 160
#define ROTATING_SPEED 225
#define MAX_SPEED 255
#define SPEED_STEP 5
#define UNIT_SECOND 1000

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

// Runtime
int currentSpeed = NO_SPEED;
int rotatingSpeed = ROTATING_SPEED;
bool toRight = true;

void releaseMotors()
{
	motor1.run(RELEASE);
	motor2.run(RELEASE);
	motor3.run(RELEASE);
	motor4.run(RELEASE);
}

void changeTransmission(int direction)
{
	// Move motors forward
	motor1.run(direction);
	motor2.run(direction);
	motor3.run(direction);
	motor4.run(direction);
}

void brake()
{
	changeTransmission(BRAKE);
}

void setSpeed(int speed)
{
	motor1.setSpeed(speed);
	motor2.setSpeed(speed);
	motor3.setSpeed(speed);
	motor4.setSpeed(speed);
	// Update runtime
	currentSpeed = speed;
}

void rotate()
{
	// Change robot speed to rotating speed
	switch (rotatingSpeed % 4)
	{
	case 0:
		rotatingSpeed = 32;
		break;
	case 1:
		rotatingSpeed = 64;
		break;
	case 2:
		rotatingSpeed = 128;
		break;
	case 3:
		rotatingSpeed = 250;
		break;
	}
	setSpeed(rotatingSpeed);
	// Perform rotate
	if (toRight)
	{
		// Rotate right
		motor1.run(FORWARD);
		motor2.run(BACKWARD);
		motor3.run(BACKWARD);
		motor4.run(FORWARD);
	}
	else
	{
		// Rotate left
		motor1.run(BACKWARD);
		motor2.run(FORWARD);
		motor3.run(FORWARD);
		motor4.run(BACKWARD);
	}
	// Update runtime
	toRight = !toRight;
}

void accelerate()
{
	for (int speed = NO_SPEED; speed <= MAX_SPEED; speed += SPEED_STEP)
	{
		setSpeed(speed);
		delay(50);
	}
}

void setup()
{
	// Trun on motors
	setSpeed(NO_SPEED);
	// Release the motors from any movement
	releaseMotors();
}

void loop()
{
	// Move forward
	changeTransmission(FORWARD);
	accelerate();
	delay(1 * UNIT_SECOND);
	// Brake
	brake();
	delay(1 * UNIT_SECOND);
	// Rotate
	rotate();
	delay(2 * UNIT_SECOND);
	// Brake
	brake();
	delay(1 * UNIT_SECOND);
	// Rotate
	rotate();
	delay(2 * UNIT_SECOND);
	// Brake
	brake();
	delay(1 * UNIT_SECOND);
	// Move backward
	changeTransmission(BACKWARD);
	accelerate();
	delay(1 * UNIT_SECOND);
	// Brake
	brake();
	delay(5 * UNIT_SECOND);
}