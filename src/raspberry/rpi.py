from random import randint
from sys import argv
from numpy import random
from robot_utils import RobotError

# Pins
PIN_SPEED = 'A0'
PIN_SIGNALS = 'A1'
PIN_A2 = 'A2'
PIN_A3 = 'A3'
PIN_A4 = 'A4'
PIN_A5 = 'A5'

# Pin states
INPUT = 0
OUTPUT = 1
INPUT_PULLUP = 2

# Digital readings states (representing 0 and 1)
LOW = 0
HIGH = 1


class GPIO:

    def __init__(self) -> None:
        self.ready = True
        self.workingPins = {}
        self.pinsValues = {}

    def setup(self):
        # TODO.. Setup GPIO pins
        if not self.ready:
            self.ready = True

    def pinMode(self, pin: str, state: int):
        if not self.ready:
            raise RobotError('You should call setup() first to setup GPIO.')
        self.workingPins[pin] = state

    def digitalRead(self, pin: str) -> int:
        if not self.ready:
            raise RobotError('You should call setup() first to setup GPIO.')
        return self.pinsValues[pin] if pin in self.pinsValues else LOW

    def digitalWrite(self, pin: str, value: int):
        if not self.ready:
            raise RobotError('You should call setup() first to setup GPIO.')
        self.pinsValues[pin] = value

    def analogRead(self, pin: str) -> int:
        if not self.ready:
            raise RobotError('You should call setup() first to setup GPIO.')
        return self.pinsValues[pin] if pin in self.pinsValues else randint(0, 1023)

    def analogWrite(self, pin: str, value: int):
        if not self.ready:
            raise RobotError('You should call setup() first to setup GPIO.')
        self.pinsValues[pin] = value

    def cleanup(self):
        self.ready = False
