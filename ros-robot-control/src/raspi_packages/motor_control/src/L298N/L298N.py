#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ---------------------------------------------
# Project:  L298N class
# Author:   Daniel Neumann, 2018
# Licence:  MIT
# Description: Python class for controlling
# L298N Board from Raspberry Pi
# ---------------------------------------------
try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO! This is probably because you need superuser privileges. "
          "You can achieve this by using 'sudo' to run your script")


class L298N:
    """
    Main class for controlling L298N Board
    via Raspberry Pi GPIO
    """
    # Motor numbering
    DC_Motor_1 = 1  # IN 1 and 2 on L298N Board
    DC_Motor_2 = 2  # IN 3 and 4 on L298N Board

    # DC Motors states and settings
    # is_running        - True if motor is running
    # running_direction - Direction of the motor (True forwards, False backwards)
    # pwm_frequency     - Frequency of pulse-width modulation (pwm)
    # pwm               - Un-/set pwm object
    _MOTORS = {
        DC_Motor_1: {"pin1": None, "pin2": None, "is_running": False, "running_direction": True, "pwm_frequency": 50, "pwm": None},
        DC_Motor_2: {"pin1": None, "pin2": None, "is_running": False, "running_direction": True, "pwm_frequency": 50, "pwm": None}
    }

    def __init__(self, use_board=False):
        """
        Initialization for L298N class

        :param bool use_board: True if GPIO.BOARD numbering will be used, otherwise GPIO.BCM
        :return:
        """
        if use_board:
            GPIO.setmode(GPIO.BOARD)
            print("PIN numbering: BOARD")
        else:
            GPIO.setmode(GPIO.BCM)
            print("PIN numbering: BCM")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            self.stop_dc_motors([self.DC_Motor_1, self.DC_Motor_2])
            GPIO.cleanup()
        except RuntimeWarning:
            return True

    def set_L298N_pins(self, MOTOR1_PIN1=None, MOTOR1_PIN2=None, MOTOR2_PIN1=None, MOTOR2_PIN2=None):
        """
        Set PINs used on Raspberry Pi to connect with L298N Board

        :param int MOTOR1_PIN1: PIN number on Raspberry Pi which is connected to L298N IN 1
        :param int MOTOR1_PIN2: PIN number on Raspberry Pi which is connected to L298N IN 2
        :param int MOTOR2_PIN1: PIN number on Raspberry Pi which is connected to L298N IN 3
        :param int MOTOR2_PIN2: PIN number on Raspberry Pi which is connected to L298N IN 4
        """

        self._MOTORS[self.DC_Motor_1]["pin1"] = MOTOR1_PIN1
        self._MOTORS[self.DC_Motor_1]["pin2"] = MOTOR1_PIN2

        self._MOTORS[self.DC_Motor_2]["pin1"] = MOTOR2_PIN1
        self._MOTORS[self.DC_Motor_2]["pin2"] = MOTOR2_PIN2

        if MOTOR1_PIN1 is not None:
            GPIO.setup(MOTOR1_PIN1, GPIO.OUT)
        if MOTOR1_PIN2 is not None:
            GPIO.setup(MOTOR1_PIN2, GPIO.OUT)
        if MOTOR2_PIN1 is not None:
            GPIO.setup(MOTOR2_PIN1, GPIO.OUT)
        if MOTOR2_PIN2 is not None:
            GPIO.setup(MOTOR2_PIN2, GPIO.OUT)

    def set_pwm_frequency(self, frequencies):
        """
        Sets the pulse-width modulation (pwm) frequencies for each motor.

        :param list of int: Values for pwm frequencies. Should be high enough to run smoothly, but
                            too high values can cause RPi.GPIO to crash.
        """
        if len(frequencies) is not len(self._MOTORS):
            print("ERROR: Size of frequency list must be equal to size of motors (".len(self._MOTORS).")!")
        else:
            self._MOTORS[self.DC_Motor_1]["pwm_frequency"] = frequencies[0]
            self._MOTORS[self.DC_Motor_2]["pwm_frequency"] = frequencies[1]

    def get_pwm_frequency(self):
        """
        Returns the current list of pulse-width modulation (pwm) frequencies for each motor.

        :return: Current pwm frequency for each motor as list.
        :rtype list of int
        """
        frequencies = []
        for m in self._MOTORS:
            frequencies += m["pwm_frequency"]

        return frequencies

    def run_dc_motor(self, dc_motor, forwards=True, speed=None):
        """
        Run motor with given direction and speed

        :param int dc_motor: number of dc motor
        :param bool forwards: True for forwards, False for backwards
        :param int speed: pwm duty cycle (range 0-100)
        :return: False in case of an ERROR, True if everything is OK
        :rtype bool
        """
        if self._MOTORS[dc_motor]["pin1"] is None:
            print("WARNING: Pin 1 for DC_Motor_{} is not set. Can not run motor. Call set_L298N_pins() before!".format(dc_motor))
            return False
        if self._MOTORS[dc_motor]["pin2"] is None:
            print("WARNING: Pin 2 for DC_Motor_{} is not set. Can not run motor. Call set_L298N_pins() before!".format(dc_motor))
            return False

        outHigh = self._MOTORS[dc_motor]["pin1"]
        outLow = self._MOTORS[dc_motor]["pin2"]

        if not forwards:
            outHigh = self._MOTORS[dc_motor]["pin2"]
            outLow = self._MOTORS[dc_motor]["pin1"]

        # turn the motor on (if speed argument is not given then full speed, otherwise set pwm according to speed)
        if speed is None or speed == 100:
            GPIO.output(outHigh, GPIO.HIGH)
            GPIO.output(outLow, GPIO.LOW)
        elif speed == 0:
            self.stop_dc_motor(dc_motor)
        elif speed > 0 and speed <= 100:
            self._MOTORS[dc_motor]["pwm"] = GPIO.PWM(
                outHigh, self._MOTORS[dc_motor]["pwm_frequency"])
            self._MOTORS[dc_motor]["pwm"].start(speed)
        else:
            print("WARNING: Speed argument must be in range 0-100! " +
                  str(speed) + " given.")
            return False

        self._MOTORS[dc_motor]["is_running"] = True
        self._MOTORS[dc_motor]["running_direction"] = forwards

        return True

    def run_dc_motors(self, dc_motors, forwards=True, speed=None):
        """
        Run motors with given direction and speed

        :param list[int] dc_motors: list of dc motor numbers
        :param bool forwards: True for forwards, False for backwards
        :param int speed: pwm duty cycle (range 0-100)
        :return: False in case of an ERROR, True if everything is OK
        :rtype bool
        """
        for dc_motor in dc_motors:
            self.run_dc_motor(dc_motor, forwards, speed)

    def stop_dc_motor(self, dc_motor):
        """
        Stop running motor

        :param int dc_motor: number of dc motor
        :return: False in case of an ERROR, True if everything is OK
        :rtype bool
        """
        if self._MOTORS[dc_motor]["pin1"] is None:
            print("WARNING: Pin 1 for DC_Motor_{} is not set. Stopping motor could not be done. Call set_L298N_pins() before!".format(dc_motor))
            return False
        if self._MOTORS[dc_motor]["pin2"] is None:
            print("WARNING: Pin 2 for DC_Motor_{} is not set. Stopping motor could not be done. Call set_L298N_pins() before!".format(dc_motor))
            return False

        if self._MOTORS[dc_motor]["pwm"] is None:
            GPIO.output(self._MOTORS[dc_motor]["pin1"], GPIO.LOW)
            GPIO.output(self._MOTORS[dc_motor]["pin2"], GPIO.LOW)
        else:
            self._MOTORS[dc_motor]["pwm"].stop()
            self._MOTORS[dc_motor]["pwm"] = None

        self._MOTORS[dc_motor]["is_running"] = False
        self._MOTORS[dc_motor]["running_direction"] = None

        return True

    def stop_dc_motors(self, dc_motors):
        """
        Stop motors set in list

        :param list[int] dc_motors: list of dc motor numbers
        :return: False in case of an ERROR, True if everything is OK
        :rtype bool
        """
        for dc_motor in dc_motors:
            if not self.stop_dc_motor(dc_motor):
                return False

        return True
