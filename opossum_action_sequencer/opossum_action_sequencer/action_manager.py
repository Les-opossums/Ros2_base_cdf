#!/usr/bin/env python3
# -*- coding: utf-8 -*-


class Version:
    def __init__(self, version):
        self.version = version


class Position:
    def __init__(self, x, y, t):
        self.x = x
        self.y = y
        self.t = t


class Speed:
    def __init__(self, vx, vy, vt):
        self.vx = vx
        self.vy = vy
        self.vt = vt


class PUMP_struct:
    def __init__(self, pump_id, enable):
        self.pump_id = pump_id
        self.enable = enable


class LED_struct:
    def __init__(self, red, green, blue):
        self.red = red
        self.green = green
        self.blue = blue


class ODOM_struct:
    def __init__(self, enable, freq):
        self.enable = enable
        self.freq = freq


class SERVO_struct:
    def __init__(self, servo_id, angle):
        self.servo_id = servo_id
        self.angle = angle


class STEPPER_struct:
    def __init__(self, mode):
        self.mode = mode
