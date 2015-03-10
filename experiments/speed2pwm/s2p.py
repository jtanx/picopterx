#!/usr/bin/env python3
import sys, os, re

AILERON_PWM_MID  = 152
AILERON_PWM_SWING  = 41
AILERON_SPEED_MID =  0

ELEVATOR_PWM_MID = 153
ELEVATOR_PWM_SWING = 42

def s2p_old(speed):
    pwm = AILERON_PWM_MID + (speed * AILERON_PWM_SWING) / 100
    if pwm > (AILERON_PWM_MID + AILERON_PWM_SWING):
        pwm = AILERON_PWM_MID + AILERON_PWM_SWING
    elif pwm < AILERON_PWM_MID - AILERON_PWM_SWING:
        pwm = AILERON_PWM_MID - AILERON_PWM_SWING
    return pwm
    
AILERON_OFFSET = 152
AILERON_RANGE = 41
def s2p_new(speed):
    speed = max(min(speed, 100), -100)
    return (speed / 100) * AILERON_RANGE + AILERON_OFFSET

def s2pe_old(speed):
    pwm = ELEVATOR_PWM_MID - (speed * ELEVATOR_PWM_SWING) / 100
    if pwm > ELEVATOR_PWM_MID + ELEVATOR_PWM_SWING:
        pwm = ELEVATOR_PWM_MID + ELEVATOR_PWM_SWING
    elif pwm < ELEVATOR_PWM_MID - ELEVATOR_PWM_SWING:
        pwm  = ELEVATOR_PWM_MID - ELEVATOR_PWM_SWING
    return pwm
    
ELEVATOR_OFFSET = 153
ELEVATOR_RANGE = 42
def s2pe_new(speed):
    speed = max(min(speed, 100), -100)
    return (-speed / 100) * ELEVATOR_RANGE + ELEVATOR_OFFSET
    
def linear_scale(x, xl, xh, yl, yh):
    return yl + ((yh - yl) * (x-xl)) / (xh - xl)

def s2pe_ls(speed):
    speed = max(min(speed, 100), -100)
    return linear_scale(-speed, -100, 100, ELEVATOR_OFFSET - ELEVATOR_RANGE, ELEVATOR_OFFSET + ELEVATOR_RANGE)
    
def s2p_ls(speed):
    speed = max(min(speed, 100), -100)
    return linear_scale(speed, -100, 100, AILERON_OFFSET - AILERON_RANGE, AILERON_OFFSET + AILERON_RANGE)