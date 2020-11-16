
#modules used in the code
import adafruit_vl53l0x #adafruit environment to control the laser sensor

#standard RPi code blocks
import RPi.GPIO as GPIO
import board
import busio
import time
import csv

#number manipulation
import numpy as np
import scipy as sp
import statistics as stats
import matplotlib.pyplot as plt

#sets up the board
GPIO.setmode(GPIO.BCM)

#laser code block
i2c = busio.I2C(board.SCL, board.SDA) #sets up
laser = adafruit_vl53l0x.VL53L0X(i2c)
def distance(self):
    return 8.5/11*self.range
   


    
#Piezo code
piezo = 11
GPIO.setup(piezo,GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
    
#Motor Code
Motor1E = 18 #  Enable pin 1 of the controller IC
Motor1A = 23 #  Input 1 of the controller IC
Motor1B = 24 #  Input 2 of the controller IC
GPIO.setup(Motor1A,GPIO.OUT)
GPIO.setup(Motor1B,GPIO.OUT)
GPIO.setup(Motor1E,GPIO.OUT)

forward=GPIO.PWM(Motor1A,100) # configuring Enable pin for PWM
reverse=GPIO.PWM(Motor1B,100) # configuring Enable pin for PWM

forward.start(0) 
reverse.start(0)

#Data analysis code blocks
def avg_dist(dist):
    '''
    Takes a list of distance measurements and outputs a radius.
    '''
    dist_small = []
    r_big  = stats.mean(dist)
    print('measured mean is: ', r_big)
    low_high= dist.copy()
    low_high.sort()
    for i in range(3):
        dist_small.append(low_high[i])
    r_small = stats.mean(dist_small)
    return (210 - r_small)


def a_c(radius, period, mass, F_or_a= 'F'):
    '''
    Outputs the centripetal force or acceleration, use 'F' or 'a' for desired output,
    Make sure to use average distance output for dist variable.
    '''
    a_c = 4*np.pi**2 *radius/period**2/1000
    F_c = mass*a_c/1000
    if F_or_a == 'F':
        return F_c
    elif F_or_a == 'a':      
        return a_c








#test code
m = float(input('What is the mass being measured? '))
sign = input('What direction of spin?(type plus or minus) ')
duty_cycle = int(input('Desired duty cycle:  '))
wait = float(input('how long before taking measurements:  '))
num_d = int(input('desired number of distance data points:  '))
num_T = int(input('desired number of period data points:  '))
t = []
dist = []
period = []
try:
        if sign == 'plus':
            GPIO.output(Motor1E,GPIO.HIGH)
            forward.ChangeDutyCycle(duty_cycle)
            reverse.ChangeDutyCycle(0)
        elif sign == 'minus':
            GPIO.output(Motor1E,GPIO.HIGH)
            forward.ChangeDutyCycle(0)
            reverse.ChangeDutyCycle(duty_cycle)
    #tickers
        i=0
        j=0
    #timing
        time.sleep(wait)
        ti = time.time()
    #distance while loop
        while i< num_d:
                l = distance(laser)
                if l < 240:
                    dist.append(l)
                    t.append(time.time()-ti)
                    i+=1
        told = time.time()
        
        while j< num_T + 1:
                if GPIO.input(11)==1:
                    tnew = time.time()
                    period.append(tnew-told)
                    told = time.time()
                    j+=1
                    time.sleep(.2)
                    
        period.pop(0)
        GPIO.cleanup()
        r = avg_dist(dist)
        T = stats.mean(period)
        print('Period is', T, ' seconds')
        print('radius is', r, ' mm')
        print("Centripetal force is:", a_c(r, T, m, 'F'), ' N')
        print('Centripetal acceleration is:', a_c(r, T, m, 'a'), ' m/s^2')
    #outputs data csv files. If taking multiple data points make sure to create new csv files so they are not overwritten
        with open('distance_data.csv', mode = 'w') as f:
            writer =csv.writer(f)
            writer.writerow(t)
            writer.writerow(dist)
        with open('period_data.csv', mode = 'w') as f:
            writer =csv.writer(f)
            writer.writerow(period)
        plt.figure()
        plt.plot(t, dist, '.')
        plt.show()
except KeyboardInterrupt:
    GPIO.cleanup()
