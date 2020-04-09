import RPi.GPIO as GPIO #standard library import so we can use GPIO pins
from time import sleep #standard libary import so we can use sleep function in the setAngle function

GPIO.setmode(GPIO.BOARD) #pin, labelling convention of pins, alternative is GPIO.BCM (broadcom is the processor on the pi)
GPIO.setup(11, GPIO.OUT)  #pin, set the pin to output signal, alternative is GPIO.IN

pwm=GPIO.PWM(11, 50) # pin, frequency (hz)
pwm.start(0) #start pwm stuff on our pin

def setAngle(angle):
	duty = angle / 18 + 3 #find out why it is / 18+3
	GPIO.output(11, True) #pin, set pin HIGH or '1'
	pwm.ChangeDutyCycle(duty) #make the duty cycle equal the answer of angle / 18+3
	sleep(1) #sleep a second
	GPIO.output(11, False) #let go of the pin
	pwm.ChangeDutyCycle(duty)
	
#####ROTATE SERVO USING BELOW#####
setAngle(95) #run the block of code inside the setAngle function by CALLING the function
################################

pwm.stop() #cease all pwm stuff from the pin 
GPIO.cleanup() #stand pin housekeeping