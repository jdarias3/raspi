import RPi.GPIO as GPIO
import time
import Robot
import led
import temp
import newtemp
from multiprocessing import Pool


GPIO.setmode(GPIO.BOARD)

ObstaclePin =  12
buzzpin = 29
RIGHT = 15
LEFT = 13
LEFT_TRIM   = 0
RIGHT_TRIM  = 0
idTemp = '28-0516933c33ff'


GPIO.setup(RIGHT,GPIO.IN)
GPIO.setup(LEFT,GPIO.IN)
GPIO.setup(ObstaclePin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(buzzpin, GPIO.OUT)

GPIO.output(buzzpin, GPIO.LOW)

# Led variables #

colors = [0xFF0000, 0x00FF00, 0x0000FF, 0xFFFF00, 0xFF00FF, 0x00FFFF]
pins = {'pin_R':16, 'pin_G':18, 'pin_B':22}  # pins is a dict

for i in pins:
        GPIO.setup(pins[i], GPIO.OUT)   # Set pins' mode is output
        GPIO.output(pins[i], GPIO.HIGH) # Set pins to high(+3.3V) to off led

p_R = GPIO.PWM(pins['pin_R'], 2000)  # set Frequece to 2KHz
p_G = GPIO.PWM(pins['pin_G'], 2000)
p_B = GPIO.PWM(pins['pin_B'], 5000)

p_R.start(0)      # Initial duty Cycle = 0(leds off)
p_G.start(0)
p_B.start(0)

# end #

robot = Robot.Robot(left_trim=LEFT_TRIM, right_trim=RIGHT_TRIM)

time.sleep(0.1)


def rightSensor():
        return GPIO.input(RIGHT)

def leftSensor():
        return GPIO.input(LEFT)

def tempSensor():
        newtemp.read_temp_raw()
        result = newtemp.read_temp()
        return(result)

if __name__ == '__main__':
        pool = Pool()
        try:
                while True:
                        result1 = pool.apply_async(tempSensor,())
                        answer1 = result1.get()
                        print(answer1)
                        if answer1 > 24000:
                                led.setColor(0x0000FF)
                                #time.sleep(20)
                        if rightSensor() == 0 and leftSensor() == 0 and 1 == GPIO.input(                                                                                                             ObstaclePin):
                                print "Move forward"
                                print "Received Signal"
                                robot.forward(70, 0.1)
                                led.setColor(0x00FF00)
                                no_barrier = True
                                #print(temp.gettemp(idTemp))
                                GPIO.output(buzzpin, GPIO.LOW)
                        if rightSensor() == 1 and leftSensor() == 0 and 1 == GPIO.input(                                                                                                             ObstaclePin):
                                print "RigthSensor, turn right"
                                robot.right(50, 0.06)
                                robot.left(30, 0.06)
                                led.setColor(0x00FF00)
                                no_barrier = True
                                #print(temp.gettemp(idTemp))
                                GPIO.output(buzzpin, GPIO.LOW)
                        if rightSensor() == 0 and leftSensor() == 1 and 1 == GPIO.input(                                                                                                             ObstaclePin):
                                print "LeftSensorn, Turn Left"
                                robot.left(50, 0.06)
                                robot.right(30, 0.06)
                                led.setColor(0x00FF00)
                                no_barrier = True
                                #print(temp.gettemp(idTemp))
                                GPIO.output(buzzpin, GPIO.LOW)
                        if rightSensor() == 0 and leftSensor() == 0 and 0 == GPIO.input(                                                                                                             ObstaclePin):
                                print "Barrier is detected !"

                                led.setColor(0xFF0000)
                                if no_barrier:
                                        time.sleep(5)
                                        no_barrier = False
                                GPIO.output(buzzpin, GPIO.HIGH)
                                print("buzzer on")
                        if rightSensor() == 1 and leftSensor() == 0 and 0 == GPIO.input(                                                                                                             ObstaclePin):
                                print "Barrier is detected !"


                                led.setColor(0xFF0000)
                                if no_barrier:
                                        time.sleep(5)
                                        no_barrier = False
                                GPIO.output(buzzpin, GPIO.HIGH)
                                print("buzzer on")


                        if rightSensor() == 0 and leftSensor() == 1 and 0 == GPIO.input(                                                                                                             ObstaclePin):
                                print "Barrier is detected !"


                                led.setColor(0xFF0000)
                                if no_barrier:
                                        time.sleep(5)
                                        no_barrier = False
                                GPIO.output(buzzpin, GPIO.HIGH)
                                print("buzzer on")

                        else:
                                print "No Signal"
                                #led.setColor(0xFF0000)
                                #time.sleep(2)

                                """if dir == True:
                                        print "RigthSensor, turn right"
                                        robot.right(30, 0.05)
                                else:
                                        print "LeftSensorn, Turn Left"
                                        robot.left(30, 0.05)"""


        except KeyboardInterrupt:
                GPIO.cleanup()
                pool.terminate()
