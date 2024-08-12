import RPi.GPIO as gp
from adafruit_motor import stepper
from adafruit_motorkit import MotorKit
import board
import time
import sys

def turn_table(steps):
    kit = MotorKit(i2c=board.I2C())
    
    for i in range(steps):
        kit.stepper1.onestep(direction=stepper.FORWARD, style=stepper.DOUBLE)
        time.sleep(0.01)
    
    kit.stepper1.release()
    gp.cleanup()

def main():
    try:
        fn = sys.argv[1]
    except IndexError:
        print("Error: No steps provided as an argument.")
        return

    try:
        steps = int(fn)
    except ValueError:
        print("Error: Provided argument is not an integer.")
        return

    turn_table(steps)

if __name__ == '__main__':
    main()
