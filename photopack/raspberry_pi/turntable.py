import RPi.GPIO as gp
from adafruit_motor import stepper
from adafruit_motorkit import MotorKit
import board
import time
import sys
from typing import Optional

from .constants import TURNTABLE_STEP_DELAY, DEFAULT_STEP_STYLE, DEFAULT_DIRECTION

def turn_table(steps: int) -> None:
    """
    Rotate the turntable by the specified number of steps.
    
    Args:
        steps: Number of steps to rotate
    """
    kit = MotorKit(i2c=board.I2C())
    
    # Get direction and style from constants
    direction = getattr(stepper, DEFAULT_DIRECTION)
    style = getattr(stepper, DEFAULT_STEP_STYLE)
    
    for i in range(steps):
        kit.stepper1.onestep(direction=direction, style=style)
        time.sleep(TURNTABLE_STEP_DELAY)
    
    kit.stepper1.release()
    gp.cleanup()

def main() -> None:
    """
    Main entry point for the turntable control script.
    Expects number of steps as command line argument.
    """
    try:
        if len(sys.argv) < 2:
            print("Error: No steps provided as an argument.")
            print("Usage: python turntable.py <number_of_steps>")
            return
            
        steps_arg = sys.argv[1]
        steps = int(steps_arg)
        
        if steps < 0:
            print("Error: Number of steps must be non-negative.")
            return
            
        if steps > 10000:  # Reasonable upper limit
            print("Error: Number of steps exceeds maximum limit (10000).")
            return
            
    except ValueError:
        print(f"Error: '{steps_arg}' is not a valid integer.")
        return

    turn_table(steps)

if __name__ == '__main__':
    main()
