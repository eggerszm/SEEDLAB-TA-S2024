"""
LCD handler functions, including current state tracking
"""

import board
import time
from smbus2 import SMBus
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

class LCD():

    def __init__(self):
        """Initialize the LCD and prepare it to display text

        Returns:
            character_lcd.Character_LCD_RGB_I2C: LCD connected
        """
        self.lcd = character_lcd.Character_LCD_RGB_I2C(board.I2C(), 16, 2)
        self.lcd.clear()
        self.lcd.text_direction = self.lcd.LEFT_TO_RIGHT
        self.current_msg = ""
        self.lcd.color = [100, 100, 100]

    def cleanup(self):
        self.lcd.color = [0,0,0]
        self.lcd.clear()


    def write_lcd(self, msg: str):
        if msg != self.current_msg:
            self.lcd.clear()
            self.lcd.message = msg
            self.current_msg = msg

