"""
Exercise 1a

Elijah Price

Get user input [0, 100], send it to the arduino, request input + 100, and display to LCD
"""

import board
import time
from smbus2 import SMBus
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

ARD_ADDR = 8


def init_lcd() -> character_lcd.Character_LCD_RGB_I2C:
    """Initialize the LCD and prepare it to display text

    Returns:
        character_lcd.Character_LCD_RGB_I2C: LCD connected
    """
    lcd_columns, lcd_rows = 16, 2
    lcd = character_lcd.Character_LCD_RGB_I2C(board.I2C(), lcd_columns, lcd_rows)
    lcd.clear()
    lcd.text_direction = lcd.LEFT_TO_RIGHT
    return lcd


