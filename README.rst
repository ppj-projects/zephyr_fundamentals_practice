This project is built for the norid nRF54LM20 board.

Example project copied using nordic's/zephyr's blinky example as a basis to just get the project layout correctly.

This is a practice repo to put together what is taught in Nordic's SDK fundamentals course for their nRF54LM20DK board.

The goal of this project is to read a temperature from a BME280. When a change in temperature is detected, the user will be notified via LED for 2 seconds, and a console print over UART.  Users will also have an option to pause the reading by pressing a button.

Onboard hardware used:
LED3 - user notification
BUTTON3 - application pause
UART - 
I2C - 
