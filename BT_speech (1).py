#!/usr/bin/env python
import time
import serial
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
ser = serial.Serial(
   port='/dev/ttyUSB0',  #ttyAMA0 ttyS0
   baudrate = 9600,
   parity=serial.PARITY_NONE,
   stopbits=serial.STOPBITS_ONE,
   bytesize=serial.EIGHTBITS,
   timeout=1
)
IN1=27
IN2=22
GPIO.setup(IN1,GPIO.OUT)
GPIO.setup(IN2,GPIO.OUT)

GPIO.output(IN1,False)
GPIO.output(IN2,False)
import time
from gtts import gTTS
from mutagen.mp3 import MP3
import time
import pygame


SW = 5
GPIO.setup(SW, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
LCD_RS = 21
LCD_E  = 20
LCD_D4 = 26
LCD_D5 = 19
LCD_D6 = 13
LCD_D7 = 6

# Define some device constants
LCD_WIDTH = 20    # Maximum characters per line
LCD_CHR = True
LCD_CMD = False

LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line
LCD_LINE_3 = 0x90 # LCD RAM address for the 2nd line
LCD_LINE_4 = 0xD0 # LCD RAM address for the 2nd line

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005
BZ = 16
####lcd end here###########
GPIO.setup(BZ, GPIO.OUT)
GPIO.output(BZ, False)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

#GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_UP)#Button to GPIO12
      # Use BCM GPIO numbers
GPIO.setup(LCD_E, GPIO.OUT)  # E
GPIO.setup(LCD_RS, GPIO.OUT) # RS
GPIO.setup(LCD_D4, GPIO.OUT) # DB4
GPIO.setup(LCD_D5, GPIO.OUT) # DB5
GPIO.setup(LCD_D6, GPIO.OUT) # DB6
GPIO.setup(LCD_D7, GPIO.OUT) # DB7

####################LCD#######################
def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  time.sleep(E_DELAY)

def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = data
  # mode = True  for character
  #        False for command

  GPIO.output(LCD_RS, mode) # RS

  # High bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x10==0x10:
    GPIO.output(LCD_D4, True)
  if bits&0x20==0x20:
    GPIO.output(LCD_D5, True)
  if bits&0x40==0x40:
    GPIO.output(LCD_D6, True)
  if bits&0x80==0x80:
    GPIO.output(LCD_D7, True)

  # Toggle 'Enable' pin
  lcd_toggle_enable()

  # Low bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x01==0x01:
    GPIO.output(LCD_D4, True)
  if bits&0x02==0x02:
    GPIO.output(LCD_D5, True)
  if bits&0x04==0x04:
    GPIO.output(LCD_D6, True)
  if bits&0x08==0x08:
    GPIO.output(LCD_D7, True)

  # Toggle 'Enable' pin
  lcd_toggle_enable()

def lcd_toggle_enable():
  # Toggle enable
  time.sleep(E_DELAY)
  GPIO.output(LCD_E, True)
  time.sleep(E_PULSE)
  GPIO.output(LCD_E, False)
  time.sleep(E_DELAY)

def lcd_string(message,line):
  # Send string to display
  message = message.ljust(LCD_WIDTH," ")

  lcd_byte(line, LCD_CMD)

  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)

#####################LCD HERE######################
def level1():
    print('move down')
    lcd_byte(0x01, LCD_CMD)
    lcd_string("BED IS MOVING  ",LCD_LINE_1)
    lcd_string("  DOWN    ",LCD_LINE_2)
    GPIO.output(IN1,True)
    GPIO.output(IN2,False)
    time.sleep(0.2)
    GPIO.output(IN1,False)
    GPIO.output(IN2,False)

def flat():
    print('move up')
    lcd_byte(0x01, LCD_CMD)
    lcd_string("BED IS MOVING  ",LCD_LINE_1)
    lcd_string("  UP    ",LCD_LINE_2)
    GPIO.output(IN1,False)
    GPIO.output(IN2,True)
    time.sleep(0.2)
    GPIO.output(IN1,False)
    GPIO.output(IN2,False)
count1=0
lcd_init()
lcd_byte(0x01, LCD_CMD)
lcd_string(" SMART BED  ",LCD_LINE_1)
lcd_string("  SYSTEM    ",LCD_LINE_2)
time.sleep(3)
lcd_byte(0x01, LCD_CMD)
lcd_string(" VOICE MODE ",LCD_LINE_1)
lcd_string("  SELECTED    ",LCD_LINE_2)
time.sleep(2)
while True:
   # flat()
   x=ser.readline()
   x = x.decode('UTF-8','ignore')
   x = x.lower()
   
   if len(x.strip())>0:
       print('You said {}'.format(x))
       y = 'say again'
       if count1<3:
           
           if x == 'move':
               flat()
               count1=count1+1
       if count1>0:
           
           if x == 'down':
               level1()
               count1=count1-1
                                 
       print(y)
   if (GPIO.input(SW)==False):
        print("EMERGENCY....!!!!!!")
        lcd_byte(0x01, LCD_CMD)
        lcd_string("EMERGENCY....!!!!!!",LCD_LINE_1)
        time.sleep(2)       
