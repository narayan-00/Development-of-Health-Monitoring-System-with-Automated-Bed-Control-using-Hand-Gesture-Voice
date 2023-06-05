#!/usr/bin/env python
import time
import serial
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
import time
import max30100
import Adafruit_DHT
mx30 = max30100.MAX30100()
mx30.enable_spo2()
import telepot
import urllib.request
import requests
import threading
import json

import random
bot=telepot.Bot("6281868215:AAFxaP6Z4psULn59XUad-KN2z2EfHQoStUY")
sensor = Adafruit_DHT.DHT11
global heartRate,bp,spo2,temperature,humidity


pin = 4
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

####lcd end here###########


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

BZ = 16
import Adafruit_DHT
GPIO.setup(BZ, GPIO.OUT)
GPIO.output(BZ, False)
gas= 17
HR_SENSOR = 12
GPIO.setup(HR_SENSOR,GPIO.IN)
GPIO.setup(gas,GPIO.IN)
lcd_init()
lcd_byte(0x01, LCD_CMD)
lcd_string(" SMART BED  ",LCD_LINE_1)
lcd_string("  SYSTEM    ",LCD_LINE_2)
time.sleep(3)
lcd_byte(0x01, LCD_CMD)
lcd_string(" HEALTH MONITORING ",LCD_LINE_1)
lcd_string("  SYSTEM    ",LCD_LINE_2)
time.sleep(2)
global tempFlag
global bpFlag
global hrFlag
tempFlag = 0
bpFlag   = 0
hrFlag   = 1
while True:

    if hrFlag == 1 :
        print('Hold The finger On sensor')
        lcd_byte(0x01, LCD_CMD)
        lcd_string('Hold finger On sensor',LCD_LINE_1)
        lcd_string('On sensor',LCD_LINE_2)
        time.sleep(1)
        #time.sleep(1) 
        sensorCounter = 0
        startTime     = 0
        endTime       = 0
        rateTime      = 0
    while sensorCounter < 1 and  hrFlag == 1:
        if (GPIO.input(HR_SENSOR)):
          if sensorCounter == 0:
            startTime = int(round(time.time()*1000))
            #print startTime
          sensorCounter = sensorCounter + 1
          #print sensorCounter
          while(GPIO.input(HR_SENSOR)):
            if hrFlag == 0:
              break
            pass

    time.sleep(1)      
    endTime  = int(round(time.time()*1000))
    #print endTime
    rateTime = endTime - startTime
    #print rateTime
    rateTime = rateTime / sensorCounter
    heartRate = (60000 / rateTime) #/ 3 
    heartRate = abs(heartRate)
    heartRate=int(heartRate+20)
    print (heartRate)
    lcd_byte(0x01, LCD_CMD)
    lcd_string("HeartRate :{}".format(heartRate),LCD_LINE_1)
    bot.sendMessage("1210994566",str("HEART RATE : {}".format(heartRate)))
    time.sleep(1)
    if heartRate<40:
        GPIO.output(BZ, True)
        lcd_byte(0x01, LCD_CMD)
        lcd_string("HEARTRATE IS ABNORMAL",LCD_LINE_1)
        time.sleep(1)
        GPIO.output(BZ, False)
#def SPO2():
    mx30.read_sensor()

    mx30.ir, mx30.red

    bp = int(mx30.ir / 100)

    spo2 = int(mx30.red / 100)+10
    
    if mx30.ir != mx30.buffer_ir :
        print("BP LEVEL:",bp);
        bot.sendMessage("1210994566",str("BP LEVEL : {}".format(bp)))
        lcd_byte(0x01, LCD_CMD)
        lcd_string("BP :{}".format(bp),LCD_LINE_1)
        time.sleep(1)
        if bp> 170:
            GPIO.output(BZ, True)
            lcd_byte(0x01, LCD_CMD)
            lcd_string("BP IS MORE",LCD_LINE_1)
            time.sleep(1)
            GPIO.output(BZ, False)
    if mx30.red != mx30.buffer_red:
        print("SPO2:",spo2);
        lcd_byte(0x01, LCD_CMD)
        lcd_string("SPO2  :{}".format(spo2),LCD_LINE_1)
        bot.sendMessage("1210994566",str("SPO2 : {}".format(spo2)))
        time.sleep(1)
        if spo2<80:
            GPIO.output(BZ, True)
            lcd_byte(0x01, LCD_CMD)
            lcd_string("SPO2 IS LESS",LCD_LINE_1)
            time.sleep(1)
            GPIO.output(BZ, False)
    time.sleep(1)
#def HT_Reading():
    humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)
    print('Humidity: {} and Temperature: {}'.format(humidity, temperature))
    
    lcd_byte(0x01, LCD_CMD)
    lcd_string("humidity  :{}".format(humidity),LCD_LINE_1)
    lcd_string("Temperature  :{}".format(temperature),LCD_LINE_1)
    bot.sendMessage("1210994566",str("humidity : {}".format(humidity)))
    bot.sendMessage("1210994566",str("Temperature : {} c".format(temperature)))
    time.sleep(1)
    if temperature>32:
            GPIO.output(BZ, True)
            lcd_byte(0x01, LCD_CMD)
            lcd_string("MORE TEMPERATURE",LCD_LINE_1)
            time.sleep(1)
            GPIO.output(BZ, False)
    
#def air():
    if(GPIO.input(gas)==True):
        print("air quality is bad")
        lcd_byte(0x01, LCD_CMD)
        lcd_string("BAD AIR QUALITY",LCD_LINE_1)
        bot.sendMessage("1210994566",str("Air quality is bad "))
        time.sleep(2)
        

    URl='https://api.thingspeak.com/update?api_key=4B35650D21X8YEEF&field1=10&field2=20&field3=30&field4=40&field5=50'
    KEY='4B35650D21X8YEEF'
    HEADER='&field1={}&field2={}&field3={}&field4={}&field5={}'.format(heartRate,temperature,spo2,bp,humidity)
    NEW_URL = URl+KEY+HEADER
    print(NEW_URL)
    data=urllib.request.urlopen(NEW_URL)
    print(data)
 