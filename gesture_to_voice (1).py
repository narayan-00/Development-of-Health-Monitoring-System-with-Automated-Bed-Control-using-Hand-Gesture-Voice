# import necessary packages
import cv2
import numpy as np
import mediapipe as mp
from tensorflow.keras.models import load_model
import time
#!/usr/bin/env python
import time
import serial
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
BZ = 16
SW = 5
GPIO.setup(SW, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
IN1=27
IN2=22
GPIO.setup(IN1,GPIO.OUT)
GPIO.setup(IN2,GPIO.OUT)

GPIO.output(IN1,False)
GPIO.output(IN2,False)
###lcd #####################
GPIO.setup(BZ, GPIO.OUT)
GPIO.output(BZ, False)
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
count2=0

mpHands = mp.solutions.hands

hands = mpHands.Hands(max_num_hands=1, min_detection_confidence=0.7)

mpDraw = mp.solutions.drawing_utils

# Load the gesture recognizer model
model = load_model('mp_hand_gesture')

# Load class names
f = open('gesture.names', 'r')
classNames = f.read().split('\n')
f.close()

print(classNames)

# Initialize the webcam
cap = cv2.VideoCapture(0)
lcd_init()
lcd_byte(0x01, LCD_CMD)
lcd_string(" SMART BED  ",LCD_LINE_1)
lcd_string("  SYSTEM    ",LCD_LINE_2)
time.sleep(3)
lcd_byte(0x01, LCD_CMD)
lcd_string(" GESTURE MODE ",LCD_LINE_1)
lcd_string("  SELECTED    ",LCD_LINE_2)
time.sleep(2)
while True:
    if (GPIO.input(SW)==False):
        print("EMERGENCY....!!!!!!")
        lcd_byte(0x01, LCD_CMD)
        lcd_string("EMERGENCY....!!!!!!",LCD_LINE_1)
        time.sleep(2)
    # Read each frame from the webcam
    _, frame = cap.read()

    x, y, c = frame.shape

    # Flip the frame vertically
    frame = cv2.flip(frame, 1)
    framergb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Get hand landmark prediction
    result = hands.process(framergb)

    # print(result)
    
    className = ''
    count = 0
    # post process the result
    if result.multi_hand_landmarks:
        landmarks = []
        #print(result.multi_hand_landmarks)
        for handslms in result.multi_hand_landmarks:
            for lm in handslms.landmark:
                # print(id, lm)
                count += 1

                lmx = int(lm.x * x)
                lmy = int(lm.y * y)
                
                landmarks.append([lmx, lmy])
                
            # Drawing landmarks on frames
            mpDraw.draw_landmarks(frame, handslms, mpHands.HAND_CONNECTIONS)

            # Predict gesture
            prediction = model.predict([landmarks])
            # print(prediction)
            classID = np.argmax(prediction)
            className = classNames[classID]
            
            
            if count1<2:
                   
                if className == 'one':
                    flat()
                    count1=count1+1
                       
            if count1>0:
                   
                if className == 'two':
                    level1()
                    count1=count1-1
            
            # show the prediction on the frame
            cv2.putText(frame, className, (300, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                           1, (0,0,255), 2 )
            
    # Show the final output
    cv2.imshow("Gesture to voice", frame) 

    if cv2.waitKey(1) == ord('q'):
        break

# release the webcam and destroy all active windows
cap.release()
cv2.destroyAllWindows()
