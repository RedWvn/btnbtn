import time
from xml.etree.ElementTree import PI
import RPi.GPIO as GPIO
import neopixel
import board

#User Params

BUTTON_PIN = 17         #declare the GPIO 23 pin for the BUTTON input
num_pixels = 6          #declare the number of NeoPixels
pixel_pin = board.D10   #declare the NeoPixel out put pin
BUTTON_PRESS_TIME = 5   #Button press state time
BLINKING_TIME_SEQUENCES = 5

#Warning: Do not change anything from here

GPIO.setmode(GPIO.BCM)  # for GPIO numbering, choose BCM
GPIO.setwarnings(False) # to disable warnings.
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  

ORDER = neopixel.RGB
# The order of the pixel colors - RGB or GRB. Some NeoPixels have red and green reversed!
# For RGBW NeoPixels, simply change the ORDER to RGBW or GRBW.

pixels = neopixel.NeoPixel(
    pixel_pin, num_pixels, brightness=1.5, pixel_order=ORDER
)


def wheel(pos):
    # Input a value 0 to 255 to get a color value.
    # The colours are a transition r - g - b - back to r.
    if pos < 0 or pos > 255:
        r = g = b = 0
    elif pos < 85:
        r = int(pos * 3)
        g = int(255 - pos * 3)
        b = 0
    elif pos < 170:
        pos -= 85
        r = int(255 - pos * 3)
        g = 0
        b = int(pos * 3)
    else:
        pos -= 170
        r = 0
        g = int(pos * 3)
        b = int(255 - pos * 3)
    return (b, g, r) if ORDER in (neopixel.RGB, neopixel.GRB) else (b, g, r, 0)

def rainbow_cycle(wait):
    for j in range(255):
        for i in range(num_pixels):
            pixel_index = (i * 255 // num_pixels) + j
            pixels[i] = wheel(pixel_index & 255)
        pixels.show()
        time.sleep(wait)

def btn_main():
    while True:

        GPIO.wait_for_edge(BUTTON_PIN, GPIO.FALLING)
        pixels.fill((175, 0, 255))
        print ("Button press detected")
        start = time.time()
        time.sleep(0.2)

        while GPIO.input(BUTTON_PIN) == GPIO.LOW:
            time.sleep(0.01)
        length = time.time() - start
        print (length)
        pixels.fill((0, 0, 0))

        if length > BUTTON_PRESS_TIME:    
        #if button pressed time is greater than BUTTON_PRESS_TIME:value then next sequence will begin
            print ("Arming sequences started")

            for i in range(BLINKING_TIME_SEQUENCES):    
                #  this FOR loop is for, blinking LED interval of 500 miliseconds 
                    pixels.fill((175, 255, 0))
                    time.sleep(0.5) 

                    pixels.fill((155, 255, 155))
                    time.sleep(0.5)  

            pixels.fill((0, 255, 0))
            time.sleep(5)

            #rainbow_cycle(0.001)


            break 
            #the break is for, this code will not run again untill reboot
            GPIO.cleanup()

        else:
            pixels.fill((0, 0, 0))
