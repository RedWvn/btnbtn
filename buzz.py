import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(19, GPIO.OUT)
 
p = GPIO.PWM(19, 1000)
#p.start(0)

def parking_beep():

    beep_sequence = [
        (1000, 0.5),  # Beep for 200ms at 1000Hz
                      
        ]

    count = 0
    # Loop through the beep sequence and play each tone
    while count < 6:

        for frequency, duration in beep_sequence:
            # Set the duty cycle for the given frequency
            p.ChangeFrequency(frequency)
            p.start(100)  # Start the PWM with a duty cycle of 50%
            
            # Wait for the given duration
            time.sleep(duration)
            
            # Stop the PWM output
            p.stop()
            
            # Pause before the next beep
            time.sleep(0.5)

        count += 1


def siren_beep():

    # Define the siren sound
    siren_sound = [
        (1000, 0.2),  # Beep for 200ms at 1000Hz
        (1500, 0.2),  # Beep for 200ms at 1500Hz
        (2000, 0.2),  # Beep for 200ms at 2000Hz
        (2500, 0.2),  # Beep for 200ms at 2500Hz
        (3000, 0.2),  # Beep for 200ms at 3000Hz
        (3500, 0.2),  # Beep for 200ms at 3500Hz
        (4000, 0.2),  # Beep for 200ms at 4000Hz
    ]

    # Loop through the siren sound and play each tone
    for frequency, duration in siren_sound:
        # Set the duty cycle for the given frequency
        p.ChangeFrequency(frequency)
        p.start(50)  # Start the PWM with a duty cycle of 50%
        
        # Wait for the given duration
        time.sleep(duration)
        
        # Stop the PWM output
        p.stop()
        
        # Pause before the next beep
        time.sleep(0.1)


def r2_d2():

    # Define the R2D2 sound sequence
    r2d2_sequence = [
        (523, 0.2),   # C5 for 200ms
        (0.1, 0.1),     # Pause for 100ms
        (523, 0.2),   # C5 for 200ms
        (0.1, 0.1),     # Pause for 100ms
        (523, 0.2),   # C5 for 200ms
        (0.1, 0.1),     # Pause for 100ms
        (523, 0.2),   # C5 for 200ms
        (0.1, 0.1),     # Pause for 100ms
        (587, 0.2),   # D5 for 200ms
        (0.1, 0.1),     # Pause for 100ms
        (523, 0.2),   # C5 for 200ms
        (0.1, 0.1),     # Pause for 100ms
        (659, 0.2),   # E5 for 200ms
        (0.1, 0.1),     # Pause for 100ms
        (783, 0.2),   # G5 for 200ms
    ]

    # Loop through the R2D2 sound sequence and play each tone
    for frequency, duration in r2d2_sequence:
        # Set the duty cycle for the given frequency
        p.ChangeFrequency(frequency)
        p.start(50)  # Start the PWM with a duty cycle of 50%
        
        # Wait for the given duration
        time.sleep(duration)
        
        # Stop the PWM output
        p.stop()
        
        # Pause before the next beep
        time.sleep(0.1)


try:
    # while True:
    #     for dc in range(0, 101, 5):
    #         p.ChangeDutyCycle(dc)
    #         time.sleep(0.1)
    #     break

    # p.stop(100)
    # Define the beep sequence
    parking_beep()
    #siren_beep()
    #r2_d2()

 
except KeyboardInterrupt:
    pass
p.stop()
GPIO.cleanup()


