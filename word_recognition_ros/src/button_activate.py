import RPi.GPIO as GPIO # Import Raspberry Pi GPIO library


GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)

while True: # Run forever
    if GPIO.input(17) == GPIO.LOW:
        print("Button was pushed!")