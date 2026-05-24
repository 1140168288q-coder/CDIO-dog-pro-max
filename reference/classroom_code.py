# Start by identifying the needed libraries
import machine, utime
from machine import Pin, PWM, ADC, time_pulse_us
from time import sleep

############################################################
# SETUP WORK

# Setup PWM for the servos
RR_Servo = PWM(Pin(5)) #Right Rear Servo
LR_Servo = PWM(Pin(2)) #Left Rear Servo
RF_Servo = PWM(Pin(10)) #Right Forward Servo
LF_Servo = PWM(Pin(21)) #Left Forward Servo
US_Servo = PWM(Pin(18)) #Ultrasonic Sensor Servo

# Set Frequency for each servo (50 Hz is standard for hobby servos)
RR_Servo.freq(50)
LR_Servo.freq(50)
RF_Servo.freq(50)
LF_Servo.freq(50)
US_Servo.freq(50)

# pause between step commands
step_time = 300 # valaue in milliseconds (ms)
# leg arc in degrees
arc = 26
# servo location values (4915 is 90 degrees)
mid_point = 90
one, two, three, four = int(mid_point-arc/2), int(mid_point-arc/6), int(mid_point+arc/6), int(mid_point+arc/2)

# Variables to track timing. NOTE: we want to avoid using US_Sensor
# or US_Servo at the same time as a walking servo to keep peak power
# demand low.
current_step = 0
last_step_time = utime.ticks_ms()
scan_index = 1 # set to 1 as the US servo initilizes to 90 degrees
scan_counter = 0
last_scan_time = utime.ticks_ms()


scan_angles = [5, 90, 175, 90] 
distances = [100, 100, 100, 100]  # Store readings here

# define pin for LDR
ldr = ADC(Pin(26)) # LDR connected to GP26 (ADC0)

# US Sensor Pins
trig = Pin(14, Pin.OUT)
echo = Pin(15, Pin.IN)

# global variables for sensors
ldr_threshold = 60000 #set the threshold value for "bright light"
ldr_value = 0

#END of SETUP WORK

############################################################
# Custom function to assign position to servo based on degree input using u16
def set_position(servo, angle):
    #Note: may need to add servo-specific adjustments later using if statements
    min_duty = 1638 # 0° = 0.5 ms pulse
    max_duty = 8192 # 180° = 2.5 ms pulse
    duty = int(min_duty + (angle / 180) * (max_duty - min_duty))
    servo.duty_u16(duty) 
# End of set_position function

# Custom function to initialize legs (set all to 90 degrees)
def initialize():
    set_position(RR_Servo, 90)
    set_position(LR_Servo, 90)
    set_position(RF_Servo, 90)
    set_position(LF_Servo, 90)
    # Set US_Servo to 90 degrees
    set_position(US_Servo, 90)
# End of initialize function

# custom function to read LDR
def read_ldr():
    value = ldr.read_u16()  # Read the LDR value and convert it to a 16-bit unsigned integer
    percent = round((ldr_value / 65535) * 100, 1)
    #print(f"Raw: {ldr_value} | Brightness: {percent}%") # Print the LDR value to the console
    return value

# Distance function
def get_distance(samples=5):
    
    total_pulse_time = 0
    valid_samples = 0
    for _ in range(samples):
    
        trig.value(0)
        utime.sleep_us(2)
        # Send 10us high pulse
        trig.value(1)
        utime.sleep_us(10)
        trig.value(0)
        
        
        # Measure the duration of the high pulse on the echo pin
        # 12000us is a ~2-meter timeout; if no echo, it returns -2 or -1
        pulse = machine.time_pulse_us(echo, 1, 12000)
        if pulse > 0:
            total_pulse_time += pulse
            valid_samples += 1
        # Small delay between samples so waves can dissipate
        utime.sleep_ms(10)
    if valid_samples == 0:
        return 100 # No valid echoes received     
    avg_pulse_time = total_pulse_time / valid_samples
    #print("average pulse time: ", avg_pulse_time)    
     
    # Speed of sound ~0.0343 cm/us
    # Distance equals (time * speed) / 2 (for send and return)
    distance = (avg_pulse_time * 0.0343) / 2
    #print("Distance in cm: ", distance) 
    return distance    
# End of distance function

# Collect an array of distances function
def US_Scan(now):
    global scan_index, last_scan_time, last_step_time, scan_counter
    
    if utime.ticks_diff(now, last_scan_time) > step_time/2:
        if scan_counter % 2 == 0: #check to see if scan_counter is an even number
            # 1. Take measurement at CURRENT position
            distances[scan_index] = get_distance()
            scan_index = (scan_index + 1) % 4
            scan_counter = (scan_counter + 1)
            last_scan_time = now
            #print("distances array: ", distances)
            return True # Indicates a new reading was just added
        else: 
            # 2. Move to NEXT position (if scan_counter is not an even number)
            #print("Moving servo to index:", scan_index)
            set_position(US_Servo, scan_angles[scan_index])
            #print("last scan time: ", last_scan_time, " Step Time: ", step_time)
            scan_counter = (scan_counter + 1)
            last_scan_time = now
            return False # indicates no reading, servo moved instead
# End of array collection

# Custom function to walk
def walk(now):
    # NOTE: you will need to adjust walking logic for turns and reversing.
    # Check if enough time has passed to take the NEXT step
    global last_step_time, current_step  # Tell Python to use the variables defined outside
    if utime.ticks_diff(now, last_step_time) > (step_time):
        last_step_time = now
        # Get the angles for the current step
        s = steps[current_step]
        #Move the Servos
        set_position(RF_Servo, s[0])
        set_position(RR_Servo, s[1])
        set_position(LF_Servo, s[2])
        set_position(LR_Servo, s[3])
        # Increment step counter and reset to 0 if it hits 4
        current_step = (current_step + 1) % len(steps)    
# end of walk function

############################################################

# Main Function
try:
    initialize()
    sleep(0.1)
    steps = [
        (four, one, three, two), # Step 1
        (three, four, four, three), # Step 2
        (two, three, one, four), # Step 3
        (one, two, two, one)  # Step 4
    ]
    while True:
        now = utime.ticks_ms()
        walk(now)
        US_Scan(now)
        ldr_value = read_ldr()
        if ldr_value > ldr_threshold:
            while ldr_value > ldr_threshold:
                #print("Bright light -> sleep")
                #print(ldr_value)
                sleep(0.2) 
                ldr_value = read_ldr()
        # It makes sense to put a stop for colour here similar to the ldr sleep loop above
    
    
except KeyboardInterrupt:
    print("Keyboard interrupt")