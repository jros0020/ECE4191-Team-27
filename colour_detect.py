import RPi.GPIO as GPIO
import time
import csv
#set pin numbers for sensors

S2 = 6
S3 = 13
OUT = 5
NUM_CYCLES = 10
readCount = 0
maxRead = 10
diffThreshold = 30 #difference in average frequency values 

def setup():
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(OUT,GPIO.IN, pull_up_down=GPIO.PUD_UP)
  GPIO.setup(S2,GPIO.OUT)
  GPIO.setup(S3,GPIO.OUT)
  print("\n")

def find_frequency():
        start = time.time()
        #see how long it takes to read NUM_CYCLES
        for impulse_count in range(NUM_CYCLES):
            GPIO.wait_for_edge(OUT, GPIO.FALLING)
        duration = time.time() - start
        frequency = NUM_CYCLES/duration
        return frequency


def detect_loop():
    # set pins to read red
    GPIO.output(S2,GPIO.LOW)
    GPIO.output(S3,GPIO.LOW)

    red  = find_frequency()   

    #set pins to read blue
    GPIO.output(S2,GPIO.LOW)
    GPIO.output(S3,GPIO.HIGH)


    blue = find_frequency() - 3000
    
    #set pins to read green
    GPIO.output(S2,GPIO.HIGH)
    GPIO.output(S3,GPIO.HIGH)

    green = find_frequency()
    

    # print("red: ", red, "blue: ", blue, "green: ", green)
    
    colour_frequencies = [red, blue, green]
    package_colour_idx = colour_frequencies.index(max(colour_frequencies))
    colour_frequencies_keys = {"red": red, "blue": blue, "green": green}
    package_colour = list(colour_frequencies_keys)[package_colour_idx]
    
    # self.package_colour_pub.publish(package_colour)
    
    time.sleep(0.1)
    
    return package_colour

    
def endprogram():
    GPIO.cleanup()     
    
if __name__=='__main__':
    S2 = 6
    S3 = 13
    OUT = 5
    NUM_CYCLES = 10
    colours = []
    setup()
    
    time_elapsed = 0
    init_time = time.time()
    
    while time_elapsed < 10:
        colours.append(detect_loop())
        time_elapsed = time.time() - init_time
        
    csv_file_name = 'ece4191/ECE4191-Team-27/data/green_colour_data.csv'
    
    with open(csv_file_name, 'w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)

        # Write the data to the CSV file row by row
        for row in colours:
            csv_writer.writerow([row])

    print(f'Data has been saved to {csv_file_name}')

