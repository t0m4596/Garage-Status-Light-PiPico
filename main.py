import neopixel
import machine
import array, time
from machine import Pin
import rp2
import uasyncio as asyncio

#Configure the pins for LED readout
led_red_pin = machine.ADC(0)  # analog input pin for LED 1
led_green_pin = machine.ADC(1)  # analog input pin for LED 2

#Configure the pins for brightness POTI
poti_pin = machine.ADC(2)

# Define Variables
SLEEPTIME = 0.1         # mSeconds refresh time
BLINK_INTERVAL = 0.5    # mSeconds between yellow blinking
LOADING_SPEED = 0.1     # mSeconds between white led transition
MAX_LOADINGTIME = 30    # seconds to Alarm (blink yellow)
LED_ACTIVATION_VOLTAGE = 3.0  
LED_MIN_BRIGHTNESS = 0.05   # Value between 0 and 1
LED_MAX_BRIGHTNESS = 1.0    # Value between LED_MIN_BRIGHTNESS and 1

# Configure the number of WS2812 LEDs.
NUM_LEDS = 3
PIN_NUM = 22
brightness = 0.1

@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=24)
def ws2812():
    T1 = 2
    T2 = 5
    T3 = 3
    wrap_target()
    label("bitloop")
    out(x, 1)               .side(0)    [T3 - 1]
    jmp(not_x, "do_zero")   .side(1)    [T1 - 1]
    jmp("bitloop")          .side(1)    [T2 - 1]
    label("do_zero")
    nop()                   .side(0)    [T2 - 1]
    wrap()

# Create the StateMachine with the ws2812 program, outputting on pin
sm = rp2.StateMachine(0, ws2812, freq=8_000_000, sideset_base=Pin(PIN_NUM))

# Start the StateMachine, it will wait for data on its FIFO.
sm.active(1)

# Display a pattern on the LEDs via an array of LED RGB values.
ar = array.array("I", [0 for _ in range(NUM_LEDS)])


#########################################################################
def pixels_show():
    dimmer_ar = array.array("I", [0 for _ in range(NUM_LEDS)])
    for i,c in enumerate(ar):
        r = int(((c >> 8) & 0xFF) * brightness)
        g = int(((c >> 16) & 0xFF) * brightness)
        b = int((c & 0xFF) * brightness)
        dimmer_ar[i] = (g<<16) + (r<<8) + b
    sm.put(dimmer_ar, 8)
    time.sleep_ms(10)

def pixels_set(i, color):
    ar[i] = (color[1]<<16) + (color[0]<<8) + color[2]

def pixels_fill(color):
    for i in range(len(ar)):
        pixels_set(i, color)


BLACK = (0, 0, 0)
RED = (255, 0, 0)
YELLOW = (255, 150, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
BLUE = (0, 0, 255)
PURPLE = (180, 0, 255)
WHITE = (255, 255, 255)
COLORS = (BLACK, RED, YELLOW, GREEN, CYAN, BLUE, PURPLE, WHITE)


#################### Lenny was here start ##########################


# Define the states
class State:
    def __init__(self, name):
        self.name = name

# Define the state machine transitions
class StateMachine:
    def __init__(self):
        self.yellow = State('LED_YELLOW')
        self.red = State('LED_RED')
        self.loading = State('LOADING')
        self.blink = State('BLINK_LED_YELLOW')
        self.green = State('LED_GREEN')
        self.state = self.yellow
        
    def transition(self, is_red_on, is_green_on, is_loading_time_exceeded):
        # LED_YELLOW
        if self.state == self.yellow:
            if not is_red_on and not is_green_on:
                self.state = self.loading
            elif is_red_on and not is_green_on:
                self.state = self.red
            elif is_green_on and not is_red_on:
                self.state = self.green
        # LED_RED
        elif self.state == self.red:
            if is_red_on and is_green_on:
                self.state = self.yellow
            elif not is_red_on and not is_green_on:
                self.state = self.loading
        # LED_LOADING
        elif self.state == self.loading:
            if is_red_on and is_green_on:
                self.state = self.yellow
            elif is_green_on and not is_red_on:
                self.state = self.green
            elif is_red_on and not is_green_on:
                self.state = self.red
            elif is_green_on and not is_red_on:
                self.state = self.green
            elif is_loading_time_exceeded:
                self.state = self.blink
        # LED_BLINK
        elif self.state == self.blink:
            if is_red_on and is_green_on:
                self.state = self.yellow
            elif is_green_on and not is_red_on:
                self.state = self.green
            elif is_red_on and not is_green_on:
                self.state = self.red
        # LED_GREEN
        elif self.state == self.green:
            if is_red_on and is_green_on:
                self.state = self.yellow
            elif not is_red_on and not is_green_on:
                self.state = self.loading
            elif is_red_on and is_green_on:
                self.state = self.yellow
        
        
    def run(self):
        time_last_loading = 0
        time_first_loading = time.time()
        global brightness
        while True:
            ############# brightness control ##############
            brightness = poti_pin.read_u16() * (LED_MAX_BRIGHTNESS - LED_MIN_BRIGHTNESS) / 65535.0 + LED_MIN_BRIGHTNESS
            print("BRIGHTNESS", poti_pin.read_u16() * 3.3 / 65535.0)
            ############# brightness control ##############
            print("STATE: ", self.state.name)
            # LED_YELLOW
            if self.state == self.yellow:
                pixels_fill(YELLOW)
                pixels_show()
            # LED_RED
            elif self.state == self.red:
                pixels_fill(RED)
                pixels_show()
            # LED_LOADING
            elif self.state == self.loading:
                pixels_fill(YELLOW)
                pixels_show()
                for i in range(len(ar)):
                    pixels_set(i, WHITE)
                    pixels_show()
                    time.sleep(LOADING_SPEED)
                    pixels_set(i, YELLOW)
                    pixels_show()
                time_last_loading = time.time()
            # LED_BLINK
            elif self.state == self.blink:
                pixels_fill(YELLOW)
                pixels_show()
                time.sleep(BLINK_INTERVAL)
                pixels_fill(BLACK)
                pixels_show()
                time.sleep(BLINK_INTERVAL - SLEEPTIME)
            # LED_GREEN
            elif self.state == self.green:
                pixels_fill(GREEN)
                pixels_show()

            if not self.state == self.loading:
                time_first_loading = time.time()

            led_red_voltage = led_red_pin.read_u16() * 3.3 / 65535.0  # convert ADC reading to voltage
            led_green_voltage = led_green_pin.read_u16() * 3.3 / 65535.0  # convert ADC reading to voltage
            print("led red", led_red_voltage, "   led green", led_green_voltage)

            print("loading_time: ", time_last_loading - time_first_loading)
            
            is_led_red_on = led_red_voltage > LED_ACTIVATION_VOLTAGE
            is_led_green_on = led_green_voltage > LED_ACTIVATION_VOLTAGE
            is_loading_time_exceeded = (time_last_loading - time_first_loading) > MAX_LOADINGTIME
            self.transition(is_led_red_on, is_led_green_on, is_loading_time_exceeded)
            time.sleep(SLEEPTIME)

async def taskSmLED():
    StateMachine().run()

async def taskTomasKram():
    global brightness
    #while True:
    #    brightness = poti_pin.read_u16() * 1.0 / 65535.0

loop = asyncio.get_event_loop()
loop.create_task(taskSmLED())
loop.create_task(taskTomasKram())
loop.run_forever()

#################### Lenny was here end ##########################
