#Code to read all analog inputs from ADC

import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

# MCP3008 configuration
CLK = 21
MISO = 19
MOSI = 20
CS = 7
mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

try:
    print("Reading position sensor value (Ctrl+C to exit):")
    while True:
        # Read position from MCP3008
        position_value1 = mcp.read_adc(0)
        position_value2 = mcp.read_adc(1)
        position_value3 = mcp.read_adc(2)
        time.sleep(0.1)



        # Print position value
        print("Position sensor value 1:", position_value1)
        print("Position sensor value 2:", position_value2)
        print("Position sensor value 3:", position_value3)


except KeyboardInterrupt:
    print("\nProgram terminated by user.")
