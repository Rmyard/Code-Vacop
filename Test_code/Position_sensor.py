import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

# MCP3008 configuration
CLK = 18
MISO = 23
MOSI = 24
CS = 25
mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

try:
    print("Reading position sensor value (Ctrl+C to exit):")
    while True:
        # Read position from MCP3008
        position_value = mcp.read_adc(0)

        # Print position value
        print("Position sensor value:", position_value)

        # Add a delay to reduce console output rate
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nProgram terminated by user.")
