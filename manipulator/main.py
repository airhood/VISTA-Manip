import arduino_uart
import drone_rpi_uart

def initSystem():
    arduino_uart.init()
    drone_rpi_uart.init()

def main():
    initSystem()

if __name__ == "__main__":
    main()