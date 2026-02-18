from . import arduino_uart

def main():
    test_result = arduino_uart.test()
    print(f"'arduino_uart' Test Result: {test_result}")

if __name__ == "__main__":
    import time
    while True:
        main()
        time.sleep(0.001)
