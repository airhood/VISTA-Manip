import asyncio
from . import arduino_uart

async def main():
    test_result = await arduino_uart.test()
    print(f"'arduino_uart' Test Result: {test_result}")

if __name__ == "__main__":
    import time
    while True:
        asyncio.run(main())
        time.sleep(0.001)
