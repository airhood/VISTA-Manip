import time
from . import arduino_uart

def main():
    start_time = time.perf_counter()
    test_result = arduino_uart.test()
    end_time = time.perf_counter()

    latency_ms = (end_time - start_time) * 1000

    print(f"'arduino_uart' Test Result: {test_result}, Latency: {latency_ms:.2f}ms")

    return latency_ms

if __name__ == "__main__":
    latencies = []

    for i in range(5):
        latency = main()
        latencies.append(latency)
        time.sleep(0.001)

    print(f"\n=== Latency Statistics ===")
    print(f"Average: {sum(latencies) / len(latencies):.2f}ms")
    print(f"Min: {min(latencies):.2f}ms")
    print(f"Max: {max(latencies):.2f}ms")
    print(f"Std Dev: {(sum((x - sum(latencies)/len(latencies))**2 for x in latencies) / len(latencies))**0.5:.2f}ms")