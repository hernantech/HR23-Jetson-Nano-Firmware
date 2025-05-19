import can
import sys

# A script to print any results from can interface for 60 attempts
# with at most one second between attempts

bus = can.Bus(channel='can0', interface='socketcan')
try:
    reads = 60
    while reads:
        message = bus.recv(1.0)
        # print(message)
        if message:
            print(f"\n{str(reads).rjust(5)}", message)
        else:
            sys.stdout.write(f"\r{str(reads).rjust(5)}: NO MESSAGE")
            sys.stdout.flush()
        reads -= 1
finally:
    bus.shutdown()
    print()

