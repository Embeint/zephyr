#!/usr/bin/env python3

import sys
import serial
import time

ser = serial.Serial()
ser.port = str(sys.argv[1])
ser.baudrate = 115200
ser.open()


def throughput(start_time, now, bytes_read):
    throughput = (bytes_read * 8) / (now - start_time)
    print(f"Read {bytes_read} in {now - start_time:.2f} seconds ({int(throughput)} kbps)")


start = time.time()
last_report = int(start)
bytes_read = 0
try:
    while True:
        read = ser.read(1024)
        if len(read) and bytes_read == 0:
            print("Successfully read first buffer...")
        bytes_read += len(read)
        # Periodic reports
        now = time.time()
        if int(now) != last_report:
            throughput(start, now, bytes_read)
            last_report = int(now)

except KeyboardInterrupt:
    pass

throughput(start, time.time(), bytes_read)
