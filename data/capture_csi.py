import sys
import time
import serial

if len(sys.argv) < 3:
    print("Usage: python3 capture_csi.py <port> <output_csv> [baud]")
    sys.exit(1)

port = sys.argv[1]
out_path = sys.argv[2]
baud = int(sys.argv[3]) if len(sys.argv) > 3 else 115200

ser = serial.Serial(port, baud, timeout=1)
ser.dtr = False
ser.rts = False
count = 0

print(f"Capturing CSI_DATA from {port} @ {baud} into {out_path}")
print("Press Ctrl+C to stop.")

with open(out_path, "w", buffering=1) as f:
    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue
            try:
                line = raw.decode("utf-8", errors="ignore").strip()
            except Exception:
                continue
            if "CSI_DATA" in line:
                f.write(line + "," + str(time.time()) + "\n")
                count += 1
                if count % 20 == 0:
                    print(f"captured {count} rows...")
    except KeyboardInterrupt:
        print(f"\nStopped. Total rows captured: {count}")
    finally:
        ser.close()
