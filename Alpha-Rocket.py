# CircuitPython logger for MPU6050 @0x68 and BMP180 @0x77 on ESP32-S3 ProS3
import time
import os
import math
import board
import busio
import adafruit_mpu6050
import bmp180  # install "bmp180" driver in /lib

# ---------- I2C SETUP ----------
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)

# Optional: show detected devices once at boot
while not i2c.try_lock():
    pass
try:
    print("I2C devices:", [hex(a) for a in i2c.scan()])
finally:
    i2c.unlock()

mpu = adafruit_mpu6050.MPU6050(i2c, address=0x68)   # IMU at 0x68
bmp = bmp180.BMP180(i2c, address=0x77)              # Barometer at 0x77

# Optional: smoother pressure mode if driver supports it
try:
    bmp.mode = bmp180.MODE_HIGHRES
except Exception:
    pass

# ---------- HELPERS ----------
def ensure_dir(path="/logs"):
    try:
        os.mkdir(path)
    except OSError:
        pass

def next_csv(path="/logs", prefix="log_", ext=".csv"):
    n = 1
    while True:
        name = f"{path}/{prefix}{n:03d}{ext}"
        try:
            with open(name, "r"):
                n += 1
        except OSError:
            return name

def accel_angles_deg(ax, ay, az):
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
    return roll, pitch

# ---------- BASELINES ----------
p0 = bmp.pressure  # hPa at boot; used for relative altitude
roll, pitch = accel_angles_deg(*mpu.acceleration)
last_t = time.monotonic()
last_status = last_t

# ---------- FILE ----------
ensure_dir("/logs")
csv_path = next_csv()
with open(csv_path, "w") as f:
    f.write("alt_m,accel_g,roll_deg,pitch_deg\n")

flush_every = 25
line_count = 0

print("Logging to:", csv_path)

# ---------- LOOP ----------
with open(csv_path, "a") as f:
    while True:
        now = time.monotonic()
        dt = now - last_t
        last_t = now

        ax, ay, az = mpu.acceleration   # m/s^2
        gx, gy, gz = mpu.gyro           # deg/s
        p = bmp.pressure                 # hPa

        # Relative altitude (meters)
        alt_m = 44330.0 * (1.0 - pow(p / p0, 1.0 / 5.255))

        # Complementary filter for angles
        roll_acc, pitch_acc = accel_angles_deg(ax, ay, az)
        alpha = 0.98
        roll  = alpha * (roll  + gx * dt) + (1 - alpha) * roll_acc
        pitch = alpha * (pitch + gy * dt) + (1 - alpha) * pitch_acc

        # Acceleration magnitude in g
        accel_g = math.sqrt(ax*ax + ay*ay + az*az) / 9.80665

        # Write CSV row
        f.write(f"{alt_m:.2f},{accel_g:.3f},{roll:.2f},{pitch:.2f}\n")
        line_count += 1
        if line_count % flush_every == 0:
            f.flush()

        # Print a quick status once per second
        if now - last_status >= 1.0:
            print(f"alt={alt_m:.2f} m, a={accel_g:.3f} g, roll={roll:.1f}°, pitch={pitch:.1f}°")
            last_status = now

        time.sleep(0.05)  # ~20 Hz
