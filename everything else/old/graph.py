import serial
import time
import matplotlib.pyplot as plt
import re
import numpy as np

# Configure serial
ser = serial.Serial('/dev/tty.usbmodem1101', 115200)  # Change COM port as needed
time.sleep(2)  # Wait for connection

# Init
object_temps = []
red_vals = []
ir_vals = []

start = time.time()

# Capture for 20 seconds
while time.time() - start < 5:
    if ser.in_waiting:
        line = ser.readline().decode(errors='ignore').strip()
        match = re.search(r'Red:(\d+),IR:(\d+),.*ObjectTempF:(\d+\.\d+)', line)
        if match:
            red = int(match.group(1))
            ir = int(match.group(2))
            obj_temp = float(match.group(3))
            red_vals.append(red)
            ir_vals.append(ir)
            object_temps.append(obj_temp)

ser.close()

# Time axis
t = np.linspace(0, len(red_vals) * 0.05, len(red_vals))

# Plot
plt.style.use('default')

# Object Temp
plt.figure(figsize=(10, 4))
plt.plot(t, object_temps, color='darkorange')
plt.title('Object Temperature (F)')
plt.xlabel('Time (s)')
plt.ylabel('Temperature (F)')
plt.grid(True, linestyle='--', alpha=0.5)
plt.gca().set_facecolor('white')
plt.tight_layout()

# Red and IR
plt.figure(figsize=(10, 4))
plt.plot(t, red_vals, label='Red', color='crimson')
plt.plot(t, ir_vals, label='IR', color='navy')
plt.title('Red and IR Values')
plt.xlabel('Time (s)')
plt.ylabel('Signal Value')
plt.legend()
plt.grid(True, linestyle='--', alpha=0.5)
plt.gca().set_facecolor('white')
plt.tight_layout()

plt.show()
