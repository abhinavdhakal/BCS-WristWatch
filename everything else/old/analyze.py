import serial
import matplotlib.pyplot as plt
import collections
import numpy as np

# --- Serial setup ---
port = "/dev/tty.usbmodem101"
baud = 115200
ser = serial.Serial(port, baud)

# --- Plot setup ---
plt.ion()
fig, (ax_ir, ax_red) = plt.subplots(2, 1, figsize=(10,6))

window_size = 500
ir_data = collections.deque([0]*window_size, maxlen=window_size)
red_data = collections.deque([0]*window_size, maxlen=window_size)

line_ir, = ax_ir.plot(ir_data, label="IR", color='r')
line_red, = ax_red.plot(red_data, label="Red", color='b')

ax_ir.set_title("IR Signal (MAX30102)")
ax_ir.set_xlabel("Samples")
ax_ir.set_ylabel("AC Value")
ax_ir.legend()

ax_red.set_title("Red Signal (MAX30102)")
ax_red.set_xlabel("Samples")
ax_red.set_ylabel("AC Value")
ax_red.legend()

def moving_average(data, n=100):
    if len(data) < n:
        return np.mean(data)
    return np.mean(list(data)[-n:])

# --- Live update loop ---
try:
    while True:
        line_bytes = ser.readline()
        line_str = line_bytes.decode("utf-8").strip()
        
        # Expect: timestamp,IR,Red
        parts = line_str.split(',')
        if len(parts) == 3:
            try:
                ir_val = int(parts[1])
                red_val = int(parts[2])
                
                # Append raw values
                ir_data.append(ir_val)
                red_data.append(red_val)
                
                # Remove DC baseline for AC component
                ir_ac = ir_val - moving_average(ir_data, n=100)
                red_ac = red_val - moving_average(red_data, n=100)
                ir_data[-1] = ir_ac
                red_data[-1] = red_ac
                
                # Update IR subplot
                line_ir.set_ydata(ir_data)
                line_ir.set_xdata(range(len(ir_data)))
                margin_ir = max(10, (max(ir_data)-min(ir_data))*1.2)
                mid_ir = np.mean(ir_data)
                ax_ir.set_ylim(mid_ir - margin_ir/2, mid_ir + margin_ir/2)
                ax_ir.set_xlim(0, len(ir_data))
                
                # Update Red subplot
                line_red.set_ydata(red_data)
                line_red.set_xdata(range(len(red_data)))
                margin_red = max(10, (max(red_data)-min(red_data))*1.2)
                mid_red = np.mean(red_data)
                ax_red.set_ylim(mid_red - margin_red/2, mid_red + margin_red/2)
                ax_red.set_xlim(0, len(red_data))
                
                plt.pause(0.001)
            except ValueError:
                continue
except KeyboardInterrupt:
    print("Exiting...")
    ser.close()

