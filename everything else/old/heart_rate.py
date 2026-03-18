import serial
import re
import time
import numpy as np
from scipy.signal import find_peaks, butter, filtfilt

ser = serial.Serial('/dev/cu.usbmodem1101', 9600, timeout=1)
time.sleep(2)

def c_to_f(c):
    return c * 9 / 5 + 32

def butter_bandpass(lowcut, highcut, fs, order=3):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return b, a

def bandpass_filter(data, lowcut, highcut, fs, order=3):
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    y = filtfilt(b, a, data)
    return y

BUFFER_SIZE = 600  # ~3 seconds at 200 Hz
SAMPLING_FREQ = 200
MIN_PEAK_DISTANCE = int(SAMPLING_FREQ * 0.25)

ir_buffer = []
red_buffer = []
temp_amb_c = 0.0
temp_obj_c = 0.0

def calc_heart_rate(peak_indices, fs):
    if len(peak_indices) < 3:
        return None
    intervals = np.diff(peak_indices) / fs  # intervals in seconds
    bpm = 60 / np.mean(intervals)
    if 40 < bpm < 180:
        return bpm
    return None

def calc_spo2(ir, red):
    ir_ac = np.ptp(ir)
    ir_dc = np.mean(ir)
    red_ac = np.ptp(red)
    red_dc = np.mean(red)
    if ir_dc == 0 or red_dc == 0:
        return None
    r = (red_ac / red_dc) / (ir_ac / ir_dc)
    spo2 = 110 - 25 * r
    if 70 <= spo2 <= 100:
        return spo2
    return None

print("Starting processing...")

try:
    while True:
        line = ser.readline().decode(errors='ignore').strip()
        if not line:
            continue

        match = re.match(
            r"R\[(\d+)\]\s+IR\[(\d+)\]\s+Temp Ambient: ([\d\.]+) C\s+Temp Object: ([\d\.]+) C", line)
        if not match:
            continue

        red = int(match.group(1))
        ir = int(match.group(2))
        temp_amb_c = float(match.group(3))
        temp_obj_c = float(match.group(4))

        print(f"Raw Data - Red: {red}, IR: {ir}, Ambient Temp (C): {temp_amb_c}, Object Temp (C): {temp_obj_c}")

        ir_buffer.append(ir)
        red_buffer.append(red)

        if len(ir_buffer) > BUFFER_SIZE:
            ir_buffer.pop(0)
            red_buffer.pop(0)

        if len(ir_buffer) == BUFFER_SIZE:
            ir_array = np.array(ir_buffer)
            red_array = np.array(red_buffer)

            filtered_ir = bandpass_filter(ir_array, 0.3, 5, SAMPLING_FREQ, order=3)

            peaks, _ = find_peaks(filtered_ir, distance=MIN_PEAK_DISTANCE)

            # print(f"Detected peaks: {len(peaks)} at indices {peaks}")

            hr = calc_heart_rate(peaks, SAMPLING_FREQ)
            spo2 = calc_spo2(ir_array, red_array)

            temp_amb_f = c_to_f(temp_amb_c)
            temp_obj_f = c_to_f(temp_obj_c)

            print(f"Heart Rate: {hr:.1f} BPM" if hr else "Heart Rate: -- BPM",
                  f"SpO2: {spo2:.1f} %" if spo2 else "SpO2: -- %",
                  f"Ambient Temp: {temp_amb_f:.1f} F",
                  f"Object Temp: {temp_obj_f:.1f} F")

except KeyboardInterrupt:
    print("Stopped by user")
finally:
    ser.close()
