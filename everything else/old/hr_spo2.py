import numpy as np
from scipy.signal import find_peaks, butter, filtfilt
from collections import deque

# Rolling history for smoothing
hr_history = deque(maxlen=5)
spo2_history = deque(maxlen=5)

def bandpass_filter(signal, fs=50, low=0.7, high=3.5):
    """Bandpass filter for HR (~42-210 bpm)"""
    nyq = 0.5 * fs
    low /= nyq
    high /= nyq
    b, a = butter(2, [low, high], btype='band')
    return filtfilt(b, a, signal)

def smooth(hr, spo2):
    """Keep rolling average to reduce jumps"""
    hr_history.append(hr)
    spo2_history.append(spo2)
    return np.mean(hr_history), np.mean(spo2_history)

def compute_hr_spo2(ir_buffer, red_buffer, fs=50):
    if len(ir_buffer) < 100:
        return 0.0, 0.0

    ir = np.array(ir_buffer)
    red = np.array(red_buffer)

    # Filter signals
    ir_f = bandpass_filter(ir, fs)
    red_f = bandpass_filter(red, fs)

    # Slight smoothing
    ir_f = np.convolve(ir_f, np.ones(3)/3, mode='same')
    red_f = np.convolve(red_f, np.ones(3)/3, mode='same')

    # Detect peaks above threshold
    threshold = 0.5 * np.std(ir_f)
    peaks, _ = find_peaks(ir_f, distance=int(0.5*fs), height=threshold)
    hr = (len(peaks) / (len(ir)/fs)) * 60  # bpm

    # SpO2: AC/DC ratio
    ac_ir = np.std(ir_f)
    dc_ir = np.mean(ir)
    ac_red = np.std(red_f)
    dc_red = np.mean(red)
    if dc_ir == 0 or dc_red == 0:
        spo2 = 0.0
    else:
        R = (ac_red/dc_red) / (ac_ir/dc_ir)
        spo2 = 110 - 25 * R
        spo2 = np.clip(spo2, 0, 100)

    # Smooth output
    hr, spo2 = smooth(hr, spo2)
    return hr, spo2
