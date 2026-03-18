import numpy as np
from scipy.signal import find_peaks, butter, filtfilt
from collections import deque

# Rolling history for smoothing
hr_history = deque(maxlen=8)
spo2_history = deque(maxlen=8)

last_valid_hr = 0.0
last_valid_spo2 = 0.0
invalid_streak = 0

# Signal quality thresholds (wrist-friendly)
MIN_IR_SIGNAL = 8000
MIN_RED_SIGNAL = 6000
MIN_AC_DC_RATIO = 0.0002
MAX_AC_DC_RATIO = 0.35
MIN_PEAKS = 2
MAX_PEAKS = 15
HR_MIN_VALID = 45
HR_MAX_VALID = 180
MIN_AUTOCORR_QUALITY = 0.08

def bandpass_filter(signal, fs=50, low=0.7, high=3.2):
    """Bandpass filter for HR (~42-192 bpm)"""
    nyq = 0.5 * fs
    low /= nyq
    high /= nyq
    b, a = butter(2, [low, high], btype='band')
    return filtfilt(b, a, signal)

def hr_from_peaks(peaks, fs):
    if len(peaks) < 2:
        return 0.0
    ibis = np.diff(peaks) / float(fs)
    median_ibi = np.median(ibis)
    if median_ibi <= 0:
        return 0.0
    return 60.0 / median_ibi


def estimate_hr_autocorr(signal, fs):
    centered = signal - np.mean(signal)
    denom = np.std(centered)
    if denom < 1e-6:
        return 0.0, 0.0

    centered = centered / denom
    ac = np.correlate(centered, centered, mode='full')
    ac = ac[len(ac)//2:]
    ac0 = ac[0] + 1e-6

    lag_min = int(fs * 60.0 / HR_MAX_VALID)
    lag_max = int(fs * 60.0 / HR_MIN_VALID)
    if lag_max >= len(ac) or lag_min < 1 or lag_min >= lag_max:
        return 0.0, 0.0

    segment = ac[lag_min:lag_max + 1]
    best_idx = int(np.argmax(segment))
    best_lag = lag_min + best_idx
    quality = float(segment[best_idx] / ac0)
    if best_lag <= 0:
        return 0.0, quality

    hr = 60.0 * fs / best_lag
    return hr, quality


def check_signal_quality(ir, red, ir_f, red_f, peaks_ir, peaks_red, fs):
    """
    Check if the signal quality is good enough for valid HR/SpO2.
    Returns True if signal is valid, False otherwise.
    """
    # Check 1: IR DC level must be high enough (sensor on skin)
    dc_ir = np.mean(ir)
    if dc_ir < MIN_IR_SIGNAL or np.mean(red) < MIN_RED_SIGNAL:
        return False
    
    # Check 2: AC/DC ratio must be reasonable
    ac_ir = np.std(ir_f)
    ac_dc_ratio = ac_ir / dc_ir if dc_ir > 0 else 0
    if ac_dc_ratio > MAX_AC_DC_RATIO:
        return False

    # Check 3: periodicity (autocorrelation) on both channels
    hr_ir, q_ir = estimate_hr_autocorr(ir_f, fs)
    hr_red, q_red = estimate_hr_autocorr(red_f, fs)
    if q_ir < MIN_AUTOCORR_QUALITY or q_red < MIN_AUTOCORR_QUALITY:
        return False

    # Check 4: IR and RED HR should agree (motion often breaks this)
    if hr_ir <= 0 or hr_red <= 0:
        return False
    if abs(hr_ir - hr_red) > 15:
        return False

    # Keep heart rate in plausible wearable range
    hr_avg = 0.5 * (hr_ir + hr_red)
    if hr_avg < HR_MIN_VALID or hr_avg > HR_MAX_VALID:
        return False

    # Check 5: Motion artifact check using derivative energy
    deriv_std = np.std(np.diff(ir_f)) if len(ir_f) > 1 else 0.0
    signal_std = np.std(ir_f) + 1e-6
    if (deriv_std / signal_std) > 2.2:
        return False
    
    return True

def smooth(hr, spo2):
    """Keep rolling average to reduce jumps"""
    hr_history.append(hr)
    spo2_history.append(spo2)
    return np.mean(hr_history), np.mean(spo2_history)

def compute_hr_spo2(ir_buffer, red_buffer, fs=50):
    global last_valid_hr, last_valid_spo2, invalid_streak

    if len(ir_buffer) < 180:
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
    threshold_ir = 0.20 * np.std(ir_f)
    threshold_red = 0.20 * np.std(red_f)
    peaks_ir, _ = find_peaks(
        ir_f,
        distance=int(0.4 * fs),
        height=threshold_ir,
        prominence=0.15 * np.std(ir_f),
    )
    peaks_red, _ = find_peaks(
        red_f,
        distance=int(0.4 * fs),
        height=threshold_red,
        prominence=0.15 * np.std(red_f),
    )
    
    # Check signal quality BEFORE calculating HR/SpO2
    if not check_signal_quality(ir, red, ir_f, red_f, peaks_ir, peaks_red, fs):
        invalid_streak += 1
        if invalid_streak <= 3 and last_valid_hr > 0 and last_valid_spo2 > 0:
            return last_valid_hr, last_valid_spo2
        hr_history.clear()
        spo2_history.clear()
        last_valid_hr = 0.0
        last_valid_spo2 = 0.0
        return 0.0, 0.0
    invalid_streak = 0
    
    # Calculate HR from autocorrelation periodicity (more robust for steady low-amplitude wrist signal)
    hr_ir, _ = estimate_hr_autocorr(ir_f, fs)
    hr_red, _ = estimate_hr_autocorr(red_f, fs)
    hr = 0.5 * (hr_ir + hr_red) if hr_ir > 0 and hr_red > 0 else 0.0

    # SpO2: AC/DC ratio
    ac_ir = np.std(ir_f)
    dc_ir = np.mean(ir)
    ac_red = np.std(red_f)
    dc_red = np.mean(red)
    if dc_ir == 0 or dc_red == 0:
        spo2 = 0.0
    else:
        if ac_ir < 1e-6 or ac_red < 1e-6:
            spo2 = last_valid_spo2 if last_valid_spo2 > 0 else 0.0
        else:
            R = (ac_red / dc_red) / (ac_ir / dc_ir)
            spo2 = 104 - 17 * R
            spo2 = np.clip(spo2, 80, 100)

    # Smooth output
    hr, spo2 = smooth(hr, spo2)
    if HR_MIN_VALID <= hr <= HR_MAX_VALID and 80 <= spo2 <= 100:
        last_valid_hr = float(hr)
        last_valid_spo2 = float(spo2)
    return hr, spo2
