import asyncio
from bleak import BleakClient, BleakScanner
import struct
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from hr_spo2 import compute_hr_spo2
from datetime import datetime

# -------------------------------
# BLE UUIDs
# -------------------------------
SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
IR_CHAR_UUID = "12345678-1234-1234-1234-123456789ab1"
RED_CHAR_UUID = "12345678-1234-1234-1234-123456789ab2"
TEMP_CHAR_UUID = "12345678-1234-1234-1234-123456789ab3"
IMU_CHAR_UUID = "12345678-1234-1234-1234-123456789ab4"
BATT_CHAR_UUID = "12345678-1234-1234-1234-123456789ab5"

# -------------------------------
# Sensor values and buffers
# -------------------------------
sensor_values = {
    "IR": 0, "Red": 0, "Temp_C": 0.0,
    "Accel_X": 0.0, "Accel_Y": 0.0, "Accel_Z": 0.0,
    "Gyro_X": 0.0, "Gyro_Y": 0.0, "Gyro_Z": 0.0,
    "Battery_V": 0.0, "Battery_%": 0.0,
    "HR": 0.0, "SpO2": 0.0,
    "HR_Avg": 0.0, "SpO2_Avg": 0.0
}

BUFFER_SIZE = 120  # ~2.5s @50Hz
ir_buffer = deque(maxlen=BUFFER_SIZE)
red_buffer = deque(maxlen=BUFFER_SIZE)

# HR and SpO2 1-minute averaging buffers (~120 readings at 0.5s intervals)
HR_AVG_SIZE = 120  # 1 minute of data
SPO2_AVG_SIZE = 120  # 1 minute of data
hr_history = deque(maxlen=HR_AVG_SIZE)
spo2_history = deque(maxlen=SPO2_AVG_SIZE)

# -------------------------------
# Averaging with outlier detection
# -------------------------------
def filter_and_average(values, min_val, max_val, std_threshold=2.0):
    """
    Average values after removing outliers.
    - Remove values outside [min_val, max_val]
    - Remove values beyond std_threshold standard deviations from median
    """
    if len(values) == 0:
        return 0.0
    
    arr = np.array(values)
    
    # Filter by valid range
    valid = arr[(arr >= min_val) & (arr <= max_val)]
    
    if len(valid) == 0:
        return 0.0
    
    # Filter by statistical outliers if we have enough data
    if len(valid) >= 3:
        median = np.median(valid)
        std = np.std(valid)
        if std > 0:
            # Keep values within std_threshold standard deviations
            valid = valid[np.abs(valid - median) <= std_threshold * std]
    
    if len(valid) == 0:
        return 0.0
    
    return np.mean(valid)

# -------------------------------
# Setup live plot
# -------------------------------
plt.ion()
fig, ax = plt.subplots(2, 1, figsize=(10, 6))
line_ir, = ax[0].plot([], [], 'r')
line_red, = ax[1].plot([], [], 'b')
ax[0].set_title("IR Signal")
ax[1].set_title("Red Signal")

def update_plot():
    ir_data = np.array(ir_buffer)
    red_data = np.array(red_buffer)
    x = np.arange(len(ir_data))
    
    if len(ir_data) == 0:
        return
    
    line_ir.set_data(x, ir_data)
    line_red.set_data(x, red_data)
    
    ax[0].set_ylim(ir_data.min() - 100, ir_data.max() + 100)
    ax[1].set_ylim(red_data.min() - 100, red_data.max() + 100)
    
    ax[0].set_xlim(0, BUFFER_SIZE)
    ax[1].set_xlim(0, BUFFER_SIZE)
    
    plt.pause(0.001)


# -------------------------------
# Terminal display
# -------------------------------
def print_sensor_values():
    print("\033[H\033[J", end="")  # Clear terminal
    now = datetime.now()
    print(f"{'XIAO Sense Data':^60}")
    print(f"{now.strftime('%Y-%m-%d %H:%M:%S'):^60}")
    print("-"*60)
    print(f"IR Light: {sensor_values['IR']:,} counts")
    print(f"Red Light: {sensor_values['Red']:,} counts")
    print(f"Temperature: {sensor_values['Temp_C']:.2f} °C")
    print(f"Accelerometer: X={sensor_values['Accel_X']:+.3f}g "
          f"Y={sensor_values['Accel_Y']:+.3f}g "
          f"Z={sensor_values['Accel_Z']:+.3f}g")
    print(f"Gyroscope:     X={sensor_values['Gyro_X']:+.2f}°/s "
          f"Y={sensor_values['Gyro_Y']:+.2f}°/s "
          f"Z={sensor_values['Gyro_Z']:+.2f}°/s")
    print(f"Heart Rate (Live): {sensor_values['HR']:.1f} bpm")
    print(f"Heart Rate (1min Avg): {sensor_values['HR_Avg']:.1f} bpm")
    print(f"SpO2 (Live): {sensor_values['SpO2']:.1f}%")
    print(f"SpO2 (1min Avg): {sensor_values['SpO2_Avg']:.1f}%")
    print("-"*60)

# -------------------------------
# BLE Decoder
# -------------------------------
class XiaoSenseDecoder:
    def __init__(self):
        self.client = None
        self.device_address = None
        self.print_counter = 0
        self.hr_counter = 0

    def decode_ir(self, data): return struct.unpack('<L', data)[0]
    def decode_red(self, data): return struct.unpack('<L', data)[0]
    def decode_temperature(self, data): return struct.unpack('<L', data)[0]/100.0
    def decode_imu(self, data):
        ax, ay, az, gx, gy, gz = struct.unpack('<ffffff', data)
        return {'accel_x': ax, 'accel_y': ay, 'accel_z': az,
                'gyro_x': gx, 'gyro_y': gy, 'gyro_z': gz}
    def decode_battery(self, data):
        batt_raw = struct.unpack('<L', data)[0]
        voltage = batt_raw / 1000.0
        percentage = max(0, min(100, ((voltage-3.0)/(4.2-3.0))*100))
        return {'voltage': voltage, 'percentage': percentage}

    async def notification_handler(self, sender, data):
        try:
            # --- Decode data ---
            if sender.uuid == IR_CHAR_UUID:
                val = self.decode_ir(data)
                sensor_values["IR"] = val
                ir_buffer.append(val)
            elif sender.uuid == RED_CHAR_UUID:
                val = self.decode_red(data)
                sensor_values["Red"] = val
                red_buffer.append(val)
            elif sender.uuid == TEMP_CHAR_UUID:
                sensor_values["Temp_C"] = self.decode_temperature(data)
            elif sender.uuid == IMU_CHAR_UUID:
                imu = self.decode_imu(data)
                sensor_values["Accel_X"] = imu['accel_x']
                sensor_values["Accel_Y"] = imu['accel_y']
                sensor_values["Accel_Z"] = imu['accel_z']
                sensor_values["Gyro_X"] = imu['gyro_x']
                sensor_values["Gyro_Y"] = imu['gyro_y']
                sensor_values["Gyro_Z"] = imu['gyro_z']

            elif sender.uuid == BATT_CHAR_UUID:
                batt = self.decode_battery(data)
                sensor_values["Battery_V"] = batt['voltage']
                sensor_values["Battery_%"] = batt['percentage']
            
            # --- HR & SpO2 update every ~0.5s ---
            self.hr_counter += 1
            if self.hr_counter % 25 == 0:
                hr, spo2 = compute_hr_spo2(ir_buffer, red_buffer)
                
                # Update live values
                sensor_values["HR"] = hr
                sensor_values["SpO2"] = spo2
                
                # Add to history buffers (only if values are reasonable)
                if 40 <= hr <= 200:  # Valid HR range
                    hr_history.append(hr)
                if 70 <= spo2 <= 100:  # Valid SpO2 range
                    spo2_history.append(spo2)
                
                # Compute filtered 1-minute averages
                hr_avg = filter_and_average(hr_history, min_val=40, max_val=200, std_threshold=1.5)
                spo2_avg = filter_and_average(spo2_history, min_val=70, max_val=100, std_threshold=1.5)
                
                # Update average values
                sensor_values["HR_Avg"] = hr_avg if hr_avg > 0 else 0.0
                sensor_values["SpO2_Avg"] = spo2_avg if spo2_avg > 0 else 0.0
            
            # --- Print every ~0.5s ---
            self.print_counter += 1
            if self.print_counter % 25 == 0:
                print_sensor_values()

        except Exception as e:
            print(f"Error decoding data: {e}")

    async def scan_for_device(self, device_name=None, timeout=10):
        if not device_name:
            device_name = input("Enter the name of your XIAO Sense device: ").strip()
        print(f"Scanning for '{device_name}'...")
        devices = await BleakScanner.discover(timeout=timeout)
        
        # Show all devices for debugging
        print(f"\nFound {len(devices)} BLE devices:")
        matches = []
        for device in devices:
            name = device.name or "(Unknown)"
            if device.name == device_name:
                print(f"  ✓ {name} - {device.address}")
                matches.append(device)
            
        if matches:
            device = matches[0]
            print(f"\n✓ Found {device_name} at {device.address}")
            self.device_address = device.address
            return device.address
        
        print(f"\n⚠️  Device '{device_name}' not found")
        print("\nTip: Make sure the Arduino is:")
        print("  1. Powered on")
        print("  2. Not connected to another device")
        print("  3. Reset recently (press reset button)")
        return None

    async def connect_and_monitor(self, address=None):
        if address is None:
            address = await self.scan_for_device()
            if address is None: return False

        print(f"\nConnecting to {address}...")
        try:
            self.client = BleakClient(address, timeout=15.0)
            
            print("Attempting connection (timeout: 15s)...")
            await self.client.connect(timeout=15.0)
            
            if not self.client.is_connected:
                print("❌ Failed to connect")
                return False
            print("✅ Connected successfully!")
            
            # Give BLE stack time to settle
            await asyncio.sleep(1)
            
            # --- Discover services ---
            print("\nDiscovering services...")
            services = self.client.services
            service_list = list(services)
            print(f"Found {len(service_list)} services")
            
            target_service = None
            for service in service_list:
                if service.uuid.lower() == SERVICE_UUID.lower():
                    target_service = service
                    print(f"✓ Found target service: {SERVICE_UUID}")
                    for char in service.characteristics:
                        props = ','.join(char.properties)
                        print(f"  └─ {char.uuid} [{props}]")
                    break
            
            if not target_service:
                print(f"\n⚠️  ERROR: Service {SERVICE_UUID} not found!")
                print("The device may not be running the correct firmware.")
                return False

            # --- Subscribe to notifications ---
            print("\nSubscribing to notifications...")
            await self.client.start_notify(IR_CHAR_UUID, self.notification_handler)
            print("  ✓ IR")
            await self.client.start_notify(RED_CHAR_UUID, self.notification_handler)
            print("  ✓ RED")
            await self.client.start_notify(TEMP_CHAR_UUID, self.notification_handler)
            print("  ✓ TEMP")
            await self.client.start_notify(IMU_CHAR_UUID, self.notification_handler)
            print("  ✓ IMU")
            await self.client.start_notify(BATT_CHAR_UUID, self.notification_handler)
            print("  ✓ BATT")

            # --- Start plot updater task ---
            asyncio.create_task(self.plot_task())

            print("Subscribed to all notifications. Press Ctrl+C to stop.\n")
            while True:
                await asyncio.sleep(1)
                if not self.client.is_connected:
                    print("Connection lost")
                    break

        except KeyboardInterrupt:
            print("\nStopping...")
        except asyncio.TimeoutError:
            print(f"\n❌ Connection timeout")
            print(f"\n💡 The device may already be connected to something else.")
            print(f"   Solutions:")
            print(f"   1. Press the RESET button on your Arduino")
            print(f"   2. Wait 3 seconds")
            print(f"   3. Run this script again")
        except Exception as e:
            import traceback
            print(f"\n❌ Error: {e}")
            print(f"Error type: {type(e).__name__}")
            print("\nFull traceback:")
            traceback.print_exc()
            print("\n💡 Troubleshooting tips:")
            print("  • Reset your Arduino board (press reset button)")
            print("  • Make sure it's not connected to another device")
            print("  • Try turning Bluetooth off/on in System Preferences")
        finally:
            if self.client and self.client.is_connected:
                await self.client.disconnect()
                print("Disconnected")

    async def plot_task(self):
        while True:
            update_plot()
            await asyncio.sleep(0.05)  # 20Hz

# -------------------------------
# Main
# -------------------------------
async def main():
    decoder = XiaoSenseDecoder()
    print("XIAO Sense BLE Data Reader with HR & SpO2")
    print("=" * 60)
    await decoder.connect_and_monitor()

if __name__ == "__main__":
    asyncio.run(main())
