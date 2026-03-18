#!/usr/bin/env python3
"""
Quick diagnostic tool to check Arduino BLE connection
"""
import asyncio
from bleak import BleakScanner, BleakClient

SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"

async def check_device(device_name="Arduino"):
    print(f"🔍 Scanning for '{device_name}'...")
    devices = await BleakScanner.discover(timeout=10.0)
    
    print(f"\n📱 Found {len(devices)} BLE devices\n")
    
    target_device = None
    for device in devices:
        name = device.name or "(Unknown)"
        if name == device_name:
            print(f"✅ FOUND: {name} - {device.address}")
            target_device = device
        elif device_name.lower() in name.lower():
            print(f"⚠️  Similar: {name} - {device.address}")
    
    if not target_device:
        print(f"\n❌ Device '{device_name}' not found")
        print("\n💡 Common device names to try:")
        print("   • Arduino")
        print("   • BCS-Watch")
        print("   • XIAO")
        return
    
    print(f"\n🔗 Attempting connection to {target_device.address}...")
    
    try:
        async with BleakClient(target_device.address, timeout=15.0) as client:
            print("✅ Connected!")
            
            # Check services
            print(f"\n📋 Services ({len(client.services)} total):")
            found_service = False
            for service in client.services:
                if service.uuid.lower() == SERVICE_UUID.lower():
                    found_service = True
                    print(f"  ✅ {service.uuid} (TARGET SERVICE)")
                    print(f"     Characteristics:")
                    for char in service.characteristics:
                        props = ', '.join(char.properties)
                        print(f"       └─ {char.uuid} [{props}]")
                else:
                    print(f"     {service.uuid}")
            
            if found_service:
                print(f"\n✅ SUCCESS! Device is ready to use.")
                print(f"\nRun: python receiver.py")
            else:
                print(f"\n❌ Target service {SERVICE_UUID} not found")
                print(f"   Device may not be running correct firmware")
    
    except Exception as e:
        print(f"\n❌ Connection failed: {e}")
        print(f"\n💡 Try:")
        print(f"   1. Press the RESET button on your Arduino")
        print(f"   2. Wait 3 seconds")
        print(f"   3. Run this script again")

if __name__ == "__main__":
    import sys
    device_name = sys.argv[1] if len(sys.argv) > 1 else "Arduino"
    asyncio.run(check_device(device_name))
