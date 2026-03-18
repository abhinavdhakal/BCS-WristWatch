#!/usr/bin/env python3
"""
BLE utility for WirstWatch_BCS.

Usage:
  python3 ble_connector.py --scan
  python3 ble_connector.py --name WirstWatch_BCS --connect
  python3 ble_connector.py --name WirstWatch_BCS
"""

import argparse
import asyncio
from bleak import BleakScanner, BleakClient


async def scan_devices(timeout: float):
    print("🔍 Scanning for BLE devices...")
    devices = await BleakScanner.discover(timeout=timeout)
    print(f"\n📱 Found {len(devices)} devices:\n")
    for i, device in enumerate(devices, 1):
        name = device.name or "(Unknown)"
        print(f"  {i:>2}. {name:<30} {device.address}")
    return devices


async def connect_first_match(devices, target_name: str, timeout: float):
    matches = [d for d in devices if d.name and target_name.lower() in d.name.lower()]
    if not matches:
        print(f"\n⚠️  No devices matched '{target_name}'.")
        return

    device = matches[0]
    print(f"\n🔗 Connecting to {device.name} ({device.address})...")
    try:
        async with BleakClient(device.address, timeout=timeout) as client:
            print("✅ Connected successfully")
            print("\n📋 Services:")
            for service in client.services:
                print(f"  {service.uuid}")
                for char in service.characteristics:
                    props = ",".join(char.properties)
                    print(f"    └─ {char.uuid} [{props}]")
            print("\n✅ Connection test complete")
    except Exception as exc:
        print(f"\n❌ Connection failed: {exc}")


async def main():
    parser = argparse.ArgumentParser(description="BLE scanner/connector for WirstWatch_BCS")
    parser.add_argument("--scan", action="store_true", help="Only scan and list devices")
    parser.add_argument("--connect", action="store_true", help="Try connecting to first matching name")
    parser.add_argument("--name", default="WirstWatch_BCS", help="Device name fragment to match")
    parser.add_argument("--timeout", type=float, default=8.0, help="Scan/connect timeout seconds")
    args = parser.parse_args()

    devices = await scan_devices(timeout=args.timeout)
    if args.scan and not args.connect:
        return

    await connect_first_match(devices, target_name=args.name, timeout=args.timeout)


if __name__ == "__main__":
    asyncio.run(main())
