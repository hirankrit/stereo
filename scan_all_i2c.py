#!/usr/bin/env python3
"""
Scan ALL addresses on ALL I2C buses to find any responding device
"""

import smbus2
import time

def scan_bus_full(bus_num):
    """Scan all addresses on a single I2C bus"""
    try:
        bus = smbus2.SMBus(bus_num)
        print(f"\n=== I2C Bus {bus_num} ===")

        found = []
        for addr in range(0x03, 0x78):  # Valid I2C address range
            try:
                # Try quick write
                bus.write_quick(addr)
                found.append(addr)
                print(f"  0x{addr:02x}: Device found", end="")

                # Try to read WHO_AM_I style register (common at 0x00 or 0x75)
                try:
                    val_00 = bus.read_byte_data(addr, 0x00)
                    print(f" [reg 0x00 = 0x{val_00:02x}]", end="")
                except:
                    pass

                try:
                    val_75 = bus.read_byte_data(addr, 0x75)
                    print(f" [reg 0x75 = 0x{val_75:02x}]", end="")
                except:
                    pass

                print()
            except:
                pass

        bus.close()
        return found
    except Exception as e:
        print(f"  Cannot access bus {bus_num}: {e}")
        return []

def main():
    """Scan all I2C buses"""
    print("Scanning ALL I2C buses for ANY devices...")
    print("(This may take a minute...)\n")

    buses_to_scan = [0, 1, 2, 4, 5, 7, 9, 10, 11]

    all_devices = {}
    for bus_num in buses_to_scan:
        devices = scan_bus_full(bus_num)
        if devices:
            all_devices[bus_num] = devices

    print("\n" + "="*60)
    print("SUMMARY:")
    if all_devices:
        for bus, addrs in all_devices.items():
            print(f"  Bus {bus}: {len(addrs)} device(s) at",
                  ", ".join([f"0x{a:02x}" for a in addrs]))
    else:
        print("  No I2C devices found!")
    print("="*60)

if __name__ == "__main__":
    main()
