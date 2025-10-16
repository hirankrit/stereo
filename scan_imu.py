#!/usr/bin/env python3
"""
Scan I2C buses to find ICM20948 IMU
ICM20948 default addresses: 0x68 or 0x69
"""

import smbus2
import time

# ICM20948 WHO_AM_I register and expected value
WHO_AM_I_REG = 0x00
WHO_AM_I_ICM20948 = 0xEA

def scan_bus(bus_num):
    """Scan a single I2C bus for ICM20948"""
    try:
        bus = smbus2.SMBus(bus_num)
        print(f"\n=== Scanning I2C bus {bus_num} ===")

        # Common IMU addresses
        addresses = [0x68, 0x69]

        for addr in addresses:
            try:
                # Try to read WHO_AM_I register
                who_am_i = bus.read_byte_data(addr, WHO_AM_I_REG)
                print(f"  Address 0x{addr:02x}: WHO_AM_I = 0x{who_am_i:02x}", end="")

                if who_am_i == WHO_AM_I_ICM20948:
                    print(" ✓ ICM20948 FOUND!")
                    return bus_num, addr
                else:
                    print(f" (not ICM20948, expected 0x{WHO_AM_I_ICM20948:02x})")
            except Exception as e:
                # Address not responding
                pass

        bus.close()
    except Exception as e:
        print(f"  Cannot access bus {bus_num}: {e}")

    return None, None

def main():
    """Scan all I2C buses"""
    print("Scanning I2C buses for ICM20948 IMU...")
    print(f"Expected WHO_AM_I value: 0x{WHO_AM_I_ICM20948:02x}")

    # Scan common I2C buses on Jetson
    buses_to_scan = [0, 1, 2, 4, 5, 7, 9, 10, 11]

    found_bus = None
    found_addr = None

    for bus_num in buses_to_scan:
        bus, addr = scan_bus(bus_num)
        if bus is not None:
            found_bus = bus
            found_addr = addr
            break

    print("\n" + "="*50)
    if found_bus is not None:
        print(f"✓ ICM20948 found on I2C bus {found_bus}, address 0x{found_addr:02x}")
        print(f"\nTo use in your code:")
        print(f"  bus = smbus2.SMBus({found_bus})")
        print(f"  IMU_ADDR = 0x{found_addr:02x}")
    else:
        print("✗ ICM20948 not found on any I2C bus")
        print("\nTroubleshooting:")
        print("  1. Check IMU hardware connection")
        print("  2. Check if device tree is loaded (may need reboot)")
        print("  3. Run: sudo i2cdetect -y -r 1")
        print("  4. Check dmesg for I2C errors: sudo dmesg | grep i2c")

if __name__ == "__main__":
    main()
