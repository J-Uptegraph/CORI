#!/usr/bin/env python3
"""
Arduino Detection Script
Helps find and test Arduino connection
"""
import serial
import serial.tools.list_ports
import time
import sys

def find_arduino_ports():
    """Find potential Arduino ports"""
    ports = list(serial.tools.list_ports.comports())
    arduino_ports = []
    
    print("🔍 Scanning for Arduino devices...")
    print(f"Found {len(ports)} serial ports:")
    
    for port in ports:
        print(f"  📍 {port.device}")
        print(f"     Description: {port.description}")
        print(f"     Hardware ID: {port.hwid}")
        print(f"     Manufacturer: {port.manufacturer}")
        
        # Check for Arduino-like devices
        if any(keyword in port.description.lower() for keyword in ['arduino', 'ch340', 'ch341', 'usb']):
            arduino_ports.append(port.device)
            print(f"     ✅ POTENTIAL ARDUINO DETECTED!")
        print()
    
    return arduino_ports

def test_arduino_connection(port, baudrate=115200):
    """Test connection to Arduino"""
    print(f"🔗 Testing connection to {port} at {baudrate} baud...")
    
    try:
        # Open serial connection
        ser = serial.Serial(port, baudrate, timeout=2)
        time.sleep(2)  # Wait for Arduino to initialize
        
        # Send test command
        ser.write(b'STATUS\n')
        time.sleep(0.5)
        
        # Read response
        response = ser.read_all().decode('utf-8', errors='ignore')
        
        if response:
            print(f"✅ Arduino responded from {port}:")
            print(f"   Response: {response}")
            
            # Check if it's our CORI Arduino
            if "CORI_HARDWARE_READY" in response:
                print(f"🎉 CORI Arduino found at {port}!")
                return True
            else:
                print(f"🤔 Device at {port} responded, but not CORI Arduino")
        else:
            print(f"❌ No response from {port}")
            
        ser.close()
        
    except Exception as e:
        print(f"❌ Error connecting to {port}: {e}")
    
    return False

def main():
    print("🤖 CORI Arduino Detection Tool")
    print("=" * 50)
    
    # Find potential Arduino ports
    arduino_ports = find_arduino_ports()
    
    if not arduino_ports:
        print("❌ No Arduino-like devices found!")
        print("\n🔧 Troubleshooting:")
        print("1. Check USB cable connection")
        print("2. Try unplugging and plugging Arduino back in")
        print("3. Check if Arduino shows up with: lsusb")
        print("4. Install CH340 drivers if needed")
        print("5. Add user to dialout group: sudo usermod -a -G dialout $USER")
        return
    
    print(f"🎯 Found {len(arduino_ports)} potential Arduino port(s)")
    print("Testing each port...")
    print()
    
    # Test each port
    for port in arduino_ports:
        if test_arduino_connection(port):
            print(f"\n🎉 SUCCESS! Arduino found at: {port}")
            print(f"Use this port for ROS bridge: {port}")
            
            # Test basic commands
            print("\n🧪 Testing basic commands...")
            try:
                ser = serial.Serial(port, 115200, timeout=2)
                time.sleep(2)
                
                # Test CENTER command
                print("Testing CENTER command...")
                ser.write(b'CENTER\n')
                time.sleep(1)
                response = ser.read_all().decode('utf-8', errors='ignore')
                print(f"Response: {response}")
                
                # Test NOD command  
                print("Testing NOD command...")
                ser.write(b'NOD\n')
                time.sleep(3)  # Nodding takes time
                response = ser.read_all().decode('utf-8', errors='ignore')
                print(f"Response: {response}")
                
                ser.close()
                
            except Exception as e:
                print(f"Error testing commands: {e}")
                
            return
    
    print("\n❌ No CORI Arduino found on any port")
    print("📋 Next steps:")
    print("1. Upload the Arduino code first")
    print("2. Make sure servos are connected to D11 and D12")
    print("3. Check wiring and power supply")

if __name__ == "__main__":
    main()