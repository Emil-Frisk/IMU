#!/usr/bin/env python3
"""
Basic ISM330
DLC 6-axis IMU sensor interface for Raspberry Pi
Supports accelerometer and gyroscope data reading via I2C
"""

INT32_MIN=-32768
INT32_MAX=32767
import pdb
import smbus
import math
import time
import struct

class ISM330DLC:
    # I2C addresses (depends on SA0 pin connection)
    ADDR_LOW = 0x6A   # SA0 connected to GND
    ADDR_HIGH = 0x6B  # SA0 connected to VDD

    # Register addresses
    WHO_AM_I = 0x0F
    CTRL2_G = 0x11    # Gyroscope control register 2
    STATUS_REG = 0x1E


    ### Accelerometer register addresses
    CTRL1_XL = 0x10   # Accelerometer control register 1
    CTRL6_C = 0x15 # 0 enable high performance XL_HM_MODE
    CTRL8_XL = 0x17 # 0 -> disabale all filters
    X_OFS_USR  = 0x73
    Y_OFS_USR  = 0x74
    Z_OFS_USR  = 0x75

    # Accelerometer constants
    XL_OFFSET_SCALE = 0.000976525 # assumes default setting in (0x15 USR_OFF_W)

    # Accelerometer values
    XL_LPF_LOWEST_BANDWITH = 192

    # Accelerometer output registers
    OUTX_L_XL = 0x28
    OUTX_H_XL = 0x29
    OUTY_L_XL = 0x2A
    OUTY_H_XL = 0x2B
    OUTZ_L_XL = 0x2C
    OUTZ_H_XL = 0x2D
    
    # Gyroscope output registers
    OUTX_L_G = 0x22
    OUTX_H_G = 0x23
    OUTY_L_G = 0x24
    OUTY_H_G = 0x25
    OUTZ_L_G = 0x26
    OUTZ_H_G = 0x27
    
    # Expected WHO_AM_I value
    WHO_AM_I_VAL = 0x6A
    
    def _enable_xl_lpf_filter(self): ### ODR / 400 - lowest bandiwth possible
        self.bus.write_byte_data(self.address, self.CTRL8_XL, self.XL_LPF_LOWEST_BANDWITH)

    def _disable_accelerometer_filters(self):
        self.bus.write_byte_data(self.address, self.CTRL8_XL, 0x0) 

    def __init__(self, bus_num=1, address=None, ODR=12.5, disable_xl_filters=True, g_range=2):
        """
        Initialize ISM330DLC sensor
        
        Args:
            bus_num: I2C bus number (default 1 for Raspberry Pi)
            address: I2C address (if None, will try to detect)
        """
        self.bus = smbus.SMBus(bus_num)

        # accelerometer register values
        self.ODR = ODR
        self._disable_xl_filters = disable_xl_filters
        self.g_range = g_range
        
        if address is None:
            # Try to detect the correct address
            self.address = self._detect_address()
        else:
            self.address = address
            
        # if not self._check_connection():
            # raise Exception("ISM330DLC not found or not responding")
            
        self._initialize_sensor()
    
    def _detect_address(self):
        """Try to detect the correct I2C address"""
        for addr in [self.ADDR_LOW, self.ADDR_HIGH]:
            try:
                who_am_i = self.bus.read_byte_data(addr, self.WHO_AM_I)
                if who_am_i == self.WHO_AM_I_VAL:
                    print(f"ISM330DLC found at address 0x{addr:02X}")
                    return addr
            except:
                continue
        raise Exception("ISM330DLC not found at either address")
    
    def _check_connection(self):
        """Verify sensor connection by reading WHO_AM_I register"""
        try:
            breakpoint()
            who_am_i = self.bus.read_byte_data(self.address, self.WHO_AM_I)
            return who_am_i == self.WHO_AM_I_VAL
        except:
            return False
    
    def _configure_accelerometer(self):
        self.bus.write_byte_data(self.address, self.CTRL6_C, 0x0) ### high performance mode
        if self._disable_xl_filters:
            self._disable_accelerometer_filters()
        else:
            self._enable_xl_lpf_filter()
        
        if self.g_range == 2:
            self.bus.write_byte_data(self.address, self.CTRL1_XL, 0x10) # ODR 12.5hz -> sample rate = 6.25hz
        else:
            raise Exception("Unsupported g_range")


    def _initialize_sensor(self):
        """Initialize sensor with basic configuration"""
        self._configure_accelerometer()
        
        # Wait for sensor to stabilize
        time.sleep(0.1)
        print("ISM330DLC initialized successfully")
    
    def _read_raw_axis(self, reg_l, reg_h):
        """Read raw 16-bit value from sensor registers"""
        low = self.bus.read_byte_data(self.address, reg_l)
        high = self.bus.read_byte_data(self.address, reg_h)
        # Combine bytes and convert to signed 16-bit
        value = (high << 8) | low
        return struct.unpack('<h', struct.pack('<H', value))[0]
    
    def calculate_pitch_roll(self, x, y, z):
        # Calculate pitch (rotation around Y-axis)
        pitch = math.atan2(x, math.sqrt(y*y + z*z))
        # Calculate roll (rotation around Y-axis)  
        roll = math.atan2(y, math.sqrt(x*x + z*z))

        # Convert from radians to degrees
        pitch_degrees = math.degrees(pitch) 
        roll_degrees = math.degrees(roll)

        return pitch_degrees, roll_degrees
    def _scale_xl_values(self, x_raw, y_raw, z_raw):
        x = x_raw * (self.g_range / 32768.0)
        y = y_raw * (self.g_range / 32768.0)
        z = z_raw * (self.g_range / 32768.0)
        return x, y, z

    def read_accelerometer(self):
        """
        Read accelerometer data
        
        Returns:
            tuple: (x, y, z) returns in angles
        """
        x_raw = self._read_raw_axis(self.OUTX_L_XL, self.OUTX_H_XL)
        y_raw = self._read_raw_axis(self.OUTY_L_XL, self.OUTY_H_XL)
        z_raw = self._read_raw_axis(self.OUTZ_L_XL, self.OUTZ_H_XL)
        
        x, y, z  = self._scale_xl_values(x_raw, y_raw, z_raw)
        print(f"{x,y,z}")
        pitch, roll = self.calculate_pitch_roll(x, y, z)
        return (pitch, roll)
    
    def read_gyroscope(self):
        """
        Read gyroscope data
        
        Returns:
            tuple: (x, y, z) angular velocity in degrees/second
        """
        x_raw = self._read_raw_axis(self.OUTX_L_G, self.OUTX_H_G)
        y_raw = self._read_raw_axis(self.OUTY_L_G, self.OUTY_H_G)
        z_raw = self._read_raw_axis(self.OUTZ_L_G, self.OUTZ_H_G)
        
        # Convert to degrees/second (±250 dps range, 16-bit resolution)
        scale = 250.0 / 32768.0
        x = x_raw * scale
        y = y_raw * scale
        z = z_raw * scale
        
        return (x, y, z)
    
    def read_all(self):
        """
        Read both accelerometer and gyroscope data
        
        Returns:
            dict: Dictionary containing 'accel' and 'gyro' data
        """
        return {
            'accel': self.read_accelerometer(),
            'gyro': self.read_gyroscope()
        }

def main():
    """Example usage of the ISM330DLC sensor"""
    try:
        # Initialize sensor (will auto-detect address)
        sensor = ISM330DLC(address=0x6A, disable_xl_filters=True)
        
        print("Starting sensor readings (Ctrl+C to stop)...")
        print("Format: Accel(x,y,z)[g] | Gyro(x,y,z)[deg/s]")
        print("-" * 60)
        
        # while True:
        #     # Read sensor data
        #     data = sensor.read_all()
        #     accel = data['accel']
        #     gyro = data['gyro']
            
        #     # Format and display data
        #     print(f"Accel: {accel[0]:6.3f}, {accel[1]:6.3f}, {accel[2]:6.3f} | "
        #           f"Gyro: {gyro[0]:7.2f}, {gyro[1]:7.2f}, {gyro[2]:7.2f}")
            
        #     time.sleep(0.1)  # 10 Hz update rate
        p, r = sensor.read_accelerometer()
        print(f"pitch: {p:.2f} | roll: {r:.2f}")
            
    except KeyboardInterrupt:
        print("\nStopping sensor readings...")
    except Exception as e:
        print(f"Error: {e}")
        print("Make sure:")
        print("1. I2C is enabled on your Raspberry Pi")
        print("2. Sensor is properly wired (VCC, GND, SDA, SCL)")
        print("3. Pull-up resistors are connected (usually built-in on Pi)")

if __name__ == "__main__":
    main()
