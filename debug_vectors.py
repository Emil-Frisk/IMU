import numpy as np
import board
from time import monotonic
from time import sleep
from adafruit_lsm6ds.ism330dhcx import ISM330DHCX

def quaternion_normalize(q):
    """Normalize a quaternion to unit magnitude """
    # Convert input to float32 to ensure consistent types
    q = np.asarray(q, dtype=np.float32)

    norm = np.sqrt(np.sum(q * q))
    if norm < 1e-10:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

    # Explicitly ensure float32 output
    return np.asarray(q / norm, dtype=np.float32)

def quaternion_exponential(v, scalar=0.0):
    """
    Compute quaternion exponential exp(q) 
    """
    # Ensure float32 input
    v = np.asarray(v, dtype=np.float32)
    scalar = np.float32(scalar)

    v_norm = np.sqrt(np.sum(v * v))

    # Handle small vector norm case
    if v_norm < 1e-10:
        return np.array([np.exp(scalar), 0.0, 0.0, 0.0], dtype=np.float32)

    # Calculate the exponential
    exp_scalar = np.float32(np.exp(scalar))
    factor = exp_scalar * np.sin(v_norm) / v_norm

    return np.array([
        exp_scalar * np.cos(v_norm),
        factor * v[0],
        factor * v[1],
        factor * v[2]
    ], dtype=np.float32)

def integrate_quaternion_exp(q, gyro, dt):
    """Integrate quaternion with exponential """
    # Ensure float32 inputs
    q = np.asarray(q, dtype=np.float32)
    gyro = np.asarray(gyro, dtype=np.float32)
    dt = np.float32(dt)

    half_angle = np.float32(0.5) * dt
    omega_vector = np.array([gyro[0], gyro[1], gyro[2]], dtype=np.float32) * half_angle
    q_exp = quaternion_exponential(omega_vector)
    q_new = quaternion_multiply(q, q_exp)
    return quaternion_normalize(q_new)

def quaternion_multiply(q1, q2):
    """
    Multiply two quaternions (body frame) 
    q1, q2: quaternions in [w, x, y, z] format
    """
    # Ensure float32 inputs
    q1 = np.asarray(q1, dtype=np.float32)
    q2 = np.asarray(q2, dtype=np.float32)

    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    return np.array([w, x, y, z], dtype=np.float32)

def rotate_vector_by_quaternion(v, q):
    """Rotate vector by quaternion """
    # Ensure float32 inputs
    v = np.asarray(v, dtype=np.float32)
    q = np.asarray(q, dtype=np.float32)

    p = np.array([0, v[0], v[1], v[2]], dtype=np.float32)
    q_conj = np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float32)
    qp = quaternion_multiply(q, p)
    rotated_p = quaternion_multiply(qp, q_conj)
    return rotated_p[1:4]

def quaternion_from_axis_angle(axis, angle):
    """
    Create quaternion from rotation axis and angle

    Args:
        axis: normalized 3D vector representing rotation axis
        angle: rotation angle in radians

    Returns:
        quaternion [w, x, y, z]
    """
    half_angle = angle * 0.5
    sin_half = np.sin(half_angle)

    w = np.cos(half_angle)
    x = axis[0] * sin_half
    y = axis[1] * sin_half
    z = axis[2] * sin_half

    return np.array([w, x, y, z], dtype=np.float32)

class ImuDebug:
    def __init__(self):
        self.rotation_axis_mag = 0
        self.corrected_zaxis = False
        self.data_array = np.zeros((6), dtype=np.float32)
        self.quaternion = np.zeros((4), dtype=np.float32)
        self.quaternion[0] = 1.0
        # accelerometers down direction first, quaternions's down vector second 
        self.debug_vectors = np.zeros((3, 3), dtype=np.float32) 
        self.dt = 0
        self.last_time = 0
        i2c = board.I2C()
        self.imu = ISM330DHCX(i2c, address=0x6A)

    def read_imus(self):
        self.data_array[0:3] = self.imu.acceleration
        self.data_array[3:6] = self.imu.gyro

        np.round(self.data_array, decimals=4, out=self.data_array)

        current_time = monotonic()
        dt = current_time - self.last_time
        self.last_time = current_time
        self.update_quaternions_exponential(dt=dt)

        return self.quaternion, self.debug_vectors, self.corrected_zaxis, self.rotation_axis_mag

    def correct_gyro(self, q_new):
        # Apply accelerometer correction if possible
        accel = self.data_array[0:3]
        acc_length = np.sqrt(np.sum(accel ** 2))

        if acc_length > 0.01:  # Only correct if we have significant acceleration
        # Normalize accelerometer
            accel_unit = accel / acc_length
            self.debug_vectors[0] = accel_unit

            # Simple accelerometer correction toward gravity
            # Create a rotation that would align current gravity vector with measured acceleration
            current_down = rotate_vector_by_quaternion(np.array([0, 0, 1]), q_new)
            self.debug_vectors[1] = current_down

            correction_strength = 0.1  # Smaller for fine movements

            # Calculate rotation axis between vectors (cross product)
            rotation_axis = np.cross(current_down, accel_unit)
            rotation_angle = np.arccos(np.clip(np.dot(current_down, accel_unit), -1.0, 1.0))

            # Apply small correction - create correction quaternion
            rotation_axis_magnitude=np.linalg.norm(rotation_axis)
            self.rotation_axis_mag = rotation_axis_magnitude
            if rotation_axis_magnitude > 1e-6:
                rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
                correction_angle = rotation_angle * correction_strength
                correction_quat = quaternion_from_axis_angle(rotation_axis, correction_angle)
                
                # Apply correction
                q_new = quaternion_multiply(correction_quat, q_new)
                correction_zaxis = rotate_vector_by_quaternion(np.array([0, 0, 2]), q_new)
                self.debug_vectors[2] = correction_zaxis 
                
                self.corrected_zaxis=True
            else:
                self.corrected_zaxis=False

            return q_new

    def update_quaternions_exponential(self, dt):
        """
        Update quaternions using direct quaternion exponential integration
        More accurate with small angular velocities
        """
        # Get current quaternion
        q = self.quaternion

        # Get gyroscope data in rad/s
        gyro = self.data_array[3:6]

        # Directly integrate using quaternion exponential formula
        q_new = integrate_quaternion_exp(q, gyro, dt)

        q_new = self.correct_gyro(q_new)

        # Store updated and normalized quaternion
        self.quaternion = quaternion_normalize(q_new)
        
        return self.quaternion

if __name__ == "__main__":
    debug = ImuDebug()
    
    for i in range(100):
        data = debug.read_imus()
        debug_vectors = data[1]
        corrected = data[2]
        correction_mag = data[3]
        print(debug_vectors, corrected, correction_mag)
        sleep(1)
