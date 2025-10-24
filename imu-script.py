
from mpu6050 import mpu6050
import numpy as np
import time

class SimpleKinematics:
    def __init__(self, address=0x68):
        self.sensor = mpu6050(address)
        
        # State variables
        self.velocity = np.array([0.0, 0.0, 0.0])  # m/s
        self.position = np.array([0.0, 0.0, 0.0])  # m
        
        # Calibration offsets (to remove sensor bias)
        self.accel_offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # Timing
        self.last_time = time.time()
    
    def calibrate(self, samples=100):
        """Average readings while stationary to find bias"""
        
        accel_sum = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        for i in range(samples):
            accel = self.sensor.get_accel_data()
            for axis in ['x', 'y', 'z']:
                accel_sum[axis] += accel[axis]
            time.sleep(0.01)
        
        # Calculate average offset
        for axis in ['x', 'y', 'z']:
            self.accel_offset[axis] = accel_sum[axis] / samples
        
        # Remove gravity from Z (assuming sensor is flat)
        self.accel_offset['z'] -= 9.81
        
    
    def update(self):
        """
        The core physics:
        1. Get acceleration from sensor
        2. Integrate acceleration to get velocity:  v = v + a*dt
        3. Integrate velocity to get position:      p = p + v*dt
        """
        # Calculate time step
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Get raw acceleration and remove offset
        raw_accel = self.sensor.get_accel_data()
        ax = raw_accel['x'] - self.accel_offset['x']
        ay = raw_accel['y'] - self.accel_offset['y']
        az = raw_accel['z'] - self.accel_offset['z']
        
        # Convert to numpy array
        accel = np.array([ax, ay, az])
        
        # Physics equations:
        # v(t) = v(t-1) + a*dt
        self.velocity = self.velocity + accel * dt
        
        # p(t) = p(t-1) + v*dt
        position_change = self.velocity * dt
        self.position = self.position + position_change
        
        return {
            'acceleration': accel,           # m/s²
            'velocity': self.velocity,       # m/s
            'position': self.position,       # m
            'position_delta': position_change  # m (change this frame)
        }
    
    def reset(self):
        """Reset to starting position"""
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.position = np.array([0.0, 0.0, 0.0])


# Simple demo
if __name__ == "__main__":
    print("=" * 50)
    print("Simple MPU6050 Kinematics - Pure Physics")
    print("=" * 50)
    print()
    
    # Create kinematics tracker
    tracker = SimpleKinematics(address=0x68)
    
    # Calibrate sensor
    tracker.calibrate(samples=100)
    
    print("Tracking motion... (Ctrl+C to stop)\n")
    
    try:
        while True:
            # Update physics
            data = tracker.update()
            
            a = data['acceleration']
            v = data['velocity']
            p = data['position']
            dp = data['position_delta']
            
            # Display
            print(f"Accel (m/s²):  X:{a[0]:7.3f}  Y:{a[1]:7.3f}  Z:{a[2]:7.3f}")
            print(f"Velocity (m/s): X:{v[0]:7.3f}  Y:{v[1]:7.3f}  Z:{v[2]:7.3f}")
            print(f"Position (m):  X:{p[0]:7.3f}  Y:{p[1]:7.3f}  Z:{p[2]:7.3f}")
            print(f"Change (m):    ΔX:{dp[0]:7.4f}  ΔY:{dp[1]:7.4f}  ΔZ:{dp[2]:7.4f}")
            print("-" * 50)
            
            time.sleep(0.05)  # 20 Hz
            
    except KeyboardInterrupt:
        print("\n✓ Stopped")
        print(f"Final position: {tracker.position}")


"""
HOW THE PHYSICS WORKS:

1. ACCELERATION (a):
   - Sensor directly measures this in m/s²
   - This is the rate of change of velocity

2. VELOCITY (v):
   - v_new = v_old + a * dt
   - We add the acceleration over the time step
   - This is integration: ∫a dt = v

3. POSITION (p):
   - p_new = p_old + v * dt
   - We add the velocity over the time step
   - This is integration: ∫v dt = p
"""