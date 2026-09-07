# --- retroflight/controller.py ---
import numpy as np

class UAVController:
    def __init__(self):
        # Position loop gains (outer loop)
        self.Kp_pos = np.array([4, 4, 0.0])
        self.Ki_pos = np.array([0.0, 0.0, 0.0])
        self.Kd_pos = np.array([0.0, 0.0, 0.0])
        
        # Velocity loop gains (inner loop)
        self.Kp_vel = np.array([5.5, 5.5, 0.0])
        self.Ki_vel = np.array([0.5, 0.5, 0.0])
        self.Kd_vel = np.array([0.0, 0.0, 0.0])

        # Position loop state
        self.pos_error_sum = np.zeros(3)
        self.pos_last_error = np.zeros(3)
        
        # Velocity loop state
        self.vel_error_sum = np.zeros(3)
        self.vel_last_error = np.zeros(3)
        
        self.thrust = np.zeros(3)

    def compute_thrust(self, state, setpoint, dt, time):
        """
        Calculate thrust for vehicle using cascaded control architecture.
        Outer loop: position to target velocity
        Inner loop: velocity to thrust
        
        :param state: np.array [x, y, z, vx, vy, vz, ax, ay, az]
        :param setpoint: np.array [target_x, target_y, target_z]
        :param dt: delta time
        :param time: absolute simulation time
        :return: np.array [thrust_x, thrust_y, thrust_z]
        """
        pos = state[0:3]
        vel = state[3:6]
        accel = state[6:9]

        # Position error
        pos_error = setpoint - pos
        pos_error[2] = 0.0  # ignore altitude for now

        # --- Outer Loop: Position to Target Velocity ---
        # Proportional term
        pos_p_term = self.Kp_pos * pos_error
        
        # Integral term
        self.pos_error_sum += pos_error * dt
        pos_i_term = self.Ki_pos * self.pos_error_sum
        
        # Derivative term (d(error)/dt = -velocity)
        pos_d_term = self.Kd_pos * (-vel)
        
        # Target velocity from position controller
        target_vel = pos_p_term + pos_i_term + pos_d_term
        
        # --- Inner Loop: Velocity to Thrust ---
        # Velocity error
        vel_error = target_vel - vel
        
        # Proportional term
        vel_p_term = self.Kp_vel * vel_error
        
        # Integral term
        self.vel_error_sum += vel_error * dt
        vel_i_term = self.Ki_vel * self.vel_error_sum
        
        # Derivative term (d(error_vel)/dt = -acceleration)
        vel_d_term = self.Kd_vel * (-accel)
        
        # Compute thrust
        self.thrust = vel_p_term + vel_i_term + vel_d_term
        
        # Update last errors
        self.pos_last_error = pos_error.copy()
        self.vel_last_error = vel_error.copy()
 
        return self.thrust

    def on_collision(self, state, type_string):
        """
        Handle collision event.
        type_string can be e.g. "level" or "battery".
        """
        # print(type_string)


