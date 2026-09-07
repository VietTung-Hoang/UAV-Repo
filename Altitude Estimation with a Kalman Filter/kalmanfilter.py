import numpy as np
import pandas as pd
from pathlib import Path
import matplotlib.pyplot as plt

def quat_to_matrix(q):
    """Unit quaternion (qw, qx, qy, qz) -> 3x3 body->world rotation matrix."""
    a, b, c, d = q
    return np.array([
        [a*a + b*b - c*c - d*d, 2*(b*c - a*d),         2*(b*d + a*c)],
        [2*(b*c + a*d),         a*a - b*b + c*c - d*d, 2*(c*d - a*b)],
        [2*(b*d - a*c),         2*(c*d + a*b),         a*a - b*b - c*c + d*d]
    ])

def pressure_to_alt(p_hpa, p0_hpa=1013.25):
    """Barometric altitude formula (international standard atmosphere)."""
    return 44330.0 * (1.0 - (p_hpa / p0_hpa) ** (1.0 / 5.255))


#------from CSV file------
def from_csv(file_path):
    # 1. Load data from flight_data.csv
    df = pd.read_csv(file_path)

    # 2. Define the standard gravity constant (G)
    G_TO_MS2 = 9.80665

    # 3. Extract required components
    # Body frame accelerations (units in G)
    ax = df['acc_x'].to_numpy()
    ay = df['acc_y'].to_numpy()
    az = df['acc_z'].to_numpy()

    # Attitude represented as Quaternions
    qw = df['qw'].to_numpy()
    qx = df['qx'].to_numpy()
    qy = df['qy'].to_numpy()
    qz = df['qz'].to_numpy()

    # Time 
    t = df['time'].to_numpy()

    # Barometer altitude
    p = df['baro_asl'].to_numpy()
    return t, ax, ay, az, qw, qx, qy, qz, p


# -----Correction (Fusion) Definition-----
# Measurement model: the barometer observes only the altitude state.
H = np.array([[1.0, 0.0, 0.0]])

# Standard deviation of accelerometer input noise.
acc_std = 0.05

# Process noise associated with accelerometer bias drift.
process_noise_acc = 0.0001

# Standard deviation of the barometer measurement noise.
# Tunning range 0.25-3
"""
To choose the barometer standard deviation, from the slide I know that it range from 0.25 to 3
Then I start with 0.25 and increase by 0.05
At 0.25 the velocity Vz is very noisy. The bias reduces remarkaby from 5s, and it is quite stable at the end.
Then when I try with 1.25 or over 1 in general, the starting Vz from 0s to 4s is pretty smooth.
Howerver the bias reduce slower, from 10s it starting to reduce and converge to a stable value at the end. 
The error band for kalman filter is also increased.
"""
baro_std = 0.25

#---- Complete Filter Loop------------
flight_df = pd.read_csv('flight_data.csv')
t, ax, ay, az, qw, qx, qy, qz, p = from_csv('flight_data.csv')

# Initial covariance matrix.
#P = np.eye(3, dtype=float)
P = np.diag([
    0.1**2,   # z uncertainty
    0.1**2,   # vz uncertainty
    0.01**2   # bias uncertainty
]).astype(float)

# Use the first barometer reading as the zero-altitude reference.
# h0 = pressure_to_alt(p[0])
#h0 = p[0]
ground_duration = 4  # seconds
ground_mask = t <= (t[0] + ground_duration)
h0 = np.median(p[ground_mask])

# State vector x = [height, vertical_velocity, accelerometer_bias].
x = np.array([p[0], 0.0, 0.0], dtype=float)

heights = [h0]
velocities = [0.0]
sigmas = [np.sqrt(P[0, 0])]
biases = [x[2]]
bias_sigmas = [np.sqrt(P[2, 2])]

for i in range(1, len(t)):
    dt = t[i] - t[i - 1]

    # ------Prediction Step------
    # Prediction matric
    Phi = np.array([
        [1.0, dt, 0.5 * dt**2],
        [0.0, 1.0, dt],
        [0.0, 0.0, 1.0]
    ], dtype=float)

    # Control input matrix
    B = np.array([
        0.5 * dt**2,
        dt,
        0.0
    ], dtype=float)

    R = quat_to_matrix([qw[i], qx[i], qy[i], qz[i]])
    a_body = np.array([ax[i], ay[i], az[i]], dtype=float)

    """
    Convert body-frame acceleration to world-frame and subtract gravity
    Since the z-axis after rotating to the world frame points up according to the convention,
    the value at rest is usually close to -1 g.
    In that case: sensor gives about +1 g when stationary, then subtracting 1.0 removes gravity;
    """
    G_TO_MS2 = 9.80665
    a = ((R @ a_body)[2] - 1.0) * G_TO_MS2

    x = Phi @ x + B * a

    # process noise for acceleration input
    q_a = acc_std**2

    # process noise for acceleration bias
    q_b = process_noise_acc**2

    B_mat = B.reshape(3, 1)
    
    """
    Q matrix depends on the acceleration noise and the process noise for the bias.
    This value is 0.05 and 0.001 respectively.

    For process_noise_acc, by starting tunning from 0.1, I see the estimated bias is drifting too much
    and not converging to a stable value. By reducing it to 0.01, 0.001 and so on. And starting from 0.0001, 
    the bias estimate becomes more stable and converges

    For acc_std, I try to vary it, but not see any noticeable difference in the estimated bias.
    """
    Q_acc = B_mat @ np.array([[q_a]]) @ B_mat.T + np.diag([0.0, 0.0, q_b])

    P = Phi @ P @ Phi.T + Q_acc

    # --------Correction Step------
    # raw altitude
    #h_baro = pressure_to_alt(p[i]) - h0   
    h_baro = p[i]

    # innovation    
    y = h_baro - (H @ x)[0]

    # S is scalar
    S = (H @ P @ H.T)[0, 0] + baro_std**2  

    # K is (3,1)
    K = (P @ H.T) / S  
                    
    x = x + (K * y).ravel()
    P = (np.eye(3) - K @ H) @ P

    heights.append(x[0])
    velocities.append(x[1])
    sigmas.append(np.sqrt(P[0, 0]))
    biases.append(x[2])
    bias_sigmas.append(np.sqrt(P[2, 2]))

    #print(f"h = {x[0]:.2f} m, v = {x[1]:.2f} m/s")

output_dir = Path(__file__).resolve().parent / "output"
output_dir.mkdir(exist_ok=True)

results = pd.DataFrame({
    'time': t,
    'h': heights,
    'v': velocities
})

merged_df = flight_df.merge(results, on='time', how='left')
output_file = output_dir / 'flight_data_with_kalman.csv'
merged_df.to_csv(output_file, index=False)
print(f"Saved merged flight data with Kalman estimates to {output_file}")

# Calculate the error bands for the altitude estimates
# sigma_h = [np.sqrt(p_val) for p_val in variances_p00] 

fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
ax1, ax2, ax3 = axes

# ----- Plot 1: Altitude -----
kalman_alt = np.array(heights) - h0
upper = kalman_alt + np.array(sigmas)
lower = kalman_alt - np.array(sigmas)

ax1.plot(t, p - h0, label='Raw Baro Altitude', alpha=0.3, color='red')
ax1.plot(t, kalman_alt, label='Kalman Altitude', color='blue', linewidth=2)
ax1.plot(t, flight_df['ref_z'].to_numpy(), label='Lighthouse Altitude (Ground Truth)', color='green', linestyle='--')
ax1.fill_between(t, lower, upper, color='blue', alpha=0.2, label='±1σ Error Band')
ax1.set_ylabel('Altitude [m]')
ax1.set_title('Kalman Filter States')
ax1.legend()
ax1.grid(True)

# ----- Plot 2: Vertical velocity -----
ax2.plot(t, velocities, label='Estimated V_z', color='orange', linewidth=2)
ax2.set_ylabel('Velocity [m/s]')
ax2.legend()
ax2.grid(True)

# ----- Plot 3: Accelerometer bias -----
upper_bias = np.array(biases) + np.array(bias_sigmas)
lower_bias = np.array(biases) - np.array(bias_sigmas)

ax3.plot(t, biases, label='Estimated Accel Bias (b_a)', color='purple', linewidth=2)
ax3.fill_between(t, lower_bias, upper_bias, color='purple', alpha=0.2)
ax3.set_xlabel('Time [s]')
ax3.set_ylabel('Bias [m/s²]')
ax3.legend()
ax3.grid(True)

plt.tight_layout()
plt.show()