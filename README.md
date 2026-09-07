# UAV Lab Repository

Coursework and experiments for UAV flight control, state estimation, and autonomous mission scripting.

## Repository Contents

### 1. 2D Drone PID Control and Waypoint Tracking

A 2D top-down UAV simulator built with Python, NumPy, and Pygame. It includes:

- PID-based waypoint tracking
- A local simulation environment
- An external mission controller using a TCP API
- A visual interface for testing flight behavior without hardware

See [the RetroFlight README](2D%20Drone%20PID%20Control%20and%20Way%20Point%20Tracking/retroflight/README.md) for simulator details.

To run it on Windows:

```powershell
cd "2D Drone PID Control and Way Point Tracking\retroflight"
.\setup.bat
.\run.bat
```

To run an external mission, start the simulator first and then use a second terminal:

```powershell
cd "2D Drone PID Control and Way Point Tracking\retroflight"
.\.venv\Scripts\activate
python external_control\my_mission.py
```

### 2. Altitude Estimation with a Kalman Filter

The [`kalmanfilter.py`](Altitude%20Estimation%20with%20a%20Kalman%20Filter/kalmanfilter.py) script estimates altitude, vertical velocity, and accelerometer bias by combining IMU, attitude, and barometer data with a Kalman filter.

Requirements:

```powershell
pip install numpy pandas matplotlib
```

Place the input file `flight_data.csv` in the `Altitude Estimation with a Kalman Filter` directory, then run:

```powershell
cd "Altitude Estimation with a Kalman Filter"
python kalmanfilter.py
```

The processed CSV is written to `output\flight_data_with_kalman.csv`, and the script displays plots for altitude, vertical velocity, and accelerometer bias.

### 3. Basic Flight Operation

A Crazyflie mission template and a hardware-free pre-flight test harness. The harness injects mock `cflib` modules, so the mission can be checked without a Crazyradio dongle or drone.

See [the Basic Flight Operation README](Basic%20Flight%20Operation/README.md) for the full test description.

Run the tests from that directory:

```powershell
cd "Basic Flight Operation"
python run_tests.py my_mission_template.py
```

The test harness checks pre-flight safety behavior, hover, and a 1 m rectangular trajectory. A passing test suite does not replace a real hardware safety check.

## Requirements

- Python 3.9 or newer for the Basic Flight Operation harness
- Python 3.8 or newer for RetroFlight
- NumPy, Pandas, and Matplotlib for the Kalman-filter analysis
- NumPy and Pygame CE for RetroFlight; these are installed by `retroflight\setup.bat`
- Crazyflie hardware and `cflib` only when running missions against a real drone

Each project is independent and should be run from its own directory.

## Reference Material

The UAV lab handouts are available at the repository root:

- [UAV Lab 1](uav_lab1.pdf)
- [UAV Lab 2](uav_lab2.pdf)
- [UAV Lab 3](uav_lab3.pdf)
