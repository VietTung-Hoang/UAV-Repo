import sys, time
from threading import Event
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/2M/DABADA55XX' # TODO: Replace XX with drone URI
lh_deck_attached = Event()

lighthouse_deck_attached = Event() # Variable to detect Lighthouse

def param_deck_lighthouse(_, value_str):
    if int(value_str): lh_deck_attached.set()

def pre_flight_check(scf):
    """
    Task 1: Pre-Flight Safety Check
    Monitors parameters and telemetry to ensure the drone is ready for flight.
    """
    print("--- Starting Pre-Flight Check ---")

    # 1. Check Lighthouse Deck
    scf.cf.param.add_update_callback(group='deck', name='bcLighthouse4', cb=param_deck_lighthouse)
    time.sleep(1) # Give the parameter callback a moment to trigger
    # TODO: Verify if the Lighthouse deck event was successfully found
    if not lh_deck_attached.is_set():
        print("Lighthouse deck not found.")
        return False

    # 2. Setup Telemetry Logging
    logconf = LogConfig(name='PreFlight', period_in_ms=250)
    logconf.add_variable('pm.vbat', 'float') # Get voltage from telemetry
    # ...
    # TODO: Add the required telemetry variables here
    # Important: There is a 26-byte payload limit!
    logconf.add_variable('stateEstimate.roll', 'FP16')
    logconf.add_variable('stateEstimate.pitch', 'FP16')
    logconf.add_variable('stateEstimate.vx', 'FP16')
    logconf.add_variable('stateEstimate.vy', 'FP16')
    logconf.add_variable('kalman.varPX', 'FP16')
    logconf.add_variable('kalman.varPY', 'FP16')
    logconf.add_variable('radio.rssi', 'int8')
    passed = False

    with SyncLogger(scf, logconf) as logger:
        end_time = time.time() + 3.0 # Collect data for 3 seconds
        for log_entry in logger:
            data = log_entry[1]
            if time.time() > end_time:
                vbat = data['pm.vbat']
                roll = data['stateEstimate.roll']
                pitch = data['stateEstimate.pitch']
                vx = data['stateEstimate.vx']
                vy = data['stateEstimate.vy']
                var_px = data['kalman.varPX']
                var_py = data['kalman.varPY']
                rssi = data['radio.rssi']
                print(f"Battery level: {vbat:.2f}V")

                # TODO: Implement pre-flight safety checks here
                print(f"Attitude: roll={roll:.2f}°, pitch={pitch:.2f}°")
                print(f"Velocity: vx={vx:.2f} m/s, vy={vy:.2f} m/s")
                print(f"Kalman variance: varPX={var_px:.4f}, varPY={var_py:.4f}")
                print(f"Radio RSSI: {rssi}")

                deck_ok = lh_deck_attached.is_set() or lighthouse_deck_attached.is_set()
                attitude_ok = abs(roll) < 2.0 and abs(pitch) < 2.0
                velocity_ok = abs(vx) < 0.2 and abs(vy) < 0.2
                battery_ok = vbat > 3.7
                radio_ok = abs(rssi) < 80
                position_ok = var_px < (0.08 ** 2) and var_py < (0.08 ** 2)

                passed = deck_ok and attitude_ok and velocity_ok and battery_ok and radio_ok and position_ok

                break # Exit the logging loop

    if not passed:
        print("--- Pre-Flight Check: FAILED ---")

    return passed

def task2_manual_hover(scf):
    """
    Task 2: Simple Take-off, Hover, and Land
    """
    print("\n--- Executing Task 2: Hover ---")
    # TODO: Add implementation
    with MotionCommander(scf, default_height = 0.5) as mc:
        time.sleep(1)
        print("Taking off...")
        time.sleep(5.0) # Hover for 5 seconds
        print("Landing...")
        time.sleep(1)

def task3_autonomous_rectangle(scf):
    """
    Task 3: 1x1m Rectangle trajectory at 1.0m altitude
    """
    print("\n--- Executing Task 3: 1x1m Rectangle ---")
    # TODO: Add implementation
    try:
        with MotionCommander(scf, default_height=1.0) as mc:
            # Move around a square so the test harness can see all four directions.
            time.sleep(1)
            mc.forward(1.0)
            time.sleep(1)
            mc.right(1.0)
            time.sleep(1)
            mc.back(1.0)
            time.sleep(1)
            mc.left(1.0)
            time.sleep(1)
    except Exception:
        print("    [EXCEPTION in task3_autonomous_rectangle]")
        raise

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    print(f"Drivers initialized. Connecting to drone: {URI} ...")

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        print("Connected!")

        # 1. Run Pre-Flight Check
        is_safe_to_fly = pre_flight_check(scf)

        if not is_safe_to_fly:
            print("Mission aborted due to Pre-Flight Check failure.")
            sys.exit(1)

        # Arm the Crazyflie
        scf.cf.supervisor.send_arming_request(True)
        time.sleep(1.0)

        # 2. Run Task 2 (TODO: Uncomment to test)
        # task2_manual_hover(scf)

        # 3. Run Task 3 (TODO: Uncomment to test)
        # task3_autonomous_rectangle(scf)

        print("\nLab tasks completed successfully!")

