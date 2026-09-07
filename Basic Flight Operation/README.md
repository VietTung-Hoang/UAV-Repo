# UAV Lab 2 — Pre-Flight Test Harness

Test your `my_mission.py` **without a drone or Crazyradio dongle**.

Mail: jan.zwiener@h-da.de

![Screenshot](./uavlab02.png)

## Quick Start

```bash
python run_tests.py my_mission.py
```

That's it. No venv switch, no hardware needed.

## What it checks

| # | Test | What it means |
|---|------|---------------|
| 1 | Healthy scenario  | The pre-flight check doesn't false-alarm on a good drone |
| 2 | Low battery | Prevent a take-off with a discharged batterie |
| 3 | Bad attitude | Take-off only from an even surface |
| 4 | Position quality | The Kalman/Lighthouse quality check works |
| 5 | Unstable velocity | No take-off if the drone is not stable |
| 6 | Radio quality | No take-off with a weak radio link |
| 7 | Lighthouse | Lighthouse System must be installed on drone |
| 8 | Task 2: hover | Hover procedure implemented |
| 9 | Task 3: rectangle | Flight procedure implemented |

A **green 19/19** means the script is structurally correct and safe to bring to the uav lab.
It does **not** guarantee perfect flight, for example sensor noise and real physics are not simulated.

## How it works (no venv needed)

`run_tests.py` inserts a `cflib_mock/` folder at the front of Python's module
search path (`sys.path`) before your script is imported. Python therefore finds
the mock `cflib` instead of the real one. No virtual environment switching
required, the real `cflib` (if installed) is simply shadowed for this run.

## Requirements

Only Python 3.9+ standard library. No extra packages needed.

```bash
python --version   # 3.9 or newer
python run_tests.py my_mission.py
```

## Troubleshooting

 - **`Required function 'pre_flight_check' not found`**
    - Make sure your function names exactly match the template: `pre_flight_check`, `task2_manual_hover`, `task3_autonomous_rectangle`.

 - **Lighthouse deck test fails**
    - Use `scf.cf.param.add_update_callback(group='deck', name='bcLighthouse4', cb=...)`
      and verify the callback fires with value `"1"` for the `lighthouse_deck_attached` variable.

 - **`LogConfig exceeds 26-byte limit`**
    - Too much data requested. Switch some variables from `float` (4 B) to `FP16` (2 B). See the lab sheet.
