# Waypoint Autopilot (GPS + LIDAR(lite) + Obstacle Avoidance + UDP Telemetry)

A simple 2D “boat autopilot” simulation that drives through waypoints while avoiding circular obstacles.  
It includes **sensor simulation** (GPS noise + dropouts, and a LIDAR-lite range sensor), a **turn-rate-limited heading controller** (realistic steering constraint), and **UDP telemetry** so a separate “ground station” script can watch the vehicle state live.

---

https://github.com/user-attachments/assets/929a9881-184e-4112-9693-1a49b2c2c6f0



---

## Features

- **Waypoint navigation**: visit targets in order until finished
- **Turn-rate limited steering**: heading changes are clamped by a max turn rate (deg/s)
- **GPS sensor simulation**:
  - Gaussian noise (jitter)
  - Random dropouts (hold-last-value behavior)
- **LIDAR-lite range sensor**:
  - Multiple rays in a forward field-of-view
  - Ray–circle intersection to detect obstacles
  - Range noise + per-ray dropouts
- **Obstacle avoidance**:
  - Converts LIDAR ranges into a repulsive “avoid vector”
  - Blends goal direction + avoidance direction
- **Speed control**:
  - Slows down near the next waypoint
  - Slows down when LIDAR sees a nearby obstacle
- **Live visualization** with Matplotlib:
  - boat position + trail
  - GPS measurement (jittery `x`)
  - heading arrow
  - LIDAR rays
  - obstacle circles + safety margin circles
- **UDP telemetry** (JSON packets) to a local receiver script

---

## Repo Structure

- `main.py` — runs the simulation + sensors + avoidance + telemetry sender
- `udp_receiver.py` — listens for UDP telemetry packets and prints them
- `run_log.csv` — generated at runtime (per-step logging of key values)

---

## Requirements

- Python 3.9+ recommended
- `matplotlib` (plus standard library modules)

Install dependencies:

```bash
pip install matplotlib
