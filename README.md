# Field Oriented Control (FOC) of PMSM - Open-Source Simulation

This repository contains Python and Scilab files for modeling and simulating the Field Oriented Control (FOC) of a Permanent Magnet Synchronous Machine (PMSM). The implementation is based on open-source tools, demonstrating the feasibility of using Python Control and Scilab for motor control research and algorithm validation.

## Project Overview

Field Oriented Control (FOC) is a widely used technique for precise control of PMSMs, commonly applied in electric vehicles and industrial automation. The simulation model in this repository includes:

- **Mathematical model of PMSM**
- **Coordinate transformations (Clarke & Park)**
- **Current and speed control loops using PI regulators**
- **Space Vector Pulse Width Modulation (SVPWM)**
- **Simulation of phase current sensing and noise filtering**
- **Torque and speed response analysis**

## Requirements

### Python
- Python 3.x
- `numpy`
- `matplotlib`
- `control` (Python Control Systems Library)

Install dependencies with:
```bash
pip install numpy matplotlib control
```

### Scilab
- Scilab 2025.0.0 or later


