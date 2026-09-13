# Mobile Robotics – Parallel Parking Simulation

MATLAB implementation of a kinematic parallel-parking proof of concept for a car-like mobile robot.

## Project Overview

The project generates curvature-constrained parking paths, applies trajectory timing, derives steering and velocity inputs, and validates the resulting motion using an independent forward simulation of the kinematic bicycle model.

Three parking scenarios are investigated:

- **Standard Parking:** 6.5 m parking space
- **Changed Start Pose:** same parking space with a modified initial heading
- **Tight Parking:** 5.5 m parking space

The tight scenario is reported as `not_found` when the implemented planner cannot identify a trajectory satisfying the imposed geometric and steering constraints. This is not presented as proof of physical impossibility.

## Requirements

- MATLAB R2025b
- Optimization Toolbox

## Run the Project

Main experiment:

```matlab
runParkingExperiments
```

Optional numerical step-size sensitivity study:

```matlab
runParkingSensitivity
```

## Repository Structure

```text
Mobile-Robotics-Parallel-Parking/
|-- runParkingExperiments.m       Main experiment runner
|-- runParkingSensitivity.m       Optional sensitivity study
|-- *.m                           Model, planning, simulation and evaluation functions
|-- private/                      Internal MATLAB helper function
|-- tests/                        Automated verification tests
|-- results/
|   |-- main/                     Main scenario plots and numerical results
|   `-- verification/             Final verification evidence
|-- README.md
`-- .gitignore
```

## Main Implementation Groups

### Vehicle Model and Parameters

- `vehicleParameters.m`
- `carKinematics.m`
- `scenarioParameters.m`

### Path Planning

- `cubicPath.m`
- `evaluateCubicPath.m`
- `validateCubicGeometry.m`
- `planCubicParking.m`

### Timing and Inputs

- `timeScale.m`
- `timeParkingPlan.m`
- `pathToInputs.m`

### Forward Simulation

- `simulateKinematics.m`
- `simulateParkingPlan.m`
- `simulateHeldParkingPlan.m`

### Evaluation and Visualization

- `evaluateParkingExperiment.m`
- `compareParkingStates.m`
- `plotParkingExperiment.m`
- `drawVehicle.m`

## Vehicle Parameters

- Vehicle length: 4.50 m
- Vehicle width: 1.80 m
- Wheelbase: 2.70 m
- Maximum steering angle: +/-35 degrees
- Minimum turning radius: approximately 3.856 m

## Method

1. Define the vehicle geometry and parking scenario.
2. Generate cubic path candidates.
3. Validate continuous curvature and steering constraints.
4. Select a feasible path.
5. Apply trajectory time parameterization.
6. Derive steering and longitudinal velocity inputs.
7. Forward-simulate the kinematic bicycle model.
8. Compare planned and simulated motion quantitatively.

## Results

Plots and numerical outputs used for the project evaluation are stored in `results/main`.

Final verification information is stored in `results/verification`.

## Tests

Run the clean repository test suite with:

```matlab
results = runtests("tests");
```

The verified clean repository test suite contains:

- **470 passed**
- **0 failed**
- **0 incomplete**
