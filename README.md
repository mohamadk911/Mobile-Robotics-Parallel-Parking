# Mobile Robotics – Parallel Parking Simulation

MATLAB implementation of a kinematic parallel-parking proof of concept for a car-like mobile robot.

## Start Here

**[`runParkingExperiments.m`](runParkingExperiments.m) is the main entry point.** It reproduces the three report scenarios using the main validated workflow: path planning, geometric validation, timing, input generation, forward simulation, quantitative evaluation and plots.

From the repository folder in MATLAB, run:

```matlab
runParkingExperiments
```

New outputs are written to `results/step10/`. The included report plots and numerical results are in [`results/main/`](results/main/). If generated outputs already exist, the runner requires a new output folder or explicit overwrite permission.

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

These functions form the main workflow called by `runParkingExperiments.m`.

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

### Evaluation and Visualization

- `evaluateParkingExperiment.m`
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

## Additional / Supporting Work

### Numerical Sensitivity Study

[`runParkingSensitivity.m`](runParkingSensitivity.m) runs the optional sample-time and integration-sensitivity study separately from the main experiment:

```matlab
runParkingSensitivity
```

It compares Euler propagation at different time steps against a higher-accuracy numerical reference, using a fixed geometric plan for each scenario. New study outputs are written to `results/step11/`.

Supporting functions:

- `studyParkingPlan.m` and `sensitivitySettings.m` — organize the study and its settings.
- `integrateParkingReference.m` and `sampleParkingReference.m` — integrate and sample the numerical reference.
- `simulateHeldParkingPlan.m` — separate held-command propagation from Euler integration effects.
- `compareParkingStates.m` and `parkingConvergence.m` — calculate numerical discrepancies and convergence.

### Acceptance and Automated Verification

- `buildParkingAcceptance.m` consolidates evidence from the main experiment and sensitivity study.
- `tests/` contains automated checks for the model, geometry, planning, timing, simulation, evaluation and reproducibility.
- `results/verification/` contains the included final verification evidence.

## Results

Plots and numerical outputs used for the project evaluation are stored in `results/main`.

Final verification information is stored in `results/verification`.

## Tests

Run the clean repository test suite with:

```matlab
results = runtests("tests");
```

**Current clean professor-facing repository:**

- **470 passed**
- **0 failed**
- **0 incomplete**

**Historical full-development / frozen-project verification:**

The retained file [`results/verification/verification.txt`](results/verification/verification.txt) documents an earlier verification run:

- **484 passed**
- **0 failed**
- **0 incomplete**

That historical run included additional development-time verification artifacts and tests that were intentionally not all included in this cleaned submission repository. The current `runtests("tests")` command runs the clean repository's 470 tests; the historical 484-test total does not describe the currently runnable suite.
