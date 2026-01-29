# API Reference

Complete reference for all classes and functions in the FRC MATLAB Control Library.

## Table of Contents

- [Motor Models](#motor-models)
- [Mechanism Templates](#mechanism-templates)
- [Control Synthesis](#control-synthesis)
- [Simulation Engine](#simulation-engine)
- [System Identification](#system-identification)
- [Visualization System](#visualization-system)

---

## Motor Models

### KrakenX40Model

High-torque motor model for FRC applications.

```matlab
motor = KrakenX40Model();
```

**Properties:**
- `stallTorque_Nm` - Stall torque (N⋅m)
- `freeSpeed_rpm` - Free speed (RPM)
- `stallCurrent_A` - Stall current (A)
- `freeCurrent_A` - Free current (A)
- `resistance_ohm` - Motor resistance (Ω)
- `Kv` - Velocity constant
- `Kt` - Torque constant

### KrakenX60Model

High-speed motor model for FRC applications.

```matlab
motor = KrakenX60Model();
```

Same properties as KrakenX40Model with different specifications.

### MotorCharacteristics

Generic motor model creator.

```matlab
motor = MotorCharacteristics(name, stallTorque, freeSpeed, stallCurrent, freeCurrent);
```

**Methods:**
- `listAvailableMotors()` - List all predefined motors
- `getMotor(name)` - Get motor by name

---

## Mechanism Templates

### SimpleRotationalMechanism

Template for rotational mechanisms (turrets, intakes, arms).

```matlab
mech = SimpleRotationalMechanism(name, motorType);
```

**Properties:**
- `name` - Mechanism name (string)
- `motor` - Motor model object
- `gearRatio` - Gear reduction ratio (motor:mechanism)
- `inertia_kg_m2` - Moment of inertia (kg⋅m²)
- `friction_Nm` - Friction torque (N⋅m)
- `kP, kI, kD` - PID gains
- `kS, kV, kA, kG` - Feedforward gains

**Methods:**

#### `calculateFeedforward()`
Auto-calculates feedforward gains (kS, kV, kA).

```matlab
mech.calculateFeedforward();
```

#### `calculatePID(settlingTime, overshoot)`
Auto-tunes PID controller.

```matlab
mech.calculatePID(0.5, 5);  % 0.5s settling, 5% overshoot
```

**Parameters:**
- `settlingTime` - Desired settling time (seconds)
- `overshoot` - Maximum overshoot (percent)

#### `getStateSpaceModel()`
Returns state-space representation for analysis.

```matlab
sys = mech.getStateSpaceModel();
```

### ElevatorMechanism

Template for linear elevator mechanisms.

```matlab
elevator = ElevatorMechanism(name, motorType);
```

**Additional Properties:**
- `mass_kg` - Carriage mass (kg)
- `drumRadius_m` - Spool/drum radius (m)
- `kG` - Gravity compensation gain

**Methods:** Same as SimpleRotationalMechanism plus:

#### `calculateGravityCompensation()`
Calculates feedforward gain to counteract gravity.

```matlab
elevator.calculateGravityCompensation();
```

### FlywheelMechanism

Template for high-speed flywheel mechanisms.

```matlab
flywheel = FlywheelMechanism(name, motorType);
```

**Properties:** Similar to SimpleRotationalMechanism
- Optimized for velocity control
- No position controller

### SwerveDriveSystem

Complete swerve drive system simulation.

```matlab
swerve = SwerveDriveSystem(name);
```

**Properties:**
- `modules` - Array of 4 SwerveModuleModel objects
- `wheelbase_m` - Distance between front/rear wheels (m)
- `trackwidth_m` - Distance between left/right wheels (m)

**Methods:**

#### `setModuleParameters(driveMotor, steerMotor, driveRatio, steerRatio)`
Configure all modules.

```matlab
swerve.setModuleParameters('KrakenX60', 'KrakenX40', 6.75, 12.8);
```

---

## Control Synthesis

### PIDTuner

Automatic PID controller tuning.

```matlab
gains = PIDTuner.tune(mechanism, settlingTime, overshoot);
```

**Inputs:**
- `mechanism` - Mechanism object
- `settlingTime` - Desired settling time (s)
- `overshoot` - Maximum overshoot (%)

**Outputs:**
- `gains.kP` - Proportional gain
- `gains.kI` - Integral gain
- `gains.kD` - Derivative gain

**Methods:**

#### `tuneFrequencyDomain(sys, specs)`
Frequency-domain PID tuning using loop shaping.

```matlab
gains = PIDTuner.tuneFrequencyDomain(sys, specs);
```

#### `tuneZieglerNichols(sys)`
Classical Ziegler-Nichols tuning method.

```matlab
gains = PIDTuner.tuneZieglerNichols(sys);
```

### FeedforwardCalculator

Calculates feedforward gains from mechanism parameters.

```matlab
ff = FeedforwardCalculator.calculate(mechanism);
```

**Returns:**
- `ff.kS` - Static friction compensation
- `ff.kV` - Velocity feedforward  
- `ff.kA` - Acceleration feedforward
- `ff.kG` - Gravity compensation (if applicable)

### MotionMagicOptimizer

Generates optimal motion profiles.

```matlab
profile = MotionMagicOptimizer.generate(startPos, endPos, maxVel, maxAccel);
```

**Inputs:**
- `startPos` - Starting position
- `endPos` - Target position
- `maxVel` - Maximum velocity
- `maxAccel` - Maximum acceleration

**Outputs:**
- `profile.time` - Time vector
- `profile.position` - Position trajectory
- `profile.velocity` - Velocity trajectory
- `profile.acceleration` - Acceleration trajectory

---

## Simulation Engine

### DynamicsSimulator

High-fidelity physics simulation engine.

```matlab
simData = DynamicsSimulator.simulateResponse(mechanism, testSequence);
```

**Inputs:**
- `mechanism` - Mechanism object
- `testSequence` - Structure with `.time` and `.voltage` fields

**Outputs:**
- `simData.time` - Time vector
- `simData.position` - Position trajectory
- `simData.velocity` - Velocity trajectory
- `simData.acceleration` - Acceleration trajectory
- `simData.current` - Current draw (A)
- `simData.voltage` - Applied voltage (V)
- `simData.torque` - Motor torque (N⋅m)
- `simData.power` - Power consumption (W)

**Methods:**

#### `simulateClosedLoop(mechanism, testSequence, controller)`
Simulate with closed-loop control.

```matlab
controller.kP = 0.5;
controller.kI = 0.1;
controller.kD = 0.05;
simData = DynamicsSimulator.simulateClosedLoop(mech, testSeq, controller);
```

#### `simulateWithDisturbances(mechanism, testSequence, disturbances)`
Add external disturbances to simulation.

```matlab
disturbances.torque = @(t) 0.1*sin(2*pi*t);  % Sinusoidal disturbance
simData = DynamicsSimulator.simulateWithDisturbances(mech, testSeq, dist);
```

### PerformanceValidator

Basic performance validation.

```matlab
results = PerformanceValidator.validate(simData, requirements);
```

**Inputs:**
- `simData` - Simulation results
- `requirements` - Structure with validation criteria

**Outputs:**
- `results.passed` - Boolean pass/fail
- `results.metrics` - Performance metrics
- `results.violations` - List of requirement violations

---

## System Identification

### AutomatedTestSequences

Generate optimal test signals for system identification.

```matlab
testSeq = AutomatedTestSequences.generateStepResponse(duration, stepVoltage);
testSeq = AutomatedTestSequences.generateChirp(duration, freqRange);
testSeq = AutomatedTestSequences.generatePRBS(duration, amplitude);
```

**Methods:**

#### `generateStepResponse(duration, stepVoltage)`
Classic step response test.

#### `generateChirp(duration, freqRange)`
Frequency sweep for system characterization.

#### `generatePRBS(duration, amplitude)`
Pseudo-random binary sequence for robust identification.

### ParameterExtraction

Extract mechanism parameters from test data.

```matlab
params = ParameterExtraction.fromStepResponse(testData);
params = ParameterExtraction.fromFrequencyResponse(testData);
```

**Outputs:**
- `params.inertia` - Identified inertia
- `params.friction` - Identified friction
- `params.kV` - Velocity constant
- `params.kA` - Acceleration constant

### ModelValidation

Validate identified models against real data.

```matlab
results = ModelValidation.compare(realData, simulatedData);
```

**Outputs:**
- `results.fit` - Goodness of fit (%)
- `results.rmse` - Root mean square error
- `results.maxError` - Maximum error
- `results.plots` - Comparison plots

---

## Visualization System

### VisualizationSystem

Extensible plugin-based visualization and validation.

#### `runVisualization(pluginName, simData)`

Run a visualization plugin.

```matlab
VisualizationSystem.runVisualization('standard', simData);
```

**Built-in Visualizations:**
- `'standard'` - Position, velocity, current
- `'detailed'` - All variables with subplots
- `'phase'` - Phase portrait
- `'power'` - Power analysis
- `'frequency'` - Frequency response
- `'animated'` - Animated mechanism
- `'comparison'` - Compare multiple runs

#### `runValidator(validatorName, simData, mechanism)`

Run a validation plugin.

```matlab
results = VisualizationSystem.runValidator('safety', simData, mech);
```

**Built-in Validators:**
- `'safety'` - Check current limits, voltage limits
- `'performance'` - Settling time, overshoot
- `'efficiency'` - Power consumption, heat generation
- `'robustness'` - Sensitivity to disturbances

#### `registerVisualization(name, function, description)`

Register custom visualization.

```matlab
myPlot = @(simData) plot(simData.time, simData.position);
VisualizationSystem.registerVisualization('myPlot', myPlot, 'Simple position plot');
```

#### `registerValidator(name, function, description)`

Register custom validator.

```matlab
myCheck = @(simData, mech) struct('passed', max(simData.current) < 40);
VisualizationSystem.registerValidator('currentCheck', myCheck, 'Max current validator');
```

#### `listPlugins()`

List all available plugins.

```matlab
VisualizationSystem.listPlugins();
```

---

## Constants and Units

### Standard Units

- **Length:** meters (m)
- **Mass:** kilograms (kg)
- **Time:** seconds (s)
- **Angle:** radians (rad)
- **Angular velocity:** radians/second (rad/s)
- **Torque:** Newton-meters (N⋅m)
- **Current:** Amperes (A)
- **Voltage:** Volts (V)
- **Power:** Watts (W)

### Conversion Utilities

```matlab
% Built-in conversions (add to MotorCharacteristics or utility class)
rpm2radps = @(rpm) rpm * 2*pi/60;
radps2rpm = @(radps) radps * 60/(2*pi);
deg2rad = @(deg) deg * pi/180;
rad2deg = @(rad) rad * 180/pi;
```

---

## Error Handling

All functions throw MATLAB errors with descriptive messages:

```matlab
try
    mech = SimpleRotationalMechanism('Test', 'InvalidMotor');
catch ME
    fprintf('Error: %s\n', ME.message);
end
```

Common error IDs:
- `FRC:InvalidMotor` - Unknown motor type
- `FRC:InvalidParameter` - Invalid parameter value
- `FRC:SimulationFailed` - Simulation failed to converge
- `FRC:ValidationFailed` - Validation check failed

---

For more examples and usage patterns, see the [Getting Started Guide](GETTING_STARTED.md) and [examples](../examples/) folder.
