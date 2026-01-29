# Getting Started with FRC MATLAB Control Library

This guide will walk you through setting up and using the FRC MATLAB Control Library to design and simulate robot mechanisms.

## Table of Contents

1. [Installation](#installation)
2. [Your First Simulation](#your-first-simulation)
3. [Understanding the Workflow](#understanding-the-workflow)
4. [Common Use Cases](#common-use-cases)
5. [Next Steps](#next-steps)

## Installation

### Prerequisites

- MATLAB R2019a or later
- Control System Toolbox (recommended)
- Basic understanding of MATLAB

### Setup

1. **Download the library** (choose one method):
   - Clone with Git: `git clone https://github.com/yourusername/frc-matlab-control-library.git`
   - Download ZIP from GitHub and extract

2. **Open MATLAB** and navigate to the library folder:
   ```matlab
   cd path/to/frc-matlab-control-library
   ```

3. **Run initialization**:
   ```matlab
   startup
   ```
   
   This adds all necessary paths to your MATLAB session.

## Your First Simulation

Let's simulate a simple turret mechanism in 5 minutes!

### Step 1: Create a Mechanism

```matlab
% Create a turret using a Kraken X40 motor
turret = SimpleRotationalMechanism('MyTurret', 'KrakenX40');

% Set physical parameters
turret.gearRatio = 10;           % 10:1 reduction
turret.inertia_kg_m2 = 0.05;     % Moment of inertia in kg⋅m²
turret.friction_Nm = 0.1;        % Friction torque in N⋅m
```

### Step 2: Auto-Calculate Controllers

```matlab
% Calculate feedforward gains (kS, kV, kA)
turret.calculateFeedforward();

% Auto-tune PID controller
% Parameters: settling time (0.5s), overshoot (5%)
turret.calculatePID(0.5, 5);

% Display calculated gains
fprintf('kP = %.3f, kI = %.3f, kD = %.3f\n', turret.kP, turret.kI, turret.kD);
fprintf('kS = %.3f, kV = %.3f, kA = %.3f\n', turret.kS, turret.kV, turret.kA);
```

### Step 3: Create Test Input

```matlab
% Generate a step input test
testSeq.time = linspace(0, 2, 200);          % 2 seconds
testSeq.voltage = [zeros(100,1); 6*ones(100,1)];  % 0V → 6V step
```

### Step 4: Run Simulation

```matlab
% Simulate the mechanism response
simData = DynamicsSimulator.simulateResponse(turret, testSeq);
```

### Step 5: Visualize Results

```matlab
% Create standard visualization plots
VisualizationSystem.runVisualization('standard', simData);

% Run safety validation
results = VisualizationSystem.runValidator('safety', simData, turret);
if results.passed
    fprintf('✓ Safety check passed!\n');
else
    fprintf('✗ Safety issues detected: %s\n', results.message);
end
```

**That's it!** You just simulated your first mechanism.

## Understanding the Workflow

### The Standard Workflow

```
1. CREATE MECHANISM
   ↓
2. SET PARAMETERS (gear ratio, inertia, etc.)
   ↓
3. AUTO-CALCULATE CONTROLLERS
   ↓
4. GENERATE TEST INPUT
   ↓
5. SIMULATE
   ↓
6. VISUALIZE & VALIDATE
```

### Available Mechanisms

| Mechanism | Use Case | Key Parameters |
|-----------|----------|----------------|
| `SimpleRotationalMechanism` | Turrets, intakes | `gearRatio`, `inertia_kg_m2` |
| `ElevatorMechanism` | Elevators | `gearRatio`, `mass_kg`, `drumRadius_m` |
| `FlywheelMechanism` | Shooters | `gearRatio`, `inertia_kg_m2` |
| `SwerveDriveSystem` | Swerve drive | Module parameters |

### Available Motors

- `KrakenX40` - High torque motor
- `KrakenX60` - High speed motor
- Custom motors via `MotorCharacteristics`

## Common Use Cases

### Example 1: Elevator Simulation

```matlab
% Create elevator
elevator = ElevatorMechanism('MyElevator', 'KrakenX40');
elevator.gearRatio = 12;
elevator.mass_kg = 5;           % Carriage mass
elevator.drumRadius_m = 0.02;   % Spool radius (2cm)

% Auto-calculate gains
elevator.calculateFeedforward();
elevator.calculatePID(0.3, 10);  % 0.3s settling, 10% overshoot

% Test with gravity
testSeq.time = linspace(0, 3, 300);
testSeq.voltage = [zeros(100,1); 8*ones(200,1)];  % Hold against gravity

% Simulate and visualize
simData = DynamicsSimulator.simulateResponse(elevator, testSeq);
VisualizationSystem.runVisualization('detailed', simData);
```

### Example 2: Flywheel Spin-Up

```matlab
% Create flywheel
flywheel = FlywheelMechanism('Shooter', 'KrakenX60');
flywheel.gearRatio = 1;         % Direct drive
flywheel.inertia_kg_m2 = 0.01;

% Calculate gains for fast response
flywheel.calculateFeedforward();
flywheel.calculatePID(0.2, 5);

% Ramp to full speed
testSeq.time = linspace(0, 2, 200);
testSeq.voltage = linspace(0, 12, 200)';  % Ramp 0→12V

% Simulate
simData = DynamicsSimulator.simulateResponse(flywheel, testSeq);

% Check if reaches target speed
VisualizationSystem.runValidator('performance', simData, flywheel);
```

### Example 3: Custom Visualization

```matlab
% Define custom plot function
function myCustomPlot(simData)
    figure;
    subplot(2,1,1);
    plot(simData.time, simData.velocity);
    title('Velocity vs Time');
    ylabel('Velocity (rad/s)');
    
    subplot(2,1,2);
    plot(simData.time, simData.current);
    title('Current Draw');
    ylabel('Current (A)');
    xlabel('Time (s)');
end

% Register your plugin
VisualizationSystem.registerVisualization('myPlot', @myCustomPlot, ...
    'Custom velocity and current plot');

% Use it
VisualizationSystem.runVisualization('myPlot', simData);
```

## Next Steps

### Learn More

1. **Run the examples**:
   ```matlab
   Example_SimpleTurret_v2()
   Example_Using_Plugin_System()
   ```

2. **Read the API Reference**: [API_REFERENCE.md](API_REFERENCE.md)

3. **Understand the architecture**: [ARCHITECTURE.md](ARCHITECTURE.md)

### Common Tasks

- **Add a new motor model**: See `src/motors/MotorCharacteristics.m`
- **Create custom mechanism**: Inherit from existing templates
- **Add visualization**: Use `VisualizationSystem.registerVisualization()`
- **System identification**: Use tools in `src/system_id/`

### Tips for Success

1. **Start simple** - Begin with provided examples
2. **Use auto-tuning** - Let the library calculate gains
3. **Validate early** - Run safety checks frequently  
4. **Iterate quickly** - Simulation is fast, experiment freely
5. **Compare to reality** - Use system ID tools to match real robot behavior

## Troubleshooting

### Issue: "Undefined function or variable"

**Solution**: Make sure you ran `startup.m` in the project root directory.

### Issue: Simulation runs slowly

**Solution**: Reduce time vector resolution or simplify the mechanism model.

### Issue: Unrealistic results

**Solution**: Check your parameter units and verify mechanism constraints.

### Issue: Can't find motor model

**Solution**: Use `MotorCharacteristics.listAvailableMotors()` to see available motors.

## Getting Help

- Check the [examples](../examples/) folder
- Read the [API Reference](API_REFERENCE.md)
- Open an issue on GitHub
- Join the discussions

---

**Ready to build amazing FRC mechanisms! 🚀**

Next: [API Reference](API_REFERENCE.md) | [Architecture Guide](ARCHITECTURE.md)
