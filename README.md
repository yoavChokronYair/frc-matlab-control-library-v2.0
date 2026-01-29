# FRC MATLAB Control Library

A comprehensive MATLAB toolbox for designing, simulating, and tuning control systems for FIRST Robotics Competition (FRC) mechanisms.

[![MATLAB](https://img.shields.io/badge/MATLAB-R2019a+-blue.svg)](https://www.mathworks.com/products/matlab.html)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

## Features

- **Motor Models**: Pre-configured models for Kraken X40, Kraken X60, and other FRC motors
- **Mechanism Templates**: Ready-to-use templates for common mechanisms (turrets, elevators, flywheels, swerve drive)
- **Auto-Tuning**: Automatic PID and feedforward controller synthesis
- **Physics Simulation**: High-fidelity dynamics simulator with motor saturation and disturbances
- **System Identification**: Tools for extracting system parameters from real robot data
- **Visualization**: Extensible plugin system with 7 built-in visualizations and 4 validators
- **Examples**: Working examples to get you started quickly

## Quick Start

### Installation

1. Clone this repository:
```bash
git clone https://github.com/yourusername/frc-matlab-control-library.git
cd frc-matlab-control-library
```

2. Open MATLAB and navigate to the project directory

3. Run the initialization script:
```matlab
startup
```

### Basic Example

Create and simulate a simple turret mechanism:

```matlab
% Create a turret mechanism using Kraken X40 motor
turret = SimpleRotationalMechanism('MyTurret', 'KrakenX40');
turret.gearRatio = 10;
turret.inertia_kg_m2 = 0.05;

% Auto-calculate feedforward gains
turret.calculateFeedforward();

% Auto-tune PID controller (0.5s settling, 5% overshoot)
turret.calculatePID(0.5, 5);

% Generate test sequence
testSeq.time = linspace(0, 2, 200);
testSeq.voltage = [zeros(100,1); 6*ones(100,1)];

% Run simulation
simData = DynamicsSimulator.simulateResponse(turret, testSeq);

% Visualize results
VisualizationSystem.runVisualization('standard', simData);

% Validate safety
results = VisualizationSystem.runValidator('safety', simData, turret);
```

## Project Structure

```
frc-matlab-control-library/
├── src/
│   ├── motors/              # Motor models and characteristics
│   │   ├── KrakenX40Model.m
│   │   ├── KrakenX60Model.m
│   │   └── MotorCharacteristics.m
│   │
│   ├── templates/           # Mechanism templates
│   │   ├── SimpleRotationalMechanism.m
│   │   ├── ElevatorMechanism.m
│   │   ├── FlywheelMechanism.m
│   │   ├── SwerveDriveSystem.m
│   │   └── SwerveModuleModel.m
│   │
│   ├── control/             # Controller synthesis
│   │   ├── PIDTuner.m
│   │   ├── FeedforwardCalculator.m
│   │   └── MotionMagicOptimizer.m
│   │
│   ├── simulation/          # Simulation engine
│   │   ├── DynamicsSimulator.m
│   │   └── PerformanceValidator.m
│   │
│   ├── system_id/           # System identification
│   │   ├── AutomatedTestSequences.m
│   │   ├── ParameterExtraction.m
│   │   └── ModelValidation.m
│   │
│   └── plugins/             # Visualization & validation plugins
│       ├── VisualizationSystem.m
│       ├── BuiltinVisualizations.m
│       ├── BuiltinValidators.m
│       └── VisualizationSystemInit.m
│
├── examples/                # Working examples
│   ├── Example_SimpleTurret_v2.0.m
│   └── Example_Using_Plugin_System.m
│
├── docs/                    # Documentation
│   ├── GETTING_STARTED.md
│   ├── API_REFERENCE.md
│   └── ARCHITECTURE.md
│
├── startup.m               # Initialization script
├── README.md              # This file
├── LICENSE                # MIT License
└── CONTRIBUTING.md        # Contribution guidelines
```

## Documentation

- **[Getting Started Guide](docs/GETTING_STARTED.md)** - Step-by-step tutorial
- **[API Reference](docs/API_REFERENCE.md)** - Complete function documentation
- **[Architecture Guide](docs/ARCHITECTURE.md)** - System design and internals

## Supported Mechanisms

| Mechanism Type | Template | Description |
|----------------|----------|-------------|
| Rotational | `SimpleRotationalMechanism` | Turrets, intakes, spinners |
| Linear | `ElevatorMechanism` | Elevators, vertical lifts |
| Flywheel | `FlywheelMechanism` | High-speed rotation systems |
| Swerve Drive | `SwerveDriveSystem` | Complete swerve drive simulation |

## Requirements

- MATLAB R2019a or later
- Control System Toolbox (recommended)
- Signal Processing Toolbox (optional, for advanced features)

## Contributing

We welcome contributions! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Developed for FIRST Robotics Competition teams
- Built with physics-based motor models and industry-standard control techniques
- Special thanks to the FRC community for feedback and testing

## Support

- **Issues**: Please report bugs and feature requests via [GitHub Issues](https://github.com/yourusername/frc-matlab-control-library/issues)
- **Discussions**: Join our [Discussions](https://github.com/yourusername/frc-matlab-control-library/discussions) for questions and ideas
- **Documentation**: Check the [docs](docs/) folder for detailed guides
