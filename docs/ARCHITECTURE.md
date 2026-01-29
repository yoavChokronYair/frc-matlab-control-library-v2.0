# Architecture Guide

Deep dive into the design and implementation of the FRC MATLAB Control Library.

## Table of Contents

- [Design Philosophy](#design-philosophy)
- [System Architecture](#system-architecture)
- [Core Components](#core-components)
- [Data Flow](#data-flow)
- [Extension Points](#extension-points)
- [Performance Considerations](#performance-considerations)

---

## Design Philosophy

### Goals

1. **Easy to use** - Minimal code to get results
2. **Flexible** - Support various mechanism types
3. **Accurate** - Physics-based simulation
4. **Extensible** - Plugin architecture for customization
5. **Fast** - Efficient simulation for rapid iteration

### Design Principles

- **Convention over configuration** - Sensible defaults
- **Separation of concerns** - Clear module boundaries
- **Composition over inheritance** - Flexible object composition
- **Fail fast** - Early error detection with clear messages

---

## System Architecture

### High-Level Overview

```
┌─────────────────────────────────────────────────────────┐
│                     User Interface                      │
│          (MATLAB Scripts / Command Window)              │
└────────────────────┬────────────────────────────────────┘
                     │
         ┌───────────┴──────────────┐
         ▼                          ▼
┌─────────────────┐        ┌─────────────────┐
│   Mechanism     │        │  Visualization  │
│   Templates     │        │     System      │
└────────┬────────┘        └────────┬────────┘
         │                          │
         ▼                          ▼
┌─────────────────┐        ┌─────────────────┐
│    Control      │        │    Plugins      │
│   Synthesis     │        │   (Built-in &   │
└────────┬────────┘        │    Custom)      │
         │                 └─────────────────┘
         ▼
┌─────────────────┐
│   Simulation    │
│     Engine      │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  Motor Models   │
└─────────────────┘
```

### Layer Architecture

**Layer 1: User Interface**
- MATLAB command window
- User scripts
- Example files

**Layer 2: Templates & Abstractions**
- `SimpleRotationalMechanism`
- `ElevatorMechanism`
- `FlywheelMechanism`
- `SwerveDriveSystem`

**Layer 3: Core Services**
- Control synthesis (PID, Feedforward)
- Simulation engine
- System identification
- Visualization system

**Layer 4: Foundation**
- Motor models
- Mathematical utilities
- Constants and conversions

---

## Core Components

### 1. Motor Models (`src/motors/`)

#### Purpose
Provide accurate motor characteristics for simulation.

#### Structure

```matlab
classdef KrakenX40Model
    properties
        stallTorque_Nm      % Stall torque
        freeSpeed_rpm       % Free speed
        stallCurrent_A      % Stall current
        freeCurrent_A       % Free current
        resistance_ohm      % Winding resistance
        Kv                  % Velocity constant (V/rad/s)
        Kt                  % Torque constant (Nm/A)
    end
    
    methods
        function motor = KrakenX40Model()
            % Initialize from manufacturer specs
        end
        
        function torque = getTorque(obj, velocity, voltage)
            % Calculate torque given velocity and voltage
            % T = Kt * (V - Kv*omega) / R
        end
    end
end
```

#### Key Equations

**Torque-Current Relationship:**
```
τ = Kt × I
```

**Back-EMF:**
```
V_emf = Kv × ω
```

**Motor Current:**
```
I = (V_applied - V_emf) / R
```

### 2. Mechanism Templates (`src/templates/`)

#### Purpose
Provide ready-to-use mechanism models with automatic parameter calculation.

#### Base Structure

```matlab
classdef SimpleRotationalMechanism
    properties
        % Physical parameters
        motor               % Motor model object
        gearRatio          % Gear reduction
        inertia_kg_m2      % Moment of inertia
        friction_Nm        % Friction torque
        
        % Controller gains
        kP, kI, kD         % PID gains
        kS, kV, kA, kG     % Feedforward gains
    end
    
    methods
        function obj = SimpleRotationalMechanism(name, motorType)
            % Constructor
        end
        
        function calculateFeedforward(obj)
            % Auto-calculate FF gains from physics
        end
        
        function calculatePID(obj, settlingTime, overshoot)
            % Auto-tune PID controller
        end
        
        function sys = getStateSpaceModel(obj)
            % Return state-space for analysis
        end
    end
end
```

#### State-Space Representation

For a rotational mechanism:

**State vector:** `x = [θ, ω]ᵀ` (position, velocity)

**State equations:**
```
θ̇ = ω
ω̇ = (τ_motor - τ_friction) / J
```

**In matrix form:**
```
ẋ = Ax + Bu
y = Cx + Du

where:
A = [0    1  ]
    [0  -b/J ]

B = [  0   ]
    [Kt/JR ]

C = [1  0]
D = 0
```

### 3. Simulation Engine (`src/simulation/`)

#### DynamicsSimulator

**Purpose:** Numerically integrate equations of motion.

**Algorithm:** Uses MATLAB's `ode45` (Runge-Kutta 4-5) for integration.

**Process:**
1. Convert mechanism to state-space
2. Define derivative function
3. Apply motor saturation limits
4. Integrate over time
5. Post-process results

**Code Structure:**

```matlab
classdef DynamicsSimulator
    methods (Static)
        function simData = simulateResponse(mechanism, testSequence)
            % Setup state-space model
            sys = mechanism.getStateSpaceModel();
            
            % Define derivative function
            derivFcn = @(t, x) simulateStep(t, x, mechanism, testSeq);
            
            % Integrate
            [t, x] = ode45(derivFcn, testSequence.time, [0; 0]);
            
            % Package results
            simData = packageResults(t, x, mechanism);
        end
        
        function dx = simulateStep(t, x, mechanism, testSeq)
            % Get voltage at current time
            voltage = interp1(testSeq.time, testSeq.voltage, t);
            
            % Apply motor limits
            voltage = saturate(voltage, -12, 12);
            
            % Calculate motor torque
            torque = mechanism.motor.getTorque(x(2), voltage);
            
            % Calculate acceleration
            accel = (torque - friction) / inertia;
            
            % Return derivatives
            dx = [x(2); accel];
        end
    end
end
```

### 4. Control Synthesis (`src/control/`)

#### PID Tuning Algorithm

**Method:** Frequency domain loop shaping

**Steps:**
1. Get open-loop transfer function
2. Design compensator to meet specs:
   - Phase margin > 45° (low overshoot)
   - Bandwidth for settling time
   - High DC gain for steady-state
3. Convert to PID form
4. Verify in simulation

**Transfer Function:**
```
         Kp s² + Ki s + Kd
C(s) = ──────────────────
                s
```

#### Feedforward Calculation

**Derivation from physics:**

For a rotational system:
```
τ = J·α + b·ω + τ_friction

Solving for voltage:
V = (τ·R)/Kt + Kv·ω

Expanding:
V = (J·R/Kt)·α + (b·R/Kt + Kv)·ω + (τ_f·R/Kt)

Therefore:
kA = J·R/Kt        (acceleration feedforward)
kV = b·R/Kt + Kv   (velocity feedforward)  
kS = τ_f·R/Kt      (static friction)
```

### 5. Visualization System (`src/plugins/`)

#### Plugin Architecture

**Design Pattern:** Registry pattern

**Structure:**
```matlab
classdef VisualizationSystem
    properties (Access = private)
        visualizations  % Map of name → function
        validators      % Map of name → function
    end
    
    methods (Static)
        function registerVisualization(name, fcn, description)
            % Add to registry
        end
        
        function runVisualization(name, simData)
            % Look up and execute
        end
        
        function listPlugins()
            % Display available plugins
        end
    end
end
```

**Benefits:**
- No library modification needed
- Runtime extensibility
- Clean separation of concerns
- Easy to add custom visualizations

---

## Data Flow

### Typical Simulation Flow

```
1. User creates mechanism
   ↓
2. Set physical parameters
   ↓
3. Call calculateFeedforward()
   → Uses motor model + mechanism parameters
   → Calculates kS, kV, kA from physics
   ↓
4. Call calculatePID()
   → Gets state-space model
   → Runs frequency-domain optimization
   → Returns kP, kI, kD
   ↓
5. Create test sequence
   ↓
6. Call DynamicsSimulator.simulateResponse()
   → Packages mechanism into derivative function
   → Calls ode45 integrator
   → Returns time-series data
   ↓
7. Visualize with VisualizationSystem
   → Looks up plugin function
   → Executes with simData
   → Displays plots
   ↓
8. Validate with VisualizationSystem
   → Looks up validator function
   → Checks requirements
   → Returns pass/fail + metrics
```

### Data Structures

**Test Sequence:**
```matlab
testSequence.time = [t1, t2, ..., tn];
testSequence.voltage = [v1, v2, ..., vn];
```

**Simulation Data:**
```matlab
simData.time          % Time vector
simData.position      % Position trajectory
simData.velocity      % Velocity trajectory  
simData.acceleration  % Acceleration
simData.current       % Current draw
simData.voltage       % Applied voltage
simData.torque        % Motor torque
simData.power         % Power consumption
```

---

## Extension Points

### Adding a New Motor

1. Create `NewMotorModel.m` in `src/motors/`
2. Define specifications from datasheet
3. Calculate Kv and Kt from specs
4. Register in motor library

### Adding a New Mechanism Type

1. Create class in `src/templates/`
2. Inherit from appropriate base or start fresh
3. Implement state-space model
4. Add mechanism-specific calculations
5. Document thoroughly

### Adding a Custom Visualization

```matlab
function myCustomViz(simData)
    % Your plotting code
end

VisualizationSystem.registerVisualization('myViz', @myCustomViz, ...
    'Description of what it shows');
```

### Adding a Custom Validator

```matlab
function results = myValidator(simData, mechanism)
    results.passed = checkSomething(simData);
    results.message = 'Validation result';
    results.metrics = calculateMetrics(simData);
end

VisualizationSystem.registerValidator('myCheck', @myValidator, ...
    'What this validates');
```

---

## Performance Considerations

### Simulation Speed

**Factors affecting speed:**
1. Time vector resolution (more points = slower)
2. Mechanism complexity (more equations = slower)
3. Integrator tolerance settings

**Optimization tips:**
- Use adaptive time steps (default in ode45)
- Reduce output resolution if not needed
- Use simpler mechanism models when appropriate
- Cache repeated calculations

### Memory Usage

**Typical memory footprint:**
- Small simulation (1000 points): ~1 MB
- Large simulation (100,000 points): ~100 MB

**Best practices:**
- Clear old simData when done
- Use sparse sampling for long simulations
- Stream data to disk for very long runs

### Numerical Stability

**Potential issues:**
- Very high gear ratios (>200:1)
- Very low friction
- Extreme parameter values

**Solutions:**
- Scale state variables appropriately
- Add small friction term (numerical damping)
- Check condition numbers of matrices

---

## Testing Strategy

### Unit Tests
Test individual components in isolation:
- Motor torque calculations
- State-space conversions
- Controller calculations

### Integration Tests
Test component interactions:
- Full simulation pipeline
- Plugin registration system
- Parameter propagation

### Validation Tests
Compare against known solutions:
- Analytical solutions for simple cases
- Manufacturer motor curves
- Published FRC data

---

## Future Architecture Considerations

### Planned Enhancements

1. **Parallel simulation** - Batch parameter sweeps
2. **Code generation** - Export to robot code
3. **Real-time mode** - Hardware-in-the-loop testing
4. **Web dashboard** - Browser-based visualization
5. **ROS integration** - Connect to ROS/Gazebo

### Scalability

The architecture supports:
- Multiple mechanisms in single model
- Complex multi-body systems
- Custom physics models
- External force models

---

For implementation details of specific components, see the source code with inline documentation.
