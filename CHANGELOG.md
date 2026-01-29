# Changelog

All notable changes to the FRC MATLAB Control Library will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.0.0] - 2025-01-29

### Added
- **Plugin System**: Extensible visualization and validation framework
  - 7 built-in visualizations
  - 4 built-in validators
  - Easy custom plugin registration
- **New Motor Models**: KrakenX40 and KrakenX60
- **SwerveDriveSystem**: Complete swerve drive simulation template
- **MotionMagicOptimizer**: Optimal motion profile generation
- **Organized folder structure**: Clean separation of components
  - `src/motors/` - Motor models
  - `src/templates/` - Mechanism templates
  - `src/control/` - Control synthesis
  - `src/simulation/` - Physics simulation
  - `src/system_id/` - System identification
  - `src/plugins/` - Visualization system
- **Comprehensive documentation**:
  - Getting Started guide
  - API Reference
  - Architecture guide
- **Examples**: Working examples for quick learning
- **MIT License**: Open source license
- **Contributing guidelines**: Clear contribution process

### Changed
- Refactored code organization into logical folders
- Simplified startup process with automatic path configuration
- Improved documentation structure
- Streamlined README for clarity

### Fixed
- Path handling for cross-platform compatibility
- Plugin initialization robustness

## [1.0.0] - Previous Version

### Features
- Basic mechanism templates (turret, elevator, flywheel)
- PID auto-tuning
- Feedforward calculation
- Physics-based simulation
- Motor models
- System identification tools
- Basic visualization

---

## Upgrade Guide

### From v1.0 to v2.0

1. **Update file references**: Files are now organized in subdirectories
   - Motor models: `src/motors/`
   - Templates: `src/templates/`
   - etc.

2. **Use new startup script**: Run `startup` from project root

3. **Adopt plugin system** (optional):
   ```matlab
   % Old way (still works):
   plot(simData.time, simData.position);
   
   % New way (recommended):
   VisualizationSystem.runVisualization('standard', simData);
   ```

4. **Update motor model references**:
   ```matlab
   % Old:
   motor = getMotor('KrakenX40');
   
   % New:
   motor = KrakenX40Model();
   ```

All v1.0 code remains compatible with v2.0!
