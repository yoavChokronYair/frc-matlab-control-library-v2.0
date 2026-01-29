function VisualizationSystemInit()
    % Initialize the Visualization Plugin System
    %
    % This script registers all built-in visualizations, validators, and reporters
    % Call this once at startup:
    %   VisualizationSystemInit();
    %
    % Or add to startup.m:
    %   VisualizationSystemInit();
    
    fprintf('\n');
    fprintf('========================================\n');
    fprintf('Visualization System Initialization\n');
    fprintf('========================================\n\n');
    
    % Clear any existing plugins
    VisualizationSystem.unregisterAll();
    
    % ===== REGISTER VISUALIZATIONS =====
    
    fprintf('Registering visualizations...\n');
    
    % Standard simulation plots
    VisualizationSystem.registerVisualization('standard', ...
        @BuiltinVisualizations.plotStandardSimulation, ...
        'Standard simulation results (pos, vel, current, voltage)');
    
    VisualizationSystem.registerVisualization('detailed', ...
        @BuiltinVisualizations.plotDetailedTrajectory, ...
        'Detailed trajectory (pos, vel, accel, jerk)');
    
    VisualizationSystem.registerVisualization('energy', ...
        @BuiltinVisualizations.plotEnergyAnalysis, ...
        'Energy consumption and power analysis');
    
    VisualizationSystem.registerVisualization('phasePortrait', ...
        @BuiltinVisualizations.plotPhasePortrait, ...
        'Phase portrait (velocity vs position)');
    
    % Motor characteristics
    VisualizationSystem.registerVisualization('motorCurves', ...
        @BuiltinVisualizations.plotMotorCurves, ...
        'Motor torque-speed and power curves');
    
    % Comparison plots
    VisualizationSystem.registerVisualization('simVsReal', ...
        @BuiltinVisualizations.plotSimVsReal, ...
        'Compare simulation vs real robot data');
    
    VisualizationSystem.registerVisualization('correlation', ...
        @BuiltinVisualizations.plotCorrelation, ...
        'Scatter plot correlation between sim and real');
    
    fprintf('  ✓ 7 visualizations registered\n\n');
    
    % ===== REGISTER VALIDATORS =====
    
    fprintf('Registering validators...\n');
    
    % Performance validation
    VisualizationSystem.registerValidator('performance', ...
        @BuiltinValidators.validatePerformanceRequirements, ...
        'Validates mechanism meets performance requirements');
    
    % Model validation
    VisualizationSystem.registerValidator('modelAccuracy', ...
        @BuiltinValidators.validateModelAccuracy, ...
        'Validates simulation matches real robot data');
    
    % Trajectory validation
    VisualizationSystem.registerValidator('trajectoryTracking', ...
        @BuiltinValidators.validateTrajectory, ...
        'Validates trajectory tracking performance');
    
    % Safety validation
    VisualizationSystem.registerValidator('safety', ...
        @BuiltinValidators.validateSafety, ...
        'Validates safety limits (position, velocity, current)');
    
    fprintf('  ✓ 4 validators registered\n\n');
    
    % ===== SUMMARY =====
    
    fprintf('========================================\n');
    fprintf('Initialization Complete!\n');
    fprintf('========================================\n\n');
    
    fprintf('Usage:\n');
    fprintf('  List plugins:      VisualizationSystem.printRegistry();\n');
    fprintf('  Run visualization: VisualizationSystem.runVisualization(name, simData);\n');
    fprintf('  Run validator:     results = VisualizationSystem.runValidator(name, simData, mechanism);\n');
    fprintf('  Add new plugin:    VisualizationSystem.registerVisualization(name, func, desc);\n\n');
end
