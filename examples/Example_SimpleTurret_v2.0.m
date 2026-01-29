%% Simple Turret Example - v2.0 (Updated with Plugin System)
%
% This example demonstrates the v2.0 approach using the plugin system
% for visualizations and validators instead of calling methods directly.
%
% Changes from v1.0:
% - Replaced DynamicsSimulator.plotSimulation() with VisualizationSystem plugin
% - Replaced DynamicsSimulator.validatePerformance() with VisualizationSystem plugin
% - All simulation code remains unchanged
%
% Run this example:
%   Example_SimpleTurret_v2();

function Example_SimpleTurret_v2()
    
    fprintf('\n');
    fprintf('========================================\n');
    fprintf('Simple Turret Example (v2.0)\n');
    fprintf('========================================\n\n');
    
    %% STEP 1: Initialize Plugin System
    % NEW IN v2.0: Initialize plugins at startup
    VisualizationSystemInit();
    
    %% STEP 2: Create Mechanism
    fprintf('Creating turret mechanism...\n');
    
    turret = SimpleRotationalMechanism('Competition Turret', 'KrakenX40');
    turret.numMotors = 1;
    turret.gearRatio = 10.0;
    turret.inertia_kg_m2 = 0.05;
    turret.mass_kg = 2.0;
    turret.staticFriction_Nm = 0.5;
    turret.viscousFriction = 0.01;
    turret.setLimits(-pi, pi, 12, 60);
    
    %% STEP 3: Calculate Controllers
    fprintf('Calculating feedforward and PID gains...\n');
    turret.calculateFeedforward();
    turret.calculatePID(0.5, 5);  % 0.5s settling, 5% overshoot
    
    fprintf('\nController Gains:\n');
    fprintf('  Feedforward:\n');
    fprintf('    kS = %.4f V\n', turret.feedforward.kS);
    fprintf('    kV = %.4f V⋅s/rad\n', turret.feedforward.kV);
    fprintf('    kA = %.4f V⋅s²/rad\n', turret.feedforward.kA);
    fprintf('  PID:\n');
    fprintf('    kP = %.4f\n', turret.pidGains.kP);
    fprintf('    kI = %.4f\n', turret.pidGains.kI);
    fprintf('    kD = %.4f\n', turret.pidGains.kD);
    
    %% STEP 4: Define Test Sequence
    fprintf('\nCreating test sequence...\n');
    
    % Simple voltage step test
    testGen = AutomatedTestSequences();
    [time, voltage] = testGen.generateMultiStepResponse([3, 6, 9], 2.0);
    
    testSeq.time = time;
    testSeq.voltage = voltage;
    
    %% STEP 5: Run Simulation
    fprintf('Running simulation...\n');
    
    simData = DynamicsSimulator.simulateResponse(turret, testSeq);
    
    fprintf('Simulation Results:\n');
    fprintf('  Duration: %.2f s\n', simData.time(end));
    fprintf('  Max velocity: %.2f rad/s\n', max(abs(simData.velocity)));
    fprintf('  Max current: %.2f A\n', max(abs(simData.current)));
    fprintf('  Max position: %.2f rad (%.1f deg)\n', ...
        max(simData.position), max(simData.position)*180/pi);
    
    %% STEP 6: Visualize Results (v2.0 Plugin System)
    % CHANGED IN v2.0: Use VisualizationSystem plugins instead of direct methods
    
    fprintf('\nGenerating visualizations...\n');
    
    % Plot 1: Standard simulation view
    % OLD v1.0: DynamicsSimulator.plotSimulation(simData);
    % NEW v2.0: Use plugin system
    VisualizationSystem.runVisualization('standard', simData, ...
        'title', 'Turret Step Response');
    
    % Plot 2: Energy analysis
    % OLD v1.0: No built-in method
    % NEW v2.0: Use plugin system
    VisualizationSystem.runVisualization('energy', simData, ...
        'title', 'Turret Energy Consumption');
    
    % Plot 3: Detailed trajectory
    % NEW v2.0: Additional visualization via plugin
    VisualizationSystem.runVisualization('detailed', simData, ...
        'title', 'Detailed Trajectory Analysis');
    
    %% STEP 7: Validate Performance
    fprintf('\nValidating performance requirements...\n');
    
    requirements.maxVelocity_rad_s = 15.0;
    requirements.maxAcceleration_rad_s2 = 70.0;
    
    % OLD v1.0: DynamicsSimulator.validatePerformance(turret, requirements);
    % NEW v2.0: Use plugin system
    results = VisualizationSystem.runValidator('performance', simData, turret, ...
        'requirements', requirements);
    
    %% STEP 8: Validate Safety
    fprintf('Validating safety limits...\n');
    
    % NEW v2.0: Safety validation via plugin
    safetyResults = VisualizationSystem.runValidator('safety', simData, turret);
    
    %% SUMMARY
    fprintf('\n');
    fprintf('========================================\n');
    fprintf('Example Complete\n');
    fprintf('========================================\n\n');
    
    fprintf('Key Differences from v1.0:\n');
    fprintf('✓ Used VisualizationSystemInit() to register plugins\n');
    fprintf('✓ Used VisualizationSystem.runVisualization() instead of direct methods\n');
    fprintf('✓ Used VisualizationSystem.runValidator() instead of direct methods\n');
    fprintf('✓ Simulation code (DynamicsSimulator) unchanged\n');
    fprintf('✓ All other calculations (controller tuning) unchanged\n');
    fprintf('✓ Can easily add new visualizations without modifying library\n\n');
    
    fprintf('Next Steps:\n');
    fprintf('1. Copy 4 plugin files to library\n');
    fprintf('2. Call VisualizationSystemInit() at startup\n');
    fprintf('3. Replace all plotting calls with VisualizationSystem plugins\n');
    fprintf('4. Replace all validation calls with VisualizationSystem plugins\n');
    fprintf('5. Add custom visualizations/validators as needed\n\n');
    
end
