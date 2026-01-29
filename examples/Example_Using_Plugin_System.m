%% Complete Example: Using the Plugin System
% 
% This example demonstrates the new v2.0 plugin-based approach
% Shows how to:
% 1. Initialize the plugin system
% 2. Create a mechanism
% 3. Simulate dynamics
% 4. Use built-in visualizations and validators
% 5. Add custom visualizations and validators
%
% Run this file:
%   Example_Using_Plugin_System()

function Example_Using_Plugin_System()
    
    %% STEP 1: Initialize Plugin System
    fprintf('\n');
    fprintf('========================================\n');
    fprintf('Plugin System Example\n');
    fprintf('========================================\n\n');
    
    % Initialize all built-in plugins
    VisualizationSystemInit();
    
    % See what's available
    fprintf('Available visualizations:\n');
    visNames = VisualizationSystem.listVisualizations();
    for i = 1:length(visNames)
        fprintf('  - %s\n', visNames{i});
    end
    
    fprintf('\nAvailable validators:\n');
    valNames = VisualizationSystem.listValidators();
    for i = 1:length(valNames)
        fprintf('  - %s\n', valNames{i});
    end
    
    %% STEP 2: Create Mechanism
    fprintf('\n--- Creating Mechanism ---\n');
    
    % Create simple rotational mechanism
    turret = SimpleRotationalMechanism('Demo Turret', 'KrakenX40');
    
    % Set mechanical properties (from CAD or estimates)
    turret.numMotors = 1;
    turret.gearRatio = 10.0;
    turret.inertia_kg_m2 = 0.05;
    turret.mass_kg = 2.0;
    turret.staticFriction_Nm = 0.5;
    turret.viscousFriction = 0.01;
    
    % Set limits
    turret.setLimits(-pi, pi, 12, 60);
    
    % Calculate controllers
    fprintf('Calculating feedforward gains...\n');
    turret.calculateFeedforward();
    
    fprintf('Calculating PID gains...\n');
    turret.calculatePID(0.5, 5);  % 0.5s settling, 5% overshoot
    
    fprintf('\nMechanism Setup Complete:\n');
    fprintf('  Motor: KrakenX40\n');
    fprintf('  Gear Ratio: %.1f\n', turret.gearRatio);
    fprintf('  Inertia: %.4f kg⋅m²\n', turret.inertia_kg_m2);
    fprintf('  Friction: %.2f N⋅m\n', turret.staticFriction_Nm);
    
    %% STEP 3: Create Test Sequence
    fprintf('\n--- Creating Test Sequence ---\n');
    
    % Simple step response test
    testSeq.time = linspace(0, 3, 300);
    testSeq.voltage = [zeros(150,1); 6*ones(150,1)];  % 0V then 6V step
    
    fprintf('Test: 0V → 6V step at t=1.5s\n');
    
    %% STEP 4: Run Simulation
    fprintf('\n--- Running Simulation ---\n');
    
    simData = DynamicsSimulator.simulateResponse(turret, testSeq);
    
    fprintf('Simulation complete: %.2f seconds\n', simData.time(end));
    fprintf('Max velocity: %.2f rad/s\n', max(abs(simData.velocity)));
    fprintf('Max current: %.2f A\n', max(abs(simData.current)));
    
    %% STEP 5: Use Built-in Visualizations
    fprintf('\n--- Using Built-in Visualizations ---\n');
    
    % Plot 1: Standard simulation results
    fprintf('\nPlotting: standard simulation view\n');
    VisualizationSystem.runVisualization('standard', simData, ...
        'title', 'Step Response to 6V');
    
    % Plot 2: Energy analysis
    fprintf('Plotting: energy analysis\n');
    VisualizationSystem.runVisualization('energy', simData, ...
        'title', 'Energy Consumption');
    
    % Plot 3: Detailed trajectory
    fprintf('Plotting: detailed trajectory\n');
    VisualizationSystem.runVisualization('detailed', simData, ...
        'title', 'Detailed Motion Profile');
    
    % Plot 4: Phase portrait
    fprintf('Plotting: phase portrait\n');
    VisualizationSystem.runVisualization('phasePortrait', simData, ...
        'title', 'Velocity vs Position');
    
    %% STEP 6: Use Built-in Validators
    fprintf('\n--- Using Built-in Validators ---\n');
    
    % Validator 1: Performance requirements
    fprintf('\nValidating performance...\n');
    requirements.maxVelocity_rad_s = 15.0;
    requirements.maxAcceleration_rad_s2 = 70.0;
    
    results = VisualizationSystem.runValidator('performance', simData, turret, ...
        'requirements', requirements);
    
    % Validator 2: Safety limits
    fprintf('Validating safety...\n');
    safetyResults = VisualizationSystem.runValidator('safety', simData, turret);
    
    %% STEP 7: Add Custom Visualization
    fprintf('\n--- Adding Custom Visualization ---\n');
    
    % Register a custom plot
    VisualizationSystem.registerVisualization('customExample', ...
        @plotVoltageVsPosition, 'Voltage vs Position (custom plot)');
    
    % Use the custom plot
    fprintf('Running custom visualization...\n');
    VisualizationSystem.runVisualization('customExample', simData);
    
    %% STEP 8: Add Custom Validator
    fprintf('\n--- Adding Custom Validator ---\n');
    
    % Register custom validator
    VisualizationSystem.registerValidator('customExample', ...
        @validateResponseTime, 'Validates 90% settling time');
    
    % Run custom validator
    customResults = VisualizationSystem.runValidator('customExample', simData, turret);
    
    %% STEP 9: Export Results
    fprintf('\n--- Export Results ---\n');
    
    fprintf('\nFeedforward Gains:\n');
    fprintf('  kS (static friction): %.4f V\n', turret.feedforward.kS);
    fprintf('  kV (velocity):        %.4f V⋅s/rad\n', turret.feedforward.kV);
    fprintf('  kA (acceleration):    %.4f V⋅s²/rad\n', turret.feedforward.kA);
    
    fprintf('\nPID Gains:\n');
    fprintf('  kP (proportional): %.4f\n', turret.pidGains.kP);
    fprintf('  kI (integral):     %.4f\n', turret.pidGains.kI);
    fprintf('  kD (derivative):   %.4f\n', turret.pidGains.kD);
    
    %% STEP 10: Batch Operations
    fprintf('\n--- Batch Operations ---\n');
    
    fprintf('\nRunning all visualizations:\n');
    VisualizationSystem.plotAllVisualizations(simData);
    
    fprintf('\nRunning all validators:\n');
    allValidators = VisualizationSystem.runAllValidators(simData, turret);
    
    %% Summary
    fprintf('\n');
    fprintf('========================================\n');
    fprintf('Example Complete!\n');
    fprintf('========================================\n\n');
    
    fprintf('Summary:\n');
    fprintf('✓ Created mechanism with auto-tuned controllers\n');
    fprintf('✓ Simulated physics response\n');
    fprintf('✓ Used 4 built-in visualizations\n');
    fprintf('✓ Used 2 built-in validators\n');
    fprintf('✓ Added custom visualization\n');
    fprintf('✓ Added custom validator\n');
    fprintf('✓ Exported gains for robot\n');
    fprintf('✓ Ran batch operations\n\n');
    
    fprintf('Key Insight: All this WITHOUT modifying any library files!\n');
    fprintf('Just register plugins and use them. That''s it.\n\n');
    
end

%% ========== CUSTOM VISUALIZATION FUNCTION ==========

function plotVoltageVsPosition(simData, varargin)
    % Custom visualization: Voltage vs Position
    
    figure('Name', 'Voltage vs Position (Custom)');
    
    % Create plot
    plot(simData.position, simData.voltage, 'b.-', 'LineWidth', 1.5, 'MarkerSize', 4);
    
    grid on;
    xlabel('Position (rad)');
    ylabel('Applied Voltage (V)');
    title('Voltage vs Position (Custom Plot)');
    
    % Add annotations
    hold on;
    scatter(simData.position(1), simData.voltage(1), 100, 'g', 'filled', ...
        'DisplayName', 'Start');
    scatter(simData.position(end), simData.voltage(end), 100, 'r', 'filled', ...
        'DisplayName', 'End');
    legend('show');
    
end

%% ========== CUSTOM VALIDATOR FUNCTION ==========

function results = validateResponseTime(simData, mechanism, varargin)
    % Custom validator: Check 90% settling time
    
    results = struct();
    results.mechanismName = mechanism.mechanismName;
    
    % Find final value
    finalVel = simData.velocity(end);
    targetVel = finalVel * 0.9;  % 90% of final
    
    % Find when velocity reaches 90%
    aboveThreshold = abs(simData.velocity) >= abs(targetVel);
    idx90 = find(aboveThreshold, 1, 'first');
    
    if ~isempty(idx90)
        time90 = simData.time(idx90);
        results.time90 = time90;
        results.passed = time90 < 1.0;  % Must be <1s
        
        fprintf('\n=== Custom Validator: Response Time ===\n');
        fprintf('Time to 90%% velocity: %.3f s\n', time90);
        fprintf('Requirement: < 1.0 s\n');
        
        if results.passed
            fprintf('Status: ✓ PASS\n\n');
        else
            fprintf('Status: ✗ FAIL\n\n');
        end
    else
        results.time90 = inf;
        results.passed = false;
        fprintf('Status: ✗ FAIL - Never reached 90%%\n\n');
    end
    
end
