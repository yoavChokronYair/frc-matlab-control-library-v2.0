classdef PerformanceValidator < handle
    % Validates mechanism performance against requirements
    % Checks if design meets specifications
    
    methods (Static)
        function results = validatePerformance(mechanism, requirements, simData)
            % Validate mechanism meets performance requirements
            %
            % Inputs:
            %   mechanism: Mechanism model
            %   requirements: struct with performance specs
            %   simData: Simulation data (optional, generates if not provided)
            %
            % Outputs:
            %   results: struct with pass/fail for each requirement
            
            fprintf('\n=== Performance Validation ===\n\n');
            
            results = struct();
            results.mechanismName = mechanism.mechanismName;
            results.timestamp = datetime('now');
            results.allPassed = true;
            
            % Generate simulation if not provided
            if nargin < 3
                % Create test trajectory
                testGen = AutomatedTestSequences();
                [time, voltage] = testGen.generateMultiStepResponse([6, 12], 2.0);
                testSeq.time = time;
                testSeq.voltage = voltage;
                simData = DynamicsSimulator.simulateResponse(mechanism, testSeq);
            end
            
            % Check maximum velocity
            if isfield(requirements, 'maxVelocity_rad_s')
                actualMaxVel = max(abs(simData.velocity));
                reqMaxVel = requirements.maxVelocity_rad_s;
                
                passed = actualMaxVel >= reqMaxVel * 0.95;  % 95% tolerance
                results.maxVelocity.required = reqMaxVel;
                results.maxVelocity.actual = actualMaxVel;
                results.maxVelocity.passed = passed;
                results.maxVelocity.margin = (actualMaxVel / reqMaxVel - 1) * 100;
                
                fprintf('Max Velocity:\n');
                fprintf('  Required: %.2f rad/s\n', reqMaxVel);
                fprintf('  Actual: %.2f rad/s\n', actualMaxVel);
                fprintf('  Status: %s (%.1f%% margin)\n', ...
                    PerformanceValidator.passFailStr(passed), ...
                    results.maxVelocity.margin);
                fprintf('\n');
                
                if ~passed
                    results.allPassed = false;
                end
            end
            
            % Check maximum acceleration
            if isfield(requirements, 'maxAcceleration_rad_s2')
                if isfield(simData, 'acceleration')
                    actualMaxAccel = max(abs(simData.acceleration));
                else
                    % Calculate from velocity
                    dt = mean(diff(simData.time));
                    accel = diff(simData.velocity) / dt;
                    actualMaxAccel = max(abs(accel));
                end
                
                reqMaxAccel = requirements.maxAcceleration_rad_s2;
                passed = actualMaxAccel >= reqMaxAccel * 0.90;  % 90% tolerance
                
                results.maxAcceleration.required = reqMaxAccel;
                results.maxAcceleration.actual = actualMaxAccel;
                results.maxAcceleration.passed = passed;
                results.maxAcceleration.margin = (actualMaxAccel / reqMaxAccel - 1) * 100;
                
                fprintf('Max Acceleration:\n');
                fprintf('  Required: %.2f rad/s²\n', reqMaxAccel);
                fprintf('  Actual: %.2f rad/s²\n', actualMaxAccel);
                fprintf('  Status: %s (%.1f%% margin)\n', ...
                    PerformanceValidator.passFailStr(passed), ...
                    results.maxAcceleration.margin);
                fprintf('\n');
                
                if ~passed
                    results.allPassed = false;
                end
            end
            
            % Check current limit compliance
            if isfield(simData, 'current')
                maxCurrent = max(abs(simData.current));
                currentLimit = mechanism.motorModel.MAX_CONTINUOUS_CURRENT_A;
                
                passed = maxCurrent <= currentLimit * 1.05;  % 5% tolerance
                
                results.currentLimit.limit = currentLimit;
                results.currentLimit.peak = maxCurrent;
                results.currentLimit.passed = passed;
                results.currentLimit.margin = (currentLimit / maxCurrent - 1) * 100;
                
                fprintf('Current Limit:\n');
                fprintf('  Limit: %.0f A\n', currentLimit);
                fprintf('  Peak: %.2f A\n', maxCurrent);
                fprintf('  Status: %s (%.1f%% margin)\n', ...
                    PerformanceValidator.passFailStr(passed), ...
                    results.currentLimit.margin);
                fprintf('\n');
                
                if ~passed
                    results.allPassed = false;
                    fprintf('  WARNING: Current limit exceeded!\n\n');
                end
            end
            
            % Check settling time
            if isfield(requirements, 'settlingTime_s')
                % Find when position reaches and stays within 2% of final value
                finalPos = simData.position(end);
                tolerance = 0.02 * abs(finalPos);
                
                withinTolerance = abs(simData.position - finalPos) < tolerance;
                settleIdx = find(withinTolerance, 1, 'first');
                
                if ~isempty(settleIdx)
                    actualSettlingTime = simData.time(settleIdx);
                    reqSettlingTime = requirements.settlingTime_s;
                    
                    passed = actualSettlingTime <= reqSettlingTime * 1.10;  % 10% tolerance
                    
                    results.settlingTime.required = reqSettlingTime;
                    results.settlingTime.actual = actualSettlingTime;
                    results.settlingTime.passed = passed;
                    
                    fprintf('Settling Time (2%% tolerance):\n');
                    fprintf('  Required: %.3f s\n', reqSettlingTime);
                    fprintf('  Actual: %.3f s\n', actualSettlingTime);
                    fprintf('  Status: %s\n', PerformanceValidator.passFailStr(passed));
                    fprintf('\n');
                    
                    if ~passed
                        results.allPassed = false;
                    end
                end
            end
            
            % Check overshoot
            if isfield(requirements, 'maxOvershoot_percent')
                finalPos = simData.position(end);
                maxPos = max(simData.position);
                
                if finalPos ~= 0
                    overshoot = ((maxPos - finalPos) / finalPos) * 100;
                else
                    overshoot = 0;
                end
                
                reqOvershoot = requirements.maxOvershoot_percent;
                passed = overshoot <= reqOvershoot * 1.20;  % 20% tolerance
                
                results.overshoot.required = reqOvershoot;
                results.overshoot.actual = overshoot;
                results.overshoot.passed = passed;
                
                fprintf('Overshoot:\n');
                fprintf('  Max Allowed: %.1f%%\n', reqOvershoot);
                fprintf('  Actual: %.2f%%\n', overshoot);
                fprintf('  Status: %s\n', PerformanceValidator.passFailStr(passed));
                fprintf('\n');
                
                if ~passed
                    results.allPassed = false;
                end
            end
            
            % Overall assessment
            fprintf('========================================\n');
            if results.allPassed
                fprintf('OVERALL: ✓ PASS - All requirements met\n');
            else
                fprintf('OVERALL: ✗ FAIL - Some requirements not met\n');
                fprintf('Review failed checks above.\n');
            end
            fprintf('========================================\n\n');
        end
        
        function str = passFailStr(passed)
            % Helper to format pass/fail
            if passed
                str = '✓ PASS';
            else
                str = '✗ FAIL';
            end
        end
        
        function plotPerformanceMetrics(simData, requirements, title_str)
            % Visualize performance metrics
            
            if nargin < 3
                title_str = 'Performance Metrics';
            end
            
            figure('Name', title_str);
            
            % Velocity profile with requirement line
            subplot(2, 2, 1);
            plot(simData.time, abs(simData.velocity), 'b-', 'LineWidth', 1.5);
            if isfield(requirements, 'maxVelocity_rad_s')
                hold on;
                yline(requirements.maxVelocity_rad_s, 'r--', 'LineWidth', 2, ...
                    'Label', 'Requirement');
            end
            grid on;
            ylabel('|Velocity| (rad/s)');
            title('Velocity vs Requirement');
            
            % Acceleration
            subplot(2, 2, 2);
            dt = mean(diff(simData.time));
            accel = [0; diff(simData.velocity)] / dt;
            plot(simData.time, abs(accel), 'g-', 'LineWidth', 1.5);
            if isfield(requirements, 'maxAcceleration_rad_s2')
                hold on;
                yline(requirements.maxAcceleration_rad_s2, 'r--', 'LineWidth', 2, ...
                    'Label', 'Requirement');
            end
            grid on;
            ylabel('|Acceleration| (rad/s²)');
            title('Acceleration vs Requirement');
            
            % Current draw
            subplot(2, 2, 3);
            if isfield(simData, 'current')
                plot(simData.time, abs(simData.current), 'm-', 'LineWidth', 1.5);
                hold on;
                yline(40, 'r--', 'LineWidth', 2, 'Label', '40A Limit');
                grid on;
                ylabel('Current (A)');
                title('Current vs Limit');
            end
            
            % Position response
            subplot(2, 2, 4);
            plot(simData.time, simData.position, 'b-', 'LineWidth', 1.5);
            
            % Mark settling time if available
            if isfield(requirements, 'settlingTime_s')
                finalPos = simData.position(end);
                tolerance = 0.02 * abs(finalPos);
                hold on;
                plot(simData.time, finalPos + tolerance * ones(size(simData.time)), ...
                    'r--', 'LineWidth', 1);
                plot(simData.time, finalPos - tolerance * ones(size(simData.time)), ...
                    'r--', 'LineWidth', 1);
                
                withinTolerance = abs(simData.position - finalPos) < tolerance;
                settleIdx = find(withinTolerance, 1, 'first');
                if ~isempty(settleIdx)
                    xline(simData.time(settleIdx), 'g--', 'LineWidth', 2, ...
                        'Label', 'Settled');
                end
            end
            
            grid on;
            ylabel('Position (rad)');
            xlabel('Time (s)');
            title('Position Response');
            
            sgtitle(title_str, 'FontSize', 14, 'FontWeight', 'bold');
        end
        
        function report = generatePerformanceReport(results, filename)
            % Generate text report
            
            report = {};
            report{end+1} = '========================================';
            report{end+1} = 'PERFORMANCE VALIDATION REPORT';
            report{end+1} = '========================================';
            report{end+1} = '';
            report{end+1} = sprintf('Mechanism: %s', results.mechanismName);
            report{end+1} = sprintf('Timestamp: %s', char(results.timestamp));
            report{end+1} = '';
            
            fields = fieldnames(results);
            for i = 1:length(fields)
                field = fields{i};
                if isstruct(results.(field)) && isfield(results.(field), 'passed')
                    metric = results.(field);
                    report{end+1} = sprintf('--- %s ---', field);
                    
                    if isfield(metric, 'required')
                        report{end+1} = sprintf('Required: %.4f', metric.required);
                    end
                    if isfield(metric, 'actual')
                        report{end+1} = sprintf('Actual: %.4f', metric.actual);
                    end
                    if isfield(metric, 'limit')
                        report{end+1} = sprintf('Limit: %.4f', metric.limit);
                    end
                    if isfield(metric, 'peak')
                        report{end+1} = sprintf('Peak: %.4f', metric.peak);
                    end
                    if isfield(metric, 'margin')
                        report{end+1} = sprintf('Margin: %.2f%%', metric.margin);
                    end
                    
                    if metric.passed
                        report{end+1} = 'Status: PASS';
                    else
                        report{end+1} = 'Status: FAIL';
                    end
                    report{end+1} = '';
                end
            end
            
            report{end+1} = '========================================';
            if results.allPassed
                report{end+1} = 'OVERALL: PASS';
            else
                report{end+1} = 'OVERALL: FAIL';
            end
            report{end+1} = '========================================';
            
            % Print
            for i = 1:length(report)
                fprintf('%s\n', report{i});
            end
            
            % Save if requested
            if nargin >= 2
                fid = fopen(filename, 'w');
                for i = 1:length(report)
                    fprintf(fid, '%s\n', report{i});
                end
                fclose(fid);
                fprintf('\nReport saved to: %s\n', filename);
            end
        end
    end
end
