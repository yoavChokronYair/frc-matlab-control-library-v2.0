classdef BuiltinValidators
    % Standard validation plugins for the system
    %
    % These are automatically registered in VisualizationSystemInit.m
    
    methods (Static)
        % ===== PERFORMANCE VALIDATION =====
        
        function results = validatePerformanceRequirements(simData, mechanism, varargin)
            % Validate mechanism meets performance requirements
            
            p = inputParser();
            addParameter(p, 'requirements', struct(), @isstruct);
            parse(p, varargin{:});
            
            requirements = p.Results.requirements;
            
            fprintf('\n=== Performance Validation ===\n\n');
            
            results = struct();
            results.mechanismName = mechanism.mechanismName;
            results.timestamp = datetime('now');
            results.allPassed = true;
            
            % Check maximum velocity
            if isfield(requirements, 'maxVelocity_rad_s')
                actualMaxVel = max(abs(simData.velocity));
                reqMaxVel = requirements.maxVelocity_rad_s;
                
                passed = actualMaxVel >= reqMaxVel * 0.95;
                results.maxVelocity.required = reqMaxVel;
                results.maxVelocity.actual = actualMaxVel;
                results.maxVelocity.passed = passed;
                results.maxVelocity.margin = (actualMaxVel / reqMaxVel - 1) * 100;
                
                fprintf('Max Velocity:\n');
                fprintf('  Required: %.2f rad/s\n', reqMaxVel);
                fprintf('  Actual: %.2f rad/s\n', actualMaxVel);
                fprintf('  Status: %s (%.1f%% margin)\n\n', ...
                    BuiltinValidators.passFailStr(passed), ...
                    results.maxVelocity.margin);
                
                if ~passed, results.allPassed = false; end
            end
            
            % Check maximum acceleration
            if isfield(requirements, 'maxAcceleration_rad_s2')
                dt = mean(diff(simData.time));
                accel = diff(simData.velocity) / dt;
                actualMaxAccel = max(abs(accel));
                reqMaxAccel = requirements.maxAcceleration_rad_s2;
                
                passed = actualMaxAccel >= reqMaxAccel * 0.90;
                
                results.maxAcceleration.required = reqMaxAccel;
                results.maxAcceleration.actual = actualMaxAccel;
                results.maxAcceleration.passed = passed;
                results.maxAcceleration.margin = (actualMaxAccel / reqMaxAccel - 1) * 100;
                
                fprintf('Max Acceleration:\n');
                fprintf('  Required: %.2f rad/s²\n', reqMaxAccel);
                fprintf('  Actual: %.2f rad/s²\n', actualMaxAccel);
                fprintf('  Status: %s (%.1f%% margin)\n\n', ...
                    BuiltinValidators.passFailStr(passed), ...
                    results.maxAcceleration.margin);
                
                if ~passed, results.allPassed = false; end
            end
            
            % Check current limit
            if isfield(simData, 'current')
                maxCurrent = max(abs(simData.current));
                currentLimit = mechanism.motorModel.MAX_CONTINUOUS_CURRENT_A;
                
                passed = maxCurrent <= currentLimit * 1.05;
                
                results.currentLimit.limit = currentLimit;
                results.currentLimit.peak = maxCurrent;
                results.currentLimit.passed = passed;
                results.currentLimit.margin = (currentLimit / maxCurrent - 1) * 100;
                
                fprintf('Current Limit:\n');
                fprintf('  Limit: %.0f A\n', currentLimit);
                fprintf('  Peak: %.2f A\n', maxCurrent);
                fprintf('  Status: %s (%.1f%% margin)\n\n', ...
                    BuiltinValidators.passFailStr(passed), ...
                    results.currentLimit.margin);
                
                if ~passed, results.allPassed = false; end
            end
            
            % Overall
            fprintf('========================================\n');
            if results.allPassed
                fprintf('OVERALL: ✓ PASS - All requirements met\n');
            else
                fprintf('OVERALL: ✗ FAIL - Some requirements not met\n');
            end
            fprintf('========================================\n\n');
        end
        
        % ===== MODEL VALIDATION =====
        
        function results = validateModelAccuracy(simData, realData, varargin)
            % Validate simulation matches real robot data
            
            p = inputParser();
            addParameter(p, 'name', 'Mechanism', @ischar);
            parse(p, varargin{:});
            
            fprintf('\n=== Model Validation ===\n\n');
            
            results = struct();
            results.mechanismName = p.Results.name;
            results.timestamp = datetime('now');
            results.allPassed = true;
            
            % Interpolate real data to sim time base
            realPos_interp = interp1(realData.time, realData.position, simData.time, 'linear');
            realVel_interp = interp1(realData.time, realData.velocity, simData.time, 'linear');
            
            % Position validation
            posError = simData.position - realPos_interp;
            rmsePos = sqrt(mean(posError.^2));
            maxErrorPos = max(abs(posError));
            
            % R² for position
            ssRes = sum(posError.^2);
            ssTot = sum((realPos_interp - mean(realPos_interp)).^2);
            r2Pos = 1 - (ssRes / ssTot);
            
            results.position.rmse = rmsePos;
            results.position.maxError = maxErrorPos;
            results.position.r2 = r2Pos;
            results.position.passed = r2Pos > 0.85;  % R² > 0.85 = acceptable
            
            fprintf('Position Validation:\n');
            fprintf('  RMSE: %.4f rad (%.2f deg)\n', rmsePos, rmsePos * 180/pi);
            fprintf('  Max Error: %.4f rad (%.2f deg)\n', maxErrorPos, maxErrorPos * 180/pi);
            fprintf('  R²: %.4f (1.0 = perfect)\n');
            fprintf('  Status: %s\n\n', BuiltinValidators.passFailStr(results.position.passed));
            
            if ~results.position.passed, results.allPassed = false; end
            
            % Velocity validation
            velError = simData.velocity - realVel_interp;
            rmseVel = sqrt(mean(velError.^2));
            maxErrorVel = max(abs(velError));
            
            % R² for velocity
            ssRes = sum(velError.^2);
            ssTot = sum((realVel_interp - mean(realVel_interp)).^2);
            r2Vel = 1 - (ssRes / ssTot);
            
            results.velocity.rmse = rmseVel;
            results.velocity.maxError = maxErrorVel;
            results.velocity.r2 = r2Vel;
            results.velocity.passed = r2Vel > 0.85;
            
            fprintf('Velocity Validation:\n');
            fprintf('  RMSE: %.4f rad/s\n', rmseVel);
            fprintf('  Max Error: %.4f rad/s\n', maxErrorVel);
            fprintf('  R²: %.4f (1.0 = perfect)\n');
            fprintf('  Status: %s\n\n', BuiltinValidators.passFailStr(results.velocity.passed));
            
            if ~results.velocity.passed, results.allPassed = false; end
            
            % Overall assessment
            fprintf('========================================\n');
            if results.allPassed
                fprintf('OVERALL: ✓ GOOD - Model is acceptable\n');
            else
                fprintf('OVERALL: ✗ NEEDS REFINEMENT\n');
            end
            fprintf('========================================\n\n');
        end
        
        % ===== TRAJECTORY VALIDATION =====
        
        function results = validateTrajectory(simData, trajectory, varargin)
            % Validate that simulated motion matches desired trajectory
            
            p = inputParser();
            addParameter(p, 'tolerance', 0.1, @isnumeric);  % rad
            parse(p, varargin{:});
            
            results = struct();
            results.mechanismName = 'Trajectory Tracking';
            results.passed = true;
            
            % Interpolate trajectory to sim time
            traj_interp = interp1(trajectory.time, trajectory.position, simData.time, 'linear');
            
            % Track error
            trackError = simData.position - traj_interp;
            maxError = max(abs(trackError));
            rmseError = sqrt(mean(trackError.^2));
            
            results.maxError = maxError;
            results.rmseError = rmseError;
            results.tolerance = p.Results.tolerance;
            results.passed = maxError < p.Results.tolerance;
            
            fprintf('Trajectory Tracking Validation:\n');
            fprintf('  Max Error: %.4f rad (tolerance: %.4f rad)\n', maxError, p.Results.tolerance);
            fprintf('  RMS Error: %.4f rad\n', rmseError);
            fprintf('  Status: %s\n\n', BuiltinValidators.passFailStr(results.passed));
        end
        
        % ===== SAFETY VALIDATION =====
        
        function results = validateSafety(simData, mechanism, varargin)
            % Validate mechanism safety limits
            
            results = struct();
            results.mechanismName = mechanism.mechanismName;
            results.allPassed = true;
            
            fprintf('\n=== Safety Validation ===\n\n');
            
            % Position limits
            if isfield(mechanism, 'minPosition_rad') && isfield(mechanism, 'maxPosition_rad')
                minPos = min(simData.position);
                maxPos = max(simData.position);
                
                posOk = minPos >= mechanism.minPosition_rad && ...
                        maxPos <= mechanism.maxPosition_rad;
                
                results.position.min = minPos;
                results.position.max = maxPos;
                results.position.limit_min = mechanism.minPosition_rad;
                results.position.limit_max = mechanism.maxPosition_rad;
                results.position.passed = posOk;
                
                fprintf('Position Limits:\n');
                fprintf('  Actual: [%.2f, %.2f] rad\n', minPos, maxPos);
                fprintf('  Limits: [%.2f, %.2f] rad\n', ...
                    mechanism.minPosition_rad, mechanism.maxPosition_rad);
                fprintf('  Status: %s\n\n', BuiltinValidators.passFailStr(posOk));
                
                if ~posOk, results.allPassed = false; end
            end
            
            % Velocity limits
            if isfield(mechanism, 'maxVelocity_rad_s')
                maxVel = max(abs(simData.velocity));
                velOk = maxVel <= mechanism.maxVelocity_rad_s * 1.05;
                
                results.velocity.max = maxVel;
                results.velocity.limit = mechanism.maxVelocity_rad_s;
                results.velocity.passed = velOk;
                
                fprintf('Velocity Limits:\n');
                fprintf('  Actual: %.2f rad/s\n', maxVel);
                fprintf('  Limit: %.2f rad/s\n', mechanism.maxVelocity_rad_s);
                fprintf('  Status: %s\n\n', BuiltinValidators.passFailStr(velOk));
                
                if ~velOk, results.allPassed = false; end
            end
            
            % Current limits
            maxCurrent = max(abs(simData.current));
            currentLimit = mechanism.motorModel.MAX_CONTINUOUS_CURRENT_A;
            currentOk = maxCurrent <= currentLimit * 1.05;
            
            results.current.max = maxCurrent;
            results.current.limit = currentLimit;
            results.current.passed = currentOk;
            
            fprintf('Current Limits:\n');
            fprintf('  Peak: %.1f A\n', maxCurrent);
            fprintf('  Limit: %.0f A\n', currentLimit);
            fprintf('  Status: %s\n\n', BuiltinValidators.passFailStr(currentOk));
            
            if ~currentOk, results.allPassed = false; end
            
            fprintf('========================================\n');
            if results.allPassed
                fprintf('OVERALL: ✓ SAFE - All limits respected\n');
            else
                fprintf('OVERALL: ✗ UNSAFE - Limits exceeded\n');
            end
            fprintf('========================================\n\n');
        end
        
        % ===== HELPER =====
        
        function str = passFailStr(passed)
            % Helper to format pass/fail
            if passed
                str = '✓ PASS';
            else
                str = '✗ FAIL';
            end
        end
    end
end
