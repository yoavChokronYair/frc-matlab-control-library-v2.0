classdef BuiltinValidators
    % BUILTINVALIDATORS Standard validation functions
    %
    % Collection of built-in validation methods for mechanism simulation
    
    methods (Static)
        
        function results = validatePerformanceRequirements(simData, mechanism, varargin)
            % Validates mechanism meets performance requirements
            
            results = struct();
            results.testName = 'Performance Requirements';
            results.passed = true;
            results.details = {};
            
            % Extract requirements from mechanism if available
            if isfield(mechanism, 'requirements')
                req = mechanism.requirements;
                
                % Check max velocity
                if isfield(req, 'maxVelocity')
                    maxVel = max(abs(simData.velocity));
                    results.maxVelocity = maxVel;
                    if maxVel > req.maxVelocity
                        results.passed = false;
                        results.details{end+1} = sprintf('Max velocity %.3f m/s exceeds limit %.3f m/s', ...
                            maxVel, req.maxVelocity);
                    end
                end
                
                % Check max acceleration
                if isfield(req, 'maxAcceleration')
                    if isfield(simData, 'acceleration')
                        maxAccel = max(abs(simData.acceleration));
                    else
                        maxAccel = max(abs(diff(simData.velocity)./diff(simData.t)));
                    end
                    results.maxAcceleration = maxAccel;
                    if maxAccel > req.maxAcceleration
                        results.passed = false;
                        results.details{end+1} = sprintf('Max acceleration %.3f m/s² exceeds limit %.3f m/s²', ...
                            maxAccel, req.maxAcceleration);
                    end
                end
                
                % Check settling time
                if isfield(req, 'settlingTime')
                    finalPos = simData.position(end);
                    settlingBand = 0.02 * abs(finalPos);
                    settledIdx = find(abs(simData.position - finalPos) > settlingBand, 1, 'last');
                    if ~isempty(settledIdx)
                        settlingTime = simData.t(settledIdx);
                        results.settlingTime = settlingTime;
                        if settlingTime > req.settlingTime
                            results.passed = false;
                            results.details{end+1} = sprintf('Settling time %.3f s exceeds limit %.3f s', ...
                                settlingTime, req.settlingTime);
                        end
                    end
                end
            else
                results.details{end+1} = 'No requirements defined in mechanism';
            end
            
            if results.passed
                fprintf('Performance Validation: PASSED\n');
            else
                fprintf('Performance Validation: FAILED\n');
            end
        end
        
        function results = validateModelAccuracy(simData, mechanism, realData, varargin)
            % Validates simulation matches real robot data
            
            if nargin < 3 || isempty(realData)
                error('Real robot data required for model accuracy validation');
            end
            
            results = struct();
            results.testName = 'Model Accuracy';
            results.passed = true;
            results.details = {};
            
            % Interpolate to common time base
            t_common = linspace(max(simData.t(1), realData.t(1)), ...
                min(simData.t(end), realData.t(end)), 100);
            
            sim_pos = interp1(simData.t, simData.position, t_common);
            real_pos = interp1(realData.t, realData.position, t_common);
            
            sim_vel = interp1(simData.t, simData.velocity, t_common);
            real_vel = interp1(realData.t, realData.velocity, t_common);
            
            % Calculate RMSE
            results.positionRMSE = sqrt(mean((sim_pos - real_pos).^2));
            results.velocityRMSE = sqrt(mean((sim_vel - real_vel).^2));
            
            % Calculate R²
            results.positionR2 = 1 - sum((sim_pos - real_pos).^2) / sum((real_pos - mean(real_pos)).^2);
            results.velocityR2 = 1 - sum((sim_vel - real_vel).^2) / sum((real_vel - mean(real_vel)).^2);
            
            % Set thresholds
            posThreshold = 0.01;  % 1 cm
            velThreshold = 0.05;  % 5 cm/s
            R2Threshold = 0.95;
            
            if results.positionRMSE > posThreshold
                results.passed = false;
                results.details{end+1} = sprintf('Position RMSE %.4f m exceeds threshold %.4f m', ...
                    results.positionRMSE, posThreshold);
            end
            
            if results.velocityRMSE > velThreshold
                results.passed = false;
                results.details{end+1} = sprintf('Velocity RMSE %.4f m/s exceeds threshold %.4f m/s', ...
                    results.velocityRMSE, velThreshold);
            end
            
            if results.positionR2 < R2Threshold
                results.passed = false;
                results.details{end+1} = sprintf('Position R² %.4f below threshold %.4f', ...
                    results.positionR2, R2Threshold);
            end
            
            fprintf('Model Accuracy: %s (Pos R²=%.3f, Vel R²=%.3f)\n', ...
                tern(results.passed, 'PASSED', 'FAILED'), ...
                results.positionR2, results.velocityR2);
        end
        
        function results = validateTrajectory(simData, mechanism, varargin)
            % Validates trajectory tracking performance
            
            results = struct();
            results.testName = 'Trajectory Tracking';
            results.passed = true;
            results.details = {};
            
            % Check if desired trajectory is provided
            if isfield(simData, 'desiredPosition')
                % Calculate tracking error
                trackingError = simData.position - simData.desiredPosition;
                results.maxTrackingError = max(abs(trackingError));
                results.rmsTrackingError = sqrt(mean(trackingError.^2));
                
                % Set thresholds
                maxErrorThreshold = 0.01;  % 1 cm
                rmsErrorThreshold = 0.005;  % 5 mm
                
                if results.maxTrackingError > maxErrorThreshold
                    results.passed = false;
                    results.details{end+1} = sprintf('Max tracking error %.4f m exceeds threshold %.4f m', ...
                        results.maxTrackingError, maxErrorThreshold);
                end
                
                if results.rmsTrackingError > rmsErrorThreshold
                    results.passed = false;
                    results.details{end+1} = sprintf('RMS tracking error %.4f m exceeds threshold %.4f m', ...
                        results.rmsTrackingError, rmsErrorThreshold);
                end
            else
                results.details{end+1} = 'No desired trajectory provided';
            end
            
            fprintf('Trajectory Validation: %s\n', tern(results.passed, 'PASSED', 'FAILED'));
        end
        
        function results = validateSafety(simData, mechanism, varargin)
            % Validates safety limits (position, velocity, current)
            
            results = struct();
            results.testName = 'Safety Limits';
            results.passed = true;
            results.details = {};
            
            % Define safety limits
            if isfield(mechanism, 'safetyLimits')
                limits = mechanism.safetyLimits;
            else
                % Default safety limits
                limits.maxPosition = 1.0;  % meters
                limits.minPosition = -1.0;
                limits.maxVelocity = 2.0;  % m/s
                limits.maxCurrent = 10.0;  % Amps
            end
            
            % Check position limits
            if isfield(limits, 'maxPosition')
                maxPos = max(simData.position);
                if maxPos > limits.maxPosition
                    results.passed = false;
                    results.details{end+1} = sprintf('Max position %.3f m exceeds limit %.3f m', ...
                        maxPos, limits.maxPosition);
                end
            end
            
            if isfield(limits, 'minPosition')
                minPos = min(simData.position);
                if minPos < limits.minPosition
                    results.passed = false;
                    results.details{end+1} = sprintf('Min position %.3f m below limit %.3f m', ...
                        minPos, limits.minPosition);
                end
            end
            
            % Check velocity limits
            if isfield(limits, 'maxVelocity')
                maxVel = max(abs(simData.velocity));
                if maxVel > limits.maxVelocity
                    results.passed = false;
                    results.details{end+1} = sprintf('Max velocity %.3f m/s exceeds limit %.3f m/s', ...
                        maxVel, limits.maxVelocity);
                end
            end
            
            % Check current limits
            if isfield(limits, 'maxCurrent') && isfield(simData, 'current')
                maxCurrent = max(abs(simData.current));
                if maxCurrent > limits.maxCurrent
                    results.passed = false;
                    results.details{end+1} = sprintf('Max current %.3f A exceeds limit %.3f A', ...
                        maxCurrent, limits.maxCurrent);
                end
            end
            
            fprintf('Safety Validation: %s\n', tern(results.passed, 'PASSED', 'FAILED'));
        end
        
    end
end

function result = tern(condition, trueVal, falseVal)
    % Ternary operator helper function
    if condition
        result = trueVal;
    else
        result = falseVal;
    end
end