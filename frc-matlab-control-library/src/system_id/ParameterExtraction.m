classdef ParameterExtraction < handle
    % Extracts physical parameters from recorded test data
    
    methods (Static)
        function [kV, kA, kS] = extractFeedforward(testData)
            % Extract feedforward constants from voltage step response
            %
            % Input: testData struct with fields:
            %   .time (s)
            %   .voltage (V)
            %   .position (rad or m)
            %   .velocity (rad/s or m/s)
            %   .current (A)
            
            time = testData.time;
            voltage = testData.voltage;
            velocity = testData.velocity;
            
            % Calculate acceleration (numerical derivative)
            dt = mean(diff(time));
            acceleration = [0; diff(velocity)] / dt;
            
            % Linear regression: V = kS*sign(v) + kV*v + kA*a
            % Split into forward and backward motion
            
            % Forward motion (velocity > 0.1 rad/s)
            fwdIdx = velocity > 0.1;
            
            if sum(fwdIdx) > 50  % Need enough data points
                % Design matrix: [sign(v), v, a]
                X = [sign(velocity(fwdIdx)), velocity(fwdIdx), acceleration(fwdIdx)];
                y = voltage(fwdIdx);
                
                % Least squares: [kS; kV; kA] = (X'X)^-1 X'y
                params = (X' * X) \ (X' * y);
                
                kS = abs(params(1));  % Static friction
                kV = params(2);        % Velocity constant
                kA = params(3);        % Acceleration constant
            else
                warning('Insufficient data for feedforward extraction');
                kS = 0; kV = 0; kA = 0;
            end
            
            % Validate results
            fprintf('Feedforward Constants:\n');
            fprintf('  kS (static friction): %.4f V\n', kS);
            fprintf('  kV (velocity): %.4f V/(rad/s)\n', kV);
            fprintf('  kA (acceleration): %.4f V/(rad/s²)\n', kA);
        end
        
        function [inertia, friction] = extractMechanicalParams(testData, motorModel, gearRatio)
            % Extract mechanism inertia and friction from motor current
            %
            % Torque balance: τ_motor = J*α + b*ω + τ_friction
            
            time = testData.time;
            current = testData.current;
            velocity = testData.velocity;
            
            % Motor torque from current
            motorTorque = motorModel.KT * current;
            outputTorque = motorTorque * gearRatio; % At mechanism output
            
            % Calculate angular acceleration
            dt = mean(diff(time));
            acceleration = [0; diff(velocity)] / dt;
            
            % Regression: τ = J*α + b*ω + c
            % Use only steady-state data (avoid transients)
            steadyIdx = abs(acceleration) < 0.5; % Low acceleration
            
            X = [acceleration(steadyIdx), velocity(steadyIdx), ones(sum(steadyIdx), 1)];
            y = outputTorque(steadyIdx);
            
            params = (X' * X) \ (X' * y);
            
            inertia = params(1);        % kg⋅m² or kg
            friction = params(2);        % Viscous friction coefficient
            staticFriction = params(3); % Coulomb friction
            
            fprintf('Mechanical Parameters:\n');
            fprintf('  Inertia: %.4f kg⋅m² (or kg for linear)\n', inertia);
            fprintf('  Viscous friction: %.4f N⋅m⋅s/rad\n', friction);
            fprintf('  Static friction: %.4f N⋅m\n', staticFriction);
        end
        
        function [naturalFreq_Hz, damping] = extractDynamics(testData)
            % Extract natural frequency and damping from frequency sweep
            
            time = testData.time;
            positionCmd = testData.positionCommand;
            positionActual = testData.position;
            
            % Tracking error
            error = positionCmd - positionActual;
            
            % FFT to find resonance peaks
            Fs = 1 / mean(diff(time));
            N = length(error);
            frequencies = (0:N-1) * (Fs/N);
            
            fftError = abs(fft(error));
            
            % Find peak (resonance)
            [~, peakIdx] = max(fftError(1:round(N/2)));
            naturalFreq_Hz = frequencies(peakIdx);
            
            % Damping from peak width (simplified)
            damping = 0.05; % Placeholder - use half-power bandwidth method
            
            fprintf('Dynamic Characteristics:\n');
            fprintf('  Natural frequency: %.2f Hz\n', naturalFreq_Hz);
            fprintf('  Damping ratio: %.3f\n', damping);
            
            % Plot Bode-like response
            figure;
            semilogx(frequencies(1:round(N/2)), 20*log10(fftError(1:round(N/2))));
            xlabel('Frequency (Hz)');
            ylabel('Magnitude (dB)');
            title('Frequency Response');
            grid on;
        end
        
        function backlash = measureBacklash(testData)
            % Measure mechanical backlash from repeatability test
            
            position = testData.position;
            command = testData.positionCommand;
            
            % Find direction reversals
            cmdDiff = diff(command);
            reversalIdx = find(abs(cmdDiff) > 0.1);
            
            % Measure position error at each reversal
            errors = [];
            for i = reversalIdx'
                if i+10 <= length(position)
                    % Error in 10 samples after reversal
                    errors = [errors; position(i+10) - command(i+10)];
                end
            end
            
            backlash = std(errors) * 2; % 2-sigma estimate
            
            fprintf('Backlash: %.4f rad (or m)\n', backlash);
        end
    end
end
