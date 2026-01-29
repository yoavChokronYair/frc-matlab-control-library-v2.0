classdef PIDTuner < handle
    % Automated PID gain calculation using various methods
    
    methods (Static)
        function [kP, kI, kD] = tunePositionPID(systemParams, desiredResponse)
            % Tune PID for position control
            %
            % Inputs:
            %   systemParams: struct with .inertia, .damping, .kV, .kA
            %   desiredResponse: struct with .settlingTime_s, .overshoot_percent
            %
            % Outputs:
            %   kP, kI, kD: PID gains
            
            % Extract parameters
            J = systemParams.inertia;
            b = systemParams.damping;
            
            % Desired closed-loop poles from response specs
            settlingTime = desiredResponse.settlingTime_s;
            overshoot = desiredResponse.overshoot_percent;
            
            % Damping ratio from overshoot: ζ = -ln(OS/100) / sqrt(π² + ln²(OS/100))
            if overshoot > 0
                zeta = -log(overshoot/100) / sqrt(pi^2 + log(overshoot/100)^2);
            else
                zeta = 1.0; % Critically damped
            end
            
            % Natural frequency from settling time: ωn = 4 / (ζ * Ts)
            wn = 4 / (zeta * settlingTime);
            
            % PID gains from pole placement
            % Second-order system: s² + 2ζωn*s + ωn² = 0
            % With PID: s² + (b+kD)/J*s + kP/J = 0
            
            kP = J * wn^2;
            kD = 2 * zeta * wn * J - b;
            kI = kP / 10; % Typical ratio, tune if needed
            
            % Ensure non-negative
            kD = max(kD, 0);
            
            fprintf('Position PID Gains:\n');
            fprintf('  kP: %.4f\n', kP);
            fprintf('  kI: %.4f\n', kI);
            fprintf('  kD: %.4f\n', kD);
            fprintf('Expected response:\n');
            fprintf('  Settling time: %.2f s\n', settlingTime);
            fprintf('  Overshoot: %.1f%%\n', overshoot);
        end
        
        function [kP, kI, kD] = tuneVelocityPID(systemParams, bandwidth_Hz)
            % Tune PID for velocity control (typically just PI)
            %
            % Inputs:
            %   systemParams: struct with motor/mechanism properties
            %   bandwidth_Hz: Desired control bandwidth
            
            J = systemParams.inertia;
            b = systemParams.damping;
            
            % Convert bandwidth to rad/s
            wc = 2 * pi * bandwidth_Hz;
            
            % PI controller for velocity
            % Open-loop: G(s) = 1/(Js + b)
            % Controller: C(s) = kP + kI/s
            
            kP = J * wc;
            kI = b * wc;
            kD = 0; % Not typically used for velocity control
            
            fprintf('Velocity PI Gains:\n');
            fprintf('  kP: %.4f\n', kP);
            fprintf('  kI: %.4f\n', kI);
            fprintf('  Bandwidth: %.1f Hz\n', bandwidth_Hz);
        end
        
        function gains = tuneWithZieglerNichols(Ku, Tu)
            % Classic Ziegler-Nichols tuning from ultimate gain/period
            % (from oscillation test on robot)
            %
            % Inputs:
            %   Ku: Ultimate gain (gain at sustained oscillation)
            %   Tu: Ultimate period (seconds)
            
            gains.kP = 0.6 * Ku;
            gains.kI = 1.2 * Ku / Tu;
            gains.kD = 0.075 * Ku * Tu;
            
            fprintf('Ziegler-Nichols PID Gains:\n');
            fprintf('  kP: %.4f\n', gains.kP);
            fprintf('  kI: %.4f\n', gains.kI);
            fprintf('  kD: %.4f\n', gains.kD);
        end
        
        function [kP, kI, kD] = autoTune(systemModel, constraints)
            % Automatic tuning using optimization
            % Minimize settling time subject to overshoot constraint
            
            % Cost function: weighted sum of settling time and overshoot
            costFn = @(gains) PIDTuner.evaluateResponse(gains, systemModel, constraints);
            
            % Initial guess
            x0 = [1.0, 0.1, 0.05]; % [kP, kI, kD]
            
            % Bounds
            lb = [0, 0, 0];
            ub = [100, 10, 5];
            
            % Optimize using fmincon
            options = optimoptions('fmincon', 'Display', 'iter');
            gainsOpt = fmincon(costFn, x0, [], [], [], [], lb, ub, [], options);
            
            kP = gainsOpt(1);
            kI = gainsOpt(2);
            kD = gainsOpt(3);
            
            fprintf('Auto-tuned PID Gains:\n');
            fprintf('  kP: %.4f\n', kP);
            fprintf('  kI: %.4f\n', kI);
            fprintf('  kD: %.4f\n', kD);
        end
        
        function cost = evaluateResponse(gains, systemModel, constraints)
            % Simulate step response and calculate cost
            % (used by autoTune)
            
            kP = gains(1);
            kI = gains(2);
            kD = gains(3);
            
            % Closed-loop transfer function
            % TF depends on system model structure
            % Simplified second-order model
            J = systemModel.inertia;
            b = systemModel.damping;
            
            num = [kP];
            den = [J, b+kD, kP, kI];
            sys = tf(num, den);
            
            % Step response
            [y, t] = step(sys, 5); % 5 second simulation
            
            % Calculate metrics
            info = stepinfo(y, t);
            settlingTime = info.SettlingTime;
            overshoot = info.Overshoot;
            
            % Cost: penalize long settling time and excessive overshoot
            cost = settlingTime;
            if overshoot > constraints.maxOvershoot_percent
                cost = cost + 100 * (overshoot - constraints.maxOvershoot_percent);
            end
        end
    end
end
