classdef MotorCharacteristics < handle
    % Utility class for motor performance curves and characteristics
    % Provides plotting and analysis tools for motor models
    
    methods (Static)
        function plotTorqueSpeedCurve(motorModel, gearRatio, title_str)
            % Plot torque-speed curve for a motor at different gear ratios
            %
            % Inputs:
            %   motorModel: KrakenX40Model or KrakenX60Model
            %   gearRatio: Gear reduction ratio
            %   title_str: Plot title (optional)
            
            if nargin < 3
                title_str = sprintf('%s Torque-Speed Curve', class(motorModel));
            end
            
            % Speed range (rad/s at output shaft)
            maxMotorSpeed = motorModel.FREE_SPEED_RAD_S;
            maxOutputSpeed = maxMotorSpeed / gearRatio;
            speeds = linspace(0, maxOutputSpeed, 100);
            
            % Calculate torque at each speed
            torques = zeros(size(speeds));
            for i = 1:length(speeds)
                motorSpeed = speeds(i) * gearRatio;
                torques(i) = motorModel.getMaxTorqueAtSpeed(motorSpeed) * gearRatio;
            end
            
            % Plot
            figure('Name', 'Motor Characteristics');
            plot(speeds, torques, 'b-', 'LineWidth', 2);
            grid on;
            xlabel('Speed (rad/s)');
            ylabel('Torque (N\cdotm)');
            title(title_str);
            
            % Add annotations
            [maxTorque, maxIdx] = max(torques);
            text(speeds(maxIdx), maxTorque, ...
                sprintf('  Max: %.2f N·m', maxTorque), ...
                'VerticalAlignment', 'bottom');
        end
        
        function plotPowerCurve(motorModel, gearRatio, title_str)
            % Plot power output vs speed
            
            if nargin < 3
                title_str = sprintf('%s Power Curve', class(motorModel));
            end
            
            maxMotorSpeed = motorModel.FREE_SPEED_RAD_S;
            maxOutputSpeed = maxMotorSpeed / gearRatio;
            speeds = linspace(0, maxOutputSpeed, 100);
            
            powers = zeros(size(speeds));
            for i = 1:length(speeds)
                motorSpeed = speeds(i) * gearRatio;
                torque = motorModel.getMaxTorqueAtSpeed(motorSpeed) * gearRatio;
                powers(i) = torque * speeds(i);  % Power = Torque × Speed
            end
            
            figure('Name', 'Motor Power');
            plot(speeds, powers, 'r-', 'LineWidth', 2);
            grid on;
            xlabel('Speed (rad/s)');
            ylabel('Power (W)');
            title(title_str);
            
            [maxPower, maxIdx] = max(powers);
            text(speeds(maxIdx), maxPower, ...
                sprintf('  Peak: %.0f W', maxPower), ...
                'VerticalAlignment', 'bottom');
        end
        
        function plotEfficiencyMap(motorModel, gearRatio, title_str)
            % Plot efficiency vs speed and torque
            
            if nargin < 3
                title_str = sprintf('%s Efficiency Map', class(motorModel));
            end
            
            maxMotorSpeed = motorModel.FREE_SPEED_RAD_S;
            maxOutputSpeed = maxMotorSpeed / gearRatio;
            
            speeds = linspace(0, maxOutputSpeed, 50);
            torques = linspace(0, motorModel.STALL_TORQUE_NM * gearRatio, 50);
            
            [S, T] = meshgrid(speeds, torques);
            efficiency = zeros(size(S));
            
            for i = 1:numel(S)
                speed = S(i);
                torque = T(i);
                motorSpeed = speed * gearRatio;
                motorTorque = torque / gearRatio;
                
                % Mechanical power out
                powerOut = torque * speed;
                
                % Electrical power in
                current = motorModel.getCurrent(motorTorque);
                voltage = motorModel.getVoltage(motorSpeed, motorTorque);
                powerIn = voltage * current;
                
                if powerIn > 0
                    efficiency(i) = (powerOut / powerIn) * 100;
                else
                    efficiency(i) = 0;
                end
            end
            
            % Clamp efficiency to reasonable range
            efficiency(efficiency > 100) = 100;
            efficiency(efficiency < 0) = 0;
            
            figure('Name', 'Motor Efficiency');
            contourf(S, T, efficiency, 20);
            colorbar;
            xlabel('Speed (rad/s)');
            ylabel('Torque (N\cdotm)');
            title(title_str);
            caxis([0 100]);
        end
        
        function compareMotors(motors, gearRatios, names)
            % Compare multiple motors on same plot
            %
            % Inputs:
            %   motors: Cell array of motor models
            %   gearRatios: Array of gear ratios
            %   names: Cell array of names for legend
            
            figure('Name', 'Motor Comparison');
            
            colors = lines(length(motors));
            
            for m = 1:length(motors)
                motor = motors{m};
                ratio = gearRatios(m);
                
                maxMotorSpeed = motor.FREE_SPEED_RAD_S;
                maxOutputSpeed = maxMotorSpeed / ratio;
                speeds = linspace(0, maxOutputSpeed, 100);
                
                torques = zeros(size(speeds));
                for i = 1:length(speeds)
                    motorSpeed = speeds(i) * ratio;
                    torques(i) = motor.getMaxTorqueAtSpeed(motorSpeed) * ratio;
                end
                
                plot(speeds, torques, 'LineWidth', 2, 'Color', colors(m,:), ...
                    'DisplayName', names{m});
                hold on;
            end
            
            grid on;
            xlabel('Speed (rad/s)');
            ylabel('Torque (N\cdotm)');
            title('Motor Comparison');
            legend('Location', 'best');
            hold off;
        end
        
        function plotCurrentVsSpeed(motorModel, gearRatio, loadTorque)
            % Plot current draw vs speed at constant load
            %
            % Inputs:
            %   motorModel: Motor model
            %   gearRatio: Gear ratio
            %   loadTorque: Constant load torque (N·m)
            
            maxMotorSpeed = motorModel.FREE_SPEED_RAD_S;
            maxOutputSpeed = maxMotorSpeed / gearRatio;
            speeds = linspace(0, maxOutputSpeed, 100);
            
            currents = zeros(size(speeds));
            for i = 1:length(speeds)
                motorTorque = loadTorque / gearRatio;
                currents(i) = motorModel.getCurrent(motorTorque);
            end
            
            figure('Name', 'Current Draw');
            plot(speeds, currents, 'g-', 'LineWidth', 2);
            grid on;
            xlabel('Speed (rad/s)');
            ylabel('Current (A)');
            title(sprintf('Current Draw (Load = %.2f N·m)', loadTorque));
            
            % Add current limit line
            hold on;
            yline(motorModel.MAX_CONTINUOUS_CURRENT_A, 'r--', ...
                'LineWidth', 1.5, 'Label', '40A Limit');
            hold off;
        end
        
        function specs = generateSpecSheet(motorModel, gearRatio)
            % Generate specification table for a motor configuration
            
            specs = struct();
            
            % Basic specs
            specs.motorType = class(motorModel);
            specs.gearRatio = gearRatio;
            specs.currentLimit_A = motorModel.MAX_CONTINUOUS_CURRENT_A;
            
            % Performance at output shaft
            specs.maxSpeed_rad_s = motorModel.FREE_SPEED_RAD_S / gearRatio;
            specs.maxSpeed_RPM = motorModel.FREE_SPEED_RPM / gearRatio;
            specs.stallTorque_Nm = motorModel.STALL_TORQUE_NM * gearRatio;
            
            % Peak power
            speeds = linspace(0, specs.maxSpeed_rad_s, 100);
            powers = zeros(size(speeds));
            for i = 1:length(speeds)
                motorSpeed = speeds(i) * gearRatio;
                torque = motorModel.getMaxTorqueAtSpeed(motorSpeed) * gearRatio;
                powers(i) = torque * speeds(i);
            end
            specs.peakPower_W = max(powers);
            
            % Feedforward constants
            [kV, kA, kS] = motorModel.getFeedforwardConstants(gearRatio);
            specs.kV = kV;
            specs.kA = kA;
            specs.kS = kS;
            
            % Display
            fprintf('\n=== Motor Specification Sheet ===\n');
            fprintf('Motor: %s\n', specs.motorType);
            fprintf('Gear Ratio: %.2f:1\n', specs.gearRatio);
            fprintf('Current Limit: %d A\n', specs.currentLimit_A);
            fprintf('\nPerformance:\n');
            fprintf('  Max Speed: %.2f rad/s (%.0f RPM)\n', ...
                specs.maxSpeed_rad_s, specs.maxSpeed_RPM);
            fprintf('  Stall Torque: %.2f N·m\n', specs.stallTorque_Nm);
            fprintf('  Peak Power: %.0f W\n', specs.peakPower_W);
            fprintf('\nFeedforward:\n');
            fprintf('  kV: %.4f V/(rad/s)\n', specs.kV);
            fprintf('  kA: %.4f V/(rad/s²)\n', specs.kA);
            fprintf('  kS: %.4f V\n', specs.kS);
            fprintf('================================\n\n');
        end
    end
end
