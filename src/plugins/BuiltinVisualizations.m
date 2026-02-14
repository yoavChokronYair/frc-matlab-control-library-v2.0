classdef BuiltinVisualizations
    % BUILTINVISUALIZATIONS Standard visualization functions
    %
    % Collection of built-in visualization methods for mechanism simulation
    
    methods (Static)
        
        function plotStandardSimulation(simData, varargin)
            % Standard simulation results (position, velocity, current, voltage)
            
            figure('Name', 'Standard Simulation Results', 'NumberTitle', 'off');
            
            subplot(4,1,1);
            plot(simData.time, simData.position, 'LineWidth', 1.5);
            ylabel('Position (m)');
            grid on;
            title('Simulation Results');
            
            subplot(4,1,2);
            plot(simData.time, simData.velocity, 'LineWidth', 1.5);
            ylabel('Velocity (m/s)');
            grid on;
            
            subplot(4,1,3);
            plot(simData.time, simData.current, 'LineWidth', 1.5);
            ylabel('Current (A)');
            grid on;
            
            subplot(4,1,4);
            plot(simData.time, simData.voltage, 'LineWidth', 1.5);
            ylabel('Voltage (V)');
            xlabel('Time (s)');
            grid on;
        end
        
        function plotDetailedTrajectory(simData, varargin)
            % Detailed trajectory (position, velocity, acceleration, jerk)
            
            figure('Name', 'Detailed Trajectory', 'NumberTitle', 'off');
            
            subplot(4,1,1);
            plot(simData.time, simData.position, 'LineWidth', 1.5);
            ylabel('Position (m)');
            grid on;
            title('Trajectory Analysis');
            
            subplot(4,1,2);
            plot(simData.time, simData.velocity, 'LineWidth', 1.5);
            ylabel('Velocity (m/s)');
            grid on;
            
            subplot(4,1,3);
            if isfield(simData, 'acceleration')
                plot(simData.time, simData.acceleration, 'LineWidth', 1.5);
            else
                % Calculate acceleration using diff
                % Ensure column vectors for proper concatenation
                time_vec = simData.time(:);  % Force column vector
                vel_vec = simData.velocity(:);  % Force column vector
                
                dt = diff(time_vec);
                dv = diff(vel_vec);
                accel = [dv./dt; 0];  % Append zero at end to match length
                plot(simData.time, accel, 'LineWidth', 1.5);
            end
            ylabel('Acceleration (m/s²)');
            grid on;
            
            subplot(4,1,4);
            if isfield(simData, 'jerk')
                plot(simData.time, simData.jerk, 'LineWidth', 1.5);
            else
                % Calculate acceleration and jerk using diff
                % Ensure column vectors for proper concatenation
                time_vec = simData.time(:);  % Force column vector
                vel_vec = simData.velocity(:);  % Force column vector
                
                dt = diff(time_vec);
                dv = diff(vel_vec);
                accel = [dv./dt; 0];  % Append zero at end
                
                dt2 = diff(time_vec);
                da = diff(accel);
                jerk = [da./dt2; 0];  % Append zero at end
                plot(simData.time, jerk, 'LineWidth', 1.5);
            end
            ylabel('Jerk (m/s³)');
            xlabel('Time (s)');
            grid on;
        end
        
        function plotEnergyAnalysis(simData, varargin)
            % Energy consumption and power analysis
            
            figure('Name', 'Energy Analysis', 'NumberTitle', 'off');
            
            power = simData.voltage .* simData.current;
            energy = cumtrapz(simData.time, power);
            
            subplot(2,1,1);
            plot(simData.time, power, 'LineWidth', 1.5);
            ylabel('Power (W)');
            grid on;
            title('Energy Analysis');
            
            subplot(2,1,2);
            plot(simData.time, energy, 'LineWidth', 1.5);
            ylabel('Energy (J)');
            xlabel('Time (s)');
            grid on;
            
            fprintf('Total Energy: %.3f J\n', energy(end));
        end
        
        function plotPhasePortrait(simData, varargin)
            % Phase portrait (velocity vs position)
            
            figure('Name', 'Phase Portrait', 'NumberTitle', 'off');
            
            plot(simData.position, simData.velocity, 'LineWidth', 1.5);
            xlabel('Position (m)');
            ylabel('Velocity (m/s)');
            title('Phase Portrait');
            grid on;
            
            hold on;
            plot(simData.position(1), simData.velocity(1), 'go', ...
                'MarkerSize', 10, 'MarkerFaceColor', 'g');
            plot(simData.position(end), simData.velocity(end), 'ro', ...
                'MarkerSize', 10, 'MarkerFaceColor', 'r');
            legend('Trajectory', 'Start', 'End', 'Location', 'best');
            hold off;
        end
        
        function plotMotorCurves(simData, varargin)
            % Motor torque-speed and power curves
            
            if ~isfield(simData, 'motorSpeed') || ~isfield(simData, 'motorTorque')
                warning('Motor data not available. Requires motorSpeed and motorTorque fields.');
                return;
            end
            
            figure('Name', 'Motor Characteristics', 'NumberTitle', 'off');
            
            subplot(2,1,1);
            plot(simData.motorSpeed, simData.motorTorque, 'LineWidth', 1.5);
            xlabel('Speed (rad/s)');
            ylabel('Torque (Nm)');
            title('Motor Torque-Speed Curve');
            grid on;
            
            subplot(2,1,2);
            power = simData.motorSpeed .* simData.motorTorque;
            plot(simData.motorSpeed, power, 'LineWidth', 1.5);
            xlabel('Speed (rad/s)');
            ylabel('Power (W)');
            title('Motor Power Curve');
            grid on;
        end
        
        function plotSimVsReal(simData, realData, varargin)
            % Compare simulation vs real robot data
            
            if nargin < 2
                error('Real robot data required');
            end
            
            figure('Name', 'Simulation vs Real', 'NumberTitle', 'off');
            
            subplot(3,1,1);
            plot(simData.time, simData.position, 'b-', 'LineWidth', 1.5);
            hold on;
            plot(realData.time, realData.position, 'r--', 'LineWidth', 1.5);
            ylabel('Position (m)');
            legend('Sim', 'Real');
            grid on;
            title('Simulation vs Real Data');
            hold off;
            
            subplot(3,1,2);
            plot(simData.time, simData.velocity, 'b-', 'LineWidth', 1.5);
            hold on;
            plot(realData.time, realData.velocity, 'r--', 'LineWidth', 1.5);
            ylabel('Velocity (m/s)');
            legend('Sim', 'Real');
            grid on;
            hold off;
            
            subplot(3,1,3);
            plot(simData.time, simData.current, 'b-', 'LineWidth', 1.5);
            hold on;
            plot(realData.time, realData.current, 'r--', 'LineWidth', 1.5);
            ylabel('Current (A)');
            xlabel('Time (s)');
            legend('Sim', 'Real');
            grid on;
            hold off;
        end
        
        function plotCorrelation(simData, realData, varargin)
            % Scatter plot correlation between sim and real
            
            if nargin < 2
                error('Real robot data required');
            end
            
            % Interpolate to common time base
            t_common = linspace(max(simData.time(1), realData.time(1)), ...
                min(simData.time(end), realData.time(end)), 100);
            
            sim_pos = interp1(simData.time, simData.position, t_common);
            real_pos = interp1(realData.time, realData.position, t_common);
            
            sim_vel = interp1(simData.time, simData.velocity, t_common);
            real_vel = interp1(realData.time, realData.velocity, t_common);
            
            figure('Name', 'Correlation Analysis', 'NumberTitle', 'off');
            
            subplot(1,2,1);
            scatter(real_pos, sim_pos, 'filled');
            hold on;
            plot([min(real_pos) max(real_pos)], [min(real_pos) max(real_pos)], 'r--');
            xlabel('Real Position (m)');
            ylabel('Sim Position (m)');
            title('Position Correlation');
            grid on;
            
            % Calculate R²
            R2_pos = 1 - sum((sim_pos - real_pos).^2) / sum((real_pos - mean(real_pos)).^2);
            text(0.05, 0.95, sprintf('R² = %.4f', R2_pos), 'Units', 'normalized');
            hold off;
            
            subplot(1,2,2);
            scatter(real_vel, sim_vel, 'filled');
            hold on;
            plot([min(real_vel) max(real_vel)], [min(real_vel) max(real_vel)], 'r--');
            xlabel('Real Velocity (m/s)');
            ylabel('Sim Velocity (m/s)');
            title('Velocity Correlation');
            grid on;
            
            R2_vel = 1 - sum((sim_vel - real_vel).^2) / sum((real_vel - mean(real_vel)).^2);
            text(0.05, 0.95, sprintf('R² = %.4f', R2_vel), 'Units', 'normalized');
            hold off;
        end
        
    end
end