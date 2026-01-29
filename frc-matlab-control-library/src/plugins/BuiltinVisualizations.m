classdef BuiltinVisualizations
    % Standard visualization plugins for the system
    % 
    % These are automatically registered in VisualizationSystemInit.m
    
    methods (Static)
        % ===== STANDARD SIMULATION PLOTS =====
        
        function plotStandardSimulation(simData, varargin)
            % Plot basic simulation results: position, velocity, current, voltage
            
            p = inputParser();
            addParameter(p, 'title', 'Simulation Results', @ischar);
            parse(p, varargin{:});
            
            figure('Name', p.Results.title);
            
            % Position
            subplot(2, 2, 1);
            plot(simData.time, simData.position, 'b-', 'LineWidth', 1.5);
            grid on;
            ylabel('Position (rad)');
            title('Position vs Time');
            
            % Velocity
            subplot(2, 2, 2);
            plot(simData.time, simData.velocity, 'g-', 'LineWidth', 1.5);
            grid on;
            ylabel('Velocity (rad/s)');
            title('Velocity vs Time');
            
            % Current
            subplot(2, 2, 3);
            plot(simData.time, simData.current, 'm-', 'LineWidth', 1.5);
            hold on;
            yline(40, 'r--', 'LineWidth', 1, 'Label', '40A Limit');
            grid on;
            ylabel('Current (A)');
            xlabel('Time (s)');
            title('Current Draw');
            
            % Voltage
            subplot(2, 2, 4);
            plot(simData.time, simData.voltage, 'c-', 'LineWidth', 1.5);
            grid on;
            ylabel('Voltage (V)');
            xlabel('Time (s)');
            title('Applied Voltage');
            
            sgtitle(p.Results.title, 'FontSize', 14, 'FontWeight', 'bold');
        end
        
        function plotEnergyAnalysis(simData, varargin)
            % Analyze energy consumption and efficiency
            
            p = inputParser();
            addParameter(p, 'title', 'Energy Analysis', @ischar);
            parse(p, varargin{:});
            
            % Calculate instantaneous power and energy
            dt = mean(diff(simData.time));
            power = simData.voltage .* simData.current;  % Instantaneous power (W)
            energy = cumsum(power * dt) / 1000;  % Energy (kJ)
            
            figure('Name', p.Results.title);
            
            % Power vs time
            subplot(2, 2, 1);
            plot(simData.time, power, 'b-', 'LineWidth', 1.5);
            grid on;
            ylabel('Power (W)');
            title('Instantaneous Power');
            
            % Cumulative energy
            subplot(2, 2, 2);
            plot(simData.time, energy, 'r-', 'LineWidth', 1.5);
            grid on;
            ylabel('Energy (kJ)');
            title('Cumulative Energy Usage');
            
            % Power histogram
            subplot(2, 2, 3);
            histogram(power, 20, 'FaceColor', 'g', 'EdgeColor', 'k');
            grid on;
            ylabel('Frequency');
            xlabel('Power (W)');
            title('Power Distribution');
            
            % Statistics
            subplot(2, 2, 4);
            axis off;
            stats = {
                sprintf('Peak Power: %.1f W', max(power));
                sprintf('Mean Power: %.1f W', mean(power));
                sprintf('Total Energy: %.3f kJ', energy(end));
                sprintf('Duration: %.2f s', simData.time(end));
                sprintf('Avg Current: %.1f A', mean(simData.current))
            };
            text(0.1, 0.9, stats, 'VerticalAlignment', 'top', ...
                'FontName', 'Monospace', 'FontSize', 10);
            
            sgtitle(p.Results.title, 'FontSize', 14, 'FontWeight', 'bold');
        end
        
        function plotDetailedTrajectory(simData, varargin)
            % Plot position, velocity, acceleration, and jerk
            
            p = inputParser();
            addParameter(p, 'title', 'Detailed Trajectory', @ischar);
            parse(p, varargin{:});
            
            % Calculate acceleration and jerk
            dt = mean(diff(simData.time));
            accel = [0; diff(simData.velocity) / dt];
            jerk = [0; diff(accel) / dt];
            
            figure('Name', p.Results.title);
            
            % Position
            subplot(4, 1, 1);
            plot(simData.time, simData.position, 'b-', 'LineWidth', 1.5);
            grid on;
            ylabel('Position (rad)');
            title('Position');
            
            % Velocity
            subplot(4, 1, 2);
            plot(simData.time, simData.velocity, 'g-', 'LineWidth', 1.5);
            grid on;
            ylabel('Velocity (rad/s)');
            title('Velocity');
            
            % Acceleration
            subplot(4, 1, 3);
            plot(simData.time, accel, 'm-', 'LineWidth', 1.5);
            grid on;
            ylabel('Acceleration (rad/s²)');
            title('Acceleration');
            
            % Jerk
            subplot(4, 1, 4);
            plot(simData.time, jerk, 'c-', 'LineWidth', 1.5);
            grid on;
            ylabel('Jerk (rad/s³)');
            xlabel('Time (s)');
            title('Jerk');
            
            sgtitle(p.Results.title, 'FontSize', 14, 'FontWeight', 'bold');
        end
        
        function plotPhasePortrait(simData, varargin)
            % Plot velocity vs position (phase portrait)
            
            p = inputParser();
            addParameter(p, 'title', 'Phase Portrait', @ischar);
            parse(p, varargin{:});
            
            figure('Name', p.Results.title);
            
            % Phase portrait with time coloring
            plot(simData.position, simData.velocity, 'b-', 'LineWidth', 2);
            hold on;
            scatter(simData.position(1), simData.velocity(1), 100, 'g', 'filled', ...
                'DisplayName', 'Start');
            scatter(simData.position(end), simData.velocity(end), 100, 'r', 'filled', ...
                'DisplayName', 'End');
            
            grid on;
            xlabel('Position (rad)');
            ylabel('Velocity (rad/s)');
            title(p.Results.title);
            legend('show');
            
            sgtitle(p.Results.title, 'FontSize', 14, 'FontWeight', 'bold');
        end
        
        % ===== MOTOR CHARACTERISTIC PLOTS =====
        
        function plotMotorCurves(simData, motor, gearRatio, varargin)
            % Plot motor torque-speed and power curves
            
            p = inputParser();
            addParameter(p, 'title', 'Motor Characteristics', @ischar);
            parse(p, varargin{:});
            
            % Motor curves
            freqPoints = 200;
            speedRange = linspace(0, motor.maxSpeed_rpm / 60 * 2 * pi, freqPoints);
            
            torque = zeros(size(speedRange));
            power = zeros(size(speedRange));
            
            for i = 1:length(speedRange)
                % Back EMF voltage
                kemf = 60 / (2*pi) / motor.KV;
                backEmf = kemf * speedRange(i);
                
                % Current at this speed (12V nominal)
                current = (12 - backEmf) / motor.resistance;
                
                % Torque and power
                torque(i) = current * (motor.torqueConstant);
                power(i) = torque(i) * speedRange(i);
            end
            
            figure('Name', p.Results.title);
            
            % Torque-speed curve
            subplot(1, 2, 1);
            plot(speedRange * gearRatio * 60 / (2*pi), torque * gearRatio, ...
                'b-', 'LineWidth', 2);
            grid on;
            xlabel('Output Speed (rpm)');
            ylabel('Output Torque (N⋅m)');
            title('Torque-Speed Curve');
            
            % Power curve
            subplot(1, 2, 2);
            plot(speedRange * gearRatio * 60 / (2*pi), power * gearRatio / 1000, ...
                'r-', 'LineWidth', 2);
            grid on;
            xlabel('Output Speed (rpm)');
            ylabel('Power (kW)');
            title('Power Curve');
            
            sgtitle(p.Results.title, 'FontSize', 14, 'FontWeight', 'bold');
        end
        
        % ===== COMPARISON PLOTS =====
        
        function plotSimVsReal(simData, realData, varargin)
            % Compare simulation vs real robot data
            
            p = inputParser();
            addParameter(p, 'title', 'Simulation vs Reality', @ischar);
            addParameter(p, 'mechanism', 'Mechanism', @ischar);
            parse(p, varargin{:});
            
            figure('Name', p.Results.title);
            
            % Position comparison
            subplot(2, 2, 1);
            plot(simData.time, simData.position, 'b-', 'LineWidth', 1.5, ...
                'DisplayName', 'Simulation');
            hold on;
            plot(realData.time, realData.position, 'r--', 'LineWidth', 1.5, ...
                'DisplayName', 'Real');
            grid on;
            ylabel('Position (rad)');
            title('Position Comparison');
            legend('show');
            
            % Velocity comparison
            subplot(2, 2, 2);
            plot(simData.time, simData.velocity, 'b-', 'LineWidth', 1.5, ...
                'DisplayName', 'Simulation');
            hold on;
            plot(realData.time, realData.velocity, 'r--', 'LineWidth', 1.5, ...
                'DisplayName', 'Real');
            grid on;
            ylabel('Velocity (rad/s)');
            title('Velocity Comparison');
            legend('show');
            
            % Position error
            subplot(2, 2, 3);
            % Interpolate to same time base
            realPos_interp = interp1(realData.time, realData.position, simData.time);
            error = simData.position - realPos_interp;
            plot(simData.time, error, 'g-', 'LineWidth', 1.5);
            grid on;
            ylabel('Position Error (rad)');
            xlabel('Time (s)');
            title('Position Error');
            
            % Velocity error
            subplot(2, 2, 4);
            realVel_interp = interp1(realData.time, realData.velocity, simData.time);
            error = simData.velocity - realVel_interp;
            plot(simData.time, error, 'c-', 'LineWidth', 1.5);
            grid on;
            ylabel('Velocity Error (rad/s)');
            xlabel('Time (s)');
            title('Velocity Error');
            
            sgtitle(p.Results.title, 'FontSize', 14, 'FontWeight', 'bold');
        end
        
        function plotCorrelation(simData, realData, varargin)
            % Plot correlation between sim and real data
            
            p = inputParser();
            addParameter(p, 'title', 'Sim vs Real Correlation', @ischar);
            parse(p, varargin{:});
            
            % Interpolate to same time base
            realPos_interp = interp1(realData.time, realData.position, simData.time);
            realVel_interp = interp1(realData.time, realData.velocity, simData.time);
            
            figure('Name', p.Results.title);
            
            % Position correlation scatter
            subplot(1, 2, 1);
            scatter(realPos_interp, simData.position, 30, 'b', 'filled');
            hold on;
            % Add unity line
            lim = [min([realPos_interp; simData.position]), ...
                   max([realPos_interp; simData.position])];
            plot(lim, lim, 'r--', 'LineWidth', 2, 'DisplayName', 'Perfect fit');
            grid on;
            xlabel('Real Position (rad)');
            ylabel('Simulated Position (rad)');
            title('Position Correlation');
            legend('show');
            axis equal;
            
            % Velocity correlation scatter
            subplot(1, 2, 2);
            scatter(realVel_interp, simData.velocity, 30, 'g', 'filled');
            hold on;
            lim = [min([realVel_interp; simData.velocity]), ...
                   max([realVel_interp; simData.velocity])];
            plot(lim, lim, 'r--', 'LineWidth', 2, 'DisplayName', 'Perfect fit');
            grid on;
            xlabel('Real Velocity (rad/s)');
            ylabel('Simulated Velocity (rad/s)');
            title('Velocity Correlation');
            legend('show');
            axis equal;
            
            sgtitle(p.Results.title, 'FontSize', 14, 'FontWeight', 'bold');
        end
    end
end
