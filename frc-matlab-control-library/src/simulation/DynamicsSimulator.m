classdef DynamicsSimulator < handle
    % Simulates mechanism dynamics given motor inputs
    % Produces predicted position, velocity, current for validation
    
    methods (Static)
        function simData = simulateResponse(mechanism, testSequence)
            % Simulate mechanism response to voltage/position commands
            %
            % Inputs:
            %   mechanism: SimpleRotationalMechanism or similar
            %   testSequence: struct with .time and .voltage (or .position)
            %
            % Outputs:
            %   simData: struct with .time, .position, .velocity, .current
            
            fprintf('Simulating mechanism dynamics...\n');
            
            % Extract parameters
            J = mechanism.inertia_kg_m2;
            b = mechanism.viscousFriction;
            kS = mechanism.staticFriction_Nm;
            motor = mechanism.motorModel;
            ratio = mechanism.gearRatio;
            numMotors = mechanism.numMotors;
            
            % Time vector
            time = testSequence.time;
            dt = mean(diff(time));
            N = length(time);
            
            % Pre-allocate outputs
            position = zeros(N, 1);
            velocity = zeros(N, 1);
            acceleration = zeros(N, 1);
            current = zeros(N, 1);
            
            % Initial conditions
            position(1) = 0;
            velocity(1) = 0;
            
            % Simulate using Euler integration
            for i = 2:N
                % Current voltage command
                V_cmd = testSequence.voltage(i);
                
                % Motor speed (rad/s at motor shaft)
                motorSpeed = velocity(i-1) * ratio;
                
                % Back-EMF
                Kemf = 1 / motor.KV * (60 / (2*pi));
                V_bemf = Kemf * motorSpeed;
                
                % Current draw
                I = (V_cmd - V_bemf) / motor.RESISTANCE_OHM;
                I = min(I, motor.currentLimit_A);  % Clamp to current limit
                current(i) = I;
                
                % Motor torque
                tau_motor = motor.KT * I;
                
                % Output torque (all motors combined)
                tau_output = tau_motor * ratio * numMotors;
                
                % Friction torque
                if abs(velocity(i-1)) > 0.01
                    tau_friction = kS * sign(velocity(i-1)) + b * velocity(i-1);
                else
                    tau_friction = kS * sign(V_cmd);  % Static friction
                end
                
                % Net torque
                tau_net = tau_output - tau_friction;
                
                % Acceleration: τ = J*α
                acceleration(i) = tau_net / J;
                
                % Integrate to get velocity and position (Euler method)
                velocity(i) = velocity(i-1) + acceleration(i) * dt;
                position(i) = position(i-1) + velocity(i) * dt;
                
                % Apply position limits if they exist
                if isfield(mechanism, 'minPosition_rad')
                    position(i) = max(position(i), mechanism.minPosition_rad);
                end
                if isfield(mechanism, 'maxPosition_rad')
                    position(i) = min(position(i), mechanism.maxPosition_rad);
                end
            end
            
            % Package results
            simData.time = time;
            simData.position = position;
            simData.velocity = velocity;
            simData.acceleration = acceleration;
            simData.current = current;
            simData.voltage = testSequence.voltage;
            
            fprintf('Simulation complete.\n');
        end
        
        function simData = simulateWithController(mechanism, trajectory, controller)
            % Simulate closed-loop response with PID + Feedforward
            %
            % Inputs:
            %   mechanism: Mechanism model
            %   trajectory: Desired trajectory (.time, .position, .velocity)
            %   controller: struct with .ff (feedforward) and .pid (PID gains)
            %
            % Outputs:
            %   simData: struct with actual response
            
            fprintf('Simulating closed-loop control...\n');
            
            % Extract parameters
            J = mechanism.inertia_kg_m2;
            b = mechanism.viscousFriction;
            kS = mechanism.staticFriction_Nm;
            motor = mechanism.motorModel;
            ratio = mechanism.gearRatio;
            numMotors = mechanism.numMotors;
            
            % Controller gains
            ff = controller.ff;
            kP = controller.pid.kP;
            kI = controller.pid.kI;
            kD = controller.pid.kD;
            
            % Time vector
            time = trajectory.time;
            dt = mean(diff(time));
            N = length(time);
            
            % Pre-allocate
            position = zeros(N, 1);
            velocity = zeros(N, 1);
            voltage = zeros(N, 1);
            current = zeros(N, 1);
            error_integral = 0;
            
            % Initial conditions
            position(1) = trajectory.position(1);
            velocity(1) = 0;
            
            % Simulation loop
            for i = 2:N
                % Desired state
                pos_des = trajectory.position(i);
                vel_des = trajectory.velocity(i);
                
                % Error
                pos_error = pos_des - position(i-1);
                vel_error = vel_des - velocity(i-1);
                error_integral = error_integral + pos_error * dt;
                
                % PID control
                V_pid = kP * pos_error + kI * error_integral + kD * vel_error;
                
                % Feedforward control
                V_ff = ff.kS * sign(vel_des) + ff.kV * vel_des;
                
                % Total voltage
                voltage(i) = V_pid + V_ff;
                voltage(i) = max(-12, min(12, voltage(i)));  % Saturation
                
                % Motor dynamics (same as simulateResponse)
                motorSpeed = velocity(i-1) * ratio;
                Kemf = 1 / motor.KV * (60 / (2*pi));
                V_bemf = Kemf * motorSpeed;
                
                I = (voltage(i) - V_bemf) / motor.RESISTANCE_OHM;
                I = min(I, motor.currentLimit_A);
                current(i) = I;
                
                tau_motor = motor.KT * I;
                tau_output = tau_motor * ratio * numMotors;
                
                % Friction
                if abs(velocity(i-1)) > 0.01
                    tau_friction = kS * sign(velocity(i-1)) + b * velocity(i-1);
                else
                    tau_friction = kS * sign(voltage(i));
                end
                
                tau_net = tau_output - tau_friction;
                accel = tau_net / J;
                
                % Integrate
                velocity(i) = velocity(i-1) + accel * dt;
                position(i) = position(i-1) + velocity(i) * dt;
            end
            
            % Package results
            simData.time = time;
            simData.position = position;
            simData.velocity = velocity;
            simData.voltage = voltage;
            simData.current = current;
            simData.trajectory = trajectory;  % Include desired for comparison
            
            fprintf('Closed-loop simulation complete.\n');
        end
        
        function plotSimulation(simData, title_str)
            % Plot simulation results
            
            if nargin < 2
                title_str = 'Simulation Results';
            end
            
            figure('Name', title_str);
            
            subplot(4, 1, 1);
            plot(simData.time, simData.position, 'b-', 'LineWidth', 1.5);
            if isfield(simData, 'trajectory')
                hold on;
                plot(simData.trajectory.time, simData.trajectory.position, ...
                    'r--', 'LineWidth', 1.5);
                legend('Actual', 'Desired');
            end
            grid on;
            ylabel('Position (rad)');
            title(title_str);
            
            subplot(4, 1, 2);
            plot(simData.time, simData.velocity, 'b-', 'LineWidth', 1.5);
            if isfield(simData, 'trajectory')
                hold on;
                plot(simData.trajectory.time, simData.trajectory.velocity, ...
                    'r--', 'LineWidth', 1.5);
                legend('Actual', 'Desired');
            end
            grid on;
            ylabel('Velocity (rad/s)');
            
            subplot(4, 1, 3);
            plot(simData.time, simData.voltage, 'g-', 'LineWidth', 1.5);
            hold on;
            yline(12, 'r--', '12V');
            yline(-12, 'r--', '-12V');
            grid on;
            ylabel('Voltage (V)');
            ylim([-13 13]);
            
            subplot(4, 1, 4);
            plot(simData.time, simData.current, 'm-', 'LineWidth', 1.5);
            hold on;
            yline(40, 'r--', '40A Limit');
            grid on;
            ylabel('Current (A)');
            xlabel('Time (s)');
        end
        
        function energy = calculateEnergyUsage(simData)
            % Calculate total electrical energy consumed
            %
            % Output: energy in Joules
            
            dt = mean(diff(simData.time));
            
            % Power = Voltage × Current
            power = abs(simData.voltage .* simData.current);  % Watts
            
            % Energy = ∫ Power dt
            energy = sum(power) * dt;  % Joules
            
            fprintf('Total Energy: %.2f J (%.4f Wh)\n', energy, energy/3600);
            fprintf('Average Power: %.2f W\n', mean(power));
            fprintf('Peak Power: %.2f W\n', max(power));
        end
    end
end
