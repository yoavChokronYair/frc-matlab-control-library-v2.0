classdef MotionMagicOptimizer < handle
    % Optimizes Motion Magic S-curve profiles for smooth, fast motion
    
    methods (Static)
        function profile = optimizeProfile(mechanism, constraints)
            % Optimize Motion Magic profile parameters
            %
            % Inputs:
            %   mechanism: Mechanism object with motor, inertia, etc.
            %   constraints: struct with .maxTime_s, .maxJerk, .safetyFactor
            %
            % Outputs:
            %   profile: struct with .cruiseVelocity, .acceleration, .jerk
            
            if nargin < 2
                constraints = struct();
            end
            
            % Default constraints
            if ~isfield(constraints, 'maxTime_s')
                constraints.maxTime_s = 2.0;  % Max time for full range
            end
            if ~isfield(constraints, 'maxJerk')
                constraints.maxJerk = 1000;  % rad/s³
            end
            if ~isfield(constraints, 'safetyFactor')
                constraints.safetyFactor = 0.80;  % 80% of theoretical max
            end
            
            profile = struct();
            
            % Calculate theoretical maximum velocity
            motorMaxSpeed = mechanism.motorModel.FREE_SPEED_RAD_S;
            theoreticalMaxVel = (motorMaxSpeed / mechanism.gearRatio) * constraints.safetyFactor;
            
            % Calculate theoretical maximum acceleration (from torque)
            maxTorque = mechanism.motorModel.getMaxTorqueAtSpeed(0);  % Stall torque
            totalTorque = maxTorque * mechanism.numMotors * mechanism.gearRatio;
            theoreticalMaxAccel = (totalTorque / mechanism.inertia_kg_m2) * constraints.safetyFactor;
            
            % Motion Magic parameters
            profile.cruiseVelocity = theoreticalMaxVel;
            profile.acceleration = theoreticalMaxAccel;
            profile.jerk = min(theoreticalMaxAccel * 10, constraints.maxJerk);
            
            % Validate profile meets time constraint
            if isfield(mechanism, 'maxPosition_rad') && isfield(mechanism, 'minPosition_rad')
                totalDistance = mechanism.maxPosition_rad - mechanism.minPosition_rad;
                estimatedTime = MotionMagicOptimizer.estimateTime(totalDistance, profile);
                
                if estimatedTime > constraints.maxTime_s
                    % Need to speed up - relax safety factor
                    fprintf('Warning: Profile too slow (%.2fs > %.2fs)\n', ...
                        estimatedTime, constraints.maxTime_s);
                    fprintf('         Increasing speeds...\n');
                    
                    scaleFactor = estimatedTime / constraints.maxTime_s;
                    profile.cruiseVelocity = profile.cruiseVelocity * scaleFactor;
                    profile.acceleration = profile.acceleration * scaleFactor;
                end
            end
            
            fprintf('\nOptimized Motion Magic Profile:\n');
            fprintf('  Cruise Velocity: %.2f rad/s (%.0f deg/s)\n', ...
                profile.cruiseVelocity, rad2deg(profile.cruiseVelocity));
            fprintf('  Acceleration: %.2f rad/s²\n', profile.acceleration);
            fprintf('  Jerk: %.2f rad/s³\n', profile.jerk);
            
            % Calculate time for various distances
            distances = [pi/4, pi/2, pi, 2*pi];
            fprintf('\nEstimated Move Times:\n');
            for d = distances
                t = MotionMagicOptimizer.estimateTime(d, profile);
                fprintf('  %.2f rad (%.0f°): %.3f s\n', d, rad2deg(d), t);
            end
        end
        
        function time = estimateTime(distance, profile)
            % Estimate time to move a given distance with S-curve profile
            %
            % Three phases: 
            %   1. Acceleration (limited by jerk)
            %   2. Cruise (constant velocity)
            %   3. Deceleration (limited by jerk)
            
            v_cruise = profile.cruiseVelocity;
            a_max = profile.acceleration;
            j_max = profile.jerk;
            
            % Time to reach max acceleration (jerk limited)
            t_jerk = a_max / j_max;
            
            % Distance during acceleration ramp-up (jerk phase)
            d_jerk_up = (1/6) * j_max * t_jerk^3;
            
            % Time at max acceleration (after jerk phase)
            % v = j*t_jerk^2/2 + a_max*(t_accel - t_jerk)
            % Solve for t_accel when v = v_cruise
            syms t_accel_sym
            v_eq = j_max * t_jerk^2 / 2 + a_max * (t_accel_sym - t_jerk);
            t_accel_solution = solve(v_eq == v_cruise, t_accel_sym);
            t_accel = double(t_accel_solution);
            
            if isempty(t_accel) || t_accel < t_jerk
                % Can't reach cruise velocity
                t_accel = t_jerk;
            end
            
            % Distance during acceleration
            d_accel = d_jerk_up + a_max * (t_accel - t_jerk)^2 / 2;
            
            % Same for deceleration (symmetric)
            d_decel = d_accel;
            t_decel = t_accel;
            
            % Distance during cruise
            d_cruise = distance - (d_accel + d_decel);
            
            if d_cruise < 0
                % Never reach cruise velocity - triangular profile
                % Simplified: symmetric acceleration/deceleration
                t_half = sqrt(distance / a_max);
                time = 2 * t_half;
            else
                % Trapezoidal profile
                t_cruise = d_cruise / v_cruise;
                time = t_accel + t_cruise + t_decel;
            end
        end
        
        function trajectory = generateTrajectory(distance, profile, dt)
            % Generate complete position/velocity/acceleration trajectory
            %
            % Inputs:
            %   distance: Total distance to move (rad)
            %   profile: Motion Magic profile
            %   dt: Time step (s)
            %
            % Outputs:
            %   trajectory: struct with .time, .position, .velocity, .acceleration
            
            if nargin < 3
                dt = 0.01;  % 10ms timestep
            end
            
            % Estimate total time
            totalTime = MotionMagicOptimizer.estimateTime(distance, profile);
            
            % Time vector
            time = 0:dt:totalTime;
            N = length(time);
            
            % Pre-allocate
            position = zeros(N, 1);
            velocity = zeros(N, 1);
            acceleration = zeros(N, 1);
            jerk = zeros(N, 1);
            
            % S-curve profile parameters
            v_max = profile.cruiseVelocity;
            a_max = profile.acceleration;
            j_max = profile.jerk;
            
            % Phase durations
            t_jerk = a_max / j_max;
            
            % Generate trajectory using numerical integration
            for i = 2:N
                t = time(i);
                
                % Determine current jerk (S-curve)
                if t < t_jerk
                    % Jerk up
                    jerk(i) = j_max;
                elseif t < 2*t_jerk
                    % Jerk down (reach max accel)
                    jerk(i) = -j_max;
                else
                    % Check if decelerating
                    remaining = distance - position(i-1);
                    stoppingDist = velocity(i-1)^2 / (2*a_max);
                    
                    if remaining < stoppingDist * 1.5
                        % Start deceleration
                        jerk(i) = -j_max;
                    else
                        jerk(i) = 0;  % Cruise
                    end
                end
                
                % Integrate: jerk → acceleration → velocity → position
                acceleration(i) = acceleration(i-1) + jerk(i) * dt;
                acceleration(i) = max(-a_max, min(a_max, acceleration(i)));  % Clamp
                
                velocity(i) = velocity(i-1) + acceleration(i) * dt;
                velocity(i) = max(0, min(v_max, velocity(i)));  % Clamp
                
                position(i) = position(i-1) + velocity(i) * dt;
                
                % Stop when reached target
                if position(i) >= distance
                    position(i:end) = distance;
                    velocity(i:end) = 0;
                    acceleration(i:end) = 0;
                    jerk(i:end) = 0;
                    break;
                end
            end
            
            % Output
            trajectory.time = time';
            trajectory.position = position;
            trajectory.velocity = velocity;
            trajectory.acceleration = acceleration;
            trajectory.jerk = jerk;
        end
        
        function plotTrajectory(trajectory, title_str)
            % Plot complete motion profile
            
            if nargin < 2
                title_str = 'Motion Magic Trajectory';
            end
            
            figure('Name', 'Motion Profile');
            
            subplot(4, 1, 1);
            plot(trajectory.time, trajectory.position, 'b-', 'LineWidth', 1.5);
            grid on;
            ylabel('Position (rad)');
            title(title_str);
            
            subplot(4, 1, 2);
            plot(trajectory.time, trajectory.velocity, 'r-', 'LineWidth', 1.5);
            grid on;
            ylabel('Velocity (rad/s)');
            
            subplot(4, 1, 3);
            plot(trajectory.time, trajectory.acceleration, 'g-', 'LineWidth', 1.5);
            grid on;
            ylabel('Accel (rad/s²)');
            
            subplot(4, 1, 4);
            plot(trajectory.time, trajectory.jerk, 'm-', 'LineWidth', 1.5);
            grid on;
            ylabel('Jerk (rad/s³)');
            xlabel('Time (s)');
        end
        
        function compareProfiles(profiles, labels, distance)
            % Compare multiple Motion Magic profiles
            %
            % Inputs:
            %   profiles: Cell array of profile structs
            %   labels: Cell array of labels
            %   distance: Test distance (rad)
            
            figure('Name', 'Profile Comparison');
            colors = lines(length(profiles));
            
            for p = 1:length(profiles)
                traj = MotionMagicOptimizer.generateTrajectory(distance, profiles{p});
                
                subplot(2, 2, 1);
                plot(traj.time, traj.position, 'LineWidth', 1.5, ...
                    'Color', colors(p,:), 'DisplayName', labels{p});
                hold on;
                
                subplot(2, 2, 2);
                plot(traj.time, traj.velocity, 'LineWidth', 1.5, ...
                    'Color', colors(p,:), 'DisplayName', labels{p});
                hold on;
                
                subplot(2, 2, 3);
                plot(traj.time, traj.acceleration, 'LineWidth', 1.5, ...
                    'Color', colors(p,:), 'DisplayName', labels{p});
                hold on;
                
                subplot(2, 2, 4);
                plot(traj.time, traj.jerk, 'LineWidth', 1.5, ...
                    'Color', colors(p,:), 'DisplayName', labels{p});
                hold on;
            end
            
            subplot(2, 2, 1);
            grid on; ylabel('Position (rad)'); title('Position');
            legend('Location', 'best');
            
            subplot(2, 2, 2);
            grid on; ylabel('Velocity (rad/s)'); title('Velocity');
            legend('Location', 'best');
            
            subplot(2, 2, 3);
            grid on; ylabel('Accel (rad/s²)'); title('Acceleration');
            xlabel('Time (s)'); legend('Location', 'best');
            
            subplot(2, 2, 4);
            grid on; ylabel('Jerk (rad/s³)'); title('Jerk');
            xlabel('Time (s)'); legend('Location', 'best');
        end
    end
end
