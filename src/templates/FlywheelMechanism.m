classdef FlywheelMechanism < handle
    % Template for flywheel/shooter mechanisms
    % Velocity control with spin-up time optimization
    
    properties
        % Configuration
        mechanismName
        motorModel
        numMotors
        gearRatio
        
        % Physical properties
        wheelDiameter_m
        wheelMass_kg
        wheelInertia_kg_m2  % Moment of inertia of flywheel
        
        % Friction
        staticFriction_Nm
        viscousFriction
        
        % Targets
        targetRPM   
        targetSpeed_rad_s
        
        % Controllers
        feedforward
        pidGains  % Velocity PID
    end
    
    methods
        function obj = FlywheelMechanism(name, motorType)
            % Constructor
            obj.mechanismName = name;
            
            if strcmp(motorType, 'KrakenX40')
                obj.motorModel = KrakenX40Model();
            elseif strcmp(motorType, 'KrakenX60')
                obj.motorModel = KrakenX60Model();
            else
                error('Unknown motor type');
            end
            
            % Defaults for flywheel shooter
            obj.numMotors = 2;
            obj.gearRatio = 1.0;  % Often direct drive
            obj.wheelDiameter_m = 0.10;  % 4 inch wheel
            obj.staticFriction_Nm = 0.05;  % Low friction
            obj.viscousFriction = 0.005;
        end
        
        function loadCADData(obj, cadData)
            % Import flywheel properties
            obj.wheelMass_kg = cadData.mass;
            obj.wheelInertia_kg_m2 = cadData.inertia;
            
            if isfield(cadData, 'diameter_m')
                obj.wheelDiameter_m = cadData.diameter_m;
            end
            
            fprintf('%s: Loaded CAD data\n', obj.mechanismName);
            fprintf('  Wheel mass: %.3f kg\n', obj.wheelMass_kg);
            fprintf('  Wheel inertia: %.6f kg⋅m²\n', obj.wheelInertia_kg_m2);
            fprintf('  Diameter: %.3f m (%.1f in)\n', ...
                obj.wheelDiameter_m, obj.wheelDiameter_m * 39.37);
        end
        
        function setTargetSpeed(obj, targetRPM)
            % Set desired flywheel speed
            obj.targetRPM = targetRPM;
            obj.targetSpeed_rad_s = targetRPM * 2*pi / 60;
            
            fprintf('Target speed: %d RPM (%.2f rad/s)\n', ...
                targetRPM, obj.targetSpeed_rad_s);
        end
        
        function calculateFeedforward(obj)
            % Feedforward for velocity control (shooter)
            
            friction.static = obj.staticFriction_Nm;
            friction.viscous = obj.viscousFriction;
            
            % Velocity feedforward is most important for shooters
            [kV_base, ~, ~] = obj.motorModel.getFeedforwardConstants(obj.gearRatio);
            
            obj.feedforward.kV = kV_base / obj.numMotors;
            obj.feedforward.kA = 0;  % Not used for velocity control
            obj.feedforward.kS = 0.05;  % Small static friction
            obj.feedforward.kG = 0;  % No gravity
            
            % Calculate voltage for target speed
            if ~isempty(obj.targetSpeed_rad_s)
                obj.feedforward.voltageAtTarget = obj.feedforward.kV * obj.targetSpeed_rad_s;
                
                fprintf('\nShooter Feedforward:\n');
                fprintf('  kV: %.5f V/(rad/s)\n', obj.feedforward.kV);
                fprintf('  kS: %.4f V\n', obj.feedforward.kS);
                fprintf('  Voltage at %d RPM: %.2f V\n', ...
                    obj.targetRPM, obj.feedforward.voltageAtTarget);
            end
        end
        
        function calculatePID(obj, bandwidth_Hz)
            % Velocity PID for shooter
            %
            % bandwidth_Hz: Desired control bandwidth (typically 10-30 Hz)
            
            if nargin < 2
                bandwidth_Hz = 20;  % Default 20 Hz
            end
            
            systemParams.inertia = obj.wheelInertia_kg_m2;
            systemParams.damping = obj.viscousFriction;
            
            [kP, kI, ~] = PIDTuner.tuneVelocityPID(systemParams, bandwidth_Hz);
            
            obj.pidGains.kP = kP / obj.numMotors;
            obj.pidGains.kI = kI / obj.numMotors;
            obj.pidGains.kD = 0;  % Typically not used for velocity
            
            fprintf('\nShooter Velocity PI:\n');
            fprintf('  kP: %.4f\n', obj.pidGains.kP);
            fprintf('  kI: %.4f\n', obj.pidGains.kI);
            fprintf('  Bandwidth: %.1f Hz\n', bandwidth_Hz);
        end
        
        function spinUpTime = estimateSpinUpTime(obj)
            % Estimate time to reach target speed from rest
            
            if isempty(obj.targetSpeed_rad_s)
                fprintf('Error: Set target speed first\n');
                return;
            end
            
            % Maximum torque at low speed
            maxTorque = obj.motorModel.getMaxTorqueAtSpeed(0);
            totalTorque = maxTorque * obj.numMotors * obj.gearRatio;
            
            % Angular acceleration: τ = J*α
            alpha = totalTorque / obj.wheelInertia_kg_m2;
            
            % Time: t = ω / α
            spinUpTime = obj.targetSpeed_rad_s / alpha;
            
            fprintf('\nSpin-up Analysis:\n');
            fprintf('  Max torque: %.2f N⋅m (all motors)\n', totalTorque);
            fprintf('  Max acceleration: %.2f rad/s²\n', alpha);
            fprintf('  Time to %d RPM: %.3f s\n', obj.targetRPM, spinUpTime);
            
            % Energy stored in flywheel
            energy = 0.5 * obj.wheelInertia_kg_m2 * obj.targetSpeed_rad_s^2;
            fprintf('  Kinetic energy at speed: %.2f J\n', energy);
        end
        
        function recovery = estimateRecoveryTime(obj, speedDrop_percent)
            % Estimate time to recover speed after shot
            %
            % speedDrop_percent: Speed loss after releasing game piece (%)
            
            if nargin < 2
                speedDrop_percent = 10;  % Default 10% drop
            end
            
            if isempty(obj.targetSpeed_rad_s)
                fprintf('Error: Set target speed first\n');
                return;
            end
            
            % Speed after shot
            speedAfter = obj.targetSpeed_rad_s * (1 - speedDrop_percent/100);
            speedLoss = obj.targetSpeed_rad_s - speedAfter;
            
            % Recovery with PI control
            maxTorque = obj.motorModel.getMaxTorqueAtSpeed(speedAfter);
            totalTorque = maxTorque * obj.numMotors * obj.gearRatio;
            alpha = totalTorque / obj.wheelInertia_kg_m2;
            
            recovery = speedLoss / alpha;
            
            fprintf('\nRecovery Analysis:\n');
            fprintf('  Speed drop: %.0f%% (%d RPM)\n', ...
                speedDrop_percent, round(speedLoss * 60/(2*pi)));
            fprintf('  Recovery time: %.3f s\n', recovery);
            fprintf('  Sustained fire rate: ~%.1f shots/s\n', 1/(recovery + 0.2));
        end
        
        function exportToRobot(obj, filename)
            % Export to JSON
            
            output.mechanism = obj.mechanismName;
            output.type = 'Flywheel';
            output.targetRPM = obj.targetRPM;
            output.targetSpeed_rad_s = obj.targetSpeed_rad_s;
            output.feedforward = obj.feedforward;
            output.pid = obj.pidGains;
            
            jsonStr = jsonencode(output);
            fid = fopen(filename, 'w');
            fprintf(fid, '%s', jsonStr);
            fclose(fid);
            
            fprintf('\nExported to: %s\n', filename);
        end
    end
end
