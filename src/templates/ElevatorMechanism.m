classdef ElevatorMechanism < handle
    % Template for elevator/linear mechanisms
    % Vertical motion with gravity compensation
    
    properties
        % Configuration
        mechanismName
        motorModel
        numMotors
        gearRatio           % Motor rotations : spool rotations
        
        % Physical properties (from CAD)
        carriageMass_kg     % Mass of moving carriage
        spoolDiameter_m     % Diameter of pulley/spool
        spoolRadius_m       % Radius for calculations
        
        % Friction (from tests)
        staticFriction_N    % Static friction force
        viscousFriction     % Viscous damping
        
        % Limits
        minHeight_m
        maxHeight_m
        maxVelocity_m_s
        maxAcceleration_m_s2
        
        % Controllers (outputs)
        feedforward
        pidGains
        motionMagicProfile
    end
    
    methods
        function obj = ElevatorMechanism(name, motorType)
            % Constructor
            obj.mechanismName = name;
            
            if strcmp(motorType, 'KrakenX40')
                obj.motorModel = KrakenX40Model();
            elseif strcmp(motorType, 'KrakenX60')
                obj.motorModel = KrakenX60Model();
            else
                error('Unknown motor type');
            end
            
            % Defaults
            obj.numMotors = 2;  % Typical: 2 motors for elevator
            obj.gearRatio = 12.0;
            obj.spoolDiameter_m = 0.05;  % 2 inch spool
            obj.spoolRadius_m = obj.spoolDiameter_m / 2;
            obj.staticFriction_N = 10.0;  % Estimate
            obj.viscousFriction = 1.0;
        end
        
        function loadCADData(obj, cadData)
            % Import data from SolidWorks
            obj.carriageMass_kg = cadData.mass;
            
            if isfield(cadData, 'spoolDiameter_m')
                obj.spoolDiameter_m = cadData.spoolDiameter_m;
                obj.spoolRadius_m = obj.spoolDiameter_m / 2;
            end
            
            fprintf('%s: Loaded CAD data\n', obj.mechanismName);
            fprintf('  Carriage mass: %.2f kg\n', obj.carriageMass_kg);
            fprintf('  Spool diameter: %.3f m (%.1f in)\n', ...
                obj.spoolDiameter_m, obj.spoolDiameter_m * 39.37);
        end
        
        function setLimits(obj, minHeight, maxHeight, maxVel, maxAccel)
            % Set motion limits (in meters)
            obj.minHeight_m = minHeight;
            obj.maxHeight_m = maxHeight;
            obj.maxVelocity_m_s = maxVel;
            obj.maxAcceleration_m_s2 = maxAccel;
        end
        
        function calculateFeedforward(obj)
            % Calculate feedforward for elevator (with gravity)
            
            % Convert linear to equivalent rotational
            % F = m*a → τ = J*α, where J = m*r²
            inertia_equiv = obj.carriageMass_kg * obj.spoolRadius_m^2;
            
            friction_rot.static = obj.staticFriction_N * obj.spoolRadius_m;
            friction_rot.viscous = obj.viscousFriction * obj.spoolRadius_m;
            
            % Basic rotational feedforward
            obj.feedforward = FeedforwardCalculator.calculateSimpleRotational(...
                obj.motorModel, obj.gearRatio, inertia_equiv, friction_rot);
            
            % Gravity feedforward (constant upward force)
            g = 9.81;  % m/s²
            gravityForce = obj.carriageMass_kg * g;
            gravityTorque = gravityForce * obj.spoolRadius_m;
            
            % Voltage to overcome gravity (constant)
            obj.feedforward.kG = (gravityTorque / (obj.motorModel.KT * obj.gearRatio)) * ...
                obj.motorModel.RESISTANCE_OHM;
            
            % Divide by number of motors
            obj.feedforward.kS = obj.feedforward.kS / obj.numMotors;
            obj.feedforward.kV = obj.feedforward.kV / obj.numMotors;
            obj.feedforward.kA = obj.feedforward.kA / obj.numMotors;
            obj.feedforward.kG = obj.feedforward.kG / obj.numMotors;
            
            fprintf('\nElevator Feedforward:\n');
            fprintf('  kS: %.4f V\n', obj.feedforward.kS);
            fprintf('  kV: %.4f V/(m/s) at motor\n', obj.feedforward.kV);
            fprintf('  kA: %.4f V/(m/s²) at motor\n', obj.feedforward.kA);
            fprintf('  kG: %.4f V (gravity compensation)\n', obj.feedforward.kG);
            fprintf('\nNote: In robot code, add kG as constant voltage offset\n');
        end
        
        function calculatePID(obj, settlingTime_s, overshoot_percent)
            % Calculate PID gains for linear motion control
            
            inertia_equiv = obj.carriageMass_kg * obj.spoolRadius_m^2;
            
            systemParams.inertia = inertia_equiv;
            systemParams.damping = obj.viscousFriction * obj.spoolRadius_m;
            
            desiredResponse.settlingTime_s = settlingTime_s;
            desiredResponse.overshoot_percent = overshoot_percent;
            
            [kP, kI, kD] = PIDTuner.tunePositionPID(systemParams, desiredResponse);
            
            obj.pidGains.kP = kP;
            obj.pidGains.kI = kI;
            obj.pidGains.kD = kD;
            
            fprintf('\nNote: These PID gains are for motor position control.\n');
            fprintf('Convert to linear if controlling height directly.\n');
        end
        
        function calculateMotionMagic(obj, safetyFactor)
            % Calculate Motion Magic for elevator
            
            if nargin < 2
                safetyFactor = 0.75;  % More conservative for vertical motion
            end
            
            % Max linear velocity
            maxMotorSpeed_rad_s = obj.motorModel.FREE_SPEED_RAD_S;
            maxSpoolSpeed = maxMotorSpeed_rad_s / obj.gearRatio;
            maxLinearVel = maxSpoolSpeed * obj.spoolRadius_m * safetyFactor;
            
            obj.motionMagicProfile.cruiseVelocity = min(maxLinearVel, obj.maxVelocity_m_s);
            
            % Max linear acceleration (accounting for gravity)
            maxTorque = obj.motorModel.getMaxTorqueAtSpeed(0);
            totalTorque = maxTorque * obj.numMotors * obj.gearRatio;
            maxForce = totalTorque / obj.spoolRadius_m;
            
            % Net upward force = total - weight
            g = 9.81;
            netForce = maxForce - obj.carriageMass_kg * g;
            maxAccel = (netForce / obj.carriageMass_kg) * safetyFactor;
            
            obj.motionMagicProfile.acceleration = min(maxAccel, obj.maxAcceleration_m_s2);
            obj.motionMagicProfile.jerk = obj.motionMagicProfile.acceleration * 10;
            
            fprintf('\n%s Motion Magic Profile:\n', obj.mechanismName);
            fprintf('  Cruise velocity: %.2f m/s\n', obj.motionMagicProfile.cruiseVelocity);
            fprintf('  Acceleration: %.2f m/s²\n', obj.motionMagicProfile.acceleration);
            fprintf('  Jerk: %.2f m/s³\n', obj.motionMagicProfile.jerk);
            fprintf('\nNote: Gravity is compensated by kG feedforward\n');
        end
        
        function exportToRobot(obj, filename)
            % Export to JSON
            
            output.mechanism = obj.mechanismName;
            output.type = 'Elevator';
            output.feedforward = obj.feedforward;
            output.pid = obj.pidGains;
            output.motionMagic = obj.motionMagicProfile;
            output.limits.min = obj.minHeight_m;
            output.limits.max = obj.maxHeight_m;
            output.spoolRadius = obj.spoolRadius_m;
            
            jsonStr = jsonencode(output);
            fid = fopen(filename, 'w');
            fprintf(fid, '%s', jsonStr);
            fclose(fid);
            
            fprintf('\nExported to: %s\n', filename);
        end
    end
end
