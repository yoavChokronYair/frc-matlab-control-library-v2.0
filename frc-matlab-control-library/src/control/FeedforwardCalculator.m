classdef FeedforwardCalculator < handle
    % Calculate feedforward gains for various mechanism types
    
    methods (Static)
        function ff = calculateSimpleRotational(motorModel, gearRatio, inertia, friction)
            % Feedforward for simple rotational mechanism (turret, single-joint arm)
            %
            % Inputs:
            %   motorModel: KrakenX40Model or KrakenX60Model instance
            %   gearRatio: Motor rotations per output rotation
            %   inertia: Mechanism inertia (kg⋅m²)
            %   friction: Struct with .static (N⋅m) and .viscous (N⋅m⋅s/rad)
            
            % Velocity feedforward (overcome back-EMF)
            [kV_base, ~, ~] = motorModel.getFeedforwardConstants(gearRatio);
            ff.kV = kV_base;
            
            % Acceleration feedforward (overcome inertia)
            % Torque needed: τ = J*α
            % Voltage needed: V = τ / (Kt * gearRatio) * R
            ff.kA = (inertia / (motorModel.KT * gearRatio)) * motorModel.RESISTANCE_OHM;
            
            % Static friction feedforward
            % Convert friction torque to voltage
            ff.kS = (friction.static / (motorModel.KT * gearRatio)) * motorModel.RESISTANCE_OHM;
            
            % Gravity feedforward (for arms) - set to zero, calculated separately
            ff.kG = 0;
            
            fprintf('Feedforward Constants (Simple Rotational):\n');
            fprintf('  kS: %.4f V\n', ff.kS);
            fprintf('  kV: %.4f V/(rad/s)\n', ff.kV);
            fprintf('  kA: %.4f V/(rad/s²)\n', ff.kA);
        end
        
        function ff = calculateArmWithGravity(motorModel, gearRatio, armParams, friction)
            % Feedforward for arm with gravity compensation
            %
            % armParams: struct with
            %   .inertia (kg⋅m²)
            %   .mass (kg)
            %   .length_CoM (m) - distance from pivot to center of mass
            
            % Basic feedforward
            ff = FeedforwardCalculator.calculateSimpleRotational(...
                motorModel, gearRatio, armParams.inertia, friction);
            
            % Gravity feedforward: kG = m * g * r / (Kt * gearRatio)
            % This gives voltage needed to hold arm horizontal
            g = 9.81; % m/s²
            gravityTorque = armParams.mass * g * armParams.length_CoM;
            ff.kG = (gravityTorque / (motorModel.KT * gearRatio)) * motorModel.RESISTANCE_OHM;
            
            fprintf('  kG (gravity): %.4f V/cos(θ)\n', ff.kG);
            fprintf('  (Apply as: kG * cos(angle))\n');
        end
        
        function ff = calculateLinear(motorModel, gearRatio, spoolDiameter_m, mass_kg, friction)
            % Feedforward for linear mechanism (elevator)
            %
            % Inputs:
            %   spoolDiameter_m: Diameter of pulley/spool
            %   mass_kg: Carriage mass
            %   friction: struct with .static (N) and .viscous (N⋅s/m)
            
            % Convert linear to rotational
            radius = spoolDiameter_m / 2;
            inertia_equiv = mass_kg * radius^2; % Equivalent rotational inertia
            friction_rot.static = friction.static * radius;
            friction_rot.viscous = friction.viscous * radius;
            
            ff = FeedforwardCalculator.calculateSimpleRotational(...
                motorModel, gearRatio, inertia_equiv, friction_rot);
            
            % Gravity feedforward for vertical motion
            g = 9.81;
            gravityForce = mass_kg * g;
            gravityTorque = gravityForce * radius;
            ff.kG = (gravityTorque / (motorModel.KT * gearRatio)) * motorModel.RESISTANCE_OHM;
            
            fprintf('Feedforward Constants (Linear):\n');
            fprintf('  Note: These are for motor control, not linear motion\n');
            fprintf('  kG (gravity): %.4f V (constant for vertical)\n', ff.kG);
        end
        
        function ff = calculateFlywheel(motorModel, gearRatio, inertia, targetRPM)
            % Feedforward for flywheel/shooter
            % Emphasis on velocity control
            
            [kV_base, ~, ~] = motorModel.getFeedforwardConstants(gearRatio);
            
            ff.kV = kV_base;
            ff.kA = 0; % Not used for velocity control
            ff.kS = 0.05; % Small static friction
            ff.kG = 0;
            
            % Calculate voltage needed for target RPM
            targetSpeed_rad_s = targetRPM * 2*pi / 60;
            ff.voltageAtTargetRPM = ff.kV * targetSpeed_rad_s;
            
            fprintf('Flywheel Feedforward:\n');
            fprintf('  kV: %.4f V/(rad/s)\n', ff.kV);
            fprintf('  Voltage at %.0f RPM: %.2f V\n', targetRPM, ff.voltageAtTargetRPM);
        end
        
        function ff = refineFromTestData(ff_initial, testData)
            % Refine feedforward using actual test data
            % (calls ParameterExtraction methods)
            
            [kV_actual, kA_actual, kS_actual] = ...
                ParameterExtraction.extractFeedforward(testData);
            
            % Average model-based and empirical
            ff.kS = (ff_initial.kS + kS_actual) / 2;
            ff.kV = (ff_initial.kV + kV_actual) / 2;
            ff.kA = (ff_initial.kA + kA_actual) / 2;
            ff.kG = ff_initial.kG; % Gravity is well-modeled
            
            fprintf('Refined Feedforward (Model + Empirical):\n');
            fprintf('  kS: %.4f V\n', ff.kS);
            fprintf('  kV: %.4f V/(rad/s)\n', ff.kV);
            fprintf('  kA: %.4f V/(rad/s²)\n', ff.kA);
        end
    end
end
