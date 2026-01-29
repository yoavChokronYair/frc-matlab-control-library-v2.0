classdef AutomatedTestSequences < handle
    % Generates test patterns for System Identification
    
    properties
        samplingRate_Hz = 100;      % Data collection rate
        testDuration_s              % Total test time
        safetyLimits                % Position/velocity limits
    end
    
    methods
        function obj = AutomatedTestSequences()
            obj.testDuration_s = 0;
            obj.safetyLimits = struct();
        end
        
        function [timeVec, voltageCommand] = generateStepResponse(obj, stepVoltage_V, duration_s)
            % Generate voltage step input for kV/kA/kS identification
            %
            % Inputs:
            %   stepVoltage_V: Voltage to command (V)
            %   duration_s: How long to hold voltage
            %
            % Outputs:
            %   timeVec: Time vector (s)
            %   voltageCommand: Voltage at each time step (V)
            
            dt = 1 / obj.samplingRate_Hz;
            timeVec = 0:dt:duration_s;
            voltageCommand = stepVoltage_V * ones(size(timeVec));
            
            % Add 0.5s ramp-up to avoid shock load
            rampSamples = round(0.5 * obj.samplingRate_Hz);
            voltageCommand(1:rampSamples) = linspace(0, stepVoltage_V, rampSamples);
            
            obj.testDuration_s = duration_s;
        end
        
        function [timeVec, voltageCommand] = generateMultiStepResponse(obj, voltages_V, duration_per_step_s)
            % Multiple voltage steps for comprehensive data
            %
            % Inputs:
            %   voltages_V: Array of voltages [3, 6, 9, 12]
            %   duration_per_step_s: Hold time per step
            
            dt = 1 / obj.samplingRate_Hz;
            totalDuration = length(voltages_V) * duration_per_step_s;
            timeVec = 0:dt:totalDuration;
            voltageCommand = zeros(size(timeVec));
            
            for i = 1:length(voltages_V)
                startIdx = round((i-1) * duration_per_step_s * obj.samplingRate_Hz) + 1;
                endIdx = round(i * duration_per_step_s * obj.samplingRate_Hz);
                voltageCommand(startIdx:endIdx) = voltages_V(i);
            end
            
            obj.testDuration_s = totalDuration;
        end
        
        function [timeVec, positionCommand] = generateFrequencySweep(obj, freqStart_Hz, freqEnd_Hz, amplitude, duration_s)
            % Logarithmic frequency sweep (chirp signal)
            % For identifying bandwidth and resonances
            %
            % Inputs:
            %   freqStart_Hz: Starting frequency
            %   freqEnd_Hz: Ending frequency
            %   amplitude: Motion amplitude (rad or m)
            %   duration_s: Sweep duration
            
            dt = 1 / obj.samplingRate_Hz;
            timeVec = 0:dt:duration_s;
            
            % Logarithmic chirp
            k = (freqEnd_Hz / freqStart_Hz)^(1 / duration_s);
            phase = 2 * pi * freqStart_Hz * (k.^timeVec - 1) / log(k);
            positionCommand = amplitude * sin(phase);
            
            obj.testDuration_s = duration_s;
        end
        
        function [timeVec, positionCommand] = generateRepeatabilityTest(obj, position_A, position_B, numCycles)
            % Move between two positions repeatedly
            % For measuring backlash and consistency
            
            dt = 1 / obj.samplingRate_Hz;
            moveDuration = 1.0; % 1 second per move
            
            timeVec = 0:dt:(numCycles * 2 * moveDuration);
            positionCommand = zeros(size(timeVec));
            
            for i = 1:numCycles
                % Move to position A
                startIdx = round((2*i-2) * moveDuration * obj.samplingRate_Hz) + 1;
                endIdx = round((2*i-1) * moveDuration * obj.samplingRate_Hz);
                positionCommand(startIdx:endIdx) = position_A;
                
                % Move to position B
                startIdx = endIdx + 1;
                endIdx = round(2*i * moveDuration * obj.samplingRate_Hz);
                positionCommand(startIdx:endIdx) = position_B;
            end
            
            obj.testDuration_s = length(timeVec) * dt;
        end
        
        function exportToRobotCode(obj, timeVec, command, filename)
            % Export test sequence to CSV for robot code to execute
            %
            % Robot code will:
            %   1. Read CSV
            %   2. Execute commands at specified times
            %   3. Log position, velocity, current
            %   4. Save log for MATLAB analysis
            
            data = [timeVec(:), command(:)];
            csvwrite(filename, data);
            
            fprintf('Test sequence exported to: %s\n', filename);
            fprintf('Duration: %.2f seconds\n', obj.testDuration_s);
            fprintf('Expected data points: %d\n', length(timeVec));
        end
    end
end
