classdef ModelValidation < handle
    % Validates simulation model against real robot data
    % Generates comprehensive comparison plots and metrics
    
    methods (Static)
        function results = validateModel(simulatedData, realData, mechanismName)
            % Compare simulated vs real data and generate validation report
            %
            % Inputs:
            %   simulatedData: struct with .time, .position, .velocity, .voltage
            %   realData: struct with same fields from robot test
            %   mechanismName: Name for plots/reports
            %
            % Outputs:
            %   results: struct with validation metrics
            
            if nargin < 3
                mechanismName = 'Mechanism';
            end
            
            fprintf('\n=== Model Validation: %s ===\n\n', mechanismName);
            
            % Initialize results
            results = struct();
            results.mechanismName = mechanismName;
            
            % Interpolate to common time base
            timeCommon = realData.time;
            simPos = interp1(simulatedData.time, simulatedData.position, timeCommon);
            simVel = interp1(simulatedData.time, simulatedData.velocity, timeCommon);
            
            % Calculate errors
            posError = realData.position - simPos;
            velError = realData.velocity - simVel;
            
            % Metrics
            results.position_RMSE = sqrt(mean(posError.^2));
            results.position_maxError = max(abs(posError));
            results.position_meanError = mean(posError);
            
            results.velocity_RMSE = sqrt(mean(velError.^2));
            results.velocity_maxError = max(abs(velError));
            results.velocity_meanError = mean(velError);
            
            % R² coefficient
            results.position_R2 = 1 - sum(posError.^2) / sum((realData.position - mean(realData.position)).^2);
            results.velocity_R2 = 1 - sum(velError.^2) / sum((realData.velocity - mean(realData.velocity)).^2);
            
            % Display metrics
            fprintf('Position Validation:\n');
            fprintf('  RMSE: %.4f rad (%.2f deg)\n', results.position_RMSE, rad2deg(results.position_RMSE));
            fprintf('  Max Error: %.4f rad (%.2f deg)\n', results.position_maxError, rad2deg(results.position_maxError));
            fprintf('  R²: %.4f (1.0 = perfect)\n', results.position_R2);
            fprintf('\n');
            
            fprintf('Velocity Validation:\n');
            fprintf('  RMSE: %.4f rad/s\n', results.velocity_RMSE);
            fprintf('  Max Error: %.4f rad/s\n', results.velocity_maxError);
            fprintf('  R²: %.4f (1.0 = perfect)\n', results.velocity_R2);
            fprintf('\n');
            
            % Overall assessment
            if results.position_R2 > 0.95 && results.velocity_R2 > 0.90
                results.assessment = 'EXCELLENT';
                fprintf('Assessment: ✓ EXCELLENT - Model is highly accurate\n');
            elseif results.position_R2 > 0.85 && results.velocity_R2 > 0.75
                results.assessment = 'GOOD';
                fprintf('Assessment: ✓ GOOD - Model is acceptable for control design\n');
            elseif results.position_R2 > 0.70 && results.velocity_R2 > 0.60
                results.assessment = 'FAIR';
                fprintf('Assessment: ⚠ FAIR - Model needs refinement\n');
            else
                results.assessment = 'POOR';
                fprintf('Assessment: ✗ POOR - Model requires significant correction\n');
            end
            
            fprintf('\n');
            
            % Generate plots
            ModelValidation.plotValidation(simulatedData, realData, mechanismName, results);
            
            % Save results
            results.timestamp = datetime('now');
        end
        
        function plotValidation(simulatedData, realData, mechanismName, metrics)
            % Generate comprehensive validation plots
            
            % Create figure with subplots
            fig = figure('Name', sprintf('%s Validation', mechanismName), ...
                'Position', [100 100 1200 800]);
            
            % Common time base
            timeCommon = realData.time;
            simPos = interp1(simulatedData.time, simulatedData.position, timeCommon);
            simVel = interp1(simulatedData.time, simulatedData.velocity, timeCommon);
            
            % Subplot 1: Position comparison
            subplot(3, 2, 1);
            plot(realData.time, realData.position, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Real');
            hold on;
            plot(simulatedData.time, simulatedData.position, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Simulated');
            grid on;
            xlabel('Time (s)');
            ylabel('Position (rad)');
            title('Position Comparison');
            legend('Location', 'best');
            
            % Subplot 2: Position error
            subplot(3, 2, 2);
            posError = realData.position - simPos;
            plot(timeCommon, posError, 'k-', 'LineWidth', 1);
            hold on;
            yline(0, 'r--');
            grid on;
            xlabel('Time (s)');
            ylabel('Error (rad)');
            title(sprintf('Position Error (RMSE=%.4f rad)', metrics.position_RMSE));
            
            % Subplot 3: Velocity comparison
            subplot(3, 2, 3);
            plot(realData.time, realData.velocity, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Real');
            hold on;
            plot(simulatedData.time, simulatedData.velocity, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Simulated');
            grid on;
            xlabel('Time (s)');
            ylabel('Velocity (rad/s)');
            title('Velocity Comparison');
            legend('Location', 'best');
            
            % Subplot 4: Velocity error
            subplot(3, 2, 4);
            velError = realData.velocity - simVel;
            plot(timeCommon, velError, 'k-', 'LineWidth', 1);
            hold on;
            yline(0, 'r--');
            grid on;
            xlabel('Time (s)');
            ylabel('Error (rad/s)');
            title(sprintf('Velocity Error (RMSE=%.4f rad/s)', metrics.velocity_RMSE));
            
            % Subplot 5: Phase plot (position vs velocity)
            subplot(3, 2, 5);
            plot(realData.position, realData.velocity, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Real');
            hold on;
            plot(simulatedData.position, simulatedData.velocity, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Simulated');
            grid on;
            xlabel('Position (rad)');
            ylabel('Velocity (rad/s)');
            title('Phase Portrait');
            legend('Location', 'best');
            
            % Subplot 6: Error histogram
            subplot(3, 2, 6);
            histogram(posError, 30, 'FaceColor', 'blue', 'EdgeColor', 'none', 'FaceAlpha', 0.5);
            hold on;
            histogram(velError, 30, 'FaceColor', 'red', 'EdgeColor', 'none', 'FaceAlpha', 0.5);
            grid on;
            xlabel('Error');
            ylabel('Frequency');
            title('Error Distribution');
            legend('Position Error', 'Velocity Error');
            
            % Overall title
            sgtitle(sprintf('%s Model Validation - Assessment: %s', ...
                mechanismName, metrics.assessment), 'FontSize', 14, 'FontWeight', 'bold');
        end
        
        function plotComprehensiveValidation(simulatedData, realData, mechanismName)
            % Even more detailed validation with additional plots
            
            fig = figure('Name', sprintf('%s Detailed Validation', mechanismName), ...
                'Position', [50 50 1400 900]);
            
            timeCommon = realData.time;
            simPos = interp1(simulatedData.time, simulatedData.position, timeCommon);
            simVel = interp1(simulatedData.time, simulatedData.velocity, timeCommon);
            simVoltage = interp1(simulatedData.time, simulatedData.voltage, timeCommon);
            
            % Subplot 1: Voltage input
            subplot(4, 2, 1);
            plot(realData.time, realData.voltage, 'b-', 'LineWidth', 1.5);
            hold on;
            plot(simulatedData.time, simulatedData.voltage, 'r--', 'LineWidth', 1.5);
            grid on;
            xlabel('Time (s)');
            ylabel('Voltage (V)');
            title('Voltage Command');
            legend('Real', 'Simulated', 'Location', 'best');
            
            % Subplot 2: Current draw (if available)
            subplot(4, 2, 2);
            if isfield(realData, 'current') && isfield(simulatedData, 'current')
                plot(realData.time, realData.current, 'b-', 'LineWidth', 1.5);
                hold on;
                simCurrent = interp1(simulatedData.time, simulatedData.current, timeCommon);
                plot(timeCommon, simCurrent, 'r--', 'LineWidth', 1.5);
                legend('Real', 'Simulated');
            else
                text(0.5, 0.5, 'Current data not available', ...
                    'HorizontalAlignment', 'center');
            end
            grid on;
            xlabel('Time (s)');
            ylabel('Current (A)');
            title('Current Draw');
            
            % Subplot 3: Position tracking
            subplot(4, 2, 3);
            plot(realData.time, realData.position, 'b-', 'LineWidth', 2);
            hold on;
            plot(simulatedData.time, simulatedData.position, 'r--', 'LineWidth', 2);
            grid on;
            xlabel('Time (s)');
            ylabel('Position (rad)');
            title('Position Response');
            legend('Real', 'Simulated', 'Location', 'best');
            
            % Subplot 4: Velocity tracking
            subplot(4, 2, 4);
            plot(realData.time, realData.velocity, 'b-', 'LineWidth', 2);
            hold on;
            plot(simulatedData.time, simulatedData.velocity, 'r--', 'LineWidth', 2);
            grid on;
            xlabel('Time (s)');
            ylabel('Velocity (rad/s)');
            title('Velocity Response');
            legend('Real', 'Simulated', 'Location', 'best');
            
            % Subplot 5: Position error over time
            subplot(4, 2, 5);
            posError = realData.position - simPos;
            plot(timeCommon, posError, 'k-', 'LineWidth', 1);
            hold on;
            plot(timeCommon, movmean(posError, 10), 'r-', 'LineWidth', 2);
            yline(0, 'b--');
            grid on;
            xlabel('Time (s)');
            ylabel('Error (rad)');
            title('Position Error (with moving average)');
            legend('Instantaneous', 'Moving Avg', 'Location', 'best');
            
            % Subplot 6: Velocity error over time
            subplot(4, 2, 6);
            velError = realData.velocity - simVel;
            plot(timeCommon, velError, 'k-', 'LineWidth', 1);
            hold on;
            plot(timeCommon, movmean(velError, 10), 'r-', 'LineWidth', 2);
            yline(0, 'b--');
            grid on;
            xlabel('Time (s)');
            ylabel('Error (rad/s)');
            title('Velocity Error (with moving average)');
            legend('Instantaneous', 'Moving Avg', 'Location', 'best');
            
            % Subplot 7: Correlation plot - Position
            subplot(4, 2, 7);
            scatter(simPos, realData.position, 20, 'filled', 'MarkerFaceAlpha', 0.3);
            hold on;
            % Perfect correlation line
            minVal = min([simPos; realData.position]);
            maxVal = max([simPos; realData.position]);
            plot([minVal maxVal], [minVal maxVal], 'r--', 'LineWidth', 2);
            grid on;
            xlabel('Simulated Position (rad)');
            ylabel('Real Position (rad)');
            title('Position Correlation');
            axis equal;
            
            % Subplot 8: Correlation plot - Velocity
            subplot(4, 2, 8);
            scatter(simVel, realData.velocity, 20, 'filled', 'MarkerFaceAlpha', 0.3);
            hold on;
            minVal = min([simVel; realData.velocity]);
            maxVal = max([simVel; realData.velocity]);
            plot([minVal maxVal], [minVal maxVal], 'r--', 'LineWidth', 2);
            grid on;
            xlabel('Simulated Velocity (rad/s)');
            ylabel('Real Velocity (rad/s)');
            title('Velocity Correlation');
            axis equal;
            
            sgtitle(sprintf('%s - Comprehensive Model Validation', mechanismName), ...
                'FontSize', 14, 'FontWeight', 'bold');
        end
        
        function report = generateValidationReport(results, outputFile)
            % Generate text report of validation results
            
            report = {};
            report{end+1} = '========================================';
            report{end+1} = 'MODEL VALIDATION REPORT';
            report{end+1} = '========================================';
            report{end+1} = '';
            report{end+1} = sprintf('Mechanism: %s', results.mechanismName);
            report{end+1} = sprintf('Timestamp: %s', char(results.timestamp));
            report{end+1} = '';
            report{end+1} = '--- Position Metrics ---';
            report{end+1} = sprintf('RMSE: %.6f rad (%.4f deg)', ...
                results.position_RMSE, rad2deg(results.position_RMSE));
            report{end+1} = sprintf('Max Error: %.6f rad (%.4f deg)', ...
                results.position_maxError, rad2deg(results.position_maxError));
            report{end+1} = sprintf('Mean Error: %.6f rad', results.position_meanError);
            report{end+1} = sprintf('R²: %.6f', results.position_R2);
            report{end+1} = '';
            report{end+1} = '--- Velocity Metrics ---';
            report{end+1} = sprintf('RMSE: %.6f rad/s', results.velocity_RMSE);
            report{end+1} = sprintf('Max Error: %.6f rad/s', results.velocity_maxError);
            report{end+1} = sprintf('Mean Error: %.6f rad/s', results.velocity_meanError);
            report{end+1} = sprintf('R²: %.6f', results.velocity_R2);
            report{end+1} = '';
            report{end+1} = sprintf('Overall Assessment: %s', results.assessment);
            report{end+1} = '========================================';
            
            % Display report
            for i = 1:length(report)
                fprintf('%s\n', report{i});
            end
            
            % Save to file if requested
            if nargin >= 2
                fid = fopen(outputFile, 'w');
                for i = 1:length(report)
                    fprintf(fid, '%s\n', report{i});
                end
                fclose(fid);
                fprintf('\nReport saved to: %s\n', outputFile);
            end
        end
    end
end
