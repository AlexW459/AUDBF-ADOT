% MATLAB Script for Analyzing Sensitivity of DBF Scoring with Baseline Variation for All Key Parameters
% Name: Lysandar Kenelm Tolkin
% Date: 08/09/2024

clear;
clc;

% Set percentage change range (-50% to 50%) for key parameters
percentage_change = linspace(-100, 100, 100);  % percentage change in range

% Define fixed baseline values for other parameters
baseline_fuel_weight = 11.02;    % baseline fuel weight (lbs)
baseline_time = 126;             % baseline flight time (seconds)
baseline_X1_weight = 0.4;        % baseline X-1 test vehicle weight (lbs)
baseline_bonus_box_score = 2.5;  % baseline bonus box score
baseline_laps_flown = 3;         % fixed baseline laps flown

% Set y-axis limits for all plots (same scale)
y_axis_limits = [-1 2];  % Adjust as needed based on expected score variations

%% 1. Sensitivity Analysis for Fuel Weight (Baseline: 1 lb to 22 lbs)
baseline_fuel_weight_values = linspace(1, 11, 5);  % Range of baseline fuel weights

for i = 1:length(baseline_fuel_weight_values)
    fuel_weight = baseline_fuel_weight_values(i) * (1 + percentage_change / 100);
    M2_fuel = 1 + (fuel_weight ./ baseline_time) / (baseline_fuel_weight / baseline_time); % M2 fuel weight sensitivity
    M3_fuel = 2 + (baseline_laps_flown + (baseline_bonus_box_score / baseline_X1_weight)) / 19; % M3 fixed
    Total_Score_fuel(i, :) = M2_fuel + M3_fuel;
end

figure;
hold on;
for i = 1:length(baseline_fuel_weight_values)
    plot(percentage_change, Total_Score_fuel(i, :) - Total_Score_fuel(i, 50), 'LineWidth', 1.5);
end
hold off;
xlabel('Percentage Change in Fuel Weight (%)');
ylabel('Change in Total Mission Score from Baseline');
title('Sensitivity of Mission Score to Fuel Weight for Different Baseline Values');
legend(arrayfun(@(x) sprintf('Baseline Fuel Weight: %.2f lbs', x), baseline_fuel_weight_values, 'UniformOutput', false), 'Location', 'best');
ylim(y_axis_limits);  % Set consistent y-axis scale
grid on;

%% 2. Sensitivity Analysis for Flight Time (Baseline: 126 sec to 186 sec)
baseline_time_values = linspace(99, 200, 5);  % Range of baseline times

for i = 1:length(baseline_time_values)
    time = baseline_time_values(i) * (1 + percentage_change / 100);
    M2_time = 1 + (baseline_fuel_weight ./ time) / (baseline_fuel_weight / baseline_time); % M2 time sensitivity
    M3_time = 2 + (baseline_laps_flown + (baseline_bonus_box_score / baseline_X1_weight)) / 19; % M3 fixed
    Total_Score_time(i, :) = M2_time + M3_time;
end

figure;
hold on;
for i = 1:length(baseline_time_values)
    plot(percentage_change, Total_Score_time(i, :) - Total_Score_time(i, 50), 'LineWidth', 1.5);
end
hold off;
xlabel('Percentage Change in Flight Time (%)');
ylabel('Change in Total Mission Score from Baseline');
title('Sensitivity of Mission Score to Flight Time for Different Baseline Values');
legend(arrayfun(@(x) sprintf('Baseline Flight Time: %.0f seconds', x), baseline_time_values, 'UniformOutput', false), 'Location', 'best');
ylim(y_axis_limits);  % Set consistent y-axis scale
grid on;

%% 3. Sensitivity Analysis for X-1 Test Vehicle Weight (Baseline: 0.001 lb to 0.551 lb)
baseline_X1_weight_values = linspace(0.01, 0.551, 5);  % Range of baseline X-1 weights

for i = 1:length(baseline_X1_weight_values)
    X1_weight = baseline_X1_weight_values(i) * (1 + percentage_change / 100);
    M2_X1 = 1 + (baseline_fuel_weight ./ baseline_time) / (baseline_fuel_weight / baseline_time); % M2 fixed
    M3_X1 = 2 + (baseline_laps_flown + (baseline_bonus_box_score ./ X1_weight)) / 19; % M3 X-1 weight sensitivity
    Total_Score_X1(i, :) = M2_X1 + M3_X1;
end

figure;
hold on;
for i = 1:length(baseline_X1_weight_values)
    plot(percentage_change, Total_Score_X1(i, :) - Total_Score_X1(i, 50), 'LineWidth', 1.5);
end
hold off;
xlabel('Percentage Change in X-1 Weight (%)');
ylabel('Change in Total Mission Score from Baseline');
title('Sensitivity of Mission Score to X-1 Weight for Different Baseline Values');
legend(arrayfun(@(x) sprintf('Baseline X-1 Weight: %.3f lbs', x), baseline_X1_weight_values, 'UniformOutput', false), 'Location', 'best');
ylim(y_axis_limits);  % Set consistent y-axis scale
grid on;

%% 4. Sensitivity Analysis for Bonus Box Score (Baseline: 0, 1, 2.5)
baseline_bonus_box_values = [0, 1, 2.5];  % Discrete baseline bonus box scores

for i = 1:length(baseline_bonus_box_values)
    bonus_box_score = baseline_bonus_box_values(i) * (1 + percentage_change / 100);
    M2_bonus = 1 + (baseline_fuel_weight ./ baseline_time) / (baseline_fuel_weight / baseline_time); % M2 fixed
    M3_bonus = 2 + (baseline_laps_flown + (bonus_box_score ./ baseline_X1_weight)) / 19; % M3 bonus box sensitivity
    Total_Score_bonus(i, :) = M2_bonus + M3_bonus;
end

figure;
hold on;
for i = 1:length(baseline_bonus_box_values)
    plot(percentage_change, Total_Score_bonus(i, :) - Total_Score_bonus(i, 50), 'LineWidth', 1.5);
end
hold off;
xlabel('Percentage Change in Bonus Box Score (%)');
ylabel('Change in Total Mission Score from Baseline');
title('Sensitivity of Mission Score to Bonus Box Score for Different Baseline Values');
legend(arrayfun(@(x) sprintf('Bonus Box Score: %.1f', x), baseline_bonus_box_values, 'UniformOutput', false), 'Location', 'best');
ylim(y_axis_limits);  % Set consistent y-axis scale
grid on;

%% 5. Sensitivity Analysis for Number of Laps Flown in Mission 3
baseline_laps_flown_values = linspace(1, 9, 5);  % Range of baseline laps flown

for i = 1:length(baseline_laps_flown_values)
    laps_flown = baseline_laps_flown_values(i) * (1 + percentage_change / 100);
    M2_laps = 1 + (baseline_fuel_weight ./ baseline_time) / (baseline_fuel_weight / baseline_time); % M2 fixed
    M3_laps = 2 + (laps_flown + (baseline_bonus_box_score / baseline_X1_weight)) / 19; % M3 laps sensitivity
    Total_Score_laps(i, :) = M2_laps + M3_laps;
end

figure;
hold on;
for i = 1:length(baseline_laps_flown_values)
    plot(percentage_change, Total_Score_laps(i, :) - Total_Score_laps(i, 50), 'LineWidth', 1.5);
end
hold off;
xlabel('Percentage Change in Laps Flown (%)');
ylabel('Change in Total Mission Score from Baseline');
title('Sensitivity of Mission Score to Laps Flown for Different Baseline Values');
legend(arrayfun(@(x) sprintf('Baseline Laps Flown: %.1f', x), baseline_laps_flown_values, 'UniformOutput', false), 'Location', 'best');
ylim(y_axis_limits);  % Set consistent y-axis scale
grid on;
