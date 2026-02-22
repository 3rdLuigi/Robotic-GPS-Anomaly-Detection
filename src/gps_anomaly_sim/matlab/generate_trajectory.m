% Load Data from C++ ROS 2 Node
data = readtable('turtle_trajectory.csv');

% Normalize time to start at 0
t = data.time - data.time(1); 

% Variables from the CSV
x_true = data.x_true;
y_true = data.y_true;
x_noisy = data.x_noisy;
y_noisy = data.y_noisy;

% Plot the Ground Truth & Noisy Data
figure;
plot(x_true, y_true, 'k-', 'LineWidth', 2);
hold on;
scatter(x_noisy, y_noisy, 15, 'r', 'filled');
title('TurtleBot Trajectory: Ground Truth vs. Noisy GPS');
xlabel('X Position (meters)');
ylabel('Y Position (meters)');
legend('Ideal Path', 'Noisy GPS Readings', 'Location', 'best');
grid on;

% Outlier Detection and Removal
window_size = 21;

% Detect the outliers
x_outlier_idx = isoutlier(x_noisy, 'movmedian', window_size);
y_outlier_idx = isoutlier(y_noisy, 'movmedian', window_size);

% Replace the outliers with the local 'center' median 
x_filtered = filloutliers(x_noisy, 'center', 'movmedian', window_size);
y_filtered = filloutliers(y_noisy, 'center', 'movmedian', window_size);

% Combine indices to find all points that were flagged as outliers
all_outliers = union(find(x_outlier_idx), find(y_outlier_idx));

% Plot the Data with Outliers Removed
figure;
plot(x_true, y_true, 'k-', 'LineWidth', 2);
hold on;
scatter(x_filtered, y_filtered, 15, 'b', 'filled');
scatter(x_noisy(all_outliers), y_noisy(all_outliers), 30, 'rx', 'LineWidth', 1.5);

title('Trajectory after Outlier Removal (Base MATLAB Filter)');
xlabel('X Position (meters)');
ylabel('Y Position (meters)');
legend('Ideal Path', 'Cleaned Data (Filtered)', 'Detected/Removed Outliers', 'Location', 'best');
grid on;

% The Kalman Filter
% Average time step from the ROS 2 data
dt = mean(diff(t)); 

% State Transition Matrix (A)
% State is [X position; Y position; X velocity; Y velocity]
A = [1 0 dt 0; 
     0 1 0 dt; 
     0 0 1  0; 
     0 0 0  1];
     
% Measurement Matrix
H = [1 0 0 0; 
     0 1 0 0];
     
% Measurement Noise Covariance 
R = [3 0; 
     0 3];
     
% Process Noise Covariance 
Q = eye(4) * 0.0005;

% Initial State Estimate: [X_pos; Y_pos; X_vel; Y_vel]
x_est = [x_filtered(1); y_filtered(1); 0; 0];

% Initial Covariance Estimate 
P = eye(4);

% Arrays to store the final smoothed Kalman Filter output
x_kf = zeros(1, length(t));
y_kf = zeros(1, length(t));

% The Kalman Filter Loop
for i = 1:length(t)
    % Predict step
    x_est_pred = A * x_est;
    P_pred = A * P * A' + Q;
    
    % Update step with current measurement
    z = [x_filtered(i); y_filtered(i)];

    % Calculate Kalman Gain
    K = P_pred * H' / (H * P_pred * H' + R); 
    
    % Update estimate with measurement
    x_est = x_est_pred + K * (z - H * x_est_pred); 
    
    % Update covariance
    P = (eye(4) - K * H) * P_pred;        
    
    % Store the results for plotting
    x_kf(i) = x_est(1);
    y_kf(i) = x_est(2);
end

% Plot the Final Result
figure;
plot(x_true, y_true, 'k-', 'LineWidth', 2);
hold on;
scatter(x_filtered, y_filtered, 10, 'b', 'filled', 'MarkerFaceAlpha', 0.3);
plot(x_kf, y_kf, 'g-', 'LineWidth', 2.5);

title('Final Trajectory Estimation (Kalman Filter)');
xlabel('X Position (meters)');
ylabel('Y Position (meters)');
legend('Ideal Ground Truth', 'Cleaned GPS Measurements', 'Kalman Filter Estimate', 'Location', 'best');
grid on;