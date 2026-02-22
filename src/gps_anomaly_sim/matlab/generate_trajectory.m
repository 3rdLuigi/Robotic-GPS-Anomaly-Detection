% Create a time vector from 0 to 50 seconds, taking a reading every 0.1 seconds
t = 0:0.1:50; 

% Generate a Figure 8 path for the robot
x_true = 10 * sin(t);
y_true = 10 * sin(t) .* cos(t);

% --- Plot the Ground Truth ---
figure;
plot(x_true, y_true, 'k-', 'LineWidth', 2);
title('Ideal Robot Trajectory (Ground Truth)');
xlabel('X Position (meters)');
ylabel('Y Position (meters)');
grid on;

% --- Simulate Sensor Jitter (Gaussian Noise) ---
noise_level = 0.5; % Half a meter of general inaccuracy
x_noisy = x_true + noise_level * randn(size(t));
y_noisy = y_true + noise_level * randn(size(t));

% --- Inject Severe Outliers (Sensor Failures) ---
% Randomly pick 15 moments where the GPS completely glitches
num_outliers = 15;
outlier_indices = randperm(length(t), num_outliers);

% Throw those specific points 10 to 20 meters off course
x_noisy(outlier_indices) = x_noisy(outlier_indices) + 15 * randn(1, num_outliers);
y_noisy(outlier_indices) = y_noisy(outlier_indices) + 15 * randn(1, num_outliers);

% --- Plot the Noisy Data ---
hold on; % Keep the previous perfect path on the screen
scatter(x_noisy, y_noisy, 15, 'r', 'filled'); % Plot the noisy data as red dots
legend('Ideal Path', 'Noisy GPS Readings');

% --- 6. Outlier Detection and Removal (Rolling Median Filter) ---
% We use a sliding window of 21 points (10 on each side + the center point).
% By default, 'movmedian' uses the exact same 3-standard-deviation threshold as hampel.
window_size = 21;

% 1. Detect the outliers (Returns a logical array of true/false)
x_outlier_idx = isoutlier(x_noisy, 'movmedian', window_size);
y_outlier_idx = isoutlier(y_noisy, 'movmedian', window_size);

% 2. Replace the outliers with the local 'center' median 
x_filtered = filloutliers(x_noisy, 'center', 'movmedian', window_size);
y_filtered = filloutliers(y_noisy, 'center', 'movmedian', window_size);

% 3. Combine indices to find all points that were flagged as outliers
all_outliers = union(find(x_outlier_idx), find(y_outlier_idx));

% --- 7. Plot the Data with Outliers Removed ---
figure;
plot(x_true, y_true, 'k-', 'LineWidth', 2);
hold on;

% Plot the cleaned data in blue
scatter(x_filtered, y_filtered, 15, 'b', 'filled');

% Highlight the specific points the algorithm caught and removed in red X's
scatter(x_noisy(all_outliers), y_noisy(all_outliers), 30, 'rx', 'LineWidth', 1.5);

title('Trajectory after Outlier Removal (Base MATLAB Filter)');
xlabel('X Position (meters)');
ylabel('Y Position (meters)');
legend('Ideal Path', 'Cleaned Data (Filtered)', 'Detected/Removed Outliers', 'Location', 'best');
grid on;

% --- 8. The Kalman Filter (Manual Implementation) ---
% Time step from our time vector
dt = 0.1; 

% State Transition Matrix (A): Models how the system moves (Kinematics)
% State is [X position; Y position; X velocity; Y velocity]
A = [1 0 dt 0; 
     0 1 0 dt; 
     0 0 1  0; 
     0 0 0  1];
     
% Measurement Matrix
H = [1 0 0 0; 
     0 1 0 0];
     
% Measurement Noise Covariance 
R = [1 0; 
     0 1];
     
% Process Noise Covariance l
Q = eye(4) * 0.01;

% Initial State Estimate: [X_pos; Y_pos; X_vel; Y_vel]
x_est = [x_filtered(1); y_filtered(1); 0; 0];

% Initial Covariance Estimate 
P = eye(4);

% Arrays to store the final smoothed Kalman Filter output
x_kf = zeros(1, length(t));
y_kf = zeros(1, length(t));

% --- The Kalman Filter Loop ---
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

% --- Plot the Final Result ---
figure;
plot(x_true, y_true, 'k-', 'LineWidth', 2);
hold on;
scatter(x_filtered, y_filtered, 10, 'b', 'filled', 'MarkerFaceAlpha', 0.3);
plot(x_kf, y_kf, 'g-', 'LineWidth', 2.5); % The final, smoothed Kalman path

title('Final Trajectory Estimation (Kalman Filter)');
xlabel('X Position (meters)');
ylabel('Y Position (meters)');
legend('Ideal Ground Truth', 'Cleaned GPS Measurements', 'Kalman Filter Estimate', 'Location', 'best');
grid on;