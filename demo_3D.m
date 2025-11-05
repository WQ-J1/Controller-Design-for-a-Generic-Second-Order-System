%% Workspace Cleanup
clear;
clc;
close all;

%% 0. Set Random Seed for Reproducibility
rng(0); % Ensures the same trajectory is generated on each run

%% 1. Parameter Definition (Shared Across All Dimensions)
% Controller parameters
delta = 4.0;
rho = 10;
tau = 1/6;
r1 = 1.0;
r2 = r1 - tau;
r3 = r2 - tau;
beta2 = 20.0;
beta1 = 0.025;
a = 10;
C_over_Km = 0.2; % Value for C(x)/Km

% Pre-calculate constants for efficiency and readability
p = (2*rho + tau) / r1;

% Define the fsign(x, a) = sign(x) * |x|^a function
fsign = @(x, a) sign(x) .* abs(x).^a;

%% 2. Generate 3D Random Reference Trajectory
N = 100; % Number of waypoints
ref_path = zeros(N, 3); % Use an Nx3 matrix to store the trajectory (xd, yd, zd)

% Initialize the first waypoint at the origin
ref_path(1, :) = [0, 0, 0];

% Initialize a random initial direction
current_direction = rand(1, 3) - 0.5;
current_direction = current_direction / norm(current_direction); % 归一化

% Define strength; a larger value results in a more tortuous path
perturbation_strength = 0.8; 

for i = 2:N
    % 1. Add a small random perturbation to the current direction
    perturbation = (rand(1, 3) - 0.5) * perturbation_strength;
    new_direction = current_direction + perturbation;
    
    % 2. Normalize the new direction vector
    new_direction = new_direction / norm(new_direction);
    
    % 3. Generate a random step length (distance) up to 3
    step_length = 2 * rand();
    
    % 4. Calculate the position of the next waypoint
    ref_path(i, :) = ref_path(i-1, :) + step_length * new_direction;
    
    % 5. Update the current direction for the next iteration
    current_direction = new_direction;
end

% Decompose the generated path into xd, yd, zd vectors for compatibility
xd = ref_path(:, 1)';
yd = ref_path(:, 2)';
zd = ref_path(:, 3)';

%% 3. System and Simulation Initialization
% Simulation time settings
dt = 0.001; % Time step
T = 2500;   % Total simulation time (need adjustment based on trajectory)
t = 0:dt:T; % Time vector

% System state initialization
% X-dim
x1 = zeros(size(t)); x2 = zeros(size(t));
% Y-dim
y1 = zeros(size(t)); y2 = zeros(size(t));
% Z-dim
z1 = zeros(size(t)); z2 = zeros(size(t));

% Initial states are all zero
x1(1) = 0; x2(1) = 0;
y1(1) = 0; y2(1) = 0;
z1(1) = 0; z2(1) = 0;

% Storage for control inputs and current reference point history
ux = zeros(size(t)); uy = zeros(size(t)); uz = zeros(size(t));
xd_hist = zeros(size(t)); yd_hist = zeros(size(t)); zd_hist = zeros(size(t));

%% 4. Simulation Loop
current_target_index = 1; % Index of the current target waypoint

% Define controller as an anonymous function to reduce code repetition
controller = @(s, s_dot) -beta2 * sec((pi * abs(s)^p) / (2 * (delta/sqrt(3))^p))^2 ...
    * fsign(fsign(s_dot, a/r2) + (beta1^(a/r2)) * fsign(s, a/r1), r3/a) ...
    - C_over_Km * sign(fsign(s_dot, a/r2) + (beta1^(a/r2)) * fsign(s, a/r1));

for k = 1:length(t)-1
    % --- 3D Waypoint Switching Logic ---
    if current_target_index < N
        axis_threshold = delta / sqrt(3) - 0.1;
        
        % 1. Calculate the error on each axis to the *next* target
        error_x = abs(x1(k) - xd(current_target_index + 1));
        error_y = abs(y1(k) - yd(current_target_index + 1));
        error_z = abs(z1(k) - zd(current_target_index + 1));
        
        % 2. Check if the errors on all three axes are below the threshold
        if (error_x < axis_threshold) && (error_y < axis_threshold) && (error_z < axis_threshold)
            % Switch to the next waypoint only if all conditions are met
            current_target_index = current_target_index + 1;
        end
    end
    
    % Get the current target reference point
    current_xd = xd(current_target_index);
    current_yd = yd(current_target_index);
    current_zd = zd(current_target_index);
    
    % Record history for plotting
    xd_hist(k) = current_xd;
    yd_hist(k) = current_yd;
    zd_hist(k) = current_zd;
    
    % --- Controller Calculation for Each Dimension Independently ---
    % X-dimension
    sx = x1(k) - current_xd;
    sx_dot = x2(k);
    ux(k) = controller(sx, sx_dot);
    
    % Y-dimension
    sy = y1(k) - current_yd;
    sy_dot = y2(k);
    uy(k) = controller(sy, sy_dot);
    
    % Z-dimension
    sz = z1(k) - current_zd;
    sz_dot = z2(k);
    uz(k) = controller(sz, sz_dot);
    
    % --- System State Update ---
    % X
    x1(k+1) = x1(k) + x2(k) * dt;
    x2(k+1) = x2(k) + ux(k) * dt;
    % Y
    y1(k+1) = y1(k) + y2(k) * dt;
    y2(k+1) = y2(k) + uy(k) * dt;
    % Z
    z1(k+1) = z1(k) + z2(k) * dt;
    z2(k+1) = z2(k) + uz(k) * dt;
end
% Record data for the final time step to ensure complete plots
xd_hist(end) = current_xd; yd_hist(end) = current_yd; zd_hist(end) = current_zd;
ux(end) = ux(end-1); uy(end) = uy(end-1); uz(end) = uz(end-1);

%% 5. Results Visualization
% Color scheme
colors.blue   = [68, 119, 170] / 255;
colors.cyan   = [102, 204, 238] / 255;
colors.green  = [34, 136, 51] / 255;
colors.yellow = [204, 187, 68] / 255;
colors.red    = [238, 102, 119] / 255;
colors.purple = [170, 51, 119] / 255;
colors.grey   = [187, 187, 187] / 255;
colors.black  = [0, 0, 0];

% Font and line style definitions for publication-quality figures
fontName      = 'Helvetica'; % or 'Arial'
axisFontSize  = 10;
labelFontSize = 12;
titleFontSize = 14;
lineWidth     = 1.5;
markerSize    = 8;


% --- Figure 1: 3D Trajectory Tracking ---
fig1 = figure('Name', '3D Trajectory Tracking', 'Position', [100, 100, 700, 600]);
ax1 = gca; % Get current axis handle

% Plot the actual trajectory
plot3(ax1, x1, y1, z1, 'Color', colors.blue, 'LineWidth', lineWidth + 0.5, 'DisplayName', 'Actual Trajectory');
hold(ax1, 'on');

% Plot the reference trajectory
plot3(ax1, xd, yd, zd, '--', 'Color', colors.red, 'LineWidth', lineWidth-0.5, 'DisplayName', 'Reference Trajectory');

% Mark the reference waypoints
scatter3(ax1, xd, yd, zd, markerSize*2, colors.black, 'filled', 'o', 'DisplayName', 'Reference Waypoints', 'MarkerFaceAlpha', 0.7);

% Mark the start and end points
scatter3(ax1, xd(1), yd(1), zd(1), markerSize*10, colors.green, 'filled', 'p', 'DisplayName', 'Start'); % 'p' is a pentagram
scatter3(ax1, xd(end), yd(end), zd(end), markerSize*10, colors.purple, 'filled', 'h', 'DisplayName', 'End'); % 'h' is a hexagram

grid(ax1, 'on');
box(ax1, 'on');
axis(ax1, 'equal'); % Set aspect ratio to be equal
view(30, 20); % Set a suitable viewing angle

% Set axis labels and title
xlabel('X-axis', 'FontName', fontName, 'FontSize', labelFontSize, 'FontWeight', 'bold');
ylabel('Y-axis', 'FontName', 'Helvetica', 'FontSize', labelFontSize, 'FontWeight', 'bold');
zlabel('Z-axis', 'FontName', 'Helvetica', 'FontSize', labelFontSize, 'FontWeight', 'bold');
title('Three-Dimensional Trajectory Tracking', 'FontName', fontName, 'FontSize', titleFontSize, 'FontWeight', 'bold');
legend(ax1, 'Location', 'northwest', 'FontSize', axisFontSize, 'FontName', fontName);

% Format the axes for a professional appearance
set(ax1, ...
    'FontName', fontName, ...
    'FontSize', axisFontSize, ...
    'LineWidth', 1, ...
    'TickDir', 'out', ...
    'XColor', colors.black, ...
    'YColor', colors.black, ...
    'ZColor', colors.black, ...
    'GridColor', colors.grey, ...
    'GridAlpha', 0.5);

hold(ax1, 'off');


% --- Figure 2: Per-Axis Performance Analysis ---
fig2 = figure('Name', 'Per-Axis Performance Analysis', 'Position', [850, 100, 600, 700]);
% Use tiledlayout for flexible subplot management
tlo = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

% Upper subplot: Tracking Error
ax2a = nexttile;
plot(ax2a, t, x1 - xd_hist, 'Color', colors.red, 'LineWidth', lineWidth, 'DisplayName', 'Error X');
hold(ax2a, 'on');
plot(ax2a, t, y1 - yd_hist, 'Color', colors.green, 'LineWidth', lineWidth, 'DisplayName', 'Error Y');
plot(ax2a, t, z1 - zd_hist, 'Color', colors.blue, 'LineWidth', lineWidth, 'DisplayName', 'Error Z');
yline(ax2a, 2.3, '--', 'Color', colors.purple, 'LineWidth', 1.2, 'DisplayName', '+\delta/\surd3');
yline(ax2a, -2.3, '--', 'Color', colors.purple, 'LineWidth', 1.2, 'DisplayName', '-\delta/\surd3');
hold(ax2a, 'off');
grid(ax2a, 'on');
box(ax2a, 'on');

% Set axis labels and title
ylabel(ax2a, 'Tracking Error', 'FontName', fontName, 'FontSize', labelFontSize, 'FontWeight', 'bold');
title(ax2a, 'Tracking Error Analysis', 'FontName', fontName, 'FontSize', titleFontSize, 'FontWeight', 'bold');
legend(ax2a, 'Location', 'northeast', 'FontSize', axisFontSize, 'FontName', fontName);
ylim(ax2a, [-5, 5]); 

% Format the axes
set(ax2a, ...
    'FontName', fontName, ...
    'FontSize', axisFontSize, ...
    'LineWidth', 1, ...
    'TickDir', 'out', ...
    'XColor', colors.black, ...
    'YColor', colors.black, ...
    'XTickLabel', [], ... 
    'GridColor', colors.grey, ...
    'GridAlpha', 0.5);

% Lower subplot: Control Input
ax2b = nexttile;
plot(ax2b, t, ux, 'Color', colors.red, 'LineWidth', lineWidth, 'DisplayName', 'Control X');
hold(ax2b, 'on');
plot(ax2b, t, uy, 'Color', colors.green, 'LineWidth', lineWidth, 'DisplayName', 'Control Y');
plot(ax2b, t, uz, 'Color', colors.blue, 'LineWidth', lineWidth, 'DisplayName', 'Control Z');
hold(ax2b, 'off');
grid(ax2b, 'on');
box(ax2b, 'on');

% Set axis labels and title
xlabel(tlo, 'Time (s)', 'FontName', fontName, 'FontSize', labelFontSize, 'FontWeight', 'bold');
ylabel(ax2b, 'Control Input', 'FontName', fontName, 'FontSize', labelFontSize, 'FontWeight', 'bold');
ylim(ax2b, [-8, 8]);
title(ax2b, 'Control Effort Analysis', 'FontName', fontName, 'FontSize', titleFontSize, 'FontWeight', 'bold');
legend(ax2b, 'Location', 'northeast', 'FontSize', axisFontSize, 'FontName', fontName);

% Format the axes
set(ax2b, ...
    'FontName', fontName, ...
    'FontSize', axisFontSize, ...
    'LineWidth', 1, ...
    'TickDir', 'out', ...
    'XColor', colors.black, ...
    'YColor', colors.black, ...
    'GridColor', colors.grey, ...
    'GridAlpha', 0.5);

% Link the x-axes of the two subplots for synchronized zooming and panning
linkaxes([ax2a, ax2b], 'x');
xlim(ax2a, [0, T]); 