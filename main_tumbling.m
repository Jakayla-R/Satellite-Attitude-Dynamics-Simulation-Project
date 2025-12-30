% Define parameters
m = 1000; % Mass of the satellite (kg)
I_xx = 1000; % Moment of inertia about x-axis (kg*m^2)
I_yy = 1500; % Moment of inertia about y-axis (kg*m^2)
I_zz = 800; % Moment of inertia about z-axis (kg*m^2)
G = 6.67430e-11; % Gravitational constant (m^3/kg/s^2)
M_Earth = 5.972e24; % Mass of the Earth (kg)
R_Earth = 6371e3; % Radius of the Earth (m)
%% 
% calculate the gravitational forces acting on the satellite due to the Earth's 
% gravity. We'll use Newton's law of universal gravitation to compute the gravitational 
% force vector. 

% Define satellite position relative to the center of the Earth (assuming circular orbit for simplicity)
r_satellite = [0; R_Earth + 500e3; 0]; % Position vector (m)

% Calculate distance between satellite and Earth's center
r_distance = norm(r_satellite);

% Calculate gravitational force vector
F_gravity = -G * M_Earth * m / r_distance^2 * r_satellite / r_distance;

% Display the gravitational force vector
disp('Gravitational force vector (N):');
disp(F_gravity);
%% 
% The next step is to implement the dynamics of Euler angles. We'll use Euler's 
% equations of motion for rigid body rotation to compute the time derivatives 
% of the Euler angles (roll, pitch, and yaw rates) based on the applied torques 
% and gravitational forces.

% Calculate gravitational torque (assuming satellite is a rigid body)
tau_gravity = cross(r_satellite, F_gravity);

% Display the gravitational torque vector
disp('Gravitational torque vector (Nm):');
disp(tau_gravity);
%ompute these derivatives based on the torques acting on the satellite. 
% Define initial Euler angles (roll, pitch, yaw) and angular velocities
phi_0 = 0.1;  % Initial roll angle (rad)
theta_0 = 0.05; % Initial pitch angle (rad)
psi_0 = 0;   % Initial yaw angle (rad)
p_0 = 0;     % Initial roll rate (rad/s)
q_0 = 0;     % Initial pitch rate (rad/s)
r_0 = 0;     % Initial yaw rate (rad/s)

% Compute time derivatives of Euler angles using Euler's equations of motion
phi_dot = p_0 + q_0 * sin(phi_0) * tan(theta_0) + r_0 * cos(phi_0) * tan(theta_0);
theta_dot = q_0 * cos(phi_0) - r_0 * sin(phi_0);
psi_dot = q_0 * sin(phi_0) * sec(theta_0) + r_0 * cos(phi_0) * sec(theta_0);

% Display the time derivatives of Euler angles
disp('Time derivatives of Euler angles (rad/s):');
disp(['Roll rate (phi_dot): ', num2str(phi_dot)]);
disp(['Pitch rate (theta_dot): ', num2str(theta_dot)]);
disp(['Yaw rate (psi_dot): ', num2str(psi_dot)]);
%% 
% The next step is to integrate the time derivatives of the Euler angles over 
% time to update the Euler angles themselves. We'll use a numerical integration 
% method to accomplish this. 

% Define simulation time span
t_span = [0 10]; % Simulation time span (start time, end time)

% Define initial conditions vector
initial_conditions = [phi_0; theta_0; psi_0; p_0; q_0; r_0];

% Define function for Euler angles dynamics + Add gravity gradient torque effects
tau_x = 3 * (I_yy - I_zz) * sin(x(2)) * cos(x(2)); % Simplified gravity gradient torque
tau_y = 3 * (I_zz - I_xx) * sin(x(1)) * cos(x(1));
tau_z = 0; % Simplified - no yaw torque

euler_dynamics = @(t, x) [x(4) + x(5) * sin(x(1)) * tan(x(2)) + x(6) * cos(x(1)) * tan(x(2));...
                          x(5) * cos(x(1)) - x(6) * sin(x(1));...
                          x(5) * sin(x(1)) * sec(x(2)) + x(6) * cos(x(1)) * sec(x(2));...
                          tau_x / I_xx; % Gravity gradient torque on roll
                          tau_y / I_yy; % Gravity gradient torque on pitch
                          tau_z / I_zz]; % No torque on yaw
% Integrate Euler angles dynamics over time
[t, euler_states] = ode45(euler_dynamics, t_span, initial_conditions);

% Extract Euler angles and rates from integration results
phi = euler_states(:, 1);
theta = euler_states(:, 2);
psi = euler_states(:, 3);
p = euler_states(:, 4);
q = euler_states(:, 5);
r = euler_states(:, 6);

% Display the final values of Euler angles and rates
disp('Final values of Euler angles and rates:');
disp(['Roll angle (phi): ', num2str(phi(end)), ' rad']);
disp(['Pitch angle (theta): ', num2str(theta(end)), ' rad']);
disp(['Yaw angle (psi): ', num2str(psi(end)), ' rad']);
disp(['Roll rate (p): ', num2str(p(end)), ' rad/s']);
disp(['Pitch rate (q): ', num2str(q(end)), ' rad/s']);
disp(['Yaw rate (r): ', num2str(r(end)), ' rad/s']);
%% 
% In this code:
% 
% We create two separate figures, each containing three subplots.
% 
% The first figure plots the Euler angles (roll, pitch, and yaw) as functions 
% of time.
% 
% The second figure plots the angular rates (p, q, and r) as functions of time.
% 
% Each subplot represents one of the Euler angles or angular rates.

% Plot Euler angles over time
figure;
subplot(3,1,1);
plot(t, phi, 'b');
xlabel('Time (s)');
ylabel('Roll angle (rad)');
title('Roll Angle (Phi) vs. Time');

subplot(3,1,2);
plot(t, theta, 'r');
xlabel('Time (s)');
ylabel('Pitch angle (rad)');
title('Pitch Angle (Theta) vs. Time');

subplot(3,1,3);
plot(t, psi, 'g');
xlabel('Time (s)');
ylabel('Yaw angle (rad)');
title('Yaw Angle (Psi) vs. Time');

% Plot angular rates over time
figure;
subplot(3,1,1);
plot(t, p, 'b');
xlabel('Time (s)');
ylabel('Roll rate (rad/s)');
title('Roll Rate (p) vs. Time');

subplot(3,1,2);
plot(t, q, 'r');
xlabel('Time (s)');
ylabel('Pitch rate (rad/s)');
title('Pitch Rate (q) vs. Time');

subplot(3,1,3);
plot(t, r, 'g');
xlabel('Time (s)');
ylabel('Yaw rate (rad/s)');
title('Yaw Rate (r) vs. Time');