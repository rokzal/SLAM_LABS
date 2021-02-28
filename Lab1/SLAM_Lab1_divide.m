%-------------------------------------------------------------------------
% University of Zaragoza
%
% Author:  U. Ram�n and L. Cano
%-------------------------------------------------------------------------
% SLAM for Karel the robot in 1D
%-------------------------------------------------------------------------

clear all; % varialbes
close all; % figures
randn('state', 1); % always use same random number sequence
rand('state', 1); % always use same random number sequence
format long

%-------------------------------------------------------------------------
% configuration parameters

global config;

% display correlation matrix and pause at every step
config.step_by_step = 0;

%number of robot motions for each local map
config.steps_per_map = 1000;

% figure counter (to always plot a new figure)
config.fig = 0;

%Size of local maps.
config.map_size = 20;

%-------------------------------------------------------------------------

%-------------------------------------------------------------------------
% 1D world characteristics

global world;

%   true_point_locations: that in the absolute W reference
%    true_robot_location: that in the absolute W reference

% will not change during program execution
world.true_point_locations = [0.5:1:10000]';

% will change as the robot moves
world.true_robot_location = 0;

%-------------------------------------------------------------------------

%-------------------------------------------------------------------------
% robot characteristics

global robot;

%    factor_x: fraction of the motion that constitutes odometry error
%     true_uk: ground true robot motion per step

robot.factor_x = 0.1; % ten percent
robot.true_uk = 1; % each true motion is 1m
%-------------------------------------------------------------------------

%-------------------------------------------------------------------------
% sensor characteristics

global sensor;

%     factor_z: measurement error is this fraction of the distance
%    range_min: minimum sensor range
%    range_max: maximum sensor range

sensor.factor_z = 0.01; % one percent
sensor.range_min = 0;
sensor.range_max = 2;
%-------------------------------------------------------------------------

%-------------------------------------------------------------------------
% all map details

global global_map;

%            R0: absolute location of base reference for map
%         hat_x: estimated robot and feature locations
%         hat_P: robot and feature covariance matrix
%             n: number of features in map
%         num_m: number of local maps.
%        true_x: true robot and feature location (with respect to R0)
%      true_ids: true label of features in map (according to world)
%  stats.true_x: true robotlocation wrt R0 for the whole trajectory
% stats.error_x: robotlocation error at every step of the trajectory
% stats.sigma_x: robot location uncertainty at every step of the trajectory
%  stats.cost_t: computational cost (elapsed time) for each step
%-------------------------------------------------------------------------


%-------------------------------------------------------------------------
% measurements (not global, but this is the structure)

%            z_k: measured distance to all visible features
%            R_k: covariance of z_k
%          ids_f: true ids of the ones that are already in the map
%          ids_n: true ids of the ones that new to the map
%        z_pos_f: positions in z_r of ids_f
%        z_pos_n: positions in z_r of ids_n
%        x_pos_f: positions in hat_x of ids_f
%            z_f: z_k(ids_f), measurements to features already in the map
%            R_f: R_k(ids_f), meas. cov. to features already in the map
%            z_n: z_k(ids_n), measurements to features new to the map
%            R_n: R_k(ids_n), Meas. cov. to features new to the map
%-------------------------------------------------------------------------

%-------------------------------------------------------------------------
% BEGIN
%-------------------------------------------------------------------------
sizes = 5:5:1000; 
trials = 10;
trial_cost = [];
all_cost = [];


%sizes = [50];
%trials = 1;
for i =1:trials
    all_cost = [];
    for size = sizes
        %config.map_size
        clear global_map;
        global global_map;
        world.true_robot_location = 0;
        [global_map] = Kalman_filter_slam (global_map, config.steps_per_map,size);
        
        tstart = tic;
        
        [final_map] = reconstruct_map(global_map);
        [improved_map] = depurate_map(final_map);
     
        total_cost = sum(final_map.stats.cost_t);
        total_cost = total_cost+ toc(tstart);
        all_cost = [all_cost,total_cost];
    end
    trial_cost = [trial_cost;all_cost];
end


average_times = mean(trial_cost);
%Make prettier.
plot(sizes',average_times);
config.fig = config.fig + 1;
figure(config.fig);
xlabel('Size of local Map.');
ylabel('Time(s).');
title('Execution time given local map size.');






display_map_results (final_map);
display_map_results (improved_map);


%-------------------------------------------------------------------------
% END
%-------------------------------------------------------------------------

%-------------------------------------------------------------------------
% add local maps.
%-------------------------------------------------------------------------
function[final_map] = reconstruct_map(global_map)
global world;
final_map = global_map.maps{1};
for k = 2:global_map.i
    map = global_map.maps{k};
    %Constants
    m = final_map.n;
    n = map.n;
    
    %A and B matrices.
    A = [eye(m+1,m+1);[ones(n,1),zeros(n,m)]];
    B = [[1,zeros(1,n)];zeros(m,n+1);[zeros(n,1),eye(n,n)]];
    
    %Add new features to state vector and covariance matrix.
    final_map.hat_x = A*final_map.hat_x + B*map.hat_x;
    final_map.hat_P = A*final_map.hat_P*A.' + B*map.hat_P*B.';
    
    %Actuallize meta-data 
    final_map.n = final_map.n + n;
    final_map.true_ids = [final_map.true_ids; map.true_ids(2:end)];
    final_map.true_x = [final_map.true_x;world.true_point_locations(map.true_ids(2:end))];
    
    final_map.stats.true_x = [final_map.stats.true_x; map.stats.true_x+map.R0];
    final_map.stats.error_x = [final_map.stats.error_x; map.stats.error_x];
    final_map.stats.sigma_x = [final_map.stats.sigma_x; map.stats.sigma_x];
    final_map.stats.cost_t = [final_map.stats.cost_t; map.stats.cost_t];


end
end

%-------------------------------------------------------------------------
% add local maps fusing repeated features.
%-------------------------------------------------------------------------
function[global_map] = depurate_map(global_map)
global world;
%Identify repeated features.
uni = unique(global_map.true_ids); % which will give you the unique elements of A in array B
Ncount = histc(global_map.true_ids,uni); % this willgive the number of occurences of each unique element
repeated_ids = uni(Ncount>1)';
first = [];
second = [];
for id = repeated_ids
    pos = find(global_map.true_ids' == id);
    first = [first,pos(1)];
    second = [second,pos(2)];
end 

%constants:  
m = global_map.n;
f = size(first,2);
i = m - f;

%Compute H_k.
H_k = zeros(f,m + 1);
indices = (first-1) * f + [1:f];
H_k(indices)=+1;
indices = (second-1) * f + [1:f];
H_k(indices)=-1;
%H_k = [zeros(f,1),zeros(f,i),eye(f,f)];

%Kalman filter update.
y_tilde_k = -H_k * global_map.hat_x;
S_k = H_k * global_map.hat_P * H_k' ;
K_k = global_map.hat_P*H_k'/S_k;
%K_k = map.hat_P*H_k'*inv(S_k);
global_map.hat_x = global_map.hat_x + K_k * y_tilde_k;
global_map.hat_P = (eye(size(global_map.hat_P))-K_k*H_k)*global_map.hat_P;

%Remove second appereance of features from map.
global_map.hat_x(second) = [];
global_map.hat_P(second,:) = [];
global_map.hat_P(:,second) = [];

global_map.true_ids(second) = [];
global_map.true_x(second) = [];
global_map.n = global_map.n - max(size(second));

end

%-------------------------------------------------------------------------
% Kalman_filter_slam
%-------------------------------------------------------------------------

function[global_map] = Kalman_filter_slam (global_map, steps,map_size)

global world;
global_map.i = 0;
global_map.maps ={};
create_map = true;
acum_pos = 0;

k=0;
while k ~= steps
    tstart = tic;
    
    if ~create_map
        [map] = compute_motion (map);
        k = k+1;
    end
    
    if create_map 
        global_map.i = global_map.i +1;
        % initial ground truth of hat_x 
        global_map.maps{global_map.i}.true_x = zeros(map_size);
        global_map.maps{global_map.i}.true_x = [0];

        % initial number of features
        global_map.maps{global_map.i}.n = 0;

        % initial map state and covariance
        global_map.maps{global_map.i}.R0 = world.true_robot_location;

        global_map.maps{global_map.i}.hat_x = zeros(map_size);
        global_map.maps{global_map.i}.hat_x = [0];

        global_map.maps{global_map.i}.hat_P = zeros(map_size,map_size);
        global_map.maps{global_map.i}.hat_P = [0];
        

        % feature ids, for robot = 0
        global_map.maps{global_map.i}.true_ids = [0];

        % useful statistics
        global_map.maps{global_map.i}.stats.true_x = [0];
        global_map.maps{global_map.i}.stats.error_x = [];
        global_map.maps{global_map.i}.stats.sigma_x = [];
        global_map.maps{global_map.i}.stats.cost_t = [];
        if global_map.i > 1
            %Initialize with 
            %global_map.maps{global_map.i}.hat_P = global_map.maps{global_map.i-1}.hat_P(1,1);
            global_map.maps{global_map.i}.stats.true_x = [global_map.maps{global_map.i-1}.stats.true_x(1)];
        end

        map = global_map.maps{global_map.i};
        create_map = false;
    end
    
    
    % get measurements
    [measurements] = get_measurements_and_data_association(map);
    
    % seen features already in the map? KF update!
    if not(isempty(measurements.z_f))
        [map] = update_map (map, measurements);
    end
    
    % some new features?
    if not(isempty(measurements.z_n))
        [map] = add_new_features (map, measurements);
    end
    
    % record statistics
    map.stats.error_x = [map.stats.error_x; (map.stats.true_x(end) - (map.hat_x(1)))];
    map.stats.sigma_x = [map.stats.sigma_x; sqrt(map.hat_P(1,1))];
    map.stats.cost_t = [map.stats.cost_t; toc(tstart)];
    
    if size(map.hat_x,1)-1 >= map_size 
       create_map = true; 
       global_map.maps{global_map.i} = map;
    end
end
global_map.maps{global_map.i} = map;
end

%-------------------------------------------------------------------------
% compute_motion
%-------------------------------------------------------------------------

function [map] = compute_motion (map)

global config;
global robot;
global world;

% move robot and update ground truth robot location
world.true_robot_location = world.true_robot_location + robot.true_uk;

% computer error
sigma_xk = robot.factor_x * robot.true_uk;
error_k = randn(1)*sigma_xk;

% computer estimated motion
map.hat_x(1) = map.hat_x(1) + robot.true_uk + error_k;
map.hat_P(1,1) = map.hat_P(1,1) + sigma_xk^2;

% update ground truth for the current map
map.true_x(1) = map.true_x(1) + robot.true_uk;
map.stats.true_x = [map.stats.true_x; map.stats.true_x(end) + robot.true_uk];

if config.step_by_step
    fprintf('Move to %d...\n',map.true_x(1));
    plot_correlation(map.hat_P);
    pause
end
end

%-------------------------------------------------------------------------
% get_measurements
%-------------------------------------------------------------------------

function [measurements] = get_measurements_and_data_association(map)

global world;
global sensor;

distances = world.true_point_locations - world.true_robot_location;
visible_ids = find((distances >= sensor.range_min) & (distances <= sensor.range_max));
visible_d = distances(visible_ids);
n_visible = length(visible_ids);

sigma_z = sensor.factor_z * visible_d;
error_z = randn(n_visible, 1) .* sigma_z;

measurements.z_k = visible_d + error_z;
measurements.R_k = diag(sigma_z.^2);

% data association
measurements.ids_f = intersect(map.true_ids, visible_ids);
measurements.ids_n = setdiff(visible_ids, map.true_ids);
measurements.z_pos_f = find(ismember(visible_ids, measurements.ids_f));
measurements.z_pos_n = find(ismember(visible_ids, measurements.ids_n));
measurements.x_pos_f = find(ismember(map.true_ids, visible_ids));

%features already in the map
measurements.z_f = measurements.z_k(measurements.z_pos_f);
measurements.R_f = measurements.R_k(measurements.z_pos_f,measurements.z_pos_f);

%new features
measurements.z_n = measurements.z_k(measurements.z_pos_n);
measurements.R_n = measurements.R_k(measurements.z_pos_n,measurements.z_pos_n);

end

%-------------------------------------------------------------------------
% update_map
%-------------------------------------------------------------------------

function[map] = update_map (map, measurements)

    global config;

    % DO SOMETHING HERE!
    % You need to compute H_k, y_k, S_k, K_k and update map.hat_x and map.hat_P
    
    %constants:  
    m = map.n;
    f = size(measurements.ids_f,1);
    i = m - f;
    
    %Compute H_k.
    H_k = [-ones(f,1),zeros(f,i),eye(f,f)];

    %Kalman filter update.
    y_tilde_k = measurements.z_f  -H_k * map.hat_x;
    S_k = H_k * map.hat_P * H_k' + measurements.R_f;
    K_k = map.hat_P*H_k'/S_k;
    %K_k = map.hat_P*H_k'*inv(S_k);
    map.hat_x = map.hat_x + K_k * y_tilde_k;
    map.hat_P = (eye(size(map.hat_P))-K_k*H_k)*map.hat_P;
    
    if config.step_by_step
        fprintf('Updape map for last %d features...\n',length(measurements.ids_f));
        plot_correlation(map.hat_P);
        pause
    end
end

%-------------------------------------------------------------------------
% add_new_features
%-------------------------------------------------------------------------

function [map] = add_new_features (map, measurements)

    global config;
    global world;

    % DO SOMETHING HERE!
    % update map.hat_x, map.hat_P, map.true_ids, map.true_x, map.n
    %The principled way
    %constants: 
    m = map.n;
    n = size(measurements.ids_n,1);
    f = size(measurements.ids_f,1);
    
    % A and B matrices.
    A = [eye(m+1,m+1);[ones(n,1),zeros(n,m)]]; % m + n +1 x m + 1
    B = [zeros(m+1,f+n);[zeros(n,f),eye(n,n)]]; % m+1 x f + n
    
    %Add new features to state vector and covariance matrix.
    map.hat_x = A*map.hat_x + B*measurements.z_k;
    map.hat_P = A*map.hat_P*A.' + B*measurements.R_k*B.';
    
    %Actuallize meta-data 
    map.n = map.n + n;
    map.true_ids = [map.true_ids; measurements.ids_n];
    map.true_x = [map.true_x;world.true_point_locations(measurements.ids_n)];

    if config.step_by_step
        fprintf('Add %d new features...\n',length(measurements.ids_n));
        plot_correlation(map.hat_P);
        pause
    end
end

%-------------------------------------------------------------------------
% display_results
%-------------------------------------------------------------------------

function  display_map_results (map)

global config;

config.fig = config.fig + 1;
figure(config.fig);
axis([0 length(map.hat_x) -2*max(sqrt(diag(map.hat_P))) 2*max(sqrt(diag(map.hat_P)))]);
grid on;
hold on;
plot(map.true_ids, map.hat_x-map.true_x, 'ro','Linewidth', 2);
plot(map.true_ids, 2*sqrt(diag(map.hat_P)), 'b+','Linewidth', 2);
plot(map.true_ids, -2*sqrt(diag(map.hat_P)), 'b+','Linewidth', 2);
xlabel('Feature number (robot = 0)');
ylabel('meters (m)');
title('Map estimation error + 2sigma bounds');

config.fig = config.fig + 1;
figure(config.fig);
plot_correlation(map.hat_P);
title(sprintf('Correlation matrix of size %d', size(map.hat_P,1)));

config.fig = config.fig + 1;
figure(config.fig);
grid on;
hold on;
plot(map.stats.true_x, map.stats.cost_t,'r-','Linewidth',2);
xlabel('step');
ylabel('seconds');
title('Cost per step');

config.fig = config.fig + 1;
figure(config.fig);
grid on;
hold on;
plot(map.stats.true_x,cumsum(map.stats.cost_t),'r-','Linewidth',2);
xlabel('step');
ylabel('seconds');
title('Cumulative cost');

config.fig = config.fig + 1;
figure(config.fig);
axis([0 map.stats.true_x(end) -2*max(map.stats.sigma_x) 2*max(map.stats.sigma_x)]);
grid on;
hold on;
plot(map.stats.true_x, map.stats.error_x,'r-','Linewidth',2);
plot(map.stats.true_x, 2*map.stats.sigma_x,'b-','Linewidth',2);
plot(map.stats.true_x,-2*map.stats.sigma_x,'b-','Linewidth',2);
xlabel('meters (m)');
ylabel('meters (m)');
title('Robot estimation error + 2sigma bounds');

end

%-------------------------------------------------------------------------
% plot_correlation
%-------------------------------------------------------------------------

function plot_correlation(P)

ncol=256 ;

% hot cold colormap
cmap=hsv2rgb([linspace(2/3, 0, ncol)' 0.9*ones(ncol,1) ones(ncol,1)]) ;
cmap(1,:)=[0 0 0] ;
colormap(cmap) ;

corr = correlation(P);
imagesc(abs(corr), [0 1]) ;  % no sign

axis image;
colorbar ;

end

%-------------------------------------------------------------------------
% correlation
%-------------------------------------------------------------------------

function Corr=correlation(Cov)

sigmas = sqrt(diag(Cov));
Corr = diag(1./sigmas)*Cov*diag(1./sigmas);

end