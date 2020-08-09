%% IE - 3006 Rob�tica 2
% Jacqueline Guarcax
% Gabriela Iriarte
% Proyecto 2: Robot gu�a tur�stico

%% Activar debugging
% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
desktop;
%keyboard;

%% Par�metros de Webots
TIME_STEP = 64;
MAX_SPEED = 1;
MAX_SENSOR_NUMBER = 16;
WHEEL_RADIUS = (195 / 2000.0);
DISTANCE_FROM_CENTER = (381 / 2000.0);
MAX_SENSOR_VALUE = 1024;
range = MAX_SENSOR_VALUE / 2;
max_speed = 5.24;
SPEED_UNIT = max_speed / 1000;
MIN_DISTANCE = 1;
WHEEL_WEIGHT_THRESHOLD = 100;  % minimal weight for the robot to turn

goal_points= [-4, 0; 
               2, 4; 
            -3.5, 3.5; 
               2, -4];

braitenberg_matrix = [
    150 0;
    200, 0;
    300, 0;
    600, 0;
    0, 600;
    0, 300;
    0, 200;
    0, 150;
    0, 0;
    0, 0;
    0, 0;
    0, 0;
    0, 0;
    0, 0;
    0, 0;
    0, 0 ];

%% Webots - Enable Devices
camera = wb_robot_get_device('camera');
wb_camera_enable(camera, TIME_STEP);
time_step  = wb_robot_get_basic_time_step();
%  motor = wb_robot_get_device('motor');

% Device IDs
left_wheel  = wb_robot_get_device('left wheel');
right_wheel = wb_robot_get_device('right wheel');
position_sensor = wb_robot_get_device('gps');
orientation_sensor = wb_robot_get_device('compass');

% Set up wheels
wb_motor_set_position(left_wheel, Inf);
wb_motor_set_position(right_wheel, Inf);
wb_motor_set_velocity(left_wheel, 0.0);
wb_motor_set_velocity(right_wheel, 0.0);

% Enables GPS and compass
wb_gps_enable(position_sensor, TIME_STEP);
wb_compass_enable(orientation_sensor, TIME_STEP);

% get and enable all distance sensors
sonar = zeros(MAX_SENSOR_NUMBER, 1);
for k = 1:MAX_SENSOR_NUMBER
    sonar(k) = wb_robot_get_device(strcat('so', num2str(k - 1)));
    wb_distance_sensor_enable(sonar(k), time_step);
end
%period = wb_gps_get_sampling_period(tag)
%x_y_z_array = wb_gps_get_values(tag)
%speed = wb_gps_get_speed(tag)

%% Valores iniciales
pos = 0;
angle = 0;
xf = goal_points(3, 1);
zf = goal_points(3, 2);
xi = 0;
zi = 0;

alpha = 0.9;
speed = zeros(1,2);

controlador = 1;
epsilon = 0.5;
sensor_values = zeros(1, MAX_SENSOR_NUMBER);

%% Variables PID
ex = 0;
ez = 0;
ep = 0;
theta_g = 0;
eo = 0;
v = 0;

e_k_1 = 0;
E_k = 0;
e_k = 0;
eD = 0;
u_k = 0;

% CONSTANTES DEL PID
kP = 0.8;
kD = 0.0001;
kI = 0.01;

%% MAIN LOOP
while wb_robot_step(TIME_STEP) ~= -1
    %  Lectura de todos los sensores
    for k = 1:MAX_SENSOR_NUMBER
        sensor_values(k) = wb_distance_sensor_get_value(sonar(k));
    end
    
    north = wb_compass_get_values(orientation_sensor);
    rad = atan2(north(1, 1), north(1, 3));
    
    pos = wb_gps_get_values(position_sensor);
    xi = pos(1, 1);
    zi = pos(1, 3);
    
    angle = rad;
    
    distance =  5 * (1.0 - (sensor_values / MAX_SENSOR_VALUE));
    
    for k = 1:MAX_SENSOR_NUMBER
        if distance(k) < MIN_DISTANCE
            controlador = 2;
        else
            controlador = 1;
        end
    end
    
    statement = zi <= zf - epsilon || zi >= zf + epsilon;
    
    if (xi >= xf - epsilon) && (xi <= xf + epsilon) && (zi >= zf - epsilon) && (zi <= zf + epsilon)
        controlador = 0;
    end
    
    formatSpec = 'xi: %.2f - zi: %.2f control: %d \n';
    fprintf(formatSpec, xi, zi, statement);
    
    % ------------- OFF ------------- 
    if controlador == 0
        left_speed = 0;
        right_speed = 0;
 
    % ------------- PID ------------- 
    elseif controlador == 1
        % Error de posicion
        ex = xf - xi;
        ez = zf - zi;
        ep = sqrt(ex*ex + ez*ez);  % error total de posicion
        theta_g = atan2(ez, ex);
        
        % Error de orientacion
        eo = atan2(sin(theta_g - angle), cos(theta_g - angle));
        e_k = eo;
        
        % Controlador
        eD = e_k - e_k_1;  % error derivat
        E_k = E_k + e_k;  % error acumulado
        u_k = kP*e_k + kI*E_k + kD*eD;  % salida del controlador
        e_k_1 = e_k;  % actualizar las variables
        
        v = MAX_SPEED*(1 - exp(-ep*ep*alpha))/ep;  % velocidad uniciclo
        
        left_speed = (v - u_k*DISTANCE_FROM_CENTER)/WHEEL_RADIUS;
        right_speed = (v + u_k*DISTANCE_FROM_CENTER)/WHEEL_RADIUS;
        
    % ------------- Braitenberg -------------     
    elseif controlador == 2
        speed_modifier = 1 - (sensor_values/range);
        speed = speed + SPEED_UNIT*(speed_modifier)*braitenberg_matrix;
        
        for k = 1:2
            if speed(k) < -max_speed
                speed(k) = -max_speed;
            elseif speed(k) > max_speed
                speed(k) = max_speed;
            end
        end
        
        left_speed = speed(1, 1);
        right_speed = speed(1, 2);
    end
    

    wb_motor_set_velocity(left_wheel, left_speed);
    wb_motor_set_velocity(right_wheel, right_speed);
    
    % read the sensors, e.g.:
    %  rgb = wb_camera_get_image(camera);
    
    % Process here sensor data, images, etc.
    
    % send actuator commands, e.g.:
    %  wb_motor_set_postion(motor, 10.0);
    
    % if your code plots some graphics, it needs to flushed like this:
    drawnow;
    
end

% cleanup code goes here: write data to files, etc.

