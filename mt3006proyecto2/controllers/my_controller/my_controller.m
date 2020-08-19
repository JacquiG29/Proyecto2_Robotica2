%% IE - 3006 Robï¿½tica 2
% Jacqueline Guarcax
% Gabriela Iriarte
% Proyecto 2: Robot guï¿½a turï¿½stico

%% Activar debugging
% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
desktop;
%keyboard;

%% Parï¿½metros de Webots
TIME_STEP = 64;
MAX_SPEED = 1;
MAX_SENSOR_NUMBER = 16;
WHEEL_RADIUS = (195 / 2000.0);
DISTANCE_FROM_CENTER = (381 / 2000.0);
MAX_SENSOR_VALUE = 1024;
range = MAX_SENSOR_VALUE / 2;
max_speed = 5.24;
SPEED_UNIT = max_speed / 1024;
MIN_DISTANCE = 0.1;
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
% get the display (not a real e-puck device !)
camera_width = floor(wb_camera_get_width(camera));
camera_height = floor(wb_camera_get_height(camera));
byte_size = camera_width * camera_height * 4
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


controlador = 1;
epsilon = 1.8;
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

speed = zeros(1,2);
state = "f";

%Parametro de la camara
BR_C = [0,0,1; -1,0,0; 0,-1,0];
Bo_C = [0;0;0.1];
BT_C = [BR_C, Bo_C; 0,0,0,1];


f = wb_camera_get_focal_length(camera);
disp(f)
half_width = floor(wb_camera_get_width(camera) / 2);
half_height = floor(wb_camera_get_height(camera) / 2);


%rojo=wb_camera_image_get_red(camera)
%success = wb_camera_save_image(camera, 'prueba', 1)
%imwrite(image,'prueba.jpg')
%% MAIN LOOP
paquita=0;

while wb_robot_step(TIME_STEP) ~= -1
    
    wheel_weight_total = zeros(1, 2);
    
    
    %Visualizaci�n constante de la c�mara
    figure(1)
    rgb = wb_camera_get_image(camera);
    image(rgb);
    title('RGB Camera');
    
    %Thresholding del color
    [BWR, maskedRGBImageR] = createMask(rgb);
    BWR = imfill(BWR, 4, 'holes');
    BWR=bwareaopen(BWR,25);
    figure(2)
    imshow(BWR)
    if sum(BWR(:))> 0
        s1 = regionprops(BWR, 'centroid');
        centroide = s1.Centroid';
        disp(centroide)
    else
        centroide=[half_width;0];
        disp(centroide)
    end
    
    
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
    
    bin_sens_val = sensor_values ~= zeros(size(sensor_values));
    bin_sens_val(9:end) = zeros(1, 8);
    distance =  5 * (1.0 - (sensor_values / MAX_SENSOR_VALUE));
    bin_dist_val =  distance < MIN_DISTANCE;
    speed_modifier = bin_sens_val.*bin_dist_val.*(1 - (distance / MIN_DISTANCE));
    wheel_weight_total = wheel_weight_total + sum(speed_modifier'.*braitenberg_matrix);
    
    for k = 1:MAX_SENSOR_NUMBER
        if not(controlador == 0)
            if distance(k) < MIN_DISTANCE
                controlador = 2;
                break
            else
                controlador = 1;
            end
            
        end
    end
    
    
    if (xi >= xf - epsilon) && (xi <= xf + epsilon) && (zi >= zf - epsilon) && (zi <= zf + epsilon)
        controlador = 0;
        
    end
    
    
    
    formatSpec = 'xi: %.2f - zi: %.2f control: %d \n';
    fprintf(formatSpec, xi, zi, controlador);
    
    % ------------- OFF -------------
    if controlador == 0
        
        u0=half_width;
        v0=half_height;
        
        s = centroide-[u0,v0];%diferencia entre 
        
        ud = 0;
        vd = -22;
        
        ev=vd-s(2);
        ew=ud-s(1);
        
        v = 0.1*sign(s(2))*(ev);
        w = 0.01*(ew);
        
        formatSpec = 's1: %.2f s2: %.2f  ev: %.2f ew: %.2f\n';
        fprintf(formatSpec, s(1),s(2), ev, ew);
        
        left_speed = (v - w*DISTANCE_FROM_CENTER)/WHEEL_RADIUS;
        right_speed = (v + w*DISTANCE_FROM_CENTER)/WHEEL_RADIUS;
        
        speed = [left_speed,right_speed]
        for k = 1:2
            if speed(k) < -max_speed
                speed(k) = -max_speed;
            elseif speed(k) > max_speed
                speed(k) = max_speed;
            end
        end
        left_speed = speed(1, 1);
        right_speed = speed(1, 2);
        
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
        %speed_modifier = 1 - (sensor_values/range);
        %speed = speed + SPEED_UNIT*(speed_modifier)*braitenberg_matrix;
        
        [speed, state] = braitenberg(state, wheel_weight_total, speed, WHEEL_WEIGHT_THRESHOLD, MAX_SPEED);
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
    
    % Process here sensor data, images, etc.
    
    % send actuator commands, e.g.:
    %  wb_motor_set_postion(motor, 10.0);
    
    % if your code plots some graphics, it needs to flushed like this:
    drawnow;
    
end

% cleanup code goes here: write data to files, etc.

