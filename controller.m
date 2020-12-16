% MATLAB controller for Webots
% File:          my_controller.m
% Date:
% Description:
% Author:
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
desktop;
keyboard;

TIME_STEP = 64;

s = 2 %speed
wheel1 = wb_robot_get_device('wheel1')
wb_motor_set_position(wheel1, inf)
wb_motor_set_velocity(wheel1, s)
wheel2 = wb_robot_get_device('wheel2')
wb_motor_set_position(wheel2, inf)
wb_motor_set_velocity(wheel2, s)
wheel3 = wb_robot_get_device('wheel3')
wb_motor_set_position(wheel3, inf)
wb_motor_set_velocity(wheel3, s)
wheel4 = wb_robot_get_device('wheel4')
wb_motor_set_position(wheel4, inf)
wb_motor_set_velocity(wheel4, s)


ds_right = wb_robot_get_device('ds_right')
ds_left = wb_robot_get_device('ds_left')
ds_center = wb_robot_get_device('ds_center')
wb_distance_sensor_enable(ds_right, TIME_STEP)
wb_distance_sensor_enable(ds_left, TIME_STEP)
wb_distance_sensor_enable(ds_center, TIME_STEP)

cm = wb_robot_get_device('CAM')
cm = wb_camera_enable(CAM, TIME_STEP)



% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination
%
while wb_robot_step(TIME_STEP) ~= -1
distance_r = wb_distance_sensor_get_value('ds_right')
distance_l = wb_distance_sensor_get_value('ds_left')
distance_c = wb_distance_sensor_get_value('ds_center')
imagine = wb_camera_get_image('cm')
 
 disp(distance_r)
 disp(distance_l)
 disp(distance_c)
 
if distance_c > 80
s = 2
wb_motor_set_velocity(wheel1, s)
wb_motor_set_velocity(wheel2, s)
wb_motor_set_velocity(wheel3, s)
wb_motor_set_velocity(wheel4, s)
elseif distance_c < 80
s = 0
wb_motor_set_velocity(wheel1, s)
wb_motor_set_velocity(wheel2, s)
wb_motor_set_velocity(wheel3, s)
wb_motor_set_velocity(wheel4, s)
end
if distance_c < 80 & distance_c < 100
s = -2
wb_motor_set_velocity(wheel1, s)
wb_motor_set_velocity(wheel2, s)
wb_motor_set_velocity(wheel3, s)
wb_motor_set_velocity(wheel4, s)
else
s = 2
wb_motor_set_velocity(wheel1, s)
wb_motor_set_velocity(wheel2, s)
wb_motor_set_velocity(wheel3, s)
wb_motor_set_velocity(wheel4, s)
end
end



  % read the sensors, e.g.:
  %  rgb = wb_camera_get_image(camera);

  % Process here sensor data, images, etc.

  % send actuator commands, e.g.:
  %  wb_motor_set_postion(motor, 10.0);

  % if your code plots some graphics, it needs to flushed like this:
  drawnow;

end

% cleanup code goes here: write data to files, etc.
