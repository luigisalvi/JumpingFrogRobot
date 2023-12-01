function path = trajectory_fun(final_position, trj)

% C.O.M. coordinates in initial pose (195°,165°,195°)
x1 = -0.134074173710932;
y1 = 0.258819045102521;
z1 = 0;

% C.O.M. final desired coordinates 
x2 = final_position(1);
y2 = final_position(2);
z2=  final_position(3);

t_values = linspace(0, 1, 10);

% 3D trajectory direction from A to B
direction_vector = [x2 - x1, y2 - y1, z2 - z1];

switch trj
    case 1
        %Line 
        x_values = x1 + t_values * direction_vector(1);
        y_values = y1 + t_values * direction_vector(2);
        z_values = z1 + t_values * direction_vector(3);
        path = [x_values; y_values; z_values];
        
    case 2
        %Parabola
        parabola_x = x1 + t_values * direction_vector(1);
        parabola_y = y1 + t_values.^2 * direction_vector(2);
        parabola_z = z1 + t_values * direction_vector(3);
        path = [parabola_x; parabola_y; parabola_z];
    
    %ADD HERE OTHER DESIRED TRAJECTORIES FOLLOWING THE ESTABLISHED SCHEME
end














end