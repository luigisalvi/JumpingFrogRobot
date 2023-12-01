function [current_position, T_total]=Direct_Kin_frog(plot,T1,T2,T3)
% DH Parameters table and const.
a1 = 0.7;
a2 = a1;
a3 = 0.4;
a4 = 0.3;

theta1= deg2rad(360-T1); %-195 init. pos.
theta2= deg2rad(-360+T2);%+165 init. pos.
theta3= deg2rad(360-T3); %-195 init. pos.

dh_params = [
    a1   0      0   pi;
    a2   0      0   pi-theta1+pi;
    a3   0      0   theta2-2*pi;
    a4   0      0   2*pi-theta3;
];

% Homogenous transformation matrix init
T_total = eye(4);

% Tiptoe initial position (origin of the reference frame)
prev = [0;0;0];

% Inizializza la cella di celle per le etichette
legend_labels = cell(1, size(dh_params, 1));
dim=size(dh_params,1);
%Direct Kinematics and plotting computation
for i = 1:size(dh_params, 1)
    a = dh_params(i, 1);
    alpha = dh_params(i, 2);
    d = dh_params(i, 3);
    theta = dh_params(i, 4);

    % Homogenous transformation matrix for the i-th DH transform
    A_i = [
        cos(theta)    -sin(theta)*cos(alpha)     sin(theta)*sin(alpha)    a*cos(theta);
        sin(theta)     cos(theta)*cos(alpha)    -cos(theta)*sin(alpha)    a*sin(theta);
        0                     sin(alpha)         cos(alpha)               d;
        0                      0                      0                   1
    ]; 

    % Updating total transformation matrix
    T_total = T_total * A_i;

    % Final joint positions extracted from each homogeneous transf. matrix
    x_final(i) = T_total(1, 4);
    y_final(i) = T_total(2, 4);
    z_final(i) = T_total(3, 4);

    %Plotting Conditions and Legend setting
    if plot == true
    legend_labels_i = frog_plot_fun(i, dim, prev,x_final(i), y_final(i),z_final(i));
    legend_labels{i} = legend_labels_i;
    legend_labels = [legend_labels{:}];
    legend(legend_labels)
    end
    
    %Update final joint positions for the subsequent link placement
    prev_x = x_final(i);
    prev_y = y_final(i);
    prev_z = z_final(i);
    prev = [prev_x; prev_y; prev_z];
end

current_position = T_total(1:3, 4);

end