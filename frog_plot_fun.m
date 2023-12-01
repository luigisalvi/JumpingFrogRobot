%Robot Plotting Function 
function legend_labels_i = frog_plot_fun(i, dim, prev,x_final, y_final,z_final)

    rand_clr = rand(2 * dim, 3);

    prev_x = prev(1);
    prev_y = prev(2);
    prev_z = prev(3);

    if i ~= dim
        % LinkPlot
        plot3([prev_x, x_final], [prev_y, y_final], [prev_z, z_final], 'Color', rand_clr(i, :), 'LineWidth', 2);
        hold on;
        % JointPlot
        plot3(x_final, y_final, z_final, 'o', 'Color', rand_clr(2 * i, :), 'MarkerSize', 9);

        % Legend Vector
        legend_labels_i{1} = ['a', num2str(i)];
        legend_labels_i{2} = ['joint ', num2str(i)];
    else
        % LinkPlot
        plot3([prev_x, x_final], [prev_y, y_final], [prev_z, z_final], 'Color', rand_clr(i, :), 'LineWidth', 2);
        hold on;
        % JointPlot
        plot3(x_final, y_final, z_final, 'hexagram', 'Color', rand_clr(2 * i, :), 'MarkerSize', 9);

        % Legend Vector
        legend_labels_i{1} = ['a', num2str(i)];
        % Here, last joint represents the center of mass of the frog-robot
        legend_labels_i{2} = 'C.O.M';
    end

    legend_labels_i = reshape(legend_labels_i, 1, []);
    
    %Plot Config
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    xlim([-1, 1]);
    ylim([-1, 1]);
    zlim([-1, 1]);
    
    title('Frog Hindlimb');
end