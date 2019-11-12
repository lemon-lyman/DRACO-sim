clc; clear; close all;

run("../src/DRACOParamsScript.m")
for ii = 1:12
    position = DRACOParams.adcs_position(ii, :);
    orientation = DRACOParams.adcs_orientation(ii, :);
    if mod(ii, 3)==0
        color = 'b';
    elseif mod(ii, 2)==0
        color = 'g';
    else
        color = 'r';
    end
    plot3([position(1), position(1)+orientation(1)],...
          [position(2), position(2)+orientation(2)],...
          [position(3), position(3)+orientation(3)], 'Color', color)
    hold on
end
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal;
grid on;