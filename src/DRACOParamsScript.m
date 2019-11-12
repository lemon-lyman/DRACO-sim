DRACOParams.m = 1000;
DRACOParams.dimensions = [3 2 2]; % in B
w = DRACOParams.dimensions(1);
l = DRACOParams.dimensions(2);
h = DRACOParams.dimensions(3);
DRACOParams.Jq = (DRACOParams.m / 12) * [(l^2 + h^2), 0, 0;
                                         0, (w^2 + h^2), 0;
                                         0, 0, (l^2 + w^2)];

% Position of adcs thrusters in B
DRACOParams.adcs_position = [0 1 0;
                             0 1 0;
                             0 1 0;
                             0 0 1;
                             0 0 1;
                             0 0 1;
                             0 -1 0;
                             0 -1 0;
                             0 -1 0;
                             0 0 -1;
                             0 0 -1;
                             0 0 -1];

% Simplifying the trig,
minor_projection = sind(30);
major_projection = cosd(30);
% Orientation of adcs thrusters in B
DRACOParams.adcs_orientation = [1 0 0;
                                -minor_projection 0 -major_projection;
                                -minor_projection 0 major_projection;
                                1 0 0;
                                -minor_projection major_projection 0;
                                -minor_projection -major_projection 0;
                                1 0 0;
                                -minor_projection 0 major_projection;
                                -minor_projection 0 -major_projection;
                                1 0 0;
                                -minor_projection -major_projection 0;
                                -minor_projection major_projection 0];