function goal_mask = make_goal_mask(map_x, map_y, spec)
%MAKE_GOAL_MASK Build a 3D goal-area mask following the original ROS code.
%   spec fields:
%     xy_resolution
%     map_origin_x
%     map_origin_y
%     goal_x
%     goal_y
%     goal_theta_deg
%     goal_radius_m
%     goal_margin_theta_deg

    p = vi_params();
    goal_mask = false(map_y, map_x, p.N_THETA);
    t_resolution = 360 / p.N_THETA;

    for iy = 0:map_y-1
        for ix = 0:map_x-1
            x0 = ix * spec.xy_resolution + spec.map_origin_x;
            y0 = iy * spec.xy_resolution + spec.map_origin_y;
            x1 = x0 + spec.xy_resolution;
            y1 = y0 + spec.xy_resolution;

            r0 = (x0 - spec.goal_x) ^ 2 + (y0 - spec.goal_y) ^ 2;
            r1 = (x1 - spec.goal_x) ^ 2 + (y1 - spec.goal_y) ^ 2;
            in_xy = r0 < spec.goal_radius_m ^ 2 && r1 < spec.goal_radius_m ^ 2;
            if ~in_xy
                continue;
            end

            for it = 0:p.N_THETA-1
                t0 = it * t_resolution;
                t1 = (it + 1) * t_resolution;
                wrapped_goal = spec.goal_theta_deg;
                if spec.goal_theta_deg > 180
                    wrapped_goal = spec.goal_theta_deg - 360;
                else
                    wrapped_goal = spec.goal_theta_deg + 360;
                end

                in_theta = (spec.goal_theta_deg - spec.goal_margin_theta_deg <= t0 ...
                         && t1 <= spec.goal_theta_deg + spec.goal_margin_theta_deg) ...
                        || (wrapped_goal - spec.goal_margin_theta_deg <= t0 ...
                         && t1 <= wrapped_goal + spec.goal_margin_theta_deg);
                if in_theta
                    goal_mask(iy + 1, ix + 1, it + 1) = true;
                end
            end
        end
    end
end
