function tb_goal_area()
%TB_GOAL_AREA Unit tests for paper-aligned goal area construction.
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));

    map_x = 16;
    map_y = 16;
    spec = struct( ...
        'xy_resolution', 0.05, ...
        'map_origin_x', 0.0, ...
        'map_origin_y', 0.0, ...
        'goal_x', 0.225, ...
        'goal_y', 0.225, ...
        'goal_theta_deg', 90, ...
        'goal_radius_m', 0.30, ...
        'goal_margin_theta_deg', 15);

    goal_mask = make_goal_mask(map_x, map_y, spec);

    % The goal must cover an area on XY plane, not just one cell.
    assert(nnz(goal_mask(:, :, 15)) > 1, 'Goal area collapsed to a single cell');

    % The center cell is inside the goal area for theta bins fully contained
    % in [75, 105] deg when theta resolution is 6 deg.
    assert(goal_mask(5, 5, 14), 'Expected theta bin 14 to be inside goal area');
    assert(goal_mask(5, 5, 17), 'Expected theta bin 17 to be inside goal area');

    % Bins that are only partially covered by the angular margin are excluded.
    assert(~goal_mask(5, 5, 13), 'Theta bin 13 should be outside goal area');
    assert(~goal_mask(5, 5, 18), 'Theta bin 18 should be outside goal area');

    % Distant cells must remain outside the goal area.
    assert(~goal_mask(16, 16, 15), 'Far cell incorrectly included in goal area');

    disp('tb_goal_area: ALL PASSED');
end
