function [value_table, sweeps, visited_states, final_delta, stats] = vi_pyramid_sweep( ...
    value_table, penalty_table, goal_mask, transitions, map_x, map_y, ...
    threshold, max_sweeps, min_size, coarse_sweeps, refine_sweeps, descend_tau)
%VI_PYRAMID_SWEEP Coarse-to-fine VI over a 2x2 spatial pyramid.
%   Starts from the coarsest 2x2-reduced map, sweeps only active regions,
%   then descends into the children of changed coarse blocks.

    if nargin < 9 || isempty(min_size)
        min_size = 4;
    end
    if nargin < 10 || isempty(coarse_sweeps)
        coarse_sweeps = 8;
    end
    if nargin < 11 || isempty(refine_sweeps)
        refine_sweeps = max_sweeps;
    end
    if nargin < 12 || isempty(descend_tau)
        descend_tau = 0;
    end

    p = vi_params();
    MV = double(p.MAX_VALUE);
    OB = double(p.PENALTY_OBSTACLE);
    NT = p.N_THETA;
    base_trans = coerce_transition_model(transitions);

    level_mx = zeros(1, 16);
    level_my = zeros(1, 16);
    values = cell(1, 16);
    penalties = cell(1, 16);
    goals = cell(1, 16);
    active_masks = cell(1, 16);

    n_levels = 1;
    level_mx(1) = map_x;
    level_my(1) = map_y;
    values{1} = value_table;
    penalties{1} = penalty_table;
    goals{1} = goal_mask;

    while n_levels < 16 && ...
            (level_mx(n_levels) > min_size || level_my(n_levels) > min_size) && ...
            (level_mx(n_levels) > 1 || level_my(n_levels) > 1)
        [values{n_levels + 1}, penalties{n_levels + 1}, goals{n_levels + 1}] = ...
            coarsen_level(values{n_levels}, penalties{n_levels}, ...
            goals{n_levels}, level_mx(n_levels), level_my(n_levels), MV, OB, NT);
        level_mx(n_levels + 1) = ceil(level_mx(n_levels) / 2);
        level_my(n_levels + 1) = ceil(level_my(n_levels) / 2);
        n_levels = n_levels + 1;
    end

    active_masks{n_levels} = any(goals{n_levels}, 3);

    sweeps = 0;
    visited_states = 0;
    final_delta = MV;
    stats = repmat(empty_stat(), 1, n_levels);
    remaining = max(0, floor(max_sweeps));

    for level = n_levels:-1:1
        if level < n_levels
            values{level} = prolongate_level(values{level + 1}, ...
                penalties{level}, goals{level}, level_mx(level), ...
                level_my(level), MV, OB, NT);
        end

        goal_spatial = any(goals{level}, 3);
        if isempty(active_masks{level})
            active_masks{level} = goal_spatial;
        else
            active_masks{level} = active_masks{level} | goal_spatial;
        end

        if ~any(active_masks{level}(:))
            break;
        end

        if level == 1
            cap = min(remaining, max(0, floor(refine_sweeps)));
        else
            cap = min(remaining, max(1, floor(coarse_sweeps)));
        end
        if cap <= 0
            break;
        end

        scale = 2 ^ (level - 1);
        trans_model = scale_transition_model(base_trans, scale);
        [mx, my, ~] = vi_frontier_max_displacement(trans_model);
        candidate_mask = dilate_spatial_mask(active_masks{level}, mx, my, ...
            level_mx(level), level_my(level));

        [values{level}, done, changed, final_delta, descend_mask] = run_masked_sweeps( ...
            values{level}, penalties{level}, goals{level}, trans_model, ...
            level_mx(level), level_my(level), threshold, cap, OB, NT, ...
            candidate_mask, descend_tau);

        sweeps = sweeps + done;
        remaining = remaining - done;
        visited = nnz(candidate_mask) * NT * done;
        visited_states = visited_states + visited;
        stats(level) = struct('level', level, 'map_x', level_mx(level), ...
            'map_y', level_my(level), 'scale', scale, 'sweeps', done, ...
            'changed_states', changed, 'visited_states', visited, ...
            'final_delta', final_delta);

        if level > 1
            descend_mask = descend_mask | any(goals{level}, 3);
            active_masks{level - 1} = prolongate_active_mask(descend_mask, ...
                level_mx(level - 1), level_my(level - 1));
        end

        if remaining <= 0
            break;
        end
    end

    value_table = values{1};
    value_table(goal_mask) = 0;
end

function stat = empty_stat()
    stat = struct('level', 0, 'map_x', 0, 'map_y', 0, 'scale', 0, ...
        'sweeps', 0, 'changed_states', 0, 'visited_states', 0, ...
        'final_delta', 0);
end

function [coarse_value, coarse_penalty, coarse_goal] = coarsen_level( ...
    value_table, penalty_table, goal_mask, map_x, map_y, MV, OB, NT)

    coarse_x = ceil(map_x / 2);
    coarse_y = ceil(map_y / 2);
    coarse_value = ones(coarse_y, coarse_x, NT) * MV;
    coarse_penalty = ones(coarse_y, coarse_x) * OB;
    coarse_goal = false(coarse_y, coarse_x, NT);

    for cy = 1:coarse_y
        y0 = (cy - 1) * 2 + 1;
        y1 = min(y0 + 1, map_y);
        for cx = 1:coarse_x
            x0 = (cx - 1) * 2 + 1;
            x1 = min(x0 + 1, map_x);

            pen_block = penalty_table(y0:y1, x0:x1);
            free_pen = pen_block(pen_block ~= OB);
            if ~isempty(free_pen)
                coarse_penalty(cy, cx) = min(free_pen(:));
            end

            for it = 1:NT
                goal_block = goal_mask(y0:y1, x0:x1, it);
                coarse_goal(cy, cx, it) = any(goal_block(:));
                if coarse_goal(cy, cx, it)
                    coarse_value(cy, cx, it) = 0;
                elseif coarse_penalty(cy, cx) ~= OB
                    val_block = value_table(y0:y1, x0:x1, it);
                    coarse_value(cy, cx, it) = min(val_block(:));
                end
            end
        end
    end
end

function fine_value = prolongate_level(coarse_value, fine_penalty, fine_goal, ...
    map_x, map_y, MV, OB, NT)

    fine_value = ones(map_y, map_x, NT) * MV;
    for iy = 1:map_y
        cy = floor((iy - 1) / 2) + 1;
        for ix = 1:map_x
            if fine_penalty(iy, ix) == OB
                continue;
            end
            cx = floor((ix - 1) / 2) + 1;
            for it = 1:NT
                fine_value(iy, ix, it) = coarse_value(cy, cx, it);
            end
        end
    end
    fine_value(fine_goal) = 0;
end

function child_mask = prolongate_active_mask(parent_mask, map_x, map_y)
    child_mask = false(map_y, map_x);
    parent_y = size(parent_mask, 1);
    parent_x = size(parent_mask, 2);
    for py = 1:parent_y
        y0 = (py - 1) * 2 + 1;
        y1 = min(y0 + 1, map_y);
        for px = 1:parent_x
            if ~parent_mask(py, px)
                continue;
            end
            x0 = (px - 1) * 2 + 1;
            x1 = min(x0 + 1, map_x);
            child_mask(y0:y1, x0:x1) = true;
        end
    end
end

function out = dilate_spatial_mask(mask, dx, dy, map_x, map_y)
    out = false(map_y, map_x);
    pts = find(mask);
    for idx = 1:numel(pts)
        [iy, ix] = ind2sub([map_y, map_x], pts(idx));
        x0 = max(1, ix - dx);
        x1 = min(map_x, ix + dx);
        y0 = max(1, iy - dy);
        y1 = min(map_y, iy + dy);
        out(y0:y1, x0:x1) = true;
    end
end

function model = scale_transition_model(model, scale)
    if scale <= 1
        return;
    end

    for a = 1:size(model.dix, 1)
        for it = 1:size(model.dix, 2)
            n_out = model.n_outcomes(a, it);
            for k = 1:n_out
                model.dix(a, it, k) = coarse_delta(model.dix(a, it, k), scale);
                model.diy(a, it, k) = coarse_delta(model.diy(a, it, k), scale);
            end
        end
    end
end

function d = coarse_delta(d, scale)
    if d == 0
        return;
    end
    d = sign(d) * max(1, ceil(abs(double(d)) / scale));
end

function [value_table, sweeps, changed_states, final_delta, changed_mask] = ...
    run_masked_sweeps(value_table, penalty_table, goal_mask, trans_model, ...
    map_x, map_y, threshold, max_sweeps, OB, NT, candidate_mask, descend_tau)

    sweeps = 0;
    changed_states = 0;
    final_delta = 0;
    changed_mask = false(map_y, map_x);

    for sweep = 1:max_sweeps
        max_delta = 0;
        changed_this = 0;
        changed_now = false(map_y, map_x);
        for iy = 1:map_y
            for ix = 1:map_x
                if ~candidate_mask(iy, ix) || penalty_table(iy, ix) == OB
                    continue;
                end

                cell_changed = false;
                cell_max_delta = 0;
                for it = 1:NT
                    if goal_mask(iy, ix, it)
                        value_table(iy, ix, it) = 0;
                        continue;
                    end

                    old_val = value_table(iy, ix, it);
                    new_val = vi_frontier_bellman(value_table, penalty_table, ...
                        trans_model, ix, iy, it, map_x, map_y);
                    value_table(iy, ix, it) = new_val;

                    d = abs(new_val - old_val);
                    if d > 0
                        cell_changed = true;
                        changed_this = changed_this + 1;
                        if d > cell_max_delta
                            cell_max_delta = d;
                        end
                        if d > max_delta
                            max_delta = d;
                        end
                    end
                end

                if cell_changed && cell_max_delta > descend_tau
                    changed_now(iy, ix) = true;
                end
            end
        end

        sweeps = sweep;
        changed_states = changed_states + changed_this;
        changed_mask = changed_mask | changed_now;
        final_delta = max_delta;
        if max_delta <= threshold
            break;
        end
    end
end
