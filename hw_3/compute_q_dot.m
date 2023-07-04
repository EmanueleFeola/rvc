function [q_k_dot] = compute_q_dot(point_k, prev_point, next_point)
    % computes velocity constraint/value given current point and next point
    
    q_k = point_k(1, 1);
    t_k = point_k(1, 2);

    q_km1 = prev_point(1, 1); % prev position
    t_km1 = prev_point(1, 2); % prev position

    q_kp1 = next_point(1, 1); % next position
    t_kp1 = next_point(1, 2); % next position
    
    v_k = (q_k - q_km1) / (t_k - t_km1);
    v_k1 = (q_kp1 - q_k) / (t_kp1 - t_k);
    
    q_k_dot = (v_k + v_k1) / 2; % current velocity
end

