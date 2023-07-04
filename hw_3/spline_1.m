function [time, q_total, q_dot_total, q_ddot_total] = spline_1(Ts, points, qi_dot, qf_dot)
% 

time = points(1, 2):Ts:points(end, 2);
q_total = [];
q_dot_total = [];
q_ddot_total = [];

for k=1:size(points, 1)-1
    points(k)

    q_k = points(k, 1); % current position
    t_k = points(k, 2); % current time

    q_k1 = points(k+1, 1); % current position
    t_k1 = points(k+1, 2); % current time

    if k > 1
        % velocity of point k
        q_k_dot = compute_q_dot(points(k, :), points(k-1, :), points(k+1, :));
    else
        % initial velocity
        q_k_dot = qi_dot;
    end

    if k+1 == size(points, 1)
        % final velocity
        q_kp1_dot = qf_dot;
    else
        % velocity of point k+1
        q_kp1_dot = compute_q_dot(points(k+1, :), points(k, :), points(k+2, :));
    end

    q_k1_dot = 0; % next velocity
    T_k = t_k1 - t_k;

    a0_k = q_k;
    a1_k = q_k_dot;
    a2_k = (3*(q_k1 - q_k) / T_k - 2*q_k_dot - q_kp1_dot) / T_k;
    a3_k = (2*(q_k - q_k1) / T_k + q_k_dot + q_kp1_dot) / (T_k^2);

    %% compute cubic trajectory
    syms t_sym
    q = a3_k*(t_sym-t_k).^3 + a2_k*(t_sym-t_k).^2 + a1_k*(t_sym-t_k) + a0_k;
    q_dot = diff(q, t_sym);
    q_ddot = diff(q_dot, t_sym);
    
    if k == size(points, 1) - 1
        time_vector_k = [t_k:Ts:t_k1];
    else
        time_vector_k = [t_k:Ts:t_k1-Ts];
    end
    q_k = subs(q, t_sym, time_vector_k);
    q_dot_k = subs(q_dot, t_sym, time_vector_k);
    q_ddot_k = subs(q_ddot, t_sym, time_vector_k);

    q_total = [q_total q_k];
    q_dot_total = [q_dot_total q_dot_k];
    q_ddot_total = [q_ddot_total q_ddot_k];
end

end