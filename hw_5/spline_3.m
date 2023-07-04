function [time, q_total, q_dot_total, q_ddot_total, q_dddot_total] = spline_3(Ts, points, qi_dot, qf_dot)
time = points(1, 2):Ts:points(end, 2);
q_total = [];
q_dot_total = [];
q_ddot_total = [];
q_dddot_total = [];

%% compute accelerations with thomas algorithm
% A * q_ddot = d
% A = matrice tridiagonale, q_ddot vettore accelerazioni incognite, d vettore
% noto che dipende dalle posizioni (note) qk e dal tempo tk (noto) e dalla
% velocità iniziale/finale

T = points(2:end, 2) - points(1:end-1, 2);
q = points(:, 1);

trid = struct();
T_0 = T(1);
trid.b{1} = 2 * T_0;
trid.c{1} = T_0;

for k=2:size(points, 1)-1
    T_k = T(k); %points(k+1, 2) - points(k, 2);
    T_km1 = T(k-1); %points(k, 2) - points(k-1, 2);

    q_km1 = q(k-1); %points(k-1, 1);
    q_k = q(k); %points(k, 1);
    q_kp1 = q(k+1); %points(k+1, 1);

    trid.a{k} = T_km1;
    trid.b{k} = 2 * (T_km1 + T_k);
    trid.c{k} = T_k;
    trid.d{k} = 6*((q_kp1 - q_k) / T_k - (q_k - q_km1) / T_km1);
end

% first row of the system
q_0 = points(1, 1);
q_1 = points(2, 1);
T_0 = points(2, 2) - points(1, 2);
trid.d{1} = 6*((q_1 - q_0) / T_0 - qi_dot); 
trid.b{1} = 2*T_0;
trid.c{1} = T_0;

% last row of the system
q_n = points(size(points, 1), 1);
q_nm1 = points(size(points, 1)-1, 1);
T_nm1 = points(size(points, 1), 2) - points(size(points, 1)-1, 2);
trid.d{size(points, 1)} = 6*(qf_dot - (q_n - q_nm1)/T_nm1); 
trid.a{size(points, 1)} = T_nm1;
trid.b{size(points, 1)} = 2*T_nm1;

%% thomas
for k=2:size(points, 1)
    m = trid.a{k} / trid.b{k - 1};
    trid.b{k} = trid.b{k} - m * trid.c{k - 1}; 
    trid.d{k} = trid.d{k} - m * trid.d{k - 1}; 
end

k = size(points, 1);
trid.x{k} = trid.d{k} / trid.b{k};
for k=size(points, 1)-1:-1:1
    trid.x{k} = (trid.d{k} - trid.c{k} * trid.x{k+1}) / trid.b{k};
end

%% compute trajectory given thomas acc (trid.x)
for k=1:size(points, 1)-1
    q_k = points(k, 1); % current position
    t_k = points(k, 2); % current time

    q_k1 = points(k+1, 1); % next position
    t_k1 = points(k+1, 2); % next time

    q_k_ddot = trid.x{k};
    q_kp1_ddot = trid.x{k+1};
    
    T_k = t_k1 - t_k;

    a0_k = q_k;
    a1_k = (q_k1 - q_k) / T_k - (T_k/6) * (q_kp1_ddot + 2*q_k_ddot);
    a2_k = q_k_ddot / 2;
    a3_k = (q_kp1_ddot - q_k_ddot) / (6 * T_k);

    %% compute cubic trajectory
    syms t_sym
    q = a3_k*(t_sym-t_k).^3 + a2_k*(t_sym-t_k).^2 + a1_k*(t_sym-t_k) + a0_k;
    q_dot = diff(q, t_sym);
    q_ddot = diff(q_dot, t_sym);
    q_dddot = diff(q_ddot, t_sym);
    
    if k == size(points, 1) - 1
        time_vector_k = [t_k:Ts:t_k1];
    else
        time_vector_k = [t_k:Ts:t_k1-Ts];
    end
    q_k = subs(q, t_sym, time_vector_k);
    q_dot_k = subs(q_dot, t_sym, time_vector_k);
    q_ddot_k = subs(q_ddot, t_sym, time_vector_k);
    q_dddot_k = subs(q_dddot, t_sym, time_vector_k);

    q_total = [q_total q_k];
    q_dot_total = [q_dot_total q_dot_k];
    q_ddot_total = [q_ddot_total q_ddot_k];
    q_dddot_total = [q_dddot_total q_dddot_k];
end

end