function [time, q_total, q_dot_total, q_ddot_total] = spline_2(Ts, points, qi_dot, qf_dot)
% 

time = points(1, 2):Ts:points(end, 2);
q_total = [];
q_dot_total = [];
q_ddot_total = [];

%% compute velocities with thomas algorithm
% A * q_dot = d
% A = matrice tridiagonale, q_dot vettore velocità incognite, d vettore
% noto che dipende dalle posizioni (note) qk e dal tempo tk (noto) e dalla
% velocità iniziale/finale

trid = struct();
T_0 = points(2, 2) - points(1, 2); % colonna 2 c'è il tempo
T_1 = points(3, 2) - points(2, 2);
trid.b{1} = 2*(T_0 + T_1);
trid.c{1} = T_0;

for k=1:size(points, 1)-2
    t_k = points(k, 2);
    t_k1 = points(k+1, 2);
    t_k2 = points(k+2, 2);
    T_k = t_k1 - t_k;
    T_k1 = t_k2 - t_k1;

    q_k2 = points(k+2, 1);
    q_k1 = points(k+1, 1);
    q_k = points(k, 1);

    trid.d{k} = 3*T_k1*(q_k1-q_k)/T_k + 3*T_k*(q_k2-q_k1)/T_k1;
    if k == 1
        continue; % al primo giro fai solo d
    end
    trid.a{k} = T_k1;
    trid.b{k} = 2*(T_k + T_k1);
    trid.c{k} = T_k;
end

% forward
for k=2:size(points, 1)-2
    m = trid.a{k} / trid.b{k - 1};
    trid.b{k} = trid.b{k} - m * trid.c{k - 1}; 
    trid.d{k} = trid.d{k} - m * trid.d{k - 1}; 
end

% backward
k = size(points, 1)-2;
trid.x{k} = trid.d{k} / trid.b{k};
for k=size(points, 1)-3:-1:1
    trid.x{k} = (trid.d{k} - trid.c{k} * trid.x{k+1}) / trid.b{k};
end

% insert initial and final velocity
trid.x(2:size(points, 1)-1) = trid.x; % shift a destra per inserire velocità iniziale e finale
trid.x{1} = qi_dot;
trid.x{size(points, 1)} = qf_dot;

%% compute trajectory given thomas velocities (trid.x)
for k=1:size(points, 1)-1
    q_k = points(k, 1); % current position
    t_k = points(k, 2); % current time

    q_k1 = points(k+1, 1); % next position
    t_k1 = points(k+1, 2); % next time

    % velocity of point k
    q_k_dot = trid.x{k};
    % velocity of point k+1
    q_kp1_dot = trid.x{k+1};
    
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