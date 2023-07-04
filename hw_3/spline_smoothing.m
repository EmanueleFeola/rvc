function [new_points] = spline_smoothing(points, mu, w_inv)

w_inv = diag(w_inv);

lambda = (1 - mu) / (6 * mu);
n_points = size(points, 1);

A = zeros(n_points);
C = zeros(n_points);

T = points(2:end, 2) - points(1:end-1, 2);
q = points(:, 1);

for k=2:n_points-1
    A(k, k) = 2*(T(k-1) + T(k));
    A(k, k+1) = T(k);
    A(k, k-1) = T(k-1);

    C(k, k) = -(6/T(k-1) + 6/T(k));
    C(k, k+1) = 6/T(k);
    C(k, k-1) = 6/T(k-1);
end
A(1,1) = 2*T(1);
A(1,2) = T(1);
A(n_points,n_points) = 2*T(n_points-1);
A(n_points,n_points-1) = T(n_points-1);

C(1,1) = -6/T(1);
C(1,2) = 6/T(1);
C(n_points,n_points) = -6/T(n_points-1);
C(n_points,n_points-1) = 6/T(n_points-1);

s = q - lambda * w_inv * C' * inv(A + lambda * C * w_inv * C') * C * q;

new_points = [s points(:, 2)];

end