function [Pt,Ft] = spherical_traj(p0,Pi,Pf,du)

ro = norm(Pi - p0);
ri = (Pi - p0) / ro;
rf = (Pf - p0) / ro;

% direction of the circle connecting Pi and Pf on the sphere 
r = cross(ri, rf);
r = r / norm(r);
Rt = [ri cross(r, ri) r];

% compute rotations
Ri = [cross(r, -ri) r -ri];  % from Eb to Ei
Rf = [cross(r, -rf) r -rf];  % from Eb to Ef
Rif = Ri'*Rf;
uf = acos((Rif(1,1) + Rif(2,2) + Rif(3,3) -1)/2);  % angle from Ei to Ef w.r.t. Ei
% r = [Rif(3,2) - Rif(2,3);
%      Rif(1,3) - Rif(3,1);
%      Rif(2,1) - Rif(1,2)] / (2*sin(uf));  % axis between Ei and Ef w.r.t. Ei
%                                           % on this sphere, this is always [0 1 0]'

% calculate points from Pi to Pf
u = 0:du:uf;
Pt = Rt * [     ro*cos(u);
                ro*sin(u);
           zeros(size(u))] + p0;
Pt = Pt';
% % calculate moving Frenet frame from Ei to Ef
% Ru = times3(r*r',1-cos(u)) + times3(skew(r),sin(u)) + times3(eye(3),cos(u));
% Ft = zeros(size(Ru));
% for i=1:size(Ru,3)
%     Ft(:,:,i) = Ri*Ru(:,:,i);
% end

end


% function [S] = skew(s)
% S = [    0 -s(3)  s(2); 
%       s(3)     0 -s(1); 
%      -s(2)  s(1)     0];
% end
% 
% function [T] = times3(M,v)
% % matrix-by-vector multiplication over the third axis
% T = bsxfun(@times,M,reshape(v,1,1,[]));
% end