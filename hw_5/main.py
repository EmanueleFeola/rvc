import numpy as np

def linear_primitive(pi, pf, Ts):
    delta_pos = pf - pi
    delta_pos_norm = np.linalg.norm(delta_pos)

    s = np.arange(0, delta_pos_norm, Ts)
    print(np.shape(s))
    
    p_x = pi[0] + s * delta_pos[0] / delta_pos_norm
    p_y = pi[1] + s * delta_pos[1] / delta_pos_norm
    p_z = pi[2] + s * delta_pos[2] / delta_pos_norm
    p = [p_x, p_y, p_z]
    
    time = s
    return p, time

Ts = 0.1
points = np.array([[0, 0, 0], [1, 0, 0.1]])
pi = points[0]
pf = points[1]
p, time = linear_primitive(pi, pf, Ts)