from re import L, T
import numpy as np

# a_x:      vehicle longitudinal acceleration
# a_y:      vehicle lateral acceleration
# A:        vehicle frontal cross-section area (1.6 m2)
# C_d:      aerodynamic drag coefficient (0.35)
# C_r:      rolling resistance coefficient (0.01)
# C_yf:     front-axle cornering stiffness (94 kN/rad)
# C_yr:     rear-axle cornering stiffness (76 kN/rad)
# F_R:      total resistance force
# F_xf:     front-axle longitudinal force
# F_hat_xf: estimated front-axle longitudinal force
# F_xr:     rear-axle longitudinal force
# F_yf:     front-axle lateral force
# F_hat_yf: estimated front-axle lateral force
# F_yr:     rear-axle lateral force
# F_hat_yr: estimated rear-axle lateral force
# F_zf:     front-axle normal force
# F_zr:     rear-axle normal force
# g:        gravity acceleration (9.81 m/s2)
# h_c:      height of the center of gravity (CG)(0.54 m)
# I_z:      vehicle yaw moment of inertia(1523 kg m2)
# l:        vehicle wheel base (2.578 m)
# l_f:      distance from the CG to the front-axle(1.016 m)
# l_r:      distance from the CG to the rear-axle(1.562 m)
# m:        total vehicle mass (1416 kg)
# T_s:      sampling interval
# v_x:      vehicle longitudinal velocity
# v_hat_kx: filtered wheel speed
# v_m_xk:   mean velocity of the rear wheels
# v_y:      vehicle lateral velocity
# v_hat_y:  estimated vehicle lateral velocity
# a_f:      front tire side-slip/slip angle
# a_r:      rear tire side-slip/slip angle
# delta:    tire steer angle
# mu_f:     tire–road friction coefficient at front tires
# mu_r:     tire–road friction coefficient at rear tires
# rho:      mass density of air (1.225 kg/m3)
# w_rl:     angular speed of rear-left wheel
# w_rr:     angular speed of rear-right wheel
# ohm_z:    vehicle yaw rate

a_x:      0
a_y:      0
A:        0
C_d:      0
C_r:      0
C_yf:     0
C_yr:     0
F_R:      0
F_xf:     0
F_hat_xf: 0
F_xr:     0
F_yf:     0
F_hat_yf: 0
F_yr:     0
F_hat_yr: 0
F_zf:     0
F_zr:     0
g:        0
h_c:      0
I_z:      0
l:        0
l_f:      0
l_r:      0
m:        0
T_s:      0
v_x:      0
v_hat_kx: 0
v_m_xk:   0
v_y:      0
v_hat_y:  0
a_f:      0
a_r:      0
delta:    0
mu_f:     0
mu_r:     0
rho:      0
w_rl:     0
w_rr:     0
ohm_z:    0

def vehicle_model():
    #Side-slip angles of the front -'a_f' and rear-tires - 'a_r'
    a_f = ((v_y + (l_f * ohm_z))/v_x) - delta
    a_r = (v_y - (l_r * ohm_z))/v_x

    #The instantaneous normal forces acting on the front 'F_zf'and rear-axle 'f_zr' tires
    F_zf = (m*g*l_r) - (m*a_x*h_c)/l
    F_zr = (m*g*l_f) - (m*a_x*h_c)/l

    return a_f,a_r,F_zf,F_zr

def tire_model():
    #In the absence of longitudinal slip, the model describes cornering force as a nonlinear function of normal load, road friction, and tire side-slip angle, as
    #C_yi = Axle Corner Stifness
    #mu_i = Friction Coefficient

    ################ FORWARD #########################
    q_f  = np.sqrt((C_yf**2) * (np.tan(a_f)**2))

    if  q_f <= 3 * mu_f * F_zf:
        F_f = q_f  - (q_f**2/(3 * mu_f * F_zf)) +  (q_f**3/(27 * mu_f * F_zf**2)) 
    else:
        F_f = mu_f * F_zf
    
    F_yf = - ((C_yf**2) * np.tan(a_f)) * F_f/q_f
    
    ################ REVERESE #########################

    q_r  = np.sqrt((C_yr**2) * (np.tan(a_r)**2))

    if q_r <= 3 * mu_r * F_zr:
        F_r = q_r - (q_r**2/(3 * mu_r * F_zr)) +  (q_r**3/(27 * mu_r * F_zr**2)) 
    else:
        F_r = mu_r * F_zr

    F_yr = - ((C_yr**2) * np.tan(a_r)) * F_r/q_r

    return 

def Velocity_estimates():
    #INITIAL STATES
    xE[k] = np.array([v_x[k], v_y[k]]).T            #System state
    zE[k] = np.array([v_x_m[k]])                    #Measurement Vector | v_x_m[k] refers to mean velocity of the rear wheels at the ground interface, can be obtained from angular speeds of the vehicles (Wrl,Wrr)
    uE[k] = np.array([a_x[k], a_y[k], ohm_z]).T     #Input Vector

    #Vehicle Velocities
    a_x   = v_dot_x - (v_y * ohm_z)
    a_y   = v_dot_y + (v_x * ohm_z)

    # fE: states’ evolution equations
    fE = np.array([[v_x[k-1] + T_s * ohm_z[k] * v_y[k-1] + T_s * a_x[k]],
                  [v_y[k-1] + T_s * ohm_z[k] * v_x[k-1] + T_s * a_y[k]]])

    #Observation Functions
    hE = v_x[k]

    #state-space system in the discrete-time
    xE[k] = fE
    zE[k] = hE

    return

def Tireforce_estimates():
    #v_hat_x[k] = Filtered Wheel Velocity
    #v_hat_y[k] = Estimated Lateral Velocity
    xU[k] = np.array([v_x[k], ohm_z[k], v_y[k], F_xf[k], F_yf[k], F_yr[k]]).T
    zU[k] = np.array([v_hat_x[k], ohm_z[k], v_hat_y[k], a_x[k], a_y[k]]).T

    #fU: evolution function vector
    fU = np.array([[v_x[k-1] + (T_s * ohm_z * v_y[k-1]) + ((T_s/m) * (F_xf[k-1] * np.cos(delta[k]) - F_yf[k-1] * np.sin(delta[k]) - (0.5 * rho * C_d * A * v_x[k-1]**2)))],
                   [ohm_z[k-1] + T_s * (l_f/I_z) * (F_xf[k-1] * np.sin(delta[k]) + F_yf[k-1] * np.cos(delta[k])) - T_s * (l_r/I_z) * F_yr[k-1]],
                   [v_y[k-1] - (T_s * ohm_z * v_x[k-1]) + ((T_s/m) * (F_xf[k-1] * np.sin(delta[k]) - F_yf[k-1] * np.cos(delta[k]) + F_yr[k-1]))],
                   [F_xf[k-1]],
                   [F_yf[k-1]],
                   [F_yr[k-1]]
                   ])
    #hU = measurement function vector
    hU = np.array([v_x[k], 
                  ohm_z[k],
                  v_y[k],
                  (1/m) *(F_xf[k] * np.cos(delta[k]) - F_yf[k] * np.sin(delta[k]) - 0.5 * rho * C_d * A * v_x[k]**2),
                  (1/m) *(F_xf[k] * np.sin(delta[k]) + F_yf[k] * np.cos(delta[k]) + F_yr[k])])

    #discrete-time state-space system model
    xU[k] = fU # xU[k] = fU(xU[k-1], uU[k]) + Noise (wU[k])
    zU[k] = hU # hU(xU[k]) + Noise(vU[k])

    return

def TRFC_estimation():
    #F_hat_yf, F_hat_yr = lateral tire forces of the two axles 
    #a_f, a_r           = side-slip angles of front- and rear-axle
    #F_zf, F_zr         = normal forces of front- and rear-axle
    #C_yf, C_yr         = cornering stiffness of front- and rear-axle
    #mu_f, mu_r         = Unknown parameters
    T[k]  = np.array([mu_f[k],mu_r[k]]).T
    zB[k] = np.array([F_hat_yf[k], F_hat_yr[k]]).T

    #Observation Function
    hB    = np.array([F_yf(a_f[k], F_zf[k], mu_hat_f[k-1]),
                      F_yr(a_r[k], F_zr[k], mu_hat_r[k-1])])


    T[k]  = T[k-1] + wB[k]
    zB[k] = hB          #hB(T[k])
    return