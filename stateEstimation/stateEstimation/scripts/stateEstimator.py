import numpy as np

class StateEstimator:
    def __init__(self):
        # C_r:      rolling resistance coefficient (0.02)
        # C_yf:     front-axle cornering stiffness (N/rad)
        # C_yr:     rear-axle cornering stiffness (N/rad)
        # g:        gravity acceleration (9.81 m/s2)
        # h_c:      height of the center of gravity (CG)(0.54 m)
        # I_z:      vehicle yaw moment of inertia(1523 kg m2)
        # l:        vehicle wheel base (2.578 m)
        # l_f:      distance from the CG to the front-axle(1.016 m)
        # l_r:      distance from the CG to the rear-axle(1.562 m)
        # m:        total vehicle mass (1416 kg)
        self.C_r =   0.02
        self.C_yf =  150
        self.C_yr =  150
        self.g =     9.81
        self.h_c =   7.5
        self.I_z =   0.0687
        self.l =     0.32
        self.l_f =   self.l*0.4
        self.l_r =   self.l*0.6
        self.m =     3.3325

    def vehicle_model(self, v_y, v_x, ohm_z, a_x, delta):
        ''''
        Returns output of vehicle dynamics given the current states and inputs

        input: v_y: float, lateral (y-axis) velocity of the vehicle
        input: v_x: float, longitudinal (x-axis) velocity of the vehicle
        input: ohm_z: float, Yaw rate of the vehicle
        input: a_x: float, longitudinal (x-axis) acceleration
        input: a_y: float, lateral (y-axis) acceleration

        output: alpha_f: float, front tire acceleration
        output: alpha_r: float, rear tire acceleration
        output: F_zf: float, front tire normal force
        output: F_zr: float, rear tire normal force
        '''
        #Side-slip angles of the front -'alpha_f' and rear-tires - 'alpha_r'
        alpha_f = ((v_y + (self.l_f * ohm_z))/v_x) - delta
        alpha_r = (v_y - (self.l_r * ohm_z))/v_x

        #The instantaneous normal forces acting on the front 'F_zf'and rear-axle 'f_zr' tires
        F_zf = (self.m*self.g*self.l_r) - (self.m*a_x*self.h_c)/self.l
        F_zr = (self.m*self.g*self.l_f) - (self.m*a_x*self.h_c)/self.l

        return alpha_f,alpha_r,F_zf,F_zr

    def tire_model(self, alpha_f, alpha_r, F_zf, F_zr, mu_f, mu_r):
        '''
        Brushed tire model, returns the front and rear lateral (y-axis) forces
        In the absence of longitudinal slip, this model describes cornering force as a nonlinear function of normal load, road friction, and tire side-slip angle
        
        input: alpha_f: float, front tire acceleration
        input: alpha_r: float, rear tire acceleration
        input: F_zf: float, front tire normal force
        input: F_zr: float, rear tire normal force
        input: mu_f: float, estimate of the front tire friction coefficient
        input: mu_r: float, estimate of the rear tire friction coefficient

        output: F_yf: float, front tire lateral (y-axis) force
        output: F_yr: float, rear tire lateral (y-axis) force 
        '''

        ################ FRONT #########################
        q_f  = np.sqrt((self.C_yf**2) * (np.tan(alpha_f)**2))

        if  q_f <= 3 * mu_f * F_zf:
            F_f = q_f  - (q_f**2/(3 * mu_f * F_zf)) +  (q_f**3/(27 * mu_f * F_zf**2)) 
        else:
            F_f = mu_f * F_zf
        
        F_yf = - ((self.C_yf**2) * np.tan(alpha_f)) * F_f/q_f
        
        ################  REAR  #########################
        q_r  = np.sqrt((self.C_yr**2) * (np.tan(alpha_r)**2))

        if q_r <= 3 * mu_r * F_zr:
            F_r = q_r - (q_r**2/(3 * mu_r * F_zr)) +  (q_r**3/(27 * mu_r * F_zr**2)) 
        else:
            F_r = mu_r * F_zr

        F_yr = - ((self.C_yr**2) * np.tan(alpha_r)) * F_r/q_r

        return F_yf, F_yr

    def estimate_velocities(self, deltaT, mu_x_k_prev, sigma_x_k_prev, a_x, a_y, ohm_z, v_x_m,  R, Q):
        '''
        Estimates the current speeds given the previous state estimates and control inputs

        input: deltaT: float, timestep
        input: mu_x_k: 2x1 float array, [v_x, v_y].T longitudinal (x-axis) and lateral (y-axis) velocities estimate from current timestep
        input: a_x: float, input longitudinal (x-axis) acceleration from IMU
        input: a_y: float, input lateral (y-axis) acceleration from IMU
        input: ohm_z: float, input yaw (z-axis) rate from IMU
        input: v_x_m: float, observed rear wheel mean speed from control input
        input: R: 2x2 float array, state noise covarian+ce
        input: Q: 2x2 float array, measurement noise covariance
        
        output: v_x: float, longitudinal (x-axis) velocity estimate from next timestep
        output: v_y: float, lateral (y-axis) velocity estimate from next timestep
        '''
        # Extract the previous state estimates
        v_hat_x = mu_x_k_prev[0]
        v_hat_y = mu_x_k_prev[1]

        ## System Dynamics
        # Î¼_x(k+1|k)
        mu_x_k_     = np.array([[v_hat_x + deltaT * ohm_z * v_hat_y + deltaT * a_x],
                                [v_hat_y + deltaT * ohm_z * v_hat_x + deltaT * a_y]])
        
        # Jacobian of the nonlinear system dynamics
        A = np.array([[        1      , deltaT * ohm_z],
                      [ deltaT * ohm_z,        1      ]])

        # Î£_x(k+1|k)
        sigma_x_k_  = A @ sigma_x_k_prev @ A.T + R

        ## Observation Step
        # Observation Jacobian for re-use
        C_ = v_hat_x

        # Kalman Gain K
        K           = sigma_x_k_ @ C_.T @ np.linalg.inv(C_ @ sigma_x_k_ @ C_.T + Q)

        # Î¼_x(k+1|k)
        mu_x_k_k    = mu_x_k_ + K * (v_x_m - mu_x_k_[0])

        # Î£_x(k+1|k+1)
        sigma_x_k_k = (np.eye(2) - K @ C_) @ sigma_x_k_

        return mu_x_k_k, sigma_x_k_k

    def tireforce_estimates(self, v_hat_x, v_hat_y, ohm_hat_z, F_hat_xf, F_hat_yf, F_hat_yr, v_x_obs, ohm_z_obs, v_y_obs, a_x, a_y, delta, deltaT):

        #fU: Dynamics Step
        fU = np.array([[v_hat_x + (deltaT * ohm_hat_z * v_hat_y) + ((deltaT/self.m) * (F_hat_xf * np.cos(delta) - F_hat_yf * np.sin(delta)))],
                       [ohm_hat_z + deltaT * (self.l_f/self.I_z) * (F_hat_xf * np.sin(delta) + F_hat_yf * np.cos(delta)) - deltaT * (self.l_r/self.I_z) * F_hat_yr],
                       [v_hat_y - (deltaT * ohm_hat_z * v_hat_x) + ((deltaT/self.m) * (F_hat_xf * np.sin(delta) - F_hat_yf * np.cos(delta) + F_hat_yr))],
                       [F_hat_xf],
                       [F_hat_yf],
                       [F_hat_yr]
                      ])

        #hU = Observation step
        hU = np.array([v_hat_x, 
                       ohm_hat_z,
                       v_hat_y,
                       (1/self.m) *(F_hat_xf * np.cos(delta) - F_hat_yf * np.sin(delta)),
                       (1/self.m) *(F_hat_xf * np.sin(delta) + F_hat_yf * np.cos(delta) + F_hat_yr)])

        # TO DO: FIND INNOVATION AS hU - (v_x_obs, ohm_z_obs, v_y_obs, a_x, a_y)

        #discrete-time state-space system model
        xU = fU # xU = fU(xU, uU) + Noise (wU)
        zU = hU # hU(xU) + Noise(vU)

        return v_hat_x, v_hat_y, ohm_hat_z, F_hat_xf, F_hat_yf, F_hat_yr

    def TRFC_estimation(self, mu_hat_f, mu_hat_r, F_hat_yf, F_hat_yr, v_y, v_x, ohm_z, a_x, delta):
        '''
        
        '''
        #F_hat_yf, F_hat_yr = lateral tire forces of the two axles 
        #alpha_f, alpha_r           = side-slip angles of front- and rear-axle
        #F_zf, F_zr         = normal forces of front- and rear-axle
        #mu_f, mu_r         = Unknown parameters

        # Dynamics Step
        T  = np.array([mu_hat_f,mu_hat_r]).T

        # Observation Step
        alpha_f, alpha_r, F_zf, F_zr = self.vehicle_model(v_y, v_x, ohm_z, a_x, delta)
        hB    = self.tire_model(alpha_f, alpha_r, F_zf, F_zr, mu_hat_f, mu_hat_r)

        T  = T
        zB = hB          #hB(T)
        return

####################################################################
  
def find_sigma_points(x_k_prev, P_k_prev, Q):
    n   = Q.shape[0]
    S_p = +np.sqrt(n)*np.linalg.cholesky(P_k_prev + Q)
    S_n = -np.sqrt(n)*np.linalg.cholesky(P_k_prev + Q)

    S   = np.hstack((S_p.copy(), S_n.copy()))

    Xi = x_k_prev + S
    Xi = np.hstack((x_k_prev,Xi))
    return Xi

def friction_observation_step(self, Yi, v_y, v_x, ohm_z, a_x, delta):
    # Check equation 17###
    Zi = np.zeros_like(Yi)
    alpha_f, alpha_r, F_zf, F_zr = self.vehicle_model(v_y, v_x, ohm_z, a_x, delta)
    
    for i, point in enumerate(Yi): 
        mu_hat_f,mu_hat_r = point[0],point[1]
        
        hB    = self.tire_model(alpha_f, alpha_r, F_zf, F_zr, mu_hat_f, mu_hat_r)
        Zi[i] = hB

    return Zi

# Equation 64
def Pk_m(W_i_prime):
    return (W_i_prime @ W_i_prime.T) / W_i_prime.shape[1]

# Equation 70
def P_zz(Z_center):
    return (Z_center @ Z_center.T) / Z_center.shape[1]

def P_xz(Wiprime, Z_center):
    return (Wiprime @ Z_center.T) / Z_center.shape[1]

def updateFrictionPrediction(x_k_prev, P_k_prev, z_k_prev, Q, R):
    ## Update Step
    # Find and transform sigma points
    Xi  = find_sigma_points(x_k_prev, P_k_prev, Q)
    Yi  = Xi.copy()

    # Find the estimates xhatminus and the innovation term
    mu_x_k  = np.mean(Yi)
    Wiprime = Yi - mu_x_k
 
    # Pk-
    sigma_x_k = Pk_m(Wiprime)

    ## Observation Step
    # Incorporate observation models H1 & H2
    Zi  = friction_observation_step(Yi)

    # Zk-
    zk_bar = np.mean(Zi, axis=1)

    # Centered Zi's
    Z_center = Zi - zk_bar.reshape(-1,1)
    
    # Equation 69
    Pvv = P_zz(Z_center) + R
    
    # Find the innovation (true - estimate)     # Equation 44
    v_k = (z_k_prev - zk_bar)

    # Equation 70
    Pxz = P_xz(Wiprime, Z_center)

    # Find the kalman gain # Equation 72
    Kk  = Pxz @ np.linalg.inv(Pvv)

    # Find the estimate of the state covariance # Equation 75
    sigma_x_k_p  = sigma_x_k - Kk @ Pvv @ Kk.T

    # X update
    update     = Kk @ v_k
    
    # Add angle-axis x to angle-axis estimate
    x_hat_k = mu_x_k + update

    return x_hat_k, sigma_x_k_p