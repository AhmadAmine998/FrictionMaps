import numpy as np

class StateEstimator:
    def __init__(self, c_r=0.02, c_yf=150, c_yr=150, G=9.81, H_c=0.075, i_z=0.0687, L=0.32, L_f=0.35, L_r=0.65, M=3.3325, TireCirc=0.319, GearRatio=11.85):
        '''
        Initializes the TRFC State Estimator. L_f and L_r are the weight distribution of the car in ratio. (i.e a 60:40 car has l_f=0.6, l_r=0.4)
        '''
        # C_r:          rolling resistance coefficient (0.02)
        # C_yf:         front-axle cornering stiffness (N/rad)
        # C_yr:         rear-axle cornering stiffness (N/rad)
        # g:            gravity acceleration (9.81 m/s2)
        # h_c:          height of the center of gravity (CG)(0.54 m)
        # I_z:          vehicle yaw moment of inertia(1523 kg m2)
        # l:            vehicle wheel base (2.578 m)
        # l_f:          distance from the CG to the front-axle(1.016 m)
        # l_r:          distance from the CG to the rear-axle(1.562 m)
        # m:            total vehicle mass (1416 kg)
        # tire_circ:    circumference of vehicle's tires
        # total_ratio:  total gear ratio of vehicle from motor to wheels
        self.C_r  = c_r               # rolling resistence
        self.C_yf = c_yf              # N/rad
        self.C_yr = c_yr              # N/rad
        self.g    = G                 # m/s2
        self.h_c  = H_c               # m
        self.I_z  = i_z               # kgm2
        self.l    = L                 # m
        self.l_f  = self.l*L_f        # m
        self.l_r  = self.l*L_r        # m
        self.m    = M                 # kg
        self.tire_circ    = TireCirc  # m
        self.total_ratio  = GearRatio # Gear Ratio

        # EKF Parameters
        self.Qe   = 10**(-3)
        self.Re   = np.diag([10, 10])

        # Force UKF Parameters
        self.Qu   = np.diag([  10,          107,        103,          400,          850, 675])*100
        self.Ru   = np.diag([20, 94, 20, 86, 65])

        # Friction UKF Paramters
        self.Qb   = np.diag([10**(-4), 10**(-4)])
        self.Rb   = np.diag([10**(4), 10**(4)])
        
    @staticmethod
    def Pk_m(W_i_prime):
        return (W_i_prime @ W_i_prime.T) / W_i_prime.shape[1]

    @staticmethod
    def P_zz(Z_center):
        return (Z_center @ Z_center.T) / Z_center.shape[1]

    @staticmethod
    def P_xz(Wiprime, Z_center):
        return (Wiprime @ Z_center.T) / Z_center.shape[1]
        
    @staticmethod
    def find_sigma_points(x_k_prev, P_k_prev, Q):     
        n   = Q.shape[0]
        S_p = +np.sqrt(n)*np.linalg.cholesky(P_k_prev + Q)
        S_n = -np.sqrt(n)*np.linalg.cholesky(P_k_prev + Q)

        S   = np.hstack((S_p.copy(), S_n.copy()))

        Xi = x_k_prev + S
        Xi = np.hstack((x_k_prev,Xi))
        return Xi

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
        alpha_f = (v_y + (self.l_f * ohm_z))/v_x - delta
        alpha_r = (v_y - (self.l_r * ohm_z))/v_x

        #The instantaneous normal forces acting on the front 'F_zf'and rear-axle 'f_zr' tires
        F_zf = ((self.m*self.g*self.l_r) - (self.m*a_x*self.h_c))/self.l
        F_zr = ((self.m*self.g*self.l_f) + (self.m*a_x*self.h_c))/self.l

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
            F_f = q_f  - ((q_f**2)/(3 * mu_f * F_zf)) +  ((q_f**3)/(27 * mu_f**2 * F_zf**2)) 
        else:
            F_f = mu_f * F_zf
        
        F_yf = - ((self.C_yf) * np.tan(alpha_f)) * F_f/q_f
        
        ################  REAR  #########################
        q_r  = np.sqrt((self.C_yr**2) * (np.tan(alpha_r)**2))

        if q_r <= 3 * mu_r * F_zr:
            F_r = q_r - (q_r**2/(3 * mu_r * F_zr)) +  (q_r**3/(27 * mu_r**2 * F_zr**2)) 
        else:
            F_r = mu_r * F_zr

        F_yr = - ((self.C_yr) * np.tan(alpha_r)) * F_r/q_r

        return F_yf, F_yr

    def estimate_velocities(self, deltaT, v_hat_x, v_hat_y, a_x, a_y, ohm_z, v_x_m, sigma_x_k_prev):
        '''
        Estimates the current speeds given the previous state estimates and control inputs

        input: deltaT: float, timestep
        input: v_x: float, longitudinal (x-axis) velocity estimate from current timestep
        input: v_y: float, lateral (y-axis) velocity estimate from current timestep
        input: a_x: float, input longitudinal (x-axis) acceleration from IMU
        input: a_y: float, input lateral (y-axis) acceleration from IMU
        input: ohm_z: float, input yaw (z-axis) rate from IMU
        input: v_x_m: float, observed rear wheel mean speed from control input
        input: sigma_x_k_prev: 2x2 float array: Previous state covariance estimate
        
        output: v_x: float, longitudinal (x-axis) velocity estimate
        output: v_y: float, lateral (y-axis) velocity estimate
        output: sigma_x_k_k: 2x2 float array: State covariance estimate
        '''
        ## System Dynamics
        # Î¼_x(k+1|k)
        mu_x_k_     = np.array([[v_hat_x + deltaT * ohm_z * v_hat_y + deltaT * a_x],
                                [v_hat_y - deltaT * ohm_z * v_hat_x + deltaT * a_y]])
        
        # Jacobian of the nonlinear system dynamics
        A = np.array([[        1      , deltaT * ohm_z],
                      [-deltaT * ohm_z,        1      ]])

        # Î£_x(k+1|k)
        sigma_x_k_  = A @ sigma_x_k_prev @ A.T + self.Re

        ## Observation Step
        # Observation Jacobian for re-use
        C_ = np.array([[1, 0]])

        # Kalman Gain K
        K           = sigma_x_k_ @ C_.T @ np.linalg.inv(C_ @ sigma_x_k_ @ C_.T + self.Qe)

        # Î¼_x(k+1|k)
        mu_x_k_k    = mu_x_k_ + K * (v_x_m - mu_x_k_[0])

        # Î£_x(k+1|k+1)
        sigma_x_k_k = (np.eye(2) - K @ C_) @ sigma_x_k_

        return mu_x_k_k[0], mu_x_k_k[1], sigma_x_k_k
    
    def tyreforce_observation(self, Yi, delta):
        
        Zi = np.zeros((Yi.shape[0]-1, Yi.shape[1]))
        for i, point in enumerate(Yi.T):
            v_hat_x, v_hat_y, ohm_hat_z, F_hat_xf, F_hat_yf, F_hat_yr = point.tolist()

            #hU = Observation step
            Zi[:, i] = np.array([v_hat_x, 
                                 ohm_hat_z,
                                 v_hat_y,
                                 (1/self.m) *(F_hat_xf * np.cos(delta) - F_hat_yf * np.sin(delta)),
                                 (1/self.m) *(F_hat_xf * np.sin(delta) + F_hat_yf * np.cos(delta) + F_hat_yr)])
        
        return Zi

    def tyreforce_estimates(self, v_hat_x, v_hat_y, ohm_hat_z, F_hat_xf, F_hat_yf, F_hat_yr, P_k_prev, v_x_obs, ohm_z_obs, v_y_obs, a_x, a_y, delta, deltaT):

        x_k_prev = np.array([[v_hat_x], [v_hat_y], [ohm_hat_z], [F_hat_xf], [F_hat_yf], [F_hat_yr]])

        Xi  = self.find_sigma_points(x_k_prev, P_k_prev, self.Qu)
        Yi = np.zeros_like(Xi)

        for i, point in enumerate(Xi.T):
            #fU: Dynamics Step
            v_hat_x, v_hat_y, ohm_hat_z, F_hat_xf, F_hat_yf, F_hat_yr = point.tolist()

            fU = np.array([[v_hat_x + (deltaT * ohm_hat_z * v_hat_y) + ((deltaT/self.m) * (F_hat_xf * np.cos(delta) - F_hat_yf * np.sin(delta)))],
                           [ohm_hat_z + deltaT * (self.l_f/self.I_z) * (F_hat_xf * np.sin(delta) + F_hat_yf * np.cos(delta)) - deltaT * (self.l_r/self.I_z) * F_hat_yr],
                           [v_hat_y - (deltaT * ohm_hat_z * v_hat_x) + ((deltaT/self.m) * (F_hat_xf * np.sin(delta) + F_hat_yf * np.cos(delta) + F_hat_yr))],
                           [F_hat_xf],
                           [F_hat_yf],
                           [F_hat_yr]
                           ])
            Yi[:, i] = fU.flatten()



        # Find the estimates xhatminus and the innovation term
        mu_x_k  = np.mean(Yi, axis=1).reshape(-1,1)
        Wiprime = Yi - mu_x_k
    
        # Pk-
        sigma_x_k = self.Pk_m(Wiprime)

        ## Observation Step
        # Incorporate observation models H1 & H2
        Zi  = self.tyreforce_observation(Yi, delta)

        # Zk-
        zk_bar = np.mean(Zi, axis=1).reshape(-1,1)

        # Centered Zi's
        Z_center = Zi - zk_bar
        
        # Equation 69
        Pvv = self.P_zz(Z_center) + self.Ru
        
        # Find the innovation (true - estimate)     # Equation 44
        z_k_prev = np.array([[v_x_obs], [ohm_z_obs], [v_y_obs], [a_x], [a_y]])
        v_k = (z_k_prev - zk_bar)

        # Equation 70
        Pxz = self.P_xz(Wiprime, Z_center)

        # Find the kalman gain # Equation 72
        Kk  = Pxz @ np.linalg.inv(Pvv)

        # Find the estimate of the state covariance # Equation 75
        sigma_x_k_p = sigma_x_k - Kk @ Pvv @ Kk.T

        # X update
        update = Kk @ v_k
        
        # Add angle-axis x to angle-axis estimate
        x_hat_k = mu_x_k + update

        return x_hat_k, sigma_x_k_p


    def friction_observation_step(self, Yi, v_y, v_x, ohm_z, a_x, delta):
        # Check equation 17###
        Zi = np.zeros_like(Yi)
        alpha_f, alpha_r, F_zf, F_zr = self.vehicle_model(v_y, v_x, ohm_z, a_x, delta)
        
        for i, point in enumerate(Yi.T): 
            mu_hat_f,mu_hat_r = point[0],point[1]
            
            hB    = self.tire_model(alpha_f, alpha_r, F_zf, F_zr, mu_hat_f, mu_hat_r)
            Zi[:, i] = hB

        return Zi

    def TRFC_estimation(self, x_k_prev, z_k_prev, P_k_prev, v_y, v_x, ohm_z, a_x, delta):
        '''
        Updates the estimate of the tire-road friction coefficients for both tires

        input: x_k_prev: 2x1 float array, mu_f, mu_r previous estimates of the TRFC 
        input: P_k_prev: 2x2 float array, previous estimates of the covariances of the TRFC 
        input: z_k_prev: 2x1 float array, F_hat_yf, F_hat_yr lateral tire forces of the two tires
        input: v_y: float, lateral (y-axis) velocity of the vehicle
        input: v_x: float, longitudinal (x-axis) velocity of the vehicle
        input: ohm_z: float, Yaw rate of the vehicle
        input: a_x: float, longitudinal (x-axis) acceleration
        input: a_y: float, lateral (y-axis) acceleration
        input: delta: float, input steering angle
        '''
        ## Update Step
        # Find and transform sigma points
        Xi  = self.find_sigma_points(x_k_prev, P_k_prev, self.Qb)
        Yi  = Xi.copy()

        # Find the estimates xhatminus and the innovation term
        mu_x_k  = np.mean(Yi, axis=1).reshape(-1,1)
        Wiprime = Yi - mu_x_k
    
        # Pk-
        sigma_x_k = self.Pk_m(Wiprime)

        ## Observation Step
        # Incorporate observation models H1 & H2
        Zi  = self.friction_observation_step(Yi, v_y, v_x, ohm_z, a_x, delta)

        # Zk-
        zk_bar = np.mean(Zi, axis=1).reshape(-1,1)

        # Centered Zi's
        Z_center = Zi - zk_bar
        
        # Equation 69
        Pvv = self.P_zz(Z_center) + self.Rb
        
        # Find the innovation (true - estimate)     # Equation 44
        v_k = (z_k_prev - zk_bar)

        # Equation 70
        Pxz = self.P_xz(Wiprime, Z_center)

        # Find the kalman gain # Equation 72
        Kk  = Pxz @ np.linalg.inv(Pvv)

        # Find the estimate of the state covariance # Equation 75
        sigma_x_k_p  = sigma_x_k - Kk @ Pvv @ Kk.T

        # X update
        update     = Kk @ v_k
        
        # Add angle-axis x to angle-axis estimate
        x_hat_k = mu_x_k + update

        return x_hat_k, sigma_x_k_p