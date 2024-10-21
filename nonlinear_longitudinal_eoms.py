import numpy as np
import control

# Stability derivatives for Cessna 172 are taken from Roskam, J., Airplane Flight Dynamics and Automatic Flight Controls
def state_derivatives(t, x, u, params):
    xdot = np.zeros((4,))
    
    V = x[0]
    alpha = x[1]
    theta = x[2]
    q = x[3]

    # Given Constants
    W = 2650 #lbs
    S = 174 #ft^2
    cbar = 4.9 #ft
    J2 = 1346 #slug*ft^2
    rho = 2.377e-3 #slug/ft^3 (sea-level)
    g = 32.2 #ft/s^2
    m = W/g #lbm

    # Dynamic Pressure
    qbar = 1/2*rho*V**2

    # Lift Coefficient
    Cl0 = 0.307
    Cla = 4.41 #/rad
    Clel = 0.43 #/rad
    Cladot = 1.7 #/rad
    Clq = 3.9 #/rad

    # Drag Coefficient
    CdM = 0.0223
    k = 0.0554
    CldM = 0

    # Pitching Moment Coefficient
    Cm0R = 0.04
    CmaR = -0.613 #/rad
    Cmel = -1.122 #/rad
    Cmadot = -7.27 #/rad
    Cmq = -12.4 #/rad

    # Other Specs
    eta = 0.7
    ep = 0

    # System Inputs
    th = u[0]
    el = -u[1]

    # Thrust
    T = 550*th*eta/V

    # Gamma
    gamma = theta-alpha

    # Calculate alpha_dot
    term1 = -qbar*S*(Cl0+Cla*alpha+Clel*el+cbar/(2*V)*Clq*q)
    num = term1 + W*np.cos(gamma)-T*np.sin(alpha-ep)+m*V*q
    den = m*V-cbar*Cladot/(2*V)
    alpha_dot = num/den

    # Calculate lift, drag, pitch coefficients
    Cl = Cl0+Cla*alpha+Clel*el+cbar/(2*V)*Clq*q+cbar/(2*V)*Cladot*alpha_dot
    Cd = CdM+k*(Cl0+Cla*alpha+Clel*el-CldM)**2
    Cm = Cm0R+CmaR*alpha+Cmel*el+cbar/(2*V)*Cmadot*alpha_dot+cbar/(2*V)*Cmq*q

    # Calculate lift, drag, pitch moment
    L = qbar*S*Cl
    D = qbar*S*Cd
    M = qbar*S*cbar*Cm

    # Calculate V_dot
    V_dot = 1/m*(-D-W*np.sin(gamma)+T*np.cos(alpha-ep))

    # Calculate theta_dot
    theta_dot = q

    # Calculate q_dot
    q_dot = M/J2

    # Derivatives
    xdot[0] = V_dot
    xdot[1] = alpha_dot
    xdot[2] = theta_dot
    xdot[3] = q_dot
    
    return xdot

def outputs(t, x, u, params):
    output = np.zeros((2,))
    # output[0] = x[0] # Output Velocity
    output[0] = x[2] # Output Theta
    output[1] = x[2]-x[1] # Output Gamma
    
    return output

def model():
    sys = control.NonlinearIOSystem(state_derivatives, outputs)
    return sys