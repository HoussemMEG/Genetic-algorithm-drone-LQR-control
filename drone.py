from numpy import tan, sin, cos
from main import Ix, Iy, Iz, Kf_ax, Kf_ay, Kf_az, m, g

# % X = [phi theta psi P Q R Z Z_dot];
class Drone():
    def __init__(self, init_state=[0]*8, ref_state=[0]*8, dt=0.01):
        # state = [phi_0, theta_0, psi_0, P_0, Q_0, R_0, Z_0, Z_dot_0]
        self.ref_state = ref_state
        self.state = init_state
        self.dt = dt

    def step(self, U):
        # this part is made to to use the values of state(t) to update state(t+1)
        # at the same time
        new_state = [0]*8
        new_state[0] = self.state[0] + self.dt * (self.state[3] + tan(self.state[1]) *
                                                 (sin(self.state[0])*self.state[4] + cos(self.state[0])*self.state[5]))
        new_state[1] = self.state[1] + self.dt * (cos(self.state[0])*self.state[4] - sin(self.state[0])*self.state[5])
        new_state[2] = self.state[2] + self.dt * (sin(self.state[0])/cos(self.state[1])*self.state[4] +
                                                  cos(self.state[0])/cos(self.state[1])*self.state[5])
        new_state[3] = self.state[3] + self.dt * ((Iy-Iz)/Ix*self.state[4]*self.state[5] + U[0]/Ix
                                                  - Kf_ax/Ix*(self.state[3])**2)
        new_state[4] = self.state[4] + self.dt * ((Iz-Ix)/Iy*self.state[3]*self.state[5]+ U[1]/Iy
                                                  - Kf_ay/Iy*(self.state[4])**2)
        new_state[5] = self.state[5] + self.dt * ((Ix-Iy)/Iz*self.state[3]*self.state[4] + U[2]/Iz
                                                  - Kf_az/Iz*(self.state[5])**2)
        new_state[6] = self.state[6] + self.dt * (self.state[7])
        new_state[7] = self.state[7] + self.dt * (g - U(3)/m*cos(self.state[0])*cos(self.state[1]))
        return new_state