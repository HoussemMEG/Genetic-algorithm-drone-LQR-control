from numpy import tan, sin, cos, deg2rad, rad2deg
import numpy as np
import matplotlib.pyplot as plt

# Initialising the model parameters
Ix = 3.8278e-3
Iy = 3.8278e-3
Iz = 7.6566e-3
Kf_ax = 5.567e-4
Kf_ay = 5.567e-4
Kf_az = 6.354e-4
g = 9.81
m = 0.486

# Simulation parameters
init_time = 0
final_time = 5
dt = 0.001
time = np.arange(init_time, final_time + dt, dt)


class Drone:
    # % X = [phi theta psi P Q R Z Z_dot];
    def __init__(self, init_state=[0]*8, ref_state=[0]*8):
        # state = [phi_0, theta_0, psi_0, P_0, Q_0, R_0, Z_0, Z_dot_0]
        self.ref_state = [*deg2rad(ref_state[0:3]), *ref_state[3:]]
        self.state = [*deg2rad(init_state[0:3]), *init_state[3:]]
        self.state_memory = [self.state]
        self.control_memory = [[0]*4]

    def step(self, U):
        # this part is made to to use the values of state(t) to update state(t+1)
        # at the same time
        new_state = [0]*8
        new_state[0] = self.state[0] + dt * (self.state[3] + tan(self.state[1]) *
                                             (sin(self.state[0])*self.state[4] + cos(self.state[0])*self.state[5]))
        new_state[1] = self.state[1] + dt * (cos(self.state[0])*self.state[4] - sin(self.state[0])*self.state[5])
        new_state[2] = self.state[2] + dt * (sin(self.state[0])/cos(self.state[1])*self.state[4] +
                                             cos(self.state[0])/cos(self.state[1])*self.state[5])
        new_state[3] = self.state[3] + dt * ((Iy-Iz)/Ix*self.state[4]*self.state[5] + U[0]/Ix
                                             - Kf_ax/Ix*(self.state[3])**2)
        new_state[4] = self.state[4] + dt * ((Iz-Ix)/Iy*self.state[3]*self.state[5]+ U[1]/Iy
                                             - Kf_ay/Iy*(self.state[4])**2)
        new_state[5] = self.state[5] + dt * ((Ix-Iy)/Iz*self.state[3]*self.state[4] + U[2]/Iz
                                             - Kf_az/Iz*(self.state[5])**2)
        new_state[6] = self.state[6] + dt * (self.state[7]) if ((self.state[6] + dt * (self.state[7])) <= 0) else 0
        new_state[7] = self.state[7] + dt * (g - U[3]/m*cos(self.state[0])*cos(self.state[1]))
        self.state = new_state

        # saving the new state and control signals
        self.state_memory.append(new_state)
        self.control_memory.append(U)
        return new_state

    def control(self, K_end):
        K = [[1.8398, -0.0459, 0.6971, 0.1687, -0.0008, 0.0348, 0.0016, 0.0001],
             [-0.0459, 1.1513, -0.0546, -0.0008, 0.1531, -0.0030, -0.0001, -0.0000],
             [1.3943, -0.1092, 2.4393, 0.0697, -0.0060, 0.2751, 0.0020, 0.0001]]
        K.append(K_end)
        K = np.array(K)
        control = -K.dot(np.array(self.state)-self.ref_state) + \
               [0, 0, 0, m*g/(cos(self.state[0])*cos(self.state[1]))]
        if control[3] > 10:
            control[3] = 10
        return control

    def simulate(self, K_end):
        for _ in range(len(time)-1):
            self.step(self.control(K_end))

    def position_show(self):
        plt.figure()
        plt.title("Z position simulation")
        ax = plt.axes()
        plt.grid(b=True)
        ax.plot(time, [row[6] for row in self.state_memory], label='Z')
        plt.legend(loc='upper right')
        ax.set(xlabel='Temps [s]', ylabel='Position [m]')
        plt.show()

    def angle_show(self):
        plt.figure()
        ax = plt.axes()
        plt.title("Angles simulation")
        plt.grid(b=True)
        plt.plot(time, [rad2deg(row[0]) for row in self.state_memory], label="Phi")
        plt.plot(time, [rad2deg(row[1]) for row in self.state_memory], label="Theta")
        plt.plot(time, [rad2deg(row[2]) for row in self.state_memory], label="Psi")
        plt.legend(loc='upper right')
        ax.set(xlabel='Temps [s]', ylabel='Angle [deg]')
        plt.show()

    def control_show(self):
        plt.figure()
        ax = plt.axes()
        plt.title("Control signals simulation")
        plt.grid(b=True)
        plt.plot(time, [row[0] for row in self.control_memory], label="Tau_phi")
        plt.plot(time, [row[1] for row in self.control_memory], label="Tau_theta")
        plt.plot(time, [row[2] for row in self.control_memory], label="Tau_psi")
        plt.plot(time, [row[3] for row in self.control_memory], label="Thrust")
        plt.legend(loc='upper right')
        ax.set(xlabel='Temps [s]', ylabel='Amplitude [SI]')
        plt.show()





#[-0.1992, 0.0156, -0.1297, -0.0100, 0.0009, -0.0065, -97.2003, -14.5800]