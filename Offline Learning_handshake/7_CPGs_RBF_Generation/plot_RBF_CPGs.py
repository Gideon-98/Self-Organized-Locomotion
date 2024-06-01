import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

SIGMA = 0.2 #0.4 # 0.4 #from the article
LR = 0.01 # Learning rate
W_D0 = 1.4  # Default synaptic weight
W_D1 = 0.18  # Default synaptic weight
ALPHA = 0.01  # Synaptic plasticity
MI = 0.1  # Extrinsic modulatory input
S1 = np.ones(100)
S2 = np.ones(100)
O_N1 = 0.0
O_N2 = 0.2
N_KERNELS = 40


# Create the CPG Class
class CPG:
    def __init__(self):
        self.w_11 = W_D0  # Fixed synaptic weight (self-connection weights)
        self.w_22 = W_D0  # Fixed synaptic weight (self-connection weights)
        self.w_12 = W_D1 + MI  # Modulated synaptic weight (cross-connection weights)
        self.w_21 = -W_D1 - MI  # Modulated synaptic weight (cross-connection weights)
        self.alpha = ALPHA  # synaptic plasticity

    def update(self, o_N1, o_N2, S1, S2):
        o_N1 = np.tanh(self.w_11 * o_N1 + self.w_12 * o_N2 + self.alpha * S1)
        o_N2 = np.tanh(self.w_22 * o_N2 + self.w_21 * o_N1 + self.alpha * S2)
        return o_N1, o_N2

# Create the RBF Class
class RBF:
    def __init__(self, sigma):
        self.sigma = sigma

    def get_activation(self, cpg_1, cpg_2, mu_1, mu_2):
        """
        Gaussian activation function, based on the distance between the CPG outputs and the RBF kernels
        """
        return np.exp(
            -((cpg_1 - mu_1) ** 2 + (cpg_2 - mu_2) ** 2) / (2 * self.sigma**2)
        )

def main():
    cpg = CPG()
    rbf = RBF(SIGMA)
    num_steps = N_KERNELS # Number of time steps
    means_kern_pos = np.linspace(0, num_steps - 1, N_KERNELS).astype(int)  # Cast to integer values because they are positions
    x = np.arange(num_steps)
    print(x)
    print(x.shape)
    print(num_steps)
    S1 = np.ones(100) #np.sin(0.1*x)  
    S2 = np.ones(100) #np.cos(0.1*x)  

    # Lists to store CPG outputs for plotting
    output_1_cpg = []
    output_2_cpg = []
    # Initialize CPG outputs
    o_N1 = O_N1 #0.0
    o_N2 = O_N2 #0.0
    # Simulation loop: CPG
    for i in range(100):
        o_N1, o_N2 = cpg.update(o_N1, o_N2, S1[i], S2[i])
        # print(o_N1, o_N2)
        output_1_cpg.append(o_N1)
        output_2_cpg.append(o_N2)
    output_1_cpg = output_1_cpg[15:55]
    output_2_cpg = output_2_cpg[15:55]
    # Lists to store RBF outputs for plotting
    kernels = []
    centers = []
    # Simulation loop: RBF
    for j in means_kern_pos:
        kernel = []
        for i in range(num_steps):
            phi = rbf.get_activation(
                output_1_cpg[i], output_2_cpg[i], output_1_cpg[j], output_2_cpg[j]
            )
            center = [output_1_cpg[j], output_2_cpg[j]]
            kernel.append(phi)
        kernels.append(kernel)
        centers.append(center)
        #print(kernels)
    # Convert the list to a numpy array
    kernels = np.array(kernels)

    print(means_kern_pos)
    print(kernels[0])
    # Plotting
    plt.figure(figsize=(6, 4), dpi=80)
    for i in range(len(means_kern_pos)):
        plt.axvline(x=means_kern_pos[i], color='r', linestyle='--')
    for i in range(N_KERNELS):
        plt.plot(x, kernels[i], label='RBF Output')
    #plt.plot(x, output_1_cpg, label='CPG 1 Output')
    #plt.plot(x, output_2_cpg, label='CPG 2 Output')
    plt.xlabel('Time Steps')
    plt.ylabel('CPG Output')
    plt.title('Central Pattern Generator (CPG) Outputs')
    #plt.legend()
    plt.show()

    plt.figure(figsize=(6, 4), dpi=80)
    for i in range(len(means_kern_pos)):
        plt.axvline(x=means_kern_pos[i], color='r', linestyle='--')
    plt.axhline(y=0, color='k', linestyle='--')
    plt.plot(x, output_1_cpg, label='CPG 1 Output')
    plt.plot(x, output_2_cpg, label='CPG 2 Output')
    #plt.plot(x, centers_df[0], label='RBF Centers')
    #plt.plot(x, centers_df[1], label='RBF Centers')
    #plt.plot(x,S1, label='S1')
    #plt.plot(x,S2, label='S2')
    plt.xlabel('Time Steps')
    plt.ylabel('CPG Output')
    plt.title('Central Pattern Generator (CPG) Outputs')
    plt.legend()
    plt.show()
    
    # ... rest of your code ...

if __name__ == "__main__":
    main()