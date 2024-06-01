import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import math
import copy

INPUT_DIR = "7_CPGs_RBF_Generation/CSVS/"
OUT_DIR = "7_CPGs_RBF_Generation/"


SIGMA = 0.2 #0.4 # 0.4 #from the article
LR = 0.01 # Learning rate
W_D0 = 1.4  # Default synaptic weight
W_D1 = 0.18  # Default synaptic weight
ALPHA = 0.01  # Synaptic plasticity
MI = 0.1  # Extrinsic modulatory input
S1 = np.ones(1000)
S2 = np.ones(1000)
O_N1 = 0.0
O_N2 = 0.2

N_SAMPLES = 40

def generate_cpg(lenght_cpg):
    o_n1 = O_N1
    o_n2 = O_N2
    
    w_11 = W_D0  # Fixed synaptic weight (self-connection weights)
    w_22 = W_D0  # Fixed synaptic weight (self-connection weights)
    w_12 = W_D1 + MI  # Modulated synaptic weight (cross-connection weights)
    w_21 = -W_D1 - MI  # Modulated synaptic weight (cross-connection weights)
    alpha = ALPHA  # synaptic plasticity
    
    out_cpg1 = []
    out_cpg2 = []
    for i in range(lenght_cpg):
        #save the previous values
        o_n1_tmp = o_n1
        o_n2_tmp = o_n2
        
        #update the values
        o_n1 = math.tanh(w_11 * o_n1_tmp + w_12 * o_n2_tmp + alpha*S1[i])
        o_n2 = math.tanh(w_22 * o_n2_tmp + w_21 * o_n1_tmp + alpha*S2[i])
        
        #save the values
        out_cpg1.append(o_n1)
        out_cpg2.append(o_n2)
    
    return out_cpg1, out_cpg2


def main():
    fig, axs = plt.subplots(1, 2, figsize=(9, 5))
    cpg1_out, cpg2_out = generate_cpg(100)
    data_joe = np.loadtxt('7_CPGs_RBF_Generation/_old/CPGS_RBFn/Joe_weights_centers/rbfn_centers.txt', delimiter=',')
    column1_joe = data_joe[:, 0]
    column2_joe = data_joe[:, 1]
    
    # axs[0].plot(column1_joe, color='k')
    # axs[1].plot(column2_joe, color='k')
    for start in range(100-N_SAMPLES):
        cpg1_out_plot = cpg1_out[start:start + N_SAMPLES]
        cpg2_out_plot = cpg2_out[start:start + N_SAMPLES]
        
        # Plotting CPG 1
        if start == 8:
            axs[0].plot(cpg1_out_plot, color='k', label='Chosen_CPG_1' if start == 8 else None)
        else:
            axs[0].plot(cpg1_out_plot, color='orange', label='CPG_1' if start == 0 else None)
        axs[0].set_title('CPG_1')
        axs[0].set_xlabel('Time Step')
        axs[0].set_ylabel('Output')

        # Plotting CPG 2
        if start == 8:
            axs[1].plot(cpg1_out_plot, color='k', label='Chosen_CPG_2' if start == 8 else None)
        else:
            axs[1].plot(cpg2_out_plot, color='orange', label='CPG_2' if start == 0 else None)
        axs[1].set_title('CPG_2')
        axs[1].set_xlabel('Time Step')
        axs[1].set_ylabel('Output')

    # Add legends after the loop
    axs[0].legend()
    axs[1].legend()

    plt.tight_layout()
    plt.show()

    
    

if __name__ == "__main__":
    main()