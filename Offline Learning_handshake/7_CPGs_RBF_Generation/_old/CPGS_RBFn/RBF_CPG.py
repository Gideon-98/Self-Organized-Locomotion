import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

N_KERNELS = 40 # Number of kernels
SIGMA = 0.4 # 0.4 #from the article
LR = 0.01 # Learning rate
W_D0 = 1.4  # Default synaptic weight
W_D1 = 0.18  # Default synaptic weight
ALPHA = 0.01  # Synaptic plasticity
MI = 0.1  # Extrinsic modulatory input


# Create the CPG Class
class CPG:
    def __init__(self):
        self.w_11 = W_D0  # Fixed synaptic weight (self-connection weights)
        self.w_22 = W_D0  # Fixed synaptic weight (self-connection weights)
        self.w_12 = W_D1 + MI  # Modulated synaptic weight (cross-connection weights)
        self.w_21 = -W_D1 - MI  # Modulated synaptic weight (cross-connection weights)
        self.alpha = ALPHA  # synaptic plasticity

    def update(self, o_N1, o_N2, S1, S2):
        a1 = self.w_11 * o_N1 + self.w_12 * o_N2 + self.alpha * S1
        a2 = self.w_22 * o_N2 + self.w_21 * o_N1 + self.alpha * S2

        o_N1 = self.tanh(a1)
        o_N2 = self.tanh(a2)

        return o_N1, o_N2

    def tanh(self, x):
        return np.tanh(x)


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
    means_kern_pos = np.linspace(0, num_steps - 1, N_KERNELS).astype(int)  # Cast to integer values because they are positions
    print(means_kern_pos)
    
    # Training: Define the target
    df = pd.read_csv("/CSVS_/1_foreleg.csv")
    target = df["jointFR1"].values

    # Simulation parameters
    num_steps = len(target)  # Number of time steps
    x = np.arange(num_steps)
    S1 = np.ones(num_steps)  # np.sin(0.1*x)
    S2 = np.ones(num_steps)  # np.cos(0.1*x)

    # Create CPG and RBF instances
    cpg = CPG()
    rbf = RBF(SIGMA)

    # Lists to store CPG outputs for plotting
    output_1_cpg = []
    output_2_cpg = []

    # Simulation loop: CPG
    for i in range(num_steps):
        o_N1, o_N2 = cpg.update(o_N1, o_N2, S1[i], S2[i])
        # print(o_N1, o_N2)
        output_1_cpg.append(o_N1)
        output_2_cpg.append(o_N2)

    # Lists to store RBF outputs for plotting
    kernels = []
    # Simulation loop: RBF
    for j in means_kern_pos:
        kernel = []
        for i in range(num_steps):
            phi = rbf.get_activation(
                output_1_cpg[i], output_2_cpg[i], output_1_cpg[j], output_2_cpg[j]
            )
            kernel.append(phi)
        kernels.append(kernel)

    # Convert the list to a numpy array
    kernels = np.array(kernels)

    # Initialize the weights
    W = np.ones(N_KERNELS)
    
    # Initialize the output DataFrame
    output_predictions_df = pd.DataFrame()
    W_trained_df = pd.DataFrame()

    # Training: initialize
    outputs = []
    output = 0
    delta_w = 0
    err_threshold = 0
    epochs = 10000
    precision = 0.01
    
    # Initialize the output DataFrame
    output_predictions_df = pd.DataFrame()
    W_trained_df = pd.DataFrame()

    for column in df.columns:
    #Skip object type columns    
        if df[column].dtype != 'object': 
            target = df[column].values
            print(column)
    # For a certain amount of epochs
        for t in range(epochs):
            err_threshold=0 # every epoch, reset the error threshold
            if t%1000 == 0: # print the epoch number every 1000 epochs
                print(t)
    # Calculate the output of the RBF layer 
    # by iterating through the positions of the kernels
    # k = index, means_kern_pos[k] = value
            for k,mean_pos in enumerate(means_kern_pos):
                output = 0
    #Cycle through the kernels calculating the output of every kernel
                for i in range(N_KERNELS):
                    output += kernels[i][mean_pos] * W[i]
    # Calculate the error at the current index based on the calculated output
                error = target[mean_pos] - output
                delta_w = LR * error 
    # Update the weights based on the error
                W[k] = W[k] + delta_w
    # Check for the error threshold, using abs to check the surronding of the target
                if abs(error) > err_threshold:
                    err_threshold = abs(error)
            if err_threshold < precision:
                break
    # Append the trained weights to the trained_weights_df
        W_trained_df[column] = W.tolist()
    # See training results
        final_output = []
        for j in range(num_steps):
            output = 0
            for i in range(N_KERNELS):
                output += kernels[i][j] * W[i]
    # Append the output to the final_output list
            final_output.append(output)
        
    
# Convert the final_output list to a DataFrame
    output_predictions_df[column] = final_output
    
    output_predictions_df.to_csv("output_predictions.csv", index=False)
    W_trained_df.to_csv("trained weights.csv", index=False)

    
# Call the main function
if __name__ == "__main__":
    main()
