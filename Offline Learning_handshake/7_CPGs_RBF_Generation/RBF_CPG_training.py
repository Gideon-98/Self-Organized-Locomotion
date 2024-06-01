import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import math
import copy
import os

TAG = "simpleCurveTraj14-5-14-31-7-data"
#TAG = "simpleCurveTraj9-5-15-34-11-data" 
#TAG = "simpleCurveTraj29-4-17-18-47-data"
#TAG = "simpleCurveTraj8-5-9-14-55-data"


INPUT_DIR = "7_CPGs_RBF_Generation/CSVS/" + TAG + "/"
OUT_DIR_TRAIN = "7_CPGs_RBF_Generation/CPGs_waves/"
OUT_DIR = "8_Centers_weights_to_Coppelia_Simulation/"


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

N_KERNELS = 40
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

def generate_rbf(cpg_1, cpg_2, mu_1, mu_2, sigma):
    return np.exp(-((cpg_1 - mu_1) ** 2 + (cpg_2 - mu_2) ** 2) / (2 * sigma**2))

def main():
    compare_with_joe = False
    Tidy_up = False
    Training = False
    check_predictions = True
    
    filenames = ["Foreleg_" + TAG + "_resampled" + str(N_SAMPLES), "Midleg_" + TAG + "_resampled" + str(N_SAMPLES), "Hindleg_" + TAG + "_resampled" + str(N_SAMPLES)]
    #filenames = ["Foreleg", "Midleg", "Hindleg"]
    prediction_suffix = "_gian_predictions"
    weights_suffix = "_gian_weights"
    centers_suffix = "_gian_centers"
    kernels_suffix = "_gian_kernels"
    
    # Cast to integer values because they are positions 
    means_kern_pos = np.linspace(0, N_SAMPLES-1, N_KERNELS).astype(int)
    #print(means_kern_pos)
    
    # Training data
    delta_w = 0
    err_threshold = 0
    epochs = 10000
    learning_precision = 0.01
    
    # Lists to store CPG outputs for plotting
    for start in range(0,100-N_SAMPLES):
        cpg1_out, cpg2_out = generate_cpg(100)
        sub_out_dir = OUT_DIR_TRAIN + "CPGs_waves_" + TAG
        if not os.path.exists(sub_out_dir):
            os.makedirs(sub_out_dir) 
        
        output_directory_train = sub_out_dir + '/CPG_' + str(start) + '/'
        if not os.path.exists(output_directory_train):
            os.makedirs(output_directory_train)    
        
        #start = 5 #60 #13
        cpg1_out = cpg1_out[start:start + N_SAMPLES]
        cpg2_out = cpg2_out[start:start + N_SAMPLES]
        
        print(len(cpg1_out))
        print(len(cpg2_out))
        
        if Training:

            for file in filenames:
                input_file = INPUT_DIR + file + ".csv"
                df = pd.read_csv(input_file)
                df = df.drop(columns = [df.columns[3],df.columns[4],df.columns[5]])
                # Initialize the output DataFrame
                output_predictions_df = pd.DataFrame(columns=df.columns)
                W_trained_df = pd.DataFrame()
                kernels_df = pd.DataFrame()
                errors_df = pd.DataFrame(np.zeros((10000, 4)), columns=df.columns)
                   
                for column in df.columns:
                    print("Training column: ", column)
                #Skip object type columns    
                    if df[column].dtype != 'object': 
                        target = df[column].values

    #-------------- Simulation Loop --------------#
                        # Lists to store RBF outputs for plotting
                        kernels = []
                        centers = []
                        
                        # Simulation loop: RBF
                        for j in means_kern_pos:
                            kernel = []
    #-------------- Simulation Loop: RBF generation --------------#                        
                            for i in range(N_SAMPLES):
                                phi = generate_rbf(cpg1_out[i], cpg2_out[i], cpg1_out[j], cpg2_out[j], SIGMA)
                                kernel.append(phi)
                            center = [cpg1_out[j], cpg2_out[j]]
                            centers.append(center)                         
                            kernels.append(kernel)
                        kernels = np.array(kernels)
                        kernels_df = pd.DataFrame(kernels)
                        # Convert the centers list to a DataFrame
                        centers_df = pd.DataFrame(centers)
                        #errors_df = pd.DataFrame()
                    W = np.ones(N_KERNELS)  # Weights
                    errors = []
    #-------------- Training Loop --------------#
                    for t in range(epochs):
                        err_threshold=0 # every epoch, reset the error threshold
                        if t%1000 == 0: # print the epoch number every 1000 epochs
                            print(t)

    # Calculate the output of the RBF layer  by iterating through the positions of the kernels, k = index, means_kern_pos[k] = value
    # #-------------- Training Loop: scroll the kern positions --------------#
                        for k,mean_pos in enumerate(means_kern_pos):
                            output = 0
                #Cycle through the kernels calculating the output of every kernel
                            for i in range(N_KERNELS):
                                output += kernels[i][mean_pos] * W[i]
                                #print(kernels[i][mean_pos])
                # Calculate the error at the current index based on the calculated output
                            error = target[mean_pos] - output
                            delta_w = LR * error 
                # Update the weights based on the error
                            W[k] = W[k] + delta_w
                # Check for the error threshold, using abs to check the surronding of the target
                            if abs(error) > err_threshold:
                                err_threshold = abs(error)
                # Append the error to the errors list
                        errors.append(err_threshold)
                        if err_threshold < learning_precision or t == epochs-1:
                            print("Training complete")
                            print("Epochs: ", t)
                            print("Max Error: ", err_threshold)
                            break                
                    # Append the trained weights to the trained_weights_df
                    W_trained_df[column] = W.tolist()
                    if len (errors) < len(errors_df):
                        errors = errors + [0] * (len(errors_df) - len(errors))
                    errors_df[column] = errors
                    #print("Max error", err_threshold)
                    err_threshold = 0
                    
    #-------------- Training Loop: use the trained weights to get the results --------------#
                    outputs = []
                    for j in range(N_SAMPLES):
                        output = 0
                        for i in range(N_KERNELS):
                            output += kernels[i][j] * W[i]
                # Append the output to the outputs list
                        outputs.append(output)
                # Convert the outputs list to a DataFrame
                    output_predictions_df[column] = outputs
                
                # Save the results to CSV files
                output_predictions_df.to_csv(output_directory_train + file + prediction_suffix + ".csv", index=False)
                W_trained_df.to_csv(output_directory_train + file + weights_suffix + ".csv", index=False)
                centers_df.to_csv(output_directory_train + file + centers_suffix + ".csv", index=False)
                kernels_df.to_csv(output_directory_train + file + kernels_suffix + ".csv", index=False)
                errors_df.to_csv(output_directory_train + file + "_errors.csv", index=False)
            print("Training complete")
        
        predictions = [filenames[0] + prediction_suffix + '.csv',filenames[1] + prediction_suffix + '.csv', filenames[2] + prediction_suffix + '.csv']
        weights = [filenames[0] + weights_suffix + '.csv', filenames[1] + weights_suffix + '.csv', filenames[2] + weights_suffix + '.csv']
        centers_list = [filenames[0] + centers_suffix + '.csv', filenames[1] + centers_suffix + '.csv', filenames[2] + centers_suffix + '.csv']
        
        output_directory = OUT_DIR + '/weights_centers_' + TAG + '/' + 'gian_net_' + str(start) + '/'
        if not os.path.exists(output_directory):
            os.makedirs(output_directory)
        
        # Save the centers used to train to a text file
        df_centers = pd.read_csv(output_directory_train + centers_list[1])
        df_centers.to_csv(output_directory + 'rbfn_centers_gian.txt', index=False, header=False) 
        
        if Tidy_up:
            i=0
            j=3
            for weight in weights:
                df = pd.read_csv(output_directory_train + weight)
                
                with open(output_directory +'rbfn_weights_gian_' + str(i) + '.txt', 'w') as file:
                    for colum in df.columns:
                        for value in df[colum]:
                            #value = round(value, 7)
                            file.write(str(value) + '\n')
                        file.write('\n')

                with open(output_directory + 'rbfn_weights_gian_' + str(j) + '.txt', 'w') as file:
                    for colum in df.columns:
                        for value in df[colum]:
                            #value = round(value, 7)
                            file.write(str(value) + '\n')
                        file.write('\n')
                i+=1
                j+=1
        
        if compare_with_joe:
            data = np.loadtxt('7_CPGs_RBF_Generation/_old/CPGS_RBFn/Joe_weights_centers/rbfn_centers.txt', delimiter=',')
            data_weights = np.loadtxt('7_CPGs_RBF_Generation/_old/CPGS_RBFn/Joe_weights_centers/rbfn_weights_0.txt', delimiter=',')
            data_gian = np.loadtxt(output_directory +'rbfn_centers_gian.txt', delimiter=',')
            data_weights_gian = np.loadtxt(output_directory + 'rbfn_weights_gian_0.txt', delimiter=',')

            print("Joe weights: ", data_weights.shape)
            print("Gian weights: ", data_weights_gian.shape)
            
            print("Joe centers: ", data.shape)
            print("Gian centers: ", data_gian.shape)
            
            
            # Separate the columns
            column1 = data[:, 0]
            column2 = data[:, 1]
            column3 = data_gian[:,0]
            column4 = data_gian[:,1]
            column5 = cpg1_out
            column6 = cpg2_out
        
            # Plotting the results
            fig, axs = plt.subplots(1, 2, figsize=(10, 4))

            # Plotting CPG 1
            axs[0].plot(column1)
            axs[0].plot(column3)
            axs[0].plot(column5)
            axs[0].set_title('Centers_1')
            axs[0].set_xlabel('Time Step')
            axs[0].set_ylabel('Output')
            axs[0].legend(['Joe', 'Gian'])

            # Plotting CPG 2
            axs[1].plot(column2)
            axs[1].plot(column4)
            axs[1].plot(column6)
            axs[1].set_title('Centers_2')
            axs[1].set_xlabel('Time Step')
            axs[1].set_ylabel('Output')
            axs[1].legend(['Joe', 'Gian'])

            plt.tight_layout()
            plt.show()

        if check_predictions:
            x = np.linspace(0, 40, 40)
            for file in predictions:
                df_1 = pd.read_csv(sub_out_dir + '/CPG_8' + '/'+ file)
                df_2 = pd.read_csv(INPUT_DIR + file.replace(prediction_suffix, ""))
                df_2 = df_2.drop(columns = [df_2.columns[3],df_2.columns[4],df_2.columns[5]])
                
                for column in df_1.columns:
                    prediction = df_1[column].values 
                    target = df_2[column].values

                    plt.figure(figsize=(4, 2))
                    plt.plot(x, target, label='Target')
                    plt.plot(x, prediction, label='Prediction', color='orange')

                    plt.xlabel('X')
                    plt.ylabel('Y')
                    plt.title('Data from CSV')
                    plt.xlabel('Time Steps')
                    plt.ylabel('Value')
                    plt.title(f'{column}_target vs {column}_prediction')
                    plt.legend()
                    plt.show()

if __name__ == "__main__":
    main()

