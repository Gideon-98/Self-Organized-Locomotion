import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import math

W_D0 = 1.4  # Default synaptic weight
W_D1 = 0.18  # Default synaptic weight
ALPHA = 0.01  # Synaptic plasticity
MI = 0.1  # Extrinsic modulatory input
O_N1 = 0.0
O_N2 = 0.2

DIR = '9_Results_from_Coppelia_Simulation/'
TAG_ALPHA = 'sf_adapt copy.csv'

TAG = 'cpg_01 copy.csv'

TAG_GAIT = 'gait_adapt.csv'
TAG_GAIT_01 = 'gait_01.csv'

def generate_cpg(x_column):
    o_n1 = O_N1
    o_n2 = O_N2
    w_11 = W_D0  # Fixed synaptic weight (self-connection weights)
    w_22 = W_D0  # Fixed synaptic weight (self-connection weights)
    w_12 = W_D1 + MI  # Modulated synaptic weight (cross-connection weights)
    w_21 = -W_D1 - MI  # Modulated synaptic weight (cross-connection weights)
    alpha = ALPHA  # synaptic plasticity
    
    out_cpg1_list = []
    out_cpg2_list = []
    
    for x in x_column:
        #save the previous values
        o_n1_tmp = o_n1
        o_n2_tmp = o_n2
        
        #update the values
        o_n1 = math.tanh(w_11 * o_n1_tmp + w_12 * o_n2_tmp + alpha)
        o_n2 = math.tanh(w_22 * o_n2_tmp + w_21 * o_n1_tmp + alpha)
        
        #save the values
        out_cpg1_list.append([x, o_n1])
        out_cpg2_list.append([x, o_n2])
        
    out_cpg1 = pd.DataFrame(out_cpg1_list, columns=['Time (s)', 'CPG_1'])
    out_cpg2 = pd.DataFrame(out_cpg2_list, columns=['Time (s)', 'CPG_2'])
    
    return out_cpg1, out_cpg2

# Function to clean rows with Null values in a CSV file
def clean_csv(file_path):
    # Read the CSV file
    df = pd.read_csv(file_path)

    # Replace 'Null' with numpy.nan
    df.replace('Null', np.nan, inplace=True)

    # Remove the last two columns
    #df = df.iloc[:, :-2]
    
    # Remove rows with Null values
    df = df.dropna() # Drop rows with Null values in any column
    
    df.to_csv(file_path, index=False)  # Save the cleaned data back to the CSV file

# Function to plot data from a CSV file
def plot_csv(file_path, x_column):
    df = pd.read_csv(file_path)
    
    #df = df.iloc[:len(df)//4]

    columns_list = df.columns[1:].tolist()
    
    for column in columns_list:
        plt.plot(df[x_column], df[column])
        plt.xlabel(x_column)
        plt.ylabel(column)
        plt.title('Plot')
    plt.legend(columns_list)
    plt.show()

def compare_wz_th_cpg(file_path,x_column):
    
    df_alpha = pd.read_csv(DIR + TAG_ALPHA)
    
    df = pd.read_csv(file_path)
    
    cpg_1, cpg_2 = generate_cpg(df[x_column])
    
    # Select only half of the x_column values
    df = df.iloc[:len(df)//4]
    cpg_1 = cpg_1.iloc[:len(cpg_1)//4]
    cpg_2 = cpg_2.iloc[:len(cpg_2)//4]
    df_alpha = df_alpha.iloc[:3847]
    
    print(len(df),len(cpg_1),len(cpg_2),len(df_alpha))

    #["Time (s)','CPG0 (User unit)','CPG1 (User unit)','Z_POS (Meters)','CL0 (Degrees)','FL0 (Degrees)','TL0 (Degrees)"]

    columns_list_df = df.columns[1:3].tolist()
    if file_path == '9_Results_from_Coppelia_Simulation/cpg_01 copy.csv':
        # Create subplots
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 5))
        for column in columns_list_df:
            ax1.plot(df[x_column], df[column], label=column)
        ax1.set_xticks(range(0, 16, 2))
        ax1.set_xlabel(x_column)
        ax1.set_title('Experimental CPGs, α = 0.01')
        ax1.legend()
            
        # Plot cpg1 and cpg2
        ax2.plot(cpg_1[x_column], cpg_1['CPG_1'], label='CPG1')
        ax2.plot(cpg_2[x_column], cpg_2['CPG_2'], label='CPG2')
        
        ax2.set_xticks(range(0, 16, 2))
        ax2.set_xlabel(x_column)
        ax2.set_title('Theoretical CPGs, α = 0.01')
        
        ax2.legend()
        # Adjust spacing between subplots
        plt.tight_layout()
        
        # Add grid
        ax1.grid(True)
        ax2.grid(True)
    else:
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(9, 5))
        # Plot all columns of df
        for column in columns_list_df:
            ax1.plot(df[x_column], df[column], label=column)
            
        ax1.set_xticks(range(0, 16, 2))
        ax1.set_xlabel(x_column)
        ax1.set_title('Experimental CPGs, adaptation enabled')
        
        ax1.legend()
        
        # Plot cpg1 and cpg2
        ax2.plot(cpg_1[x_column], cpg_1['CPG_1'], label='CPG1')
        ax2.plot(cpg_2[x_column], cpg_2['CPG_2'], label='CPG2')
        
        ax2.set_xticks(range(0, 16, 2))
        ax2.set_xlabel(x_column)
        ax2.set_title('Theoretical CPGs, α = 0.01')
        
        ax2.legend()
        
        ax3.plot(df_alpha[x_column], df_alpha['sf3'], label='α_R3', color='purple')
        ax3.plot(df_alpha[x_column], df_alpha['sf6'], label='α_L3', color='red')
        
        # Set x-axis sensitivity
        # ax3.set_xticks(range(0, 16, 2))
        ax3.set_xlabel(x_column)
        ax3.set_title('α values')
        
        ax3.legend()   
        
        # Adjust spacing between subplots
        plt.tight_layout()
        
        # Add grid
        ax1.grid(True)
        ax2.grid(True)
        ax3.grid(True)
        
    
    # Show the figure
    plt.show()

def plot_gait(file_path, x_column):
    df = pd.read_csv(file_path)
    print(df)
    df_1 = df
    df_2 = df.iloc[:501]
    df_3 = df.iloc[501:1001]
    df_4 = df.iloc[1001:1501]
    #df = df.iloc[:len(df)//4]
    
    # Create subplots
    fig = plt.figure(figsize=(9, 5))
    grid = gridspec.GridSpec(2, 3)
    
    ax1 = plt.subplot(grid[0, :])
    ax2 = plt.subplot(grid[1, 0])
    ax3 = plt.subplot(grid[1, 1])
    ax4 = plt.subplot(grid[1, 2])
    
    for column in df_1.columns[1:]:
        ax1.plot(df_1[x_column], df_1[column], label=column)
        ax1.legend()
        ax1.set_xlabel(x_column)
        ax1.set_ylabel('Value')
        #ax1.set_title('Gait entire Simulation, adpatation enabled')
        ax1.set_title('Gait entire Simulation, α = 0.01')
    
    for columns in df_1.columns[1:]:
        ax2.plot(df_2[x_column], df_2[columns])
        ax2.set_xlabel(x_column)
        ax2.set_ylabel('Value')
        ax2.set_title('Gait Data from 0 to 5s')
    
    for columns in df_1.columns[1:]:
        ax3.plot(df_3[x_column], df_3[columns])
        ax3.set_xlabel(x_column)
        ax3.set_ylabel('Value')
        ax3.set_title('Gait Data from 5 to 10s')
    
    for columns in df_1.columns[1:]:
        ax4.plot(df_4[x_column], df_4[columns])
        ax4.set_xlabel(x_column)
        ax4.set_ylabel('Value')
        ax4.set_title('Gait Data from 10 to 15s')
    
    plt.tight_layout()
    plt.show()    

def create_csv_gait_th(file_path,x_column):
    df = pd.read_csv(file_path)

    ones = np.ones(500)
    zeros = np.zeros(500)
    array = np.concatenate((zeros,ones),axis=0)
    n_gaits = 15
    
    df_new = pd.DataFrame()
    df_new[x_column] = df[x_column]
    print(df_new)
    
    output = np.array([])
    
    for i in range(n_gaits):
        output = np.concatenate((output, array))
    
    
    
    for column in df.columns[1:4]:
        horizontal_displacement = output[:375]
        output = np.concatenate((horizontal_displacement, output))
        output = output[:len(df_new)]
        df_new[column] = output
        output = output + 1.1
    
    output = np.array([])
    for i in range(n_gaits):
        output = np.concatenate((output, array))
    output = output + 3.3
    
    
    for column in df.columns[4:]:
        horizontal_displacement = output[:375]
        if column == df.columns[4]:
            horizontal_displacement = np.full(625, 3.3)
        output = np.concatenate((horizontal_displacement, output))
        output = output[:len(df_new)]
        df_new[column] = output
        output = output + 1.1
    print(df_new)
    
    # Plot df_new
    fig, ax = plt.subplots(figsize=(6, 4))
    
    df_new = df_new.iloc[500:]
    
    for column in df_new.columns[1:]:
        ax.plot(df_new[x_column], df_new[column], label=column)
        ax.set_xlabel(x_column)
        ax.set_ylabel('Value')
        ax.set_title('Theoretical Gait Data')
        ax.legend()
        
        # Add vertical lines every 500
    for i in range(1000, len(df_new), 500):
        ax.axvline(x=df_new[x_column][i], color='gray', linestyle='--')
        
    plt.show()
    ax.set_xlabel(x_column)
    ax.set_ylabel('Value')
    ax.set_title('Generated Gait Data')
    ax.legend()
    plt.show()
# Global flags to activate/deactivate functions
clean_csv_enabled = False
plot_csv_enabled = False 
compare_cpg = False
plot_gait_enabled = True
create_csv_gait = False

# Main function
def main():
    dir = '9_Results_from_Coppelia_Simulation/'
    file_path = dir + TAG
    x_column = 'Time (s)'
    
    if clean_csv_enabled:
        clean_csv(file_path)
    if plot_csv_enabled:
        #y_column = 'value'
        plot_csv(file_path, x_column)
    if compare_cpg:
        compare_wz_th_cpg(file_path,x_column)
    if plot_gait_enabled:
        plot_gait(dir + TAG_GAIT_01, x_column)
    if create_csv_gait:
        create_csv_gait_th(dir + TAG_GAIT_01, x_column)

# Run the main function
if __name__ == '__main__':
    main()