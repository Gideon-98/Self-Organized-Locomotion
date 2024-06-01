import os
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D  # Import the 3D projection module
import matplotlib.ticker as ticker
import pandas as pd
import numpy as np

INPUT_DIR = "2_Prepare_Trackers/Plots_procedure/Offset_Traj/"#Final_Traj/" #Resampled_Traj/"
OUTPUT_DIR = "3_Combine_Trackers_to_3D/Plots_procedure_3D"
N_PAD = 10
N_RESAMPLE = 100

def plot_3d_scatter(dataframe, title, mode):
    x, y, z = dataframe['x_column'], dataframe['y_column'], dataframe['z_column']
    fig = plt.figure(figsize = (10,10))
    ax = plt.axes(projection='3d')
    ax.scatter(x, y, z, label=f'Data from {title}')
    ax.scatter(x.iloc[-1], y.iloc[-1], z.iloc[-1], label=f'Last Point of {title}_'+ mode, c='red')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.legend()
    plt.title(f'3D Plot for Data from {title}_'+ mode)
    if not os.path.exists(OUTPUT_DIR + "/3D_scatter/" + mode):
        os.makedirs(OUTPUT_DIR + "/3D_scatter/" + mode)
    plt.savefig(OUTPUT_DIR + "/3D_scatter/" + mode + "/" + title + ".png")
    #plt.show()
    
    fig = plt.figure(figsize = (10,10))
    ax = plt.axes(projection='3d')
    ax.plot(x, y, z, label=f'Data from {title}_'+ mode)
    
    x = np.linspace(min(x), max(x), 10)
    y = np.linspace(min(y), max(y), 10)
    x, y = np.meshgrid(x, y)
    z = np.zeros_like(x)
    
    #ax.plot_surface(x, y, z, alpha=0.5, rstride=100, cstride=100)
    
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.legend()
    plt.title(f'3D Plot for Data from {title}_'+ mode)
    if not os.path.exists(OUTPUT_DIR + "/3D_plot/" + mode):
        os.makedirs(OUTPUT_DIR + "/3D_plot/" + mode)
    plt.savefig(OUTPUT_DIR + "/3D_plot/" + mode + "/" + title + ".png")
    #plt.show()
    
def resample_data(data, target_samples=N_RESAMPLE):
    downsampled_df = pd.DataFrame()

    for column in data.columns:
        if data[column].dtype != 'object':  # Skip object type columns
            downsampled_column = np.interp(np.linspace(0, len(data[column]) - 1, target_samples),
                                            np.arange(len(data[column])),
                                            data[column].values)
            downsampled_df[column] = downsampled_column

    return downsampled_df

def close_path(data):
    last_value =  data.iloc[-1]
    first_value = data.iloc[0]
    
    # print(last_value)
    # print(first_value)
    
    step = (last_value-first_value)/N_PAD

    x_column = np.linspace(last_value.iloc[0], first_value.iloc[0], N_PAD)
    y_column = np.linspace(last_value.iloc[1], first_value.iloc[1], N_PAD)
    z_column = np.linspace(last_value.iloc[2], first_value.iloc[2], N_PAD)
    
    print(len(x_column), len(y_column), len(z_column))
    print(z_column)    
    new_data = pd.DataFrame({'x_column': x_column, 'y_column': y_column, 'z_column': z_column})
    # print(new_data)
    # print(new_data.shape)
    data = pd.concat([data, new_data], ignore_index=True)
    
    return data

def scale_scalar(data, scale_factor):
    scaled_data = data.copy()  # Create a copy of the data to avoid modifying the original DataFrame
    scaled_data.iloc[:, :3] = data.iloc[:, :3] * scale_factor  # Scale only the first three columns
    # print(scale_factor)

    return scaled_data

def main():

    factors = [0.1, 0.125, 0.15, 0.175, 0.2, 0.25, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
    legs = ["hindleg", "midleg", "foreleg"]
    modes = ["1_combine_3D", "2_scale_3D", "3_clean_3D","4_resample_3D","5_close_path_3D","coppelia_format"]
    
    for leg in legs:
        for mode in modes:
            
            if leg == "hindleg":
               leg_xz = pd.read_csv(INPUT_DIR + "Tracker1_offset.txt", sep=",", header=0, names=["x_column", "z_column"])
               leg_xy = pd.read_csv(INPUT_DIR + "Tracker4_offset.txt", sep=",", header=0, names=["x_column", "y_column"])
               title = "Hindleg"
            
            if leg == "midleg":
                leg_xz = pd.read_csv(INPUT_DIR + "Tracker2_offset.txt", sep=",", header=0, names=["x_column", "z_column"])
                leg_xy = pd.read_csv(INPUT_DIR + "Tracker5_offset.txt", sep=",", header=0, names=["x_column", "y_column"])
                title = "Midleg"
            
            if leg == "foreleg":
                leg_xz = pd.read_csv(INPUT_DIR + "Tracker3_offset.txt", sep=",", header=0, names=["x_column", "z_column"])
                leg_xy = pd.read_csv(INPUT_DIR + "Tracker6_offset.txt", sep=",", header=0, names=["x_column", "y_column"])
                title = "Foreleg"
            
            if mode == "1_combine_3D":
                leg_xyz = pd.concat([leg_xy, leg_xz['z_column']], axis=1)
                plot_3d_scatter(leg_xyz, title, mode)
            
            if mode == "2_scale_3D":
                # Normalize data        
                data_max = leg_xyz.max()
                data_min = leg_xyz.iloc[0]
                leg_xyz = (leg_xyz - data_min) / (data_max - data_min)
                plot_3d_scatter(leg_xyz, title, mode)

            if mode == "3_clean_3D":
                # Clean everything that is under zero in z_column
                leg_xyz = leg_xyz.loc[leg_xyz["z_column"] >= 0]
                # print(leg_xyz.shape)
                # print(leg_xyz)
                plot_3d_scatter(leg_xyz, title, mode)
                
            if mode == "4_resample_3D":
                leg_xyz = resample_data(leg_xyz)
                plot_3d_scatter(leg_xyz, title, mode)

            if mode == "5_close_path_3D":
                leg_xyz = close_path(leg_xyz)
                print(leg)
                plot_3d_scatter(leg_xyz, title, mode)
            
            if mode == "coppelia_format":
                leg_xyz['qx_column'] = 0.0
                leg_xyz['qy_column'] = 0.0
                leg_xyz['qz_column'] = 0.0
                leg_xyz['qw_column'] = 1.0
                leg_xyz['y_column'] = -leg_xyz['y_column']
                leg_xyz.columns = ['x_column', 'y_column', 'z_column', 'qx_column', 'qy_column', 'qz_column', 'qw_column']
                                    
        leg_xyz.to_csv(OUTPUT_DIR + "/Final_Traj/" + title + "_3D.csv", index=False)
            
        for factor in factors:
            if not os.path.exists(OUTPUT_DIR + "/Final_Traj/Scaled_" + str(int(factor*1000)) + "/"):
                os.makedirs(OUTPUT_DIR + "/Final_Traj/Scaled_" + str(int(factor*1000)) + "/")
            leg_xyz_scaled = scale_scalar(leg_xyz, factor)
            leg_xyz_scaled.to_csv(OUTPUT_DIR + "/Final_Traj/Scaled_" + str(int(factor*1000)) + "/" + title + "_3D_" + str(int(factor*1000)) + ".csv", index=False)
            
        
        

    
if __name__ == "__main__":
    main()
