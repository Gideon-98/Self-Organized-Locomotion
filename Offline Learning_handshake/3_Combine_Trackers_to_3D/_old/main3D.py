import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Import the 3D projection module
import matplotlib.ticker as ticker
import pandas as pd
import numpy as np

INPUT_DIR = "3D trajectory data/Trajectories"
OUTPUT_DIR = "plots"
OUTPUT_DIR_3D = "plots/_3D"
OUTPUT_OFFSET_CSV_3D = "3D trajectory data/Trajectories/3D_Traj"
OUTPUT_FINAL_CSV = "3D trajectory data/Trajectories/Final_Traj"
HINDLEG_PATH_XZ = "3D trajectory data/Trajectories/Offset_Traj/Tracker1_offset.txt"
HINDLEG_PATH_XY = "3D trajectory data/Trajectories/Offset_Traj/Tracker4_offset.txt"
MIDLEG_PATH_XZ  = "3D trajectory data/Trajectories/Offset_Traj/Tracker2_offset.txt"
MIDLEG_PATH_XY  = "3D trajectory data/Trajectories/Offset_Traj/Tracker5_offset.txt"
FORELEG_PATH_XZ = "3D trajectory data/Trajectories/Offset_Traj/Tracker3_offset.txt"
FORELEG_PATH_XY = "3D trajectory data/Trajectories/Offset_Traj/Tracker6_offset.txt"

def plot_3d_scatter(dataframe, title):
    x, y, z = dataframe['x_column'], dataframe['y_column'], dataframe['z_column']
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z, label=f'Data from {title}')
    #ax.plot(x, y, z, label=f'Data from {title}')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.legend()
    plt.title(f'3D Plot for Data from {title}')
    plt.savefig(OUTPUT_DIR_3D + "/Not_Cleaned/" + title + ".png")
    plt.show()

def main():
    hindleg_xz = pd.read_csv(HINDLEG_PATH_XZ, sep=",", header=0, names=["x_column", "z_column"])
    hindleg_xy = pd.read_csv(HINDLEG_PATH_XY, sep=",", header=0, names=["x_column", "y_column"])
    
    midleg_xz = pd.read_csv(MIDLEG_PATH_XZ, sep=",", header=0, names=["x_column", "z_column"])
    midleg_xy = pd.read_csv(MIDLEG_PATH_XY, sep=",", header=0, names=["x_column", "y_column"])
    
    foreleg_xz = pd.read_csv(FORELEG_PATH_XZ, sep=",", header=0, names=["x_column", "z_column"])
    foreleg_xy = pd.read_csv(FORELEG_PATH_XY, sep=",", header=0, names=["x_column", "y_column"])

    # Merge hindleg
    hindleg_xyz = pd.concat([hindleg_xy, hindleg_xz['z_column']], axis=1)
    
    # Merge midleg
    midleg_xyz = pd.concat([midleg_xy, midleg_xz['z_column']], axis=1)
    
    # Merge foreleg
    foreleg_xyz = pd.concat([foreleg_xy, foreleg_xz['z_column']], axis=1)
    
    # Scale hindleg
    hindleg_max = hindleg_xyz.max()
    hindleg_min = hindleg_xyz.min()
    hindleg_xyz = (hindleg_xyz - hindleg_min) / (hindleg_max - hindleg_min)

    # Scale midleg
    midleg_max = midleg_xyz.max()
    midleg_min = midleg_xyz.min()
    midleg_xyz = (midleg_xyz - midleg_min) / (midleg_max - midleg_min)

    # Scale foreleg
    foreleg_max = foreleg_xyz.max()
    foreleg_min = foreleg_xyz.min()
    foreleg_xyz = (foreleg_xyz - foreleg_min) / (foreleg_max - foreleg_min)

    # Check the existence of output directory
    if not os.path.exists(OUTPUT_OFFSET_CSV_3D):
        os.mkdir(OUTPUT_OFFSET_CSV_3D)

    # Specify output file paths
    hindleg_output_path = os.path.join(OUTPUT_OFFSET_CSV_3D, "hindleg_3d.csv")
    midleg_output_path = os.path.join(OUTPUT_OFFSET_CSV_3D, "midleg_3d.csv")
    foreleg_output_path = os.path.join(OUTPUT_OFFSET_CSV_3D, "foreleg_3d.csv")

    # Write DataFrames to CSV files
    hindleg_xyz.to_csv(hindleg_output_path, index=False)
    midleg_xyz.to_csv(midleg_output_path, index=False)
    foreleg_xyz.to_csv(foreleg_output_path, index=False)

    # Check the existence of output directory
    if not os.path.exists(OUTPUT_FINAL_CSV):
        os.mkdir(OUTPUT_FINAL_CSV)


    # Plot Data 3D for Hindleg
    plot_3d_scatter(hindleg_xyz, 'Hindleg_3D')

    # Plot Data 3D for Midleg
    plot_3d_scatter(midleg_xyz, 'Midleg_3D')

    # Plot Data 3D for Foreleg
    plot_3d_scatter(foreleg_xyz, 'Foreleg_3D')


    
if __name__ == "__main__":
    main()
