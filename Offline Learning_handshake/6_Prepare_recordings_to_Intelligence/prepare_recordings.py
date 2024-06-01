import os
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D  # Import the 3D projection module
import matplotlib.ticker as ticker
import pandas as pd
import numpy as np

FILE = "simpleCurveTraj14-5-14-31-7-data.csv"#"simpleCurveTraj9-5-15-34-11-data.csv"#"simpleCurveTraj9-5-15-9-31-data.csv" #"simpleCurveTraj9-5-9-47-50-data.csv" #"simpleCurveTraj8-5-9-14-55-data.csv" # "simpleCurveTraj29-4-17-18-47-data.csv"


INPUT_DIR = "5_Coppelia_recordings/"
OUTPUT_DIR = "7_CPGs_RBF_Generation/CSVS/"

PLOT = True
N_RESAMPLE = 40

def plot_data(dataframe,title,n):
    x, y, z = dataframe.iloc[:, 3], dataframe.iloc[:, 4], dataframe.iloc[:, 5]
    joint_1, joint_2, joint_3, force = dataframe.iloc[:, 0], dataframe.iloc[:, 1], dataframe.iloc[:, 2], dataframe.iloc[:, -1]
    max_x = find_max(x,n)
    min_z = find_min(z,n)
    
    # plot the seven graphs as subplots
    fig, axs = plt.subplots(4, 2, figsize=(15, 15))
    fig.suptitle(title)
    axs[0, 0].plot(x)
    axs[0, 0].set_title('X')
    for i in max_x:
        axs[0, 0].axvline(x=i, color='r', linestyle='--')
        
    axs[1, 0].plot(y)
    axs[1, 0].set_title('Y')
    for i in max_x:
        axs[1, 0].axvline(x=i, color='r', linestyle='--')
        
    axs[2, 0].plot(z)
    axs[2, 0].set_title('Z')
    for i in min_z:
        axs[2, 0].axvline(x=i, color='g', linestyle='--')
    for i in max_x:
        axs[2, 0].axvline(x=i, color='r', linestyle='--')

#        joints

    axs[0, 1].plot(joint_1)
    axs[0, 1].set_title('Joint 1')
    # for i in min_z:
    #     axs[0, 1].axvline(x=i, color='g', linestyle='--')
    # for i in max_x:
    #     axs[0, 1].axvline(x=i, color='r', linestyle='--')
        
    axs[1, 1].plot(joint_2)
    axs[1, 1].set_title('Joint 2')
    # for i in min_z:
    #     axs[1, 1].axvline(x=i, color='g', linestyle='--')
    # for i in max_x:
    #     axs[1, 1].axvline(x=i, color='r', linestyle='--')

    axs[2, 1].plot(joint_3)
    axs[2, 1].set_title('Joint 3')
    # for i in min_z:
    #     axs[2, 1].axvline(x=i, color='g', linestyle='--')
    # for i in max_x:
    #     axs[2, 1].axvline(x=i, color='r', linestyle='--')
    
    axs[3, 0].plot(force)
    axs[3, 0].set_title('Force')
    for i in min_z:
        axs[3, 0].axvline(x=i, color='g', linestyle='--')
    for i in max_x:
        axs[3, 0].axvline(x=i, color='r', linestyle='--')


    axs[3, 1].axis('off')
    plt.show()
    
    # plot the 3D scatter plot
    fig = plt.figure(figsize=(10, 10))
    ax = plt.axes(projection='3d')
    ax.scatter(x, y, z, label=f'Data from {title}')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.legend()
    
    plt.show()
    
    # plot the 3D line plot
    #fig = plt.figure(figsize=(10, 10))
    #ax = plt.axes(projection='3d')
    #ax.plot3D(x, y, z, label = f'Data from {title}')
    #ax.set_xlabel('X Label')
    #ax.set_ylabel('Y Label')
    #ax.set_zlabel('Z Label')
    #plt.show()
    
def find_min(dataframe, n):
    index_array = []
    for i in range(n, len(dataframe) - n):
        curr = dataframe.iloc[i]
        window = dataframe.iloc[i-n : i+n+1]
        if curr == window.min():
            index_array.append(i)
                # print("Min value is: ", curr)
                # print("Index is: ", i)
    #print("Index mins_z is: ", index_array)
    # plt.plot(dataframe)
    # for index in index_array:
        # plt.axvline(x=index, color='r', linestyle='--')
    # plt.title("mins")
    # plt.show()
    return index_array

def find_max(dataframe, n):
    index_array = []
    for i in range(n, len(dataframe) - n):
        curr = dataframe.iloc[i]
        window = dataframe.iloc[i-n : i+n+1]
        if curr == window.max():
            index_array.append(i)
                # print("Min value is: ", curr)
                # print("Index is: ", i)
    #print("Index max_x is: ", index_array)
    # plt.plot(dataframe)
    # for index in index_array:
        # plt.axvline(x=index, color='r', linestyle='--')
    # plt.title("maxs")
    # plt.show()
    return index_array
    
def force_creation(dataframe):
    mins_z = find_min(dataframe.iloc[:, 5],10) # minimums in z, that tells us at which index the period restarts
    mins_x = find_max(dataframe.iloc[:, 3],20) # maximum in x, that tells us at which index the force is applied
    merged_vector = np.concatenate((mins_x, mins_z))
    sorted_vector = np.sort(merged_vector)
    # print(sorted_vector)
    # print(len(sorted_vector))
    
    force_array = []
    zero = True
    
    for i in range(len(dataframe)):
        if i in sorted_vector:
            if zero:
                zero = False
            else:
                zero = True
        
        if zero:
            force_array.append(0)
        else:
            force_array.append(1)
    
    dataframe = dataframe.assign(force=force_array)
    
    # plt.plot(force_array)
    # plt.title("force")
    # plt.show()

    return dataframe

def resample_data(dataframe):
    
    downsampled_df = pd.DataFrame()

    for column in dataframe.columns:
        if dataframe[column].dtype != 'object':  # Skip object type columns
            downsampled_column = np.interp(np.linspace(0, len(dataframe[column]) - 1, N_RESAMPLE),
                                            np.arange(len(dataframe[column])),
                                            dataframe[column].values)
            downsampled_df[column] = downsampled_column

    return downsampled_df

def main():
    file = FILE
    
    input_file = INPUT_DIR + FILE
    file = pd.read_csv(input_file) #file = pd.read_csv(INPUT_DIR + "simpleCurveTraj29-4-17-18-47-data.csv")
    tag_input = input_file.split("/")[-1][:-4]
    
    file.drop(file.columns[-1], axis=1, inplace=True)

    legs = ["hindleg", "midleg", "foreleg"]
    if not os.path.exists(OUTPUT_DIR + "/"+ tag_input):
        os.makedirs(OUTPUT_DIR + "/"+ tag_input)    
    
    for leg in legs:
        if leg == "hindleg":
            hindleg_df = file.iloc[:, 12:]
            title = "Hindleg"
            hindleg_df = force_creation(hindleg_df)
            hindleg_df.to_csv(OUTPUT_DIR + "/"+ tag_input +"/" + title + "_" + tag_input + ".csv", index=False)
            first_z_min = find_min(hindleg_df.iloc[:, 5],10)[0]
            hindleg_df_1_period = hindleg_df.iloc[0:first_z_min, :]
            
            #drop the columns I do not need
            #hindleg_df_1_period = hindleg_df_1_period.drop(columns=[hindleg_df_1_period.columns[3], hindleg_df_1_period.columns[4],hindleg_df_1_period.columns[5]])
            hindleg_df_resampled = resample_data(hindleg_df_1_period)
            hindleg_df_resampled.to_csv(OUTPUT_DIR + "/"+ tag_input +"/" + title + "_" + tag_input + "_resampled" + str(N_RESAMPLE) + ".csv", index=False)
            
            if PLOT:
                plot_data(hindleg_df, title, n=20)
                plot_data(hindleg_df_resampled, title,n=1)
            
        
        if leg == "midleg":
            midleg_df = file.iloc[:, 6:12]
            title = "Midleg"
            midleg_df = force_creation(midleg_df)
            midleg_df.to_csv(OUTPUT_DIR + "/"+ tag_input +"/" + title + "_" + tag_input + ".csv", index=False)
            
            first_z_min = find_min(midleg_df.iloc[:, 5],10)[0]
            midleg_df_1_period = midleg_df.iloc[0:first_z_min, :]
            
            #drop the columns I do not need
            #midleg_df_1_period = midleg_df_1_period.drop(columns=[midleg_df_1_period.columns[3], midleg_df_1_period.columns[4],midleg_df_1_period.columns[5]])
            midleg_df_resampled = resample_data(midleg_df_1_period)
            midleg_df_resampled.to_csv(OUTPUT_DIR + "/"+ tag_input +"/"+ title + "_" + tag_input + "_resampled" + str(N_RESAMPLE) + ".csv", index=False, header=True)
            
            if PLOT:
                plot_data(midleg_df, title, n=20)
                plot_data(midleg_df_resampled, title, n=5)
        
        if leg == "foreleg":
            foreleg_df = file.iloc[:, :6]
            title = "Foreleg"    
            foreleg_df = force_creation(foreleg_df)
            foreleg_df.to_csv(OUTPUT_DIR + "/"+ tag_input +"/" + title + "_" + tag_input + ".csv", index=False)
            first_z_min = find_min(foreleg_df.iloc[:, 5],10)[0]
            foreleg_df_1_period = foreleg_df.iloc[0:first_z_min, :]
            #drop the columns I do not need
            #foreleg_df_1_period = foreleg_df_1_period.drop(columns=[foreleg_df_1_period.columns[3], foreleg_df_1_period.columns[4],foreleg_df_1_period.columns[5]])
            foreleg_df_resampled = resample_data(foreleg_df_1_period)
            foreleg_df_resampled.to_csv(OUTPUT_DIR + "/"+ tag_input +"/" + title + "_" + tag_input + "_resampled" + str(N_RESAMPLE) + ".csv", index=False)
            
            if PLOT:
                plot_data(foreleg_df, title, n=20)
                plot_data(foreleg_df_resampled, title, n=1)


if __name__ == "__main__":
    main()
