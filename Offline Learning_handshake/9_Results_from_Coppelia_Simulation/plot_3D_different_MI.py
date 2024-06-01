import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.stats import norm

# List of CSV files to read
CSV_FILES = ['MI_01.csv', 'MI_015.csv', 'MI_02.csv', 'MI_025.csv', 'MI_03.csv']

# List of columns to plot
COLUMNS_TO_PLOTS_1 = ["x_toe_1","y_toe_1","z_toe_1"]
COLUMNS_TO_PLOTS_2 = ["x_toe_2","y_toe_2","z_toe_2"] 
COLUMNS_TO_PLOTS_3 = ["x_toe_3","y_toe_3","z_toe_3"]

PLOTS = [COLUMNS_TO_PLOTS_1, COLUMNS_TO_PLOTS_2, COLUMNS_TO_PLOTS_3]
LEGS = ['ForeLeg', 'MidLeg', 'HindLeg']

COLORS = plt.rcParams['axes.prop_cycle'].by_key()['color']

def plot_3D(ax, dir, leg):
    if leg == 'ForeLeg':
        columns_to_plot = PLOTS[0]
    if leg == 'MidLeg':
        columns_to_plot = PLOTS[1]
    if leg == 'HindLeg':
        columns_to_plot = PLOTS[2]
    
    # Loop over the CSV files and save the data in a DataFrame
    for i, csv_file in enumerate(CSV_FILES):
        df = pd.read_csv(dir + csv_file)
        df = df.iloc[200:300]  # Slice the DataFrame
        ax.plot(df[columns_to_plot[0]], df[columns_to_plot[1]], df[columns_to_plot[2]], label=csv_file[:-4], color=COLORS[i])

    # Set the labels of the axes
    ax.set_xlabel(columns_to_plot[0])
    ax.set_ylabel(columns_to_plot[1])
    ax.set_zlabel(columns_to_plot[2])

    # Add a legend
    plt.legend()
    
    # Set the title of the plot
    if columns_to_plot == PLOTS[0]:
        plt.title('3D plot of the foot position ForeLeg')
        
    if columns_to_plot == PLOTS[1]:
        plt.title('3D plot of the foot position MidLeg')
        
    if columns_to_plot == PLOTS[2]:
        plt.title('3D plot of the foot position HindLeg')
    
    plt.savefig(dir+'3D_plot_MI_'+leg+'.png')

def plot_2D(ax2, dir, leg):
    if leg == 'ForeLeg':
        columns_to_plot = PLOTS[0]
        plt.title('2D plot of the foot position '+leg)
    if leg == 'MidLeg':
        columns_to_plot = PLOTS[1]
        plt.title('2D plot of the foot position '+leg)
    if leg == 'HindLeg':
        columns_to_plot = PLOTS[2]
        plt.title('2D plot of the foot position '+leg)
    
    # Loop over the CSV files and save the data in a DataFrame
    for i,csv_file in enumerate(CSV_FILES):
        df = pd.read_csv(dir + csv_file)
        df = df.iloc[200:300]  # Slice the DataFrame
        ax2.plot(df[columns_to_plot[0]], df[columns_to_plot[1]], label=csv_file[:-4], color=COLORS[i])

    # Set the labels of the axes
    ax2.set_xlabel(columns_to_plot[0])
    ax2.set_ylabel(columns_to_plot[1])
    
    # Add a legend
    plt.legend()
    plt.savefig(dir+'2D_plot_MI_'+leg+'.png')

def plot_error(ax3, dir, leg):
    if leg == 'ForeLeg':
        columns_to_plot = 'error_1'
        plt.title('Error on MI for '+leg)
    if leg == 'MidLeg':
        columns_to_plot = 'error_2'
        plt.title('Error on MI for '+leg)
    if leg == 'HindLeg':
        columns_to_plot = 'error_3'
        plt.title('Error on MI for '+leg)
    
    csv_ordered = ['MI_01.csv', 'MI_015.csv', 'MI_02.csv', 'MI_03.csv', 'MI_025.csv']
    #csv_ordered = []
    for i,csv_file in enumerate(csv_ordered):
        df = pd.read_csv(dir + csv_file)
        df = df.iloc[200:300]  # Slice the DataFrame
        
        #mean_error, std_error = norm.fit(df[columns_to_plot])
        meandata = df[columns_to_plot].tolist()
        negative = (-1*df[columns_to_plot]).tolist()
        meandata = meandata+negative
        meandata = np.array(meandata)
        mean_error = meandata.mean()
        std_error = meandata.std()
        
        x_axis = np.linspace(0, 35 , 100)
        plt.plot(x_axis,4*norm.pdf(x_axis,mean_error,std_error), label=csv_file[:-4], color=COLORS[i],linewidth=2.5)
        plt.hist(df[columns_to_plot], density=True, color=COLORS[i], alpha=0.7)
        
        #avg_error = df[columns_to_plot].mean()
        #ax3.axvline(avg_error, linestyle='--', label=csv_file, color=np.random.rand(3,))
        #ax3.plot(df[df.columns[0]],df[columns_to_plot], label=csv_file)    
    
    # Set the labels of the axes
    ax3.set_xlabel('Error')
    ax3.set_xlim(0,35)
    ax3.set_ylabel('Frequency')

    # Add a legend
    plt.legend()
    plt.savefig(dir+'Error_plot_MI_'+leg+'.png')
    
def main():
    
    dir = '9_Results_from_Coppelia_Simulation/'

    for leg in LEGS:
        # Create a figure and a 3D Axes
        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # plot_3D(ax, dir, leg)
        # # Show the plot
        # plt.show()
        
        # fig2 = plt.figure()
        # ax2 = fig2.add_subplot(111)
        # plot_2D(ax2, dir, leg)
        # # Show the plot
        # plt.show()
        
        fig3 = plt.figure(figsize=(6, 4))
        ax3 = fig3.add_subplot(111)
        plot_error(ax3, dir, leg)
        # Show the plot
        plt.show()
        
        
if __name__ == "__main__":
    main()
    