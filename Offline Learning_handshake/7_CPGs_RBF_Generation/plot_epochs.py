import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import math
import copy

DIR = "7_CPGs_RBF_Generation/"
NAME = "Hindleg_simpleCurveTraj14-5-14-31-7-data_resampled40_errors.csv"

def main():
    # Read the .csv file
    data = pd.read_csv(DIR + NAME)

    # Get the column names
    columns = data.columns
    # Define the size of the window plot
    plt.figure(figsize=(6, 4))
    plt.grid(True, which='both', axis='both', linestyle='--')
    plt.xticks(np.arange(0, len(data), 1000))
    plt.yticks(np.arange(min(data.min()), max(data.max()), 1))
    
    # Plot each column
    for column in columns:
        
        non_zero_data = data[column][data[column] != 0]
        max_error = round(non_zero_data.iloc[-1],4)
        lbl = column + " max error = " + str(max_error) + " at epoch " + str(non_zero_data.index[-1])
        plt.plot(non_zero_data, label=lbl)
        plt.plot(non_zero_data.index[-1], non_zero_data.iloc[-1], marker='o', color=plt.gca().lines[-1].get_color())
    
    plt.title('Error/epoch_' + NAME[:NAME.index("_")])
    plt.legend()
    plt.show()
    

if __name__ == "__main__":
    main()

