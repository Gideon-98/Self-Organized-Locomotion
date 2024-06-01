import os
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import pandas as pd
import numpy as np

INPUT_DIR = "3D trajectory data/Trajectories"
OUTPUT_DIR = "plots"
STEP = 0 #0.01


def plot_data(file_path, plot_path, mode, data):
    plt.plot(data["x_column"], data["y_column"], label="Transformed Data")
    plt.title(f"Plot for {os.path.basename(file_path)}" + f" ({mode})")
    plt.grid(True)
    plt.gca().xaxis.set_major_locator(ticker.MultipleLocator(100))
    plt.gca().yaxis.set_major_locator(ticker.MultipleLocator(100))
    plt.xlabel("X-axis Label")
    plt.ylabel("Y-axis Label")
    plt.legend()
    plt.savefig(plot_path)
    #plt.show()

def mirror_data(data):
    mean_y = data["y_column"].mean()
    for index, row in data.iterrows():
        data.loc[index, "y_column"] = mean_y - (row["y_column"] - mean_y)
    return data

def offset_data(data):
    mean_y = data["y_column"].mean()
    data = mirror_data(data)
    for index, row in data.iterrows():
        data.loc[index, "y_column"] = row["y_column"] - mean_y
    return data

def clean_data(data):
    data = offset_data(data)
    data = data.loc[data["y_column"] >= 0]
    
    first_x_value = data["x_column"].iloc[0]
    last_x_value = data["x_column"].iloc[-1]
    
    new_row = pd.DataFrame({'x_column': [first_x_value], 'y_column': [0]})
    data = pd.concat([new_row, data]).reset_index(drop=True)
    
    new_row = pd.DataFrame({'x_column': [last_x_value], 'y_column': [0]})
    data = pd.concat([data, new_row])
    return data

def scale_data(data):
    data = clean_data(data)
    min = data.min()
    max = data.max()
    data = (data - min) / (max - min)
    return data

def lateral_x_decrescent_pad(data):
    data = scale_data(data)
    data_copy = data.copy()
    # Reverse the order of the copy
    data_copy = data_copy.iloc[::-1]
    data_copy["y_column"] = 0
    # Concatenate the original data and the modified copy
    data = pd.concat([data, data_copy], ignore_index=True)

    return data

def lateral_x_decrescent_pad_step(data):
    data = scale_data(data)
    # Calculate the step
    step = (data["x_column"].iloc[-1] - data["x_column"].iloc[0]) / 100

    # Get the last and first values
    last_value = data["x_column"].iloc[-2] 
    first_value = data["x_column"].iloc[0] 

    # Generate the "x_column" values
    x_column_values = np.arange(last_value, first_value - step, -step)

    # Create a new DataFrame
    new_data = pd.DataFrame({
        'x_column': x_column_values,
        'y_column': [0] * len(x_column_values)  # Replace 0 with the values you want for 'y_column'
    })
    
    data = pd.concat([data, new_data])

    return data

def process_file(file_name, mode, plot_path):
    file_path = os.path.join(INPUT_DIR, file_name)
    data = pd.read_csv(file_path, sep=",", header=0, names=["x_column", "y_column"])
    original_data = data.copy()
# # mirroring the data
#     mirr_data = mirror_data(data)
# # offsetting the data
#     offset_data = offset_data(mirr_data)
# # cleaning the data    
#     clean_data = clean_data(offset_data)
# # scaling the data
#     scaled_data = scale_data(clean_data)
# # padding the data
#     final_data = lateral_x_decrescent_pad(scaled_data)
       
    if mode == "original":
        plot_data(file_path, plot_path, mode, data)
    elif mode == "mirrored":
        data = mirror_data(data)
        plot_data(file_path, plot_path, mode, data)
    elif mode == "both":
        data = mirror_data(data)
        plt.plot(original_data["x_column"], original_data["y_column"], label="Original Data")
        plt.axhline(data["y_column"].mean(), color="r", linestyle="--", label="Mean")
        plot_data(file_path, plot_path, mode, data)
    elif mode == "offset":
        data = offset_data(data)
        plot_data(file_path, plot_path, mode, data)
        if not os.path.exists(INPUT_DIR + "/Offset_Traj"):
            os.mkdir(INPUT_DIR + "/Offset_Traj")
        output_file_path = os.path.join(INPUT_DIR + "/Offset_Traj", os.path.basename(file_path).replace(".txt", "_offset.txt"))
        data.to_csv(output_file_path, index=False, header=False, sep=",")
    elif mode == "clean":
        data = clean_data(data)
        plot_data(file_path, plot_path, mode, data)
    elif mode == "scaled":
        data = scale_data(data) 
        plot_data(file_path, plot_path, mode, data)
    elif mode == "final":
        data = lateral_x_decrescent_pad_step(data)
        plot_data(file_path, plot_path, mode, data)
        if not os.path.exists(INPUT_DIR + "/Final_Traj_old"):
            os.mkdir(INPUT_DIR + "/Final_Traj_old")
        output_file_path = os.path.join(INPUT_DIR + "/Final_Traj_old", os.path.basename(file_path).replace(".txt", "_final.txt"))
        data.to_csv(output_file_path, index=False, header=False, sep=",")
        
    else:
        print("Invalid mode2:" + mode + " for file: " + file_name)

    plt.close()

def txt2csv(file_name):
    file_path = os.path.join(INPUT_DIR, file_name)
    data = pd.read_csv(file_path, sep=",", header=0, names=["x_column", "y_column", "z_column"])
    output_file_path = os.path.join(
        INPUT_DIR + "/csv",
        os.path.basename(file_path).replace(".txt", ".csv"),
    )
    data.to_csv(output_file_path, index=False, header=False, sep=",")
def main():
    modes = ["original", "mirrored", "both", "offset", "clean", "scaled", "final"]

    # Now I want to iterate over all the files in the 'Trajectories' folder
    for file in os.listdir(INPUT_DIR):
        if file.endswith(".txt"):
            plot_path = os.path.join(
                OUTPUT_DIR,
                "1_original",
                os.path.basename(file).replace(".txt", "_plot.png"),
            )
            if not os.path.exists(OUTPUT_DIR + "/1_original"):
                os.mkdir(OUTPUT_DIR + "/1_original")

            for mode in modes:
                if mode == "original":
                    pass
                elif mode == "mirrored":
                    if not os.path.exists(OUTPUT_DIR + "/2_mirrored"):
                        os.mkdir(OUTPUT_DIR + "/2_mirrored")
                    plot_path = os.path.join(
                        OUTPUT_DIR,
                        "2_mirrored",
                        os.path.basename(file).replace(".txt", "_mirr_plot.png"),
                    )
                elif mode == "both":
                    if not os.path.exists(OUTPUT_DIR + "/3_both"):
                        os.mkdir(OUTPUT_DIR + "/3_both")
                    plot_path = os.path.join(
                        OUTPUT_DIR,
                        "3_both",
                        os.path.basename(file).replace(".txt", "_both_plot.png"),
                    )
                elif mode == "offset":
                    if not os.path.exists(OUTPUT_DIR + "/4_offset"):
                        os.mkdir(OUTPUT_DIR + "/4_offset")
                    plot_path = os.path.join(
                        OUTPUT_DIR,
                        "4_offset",
                        os.path.basename(file).replace(".txt", "_offset_plot.png"),
                    )
                elif mode == "clean":
                    if not os.path.exists(OUTPUT_DIR + "/5_clean"):
                        os.mkdir(OUTPUT_DIR + "/5_clean")
                    plot_path = os.path.join(
                        OUTPUT_DIR,
                        "5_clean",
                        os.path.basename(file).replace(".txt", "_clean_plot.png"),
                    )
                elif mode == "scaled":
                    if not os.path.exists(OUTPUT_DIR + "/6_scaled"):
                        os.mkdir(OUTPUT_DIR + "/6_scaled")
                    plot_path = os.path.join(
                        OUTPUT_DIR,
                        "6_scaled",
                        os.path.basename(file).replace(".txt", "_scaled_plot.png"),
                    )
                elif mode == "final":
                    if not os.path.exists(OUTPUT_DIR + "/7_final/"):
                        os.mkdir(OUTPUT_DIR + "/7_final/")
                    plot_path = os.path.join(
                        OUTPUT_DIR,
                        "7_final/",
                        os.path.basename(file).replace(".txt", "_final_plot.png"),
                    )
                else:
                    print("Invalid mode1" + mode + " for file: " + file)
                process_file(file, mode, plot_path)


if __name__ == "__main__":
    main()
