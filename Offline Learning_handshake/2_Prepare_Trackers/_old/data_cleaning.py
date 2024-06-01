import os
import argparse
import matplotlib.pyplot as plt
import pandas as pd

class TrajectoryAnalyzer:
    def __init__(self, txt_path='3D trajectory data/Trajectories', output_dir='plots'):
        self.txt_path = txt_path
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)

    def plot_trajectory(self, data, save_plot=False, custom_output_dir=None):
        plt.plot(data['x_column'], data['y_column'])
        plt.title(f'Trajectory Plot')
        plt.xlabel('X-axis Label')
        plt.ylabel('Y-axis Label')

        if save_plot:
            output_dir = custom_output_dir or self.output_dir
            plot_output_path = os.path.join(output_dir, 'trajectory_plot.png')
            plt.savefig(plot_output_path)
            print(f"Plot saved to: {plot_output_path}")

        plt.show()
        plt.clf()

    def analyze_data(self, data):
        # Placeholder for data analysis logic
        pass

    def process_files(self, mode='plot', num_files=None, save_plot=False, custom_output_dir=None):
        files_to_process = os.listdir(self.txt_path)

        if num_files:
            files_to_process = files_to_process[:num_files]

        for filename in files_to_process:
            if filename.endswith('.txt'):
                file_path = os.path.join(self.txt_path, filename)
                data = pd.read_csv(file_path, header=0, names=['x_column', 'y_column'])

                if mode == 'plot':
                    self.plot_trajectory(data, save_plot, custom_output_dir)
                elif mode == 'analyze':
                    self.analyze_data(data)
                elif mode == 'save&plot':
                    self.plot_trajectory(data, True, custom_output_dir)
                # Add more modes as needed

def main():
    parser = argparse.ArgumentParser(description='Trajectory Analysis Script')
    parser.add_argument('mode', choices=['plot', 'analyze', 'save&plot'], help='Mode of analysis: "plot", "analyze" or "save&plot"')
    parser.add_argument('--directory', help='Directory containing trajectory files')
    parser.add_argument('--num-files', type=int, help='Number of files to analyze')
    parser.add_argument('--save-plot', action='store_true', help='Save plots to the output directory')
    parser.add_argument('--output-dir', help='Custom output directory for saving plots')

    args = parser.parse_args()

    if args.directory:
        txt_path = args.directory
    else:
        txt_path = '3D trajectory data/Trajectories'

    analyzer = TrajectoryAnalyzer(txt_path)

    analyzer.process_files(
        mode=args.mode,
        num_files=args.num_files,
        save_plot=args.save_plot,
        custom_output_dir=args.output_dir
    )

if __name__ == "__main__":
    main()
