import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import argparse
import os
def parse_and_plot(file_path, mode):
    df = pd.read_csv(file_path)
    df.columns = df.columns.str.strip()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    if mode in ['real', 'both']:
        df_real = df.dropna(subset=['real_lat', 'real_lon', 'real_alt_m'])
        if not df_real.empty:
            ax.plot(
                df_real['real_lon'], df_real['real_lat'], df_real['real_alt_m'],
                label='Real Location', color='blue', marker='o'
            )
            print("Plotted real data ({} points)".format(len(df_real)))
        else:
            print("No valid real data found.")

    if mode in ['spoof', 'both']:
        required_cols = ['spoof_lat', 'spoof_lon', 'spoof_alt_m']
        if all(col in df.columns for col in required_cols):
            df_spoof = df.dropna(subset=required_cols)
            if not df_spoof.empty:
                ax.plot(
                    df_spoof['spoof_lon'], df_spoof['spoof_lat'], df_spoof['spoof_alt_m'],
                    label='Spoofed Location', color='red', marker='x'
                )
                print("Plotted spoofed data ({} points)".format(len(df_spoof)))
            else:
                print("Spoof columns exist but no valid spoofed data.")
        else:
            print("Spoof columns missing in the file.")

    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.set_zlabel('Altitude (m)')
    ax.legend()
    plt.title("3D Trajectory: Real vs. Spoofed GPS" if mode == 'both' else f"3D Trajectory: {mode.capitalize()} GPS")
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="3D plot of real and/or spoofed GPS trajectories from CSV.")
    parser.add_argument("file", help="Path to the CSV file.")
    parser.add_argument("--mode", choices=["real", "spoof", "both"], default="both",
                        help="Which data to plot: real, spoof, or both. Default is 'both'.")

    args = parser.parse_args()

    if not os.path.isfile(args.file):
        print(f"File not found: {args.file}")
        sys.exit(1)
    parse_and_plot(args.file, args.mode)
