#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys



def load_data(data_dir, session_id):
    formation_file = Path(data_dir) / f"formation_{session_id}.csv"
    racing_file = Path(data_dir) / f"racing_{session_id}.csv"
    
    formation_df = None
    racing_df = None
    
    if formation_file.exists():
        formation_df = pd.read_csv(formation_file)
    
    if racing_file.exists():
        racing_df = pd.read_csv(racing_file)
        
        if racing_df is not None and len(racing_df) > 10:
            first_10 = racing_df.head(10)
            median_x = first_10['pos_x'].median()
            median_y = first_10['pos_y'].median()
            
            racing_df = racing_df[
                (abs(racing_df['pos_x'] - median_x) < 20) &
                (abs(racing_df['pos_y'] - median_y) < 20)
            ].reset_index(drop=True)
    
    return formation_df, racing_df





def plot_speed_over_time(df, output_dir):
    if df is None:
        return
    
    time = df['timestamp'] - df['timestamp'].iloc[0]
    
    plt.figure(figsize=(12, 5))
    plt.plot(time, df['vel_linear'], 'b-', linewidth=0.8, alpha=0.7)
    plt.xlabel('Time (s)')
    plt.ylabel('Linear Speed (m/s)')
    plt.title('Speed Profile Over Time')
    plt.grid(True, alpha=0.3)
    
    output_file = Path(output_dir) / 'speed_over_time.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.close()

def plot_steering_analysis(df, output_dir):
    if df is None:
        return
    
    time = df['timestamp'] - df['timestamp'].iloc[0]
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    ax1.plot(time, df['steering_cmd'], 'r-', linewidth=0.8, alpha=0.7)
    ax1.set_ylabel('Steering Command (rad/s)')
    ax1.set_title('Steering Control Over Time')
    ax1.grid(True, alpha=0.3)
    
    ax2.plot(time, df['vel_angular'], 'g-', linewidth=0.8, alpha=0.7)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Angular Velocity (rad/s)')
    ax2.set_title('Actual Angular Velocity')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_file = Path(output_dir) / 'steering_analysis.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.close()

def plot_clearance_distances(df, output_dir):
    if df is None:
        return
    
    time = df['timestamp'] - df['timestamp'].iloc[0]
    
    plt.figure(figsize=(12, 6))
    plt.plot(time, df['left_dist'], 'b-', label='Left', linewidth=0.8, alpha=0.7)
    plt.plot(time, df['right_dist'], 'r-', label='Right', linewidth=0.8, alpha=0.7)
    plt.plot(time, df['front_dist'], 'g-', label='Front', linewidth=0.8, alpha=0.7)
    
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.title('Lidar Clearance Distances')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    output_file = Path(output_dir) / 'clearance_distances.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.close()

def plot_control_commands(df, output_dir):
    if df is None:
        return
    
    time = df['timestamp'] - df['timestamp'].iloc[0]
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    ax1.plot(time, df['speed_cmd'], 'b-', linewidth=0.8, alpha=0.7)
    ax1.set_ylabel('Speed Command (m/s)')
    ax1.set_title('Speed Commands')
    ax1.grid(True, alpha=0.3)
    
    ax2.plot(time, df['steering_cmd'], 'r-', linewidth=0.8, alpha=0.7)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Steering Command (rad/s)')
    ax2.set_title('Steering Commands')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_file = Path(output_dir) / 'control_commands.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.close()

def main():
    if len(sys.argv) < 3:
        sys.exit(1)
    
    data_dir = sys.argv[1]
    session_id = sys.argv[2]
    output_dir = Path(data_dir) / "images"
    output_dir.mkdir(exist_ok=True)
    
    formation_df, racing_df = load_data(data_dir, session_id)
    
    if formation_df is None and racing_df is None:
        sys.exit(1)
    
    if racing_df is not None:
        plot_speed_over_time(racing_df, output_dir)
        plot_steering_analysis(racing_df, output_dir)
        plot_clearance_distances(racing_df, output_dir)
        plot_control_commands(racing_df, output_dir)

if __name__ == "__main__":
    main()
