#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import sys

def load_racing_data(data_dir, session_id):
    racing_file = Path(data_dir) / f"racing_{session_id}.csv"
    
    if not racing_file.exists():
        return None
    
    df = pd.read_csv(racing_file)
    df = df[df['segment_id'] >= 0].copy()
    
    return df

def detect_laps(df):
    if df is None or len(df) == 0:
        return []
    
    lap_starts = [0]
    for i in range(1, len(df)):
        if df['segment_id'].iloc[i] < df['segment_id'].iloc[i-1]:
            lap_starts.append(i)
    
    return lap_starts

def plot_lap_times(df, output_dir):
    lap_starts = detect_laps(df)
    
    if len(lap_starts) < 2:
        return
    
    lap_times = []
    lap_labels = []
    
    for i in range(len(lap_starts) - 1):
        start_idx = lap_starts[i]
        end_idx = lap_starts[i + 1]
        lap_time = df['timestamp'].iloc[end_idx] - df['timestamp'].iloc[start_idx]
        lap_times.append(lap_time)
        lap_labels.append(f"Lap {i+1}")
    
    if len(lap_starts) > 0:
        start_idx = lap_starts[-1]
        lap_time = df['timestamp'].iloc[-1] - df['timestamp'].iloc[start_idx]
        lap_times.append(lap_time)
        lap_labels.append(f"Lap {len(lap_starts)}")
    
    plt.figure(figsize=(10, 6))
    colors = ['#ff6b6b', '#feca57', '#48dbfb'][:len(lap_times)]
    bars = plt.bar(lap_labels, lap_times, color=colors, edgecolor='black', linewidth=1.5)
    
    for bar, time in zip(bars, lap_times):
        height = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2., height,
                f'{time:.1f}s',
                ha='center', va='bottom', fontsize=12, fontweight='bold')
    
    plt.xlabel('Lap Number', fontsize=13, fontweight='bold')
    plt.ylabel('Lap Time (seconds)', fontsize=13, fontweight='bold')
    plt.title('Lap Time Comparison - Learning Progress', fontsize=15, fontweight='bold')
    plt.grid(True, alpha=0.3, axis='y')
    
    if len(lap_times) > 1:
        improvement = lap_times[0] - lap_times[-1]
        improvement_pct = (improvement / lap_times[0]) * 100
        plt.text(0.5, 0.95, f'Improvement: {improvement:.1f}s ({improvement_pct:.1f}%)',
                transform=plt.gca().transAxes,
                ha='center', va='top',
                bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8),
                fontsize=11, fontweight='bold')
    
    output_file = Path(output_dir) / 'lap_times_comparison.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.close()

def plot_segment_times(df, output_dir):
    lap_starts = detect_laps(df)
    
    if len(lap_starts) < 2:
        return
    
    segment_times = {}
    
    for lap_idx in range(len(lap_starts)):
        start_idx = lap_starts[lap_idx]
        end_idx = lap_starts[lap_idx + 1] if lap_idx + 1 < len(lap_starts) else len(df)
        
        lap_df = df.iloc[start_idx:end_idx]
        
        for seg_id in lap_df['segment_id'].unique():
            seg_data = lap_df[lap_df['segment_id'] == seg_id]
            if len(seg_data) > 0:
                seg_time = seg_data['timestamp'].iloc[-1] - seg_data['timestamp'].iloc[0]
                
                if seg_id not in segment_times:
                    segment_times[seg_id] = []
                segment_times[seg_id].append(seg_time)
    
    num_segments = len(segment_times)
    if num_segments == 0:
        return
    
    x = np.arange(num_segments)
    width = 0.25
    
    fig, ax = plt.subplots(figsize=(14, 7))
    
    seg_ids = sorted(segment_times.keys())
    lap_colors = ['#e74c3c', '#f39c12', '#3498db']
    
    for lap_idx in range(min(3, max(len(times) for times in segment_times.values()))):
        times = [segment_times[seg][lap_idx] if lap_idx < len(segment_times[seg]) else 0 
                for seg in seg_ids]
        ax.bar(x + lap_idx * width, times, width, 
               label=f'Lap {lap_idx + 1}', color=lap_colors[lap_idx], edgecolor='black')
    
    ax.set_xlabel('Segment ID', fontsize=13, fontweight='bold')
    ax.set_ylabel('Time (seconds)', fontsize=13, fontweight='bold')
    ax.set_title('Segment Time Comparison - Where is the Improvement?', fontsize=15, fontweight='bold')
    ax.set_xticks(x + width)
    ax.set_xticklabels([f'S{seg}' for seg in seg_ids])
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3, axis='y')
    
    output_file = Path(output_dir) / 'segment_time_comparison.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.close()

def plot_segment_improvement_heatmap(df, output_dir):
    lap_starts = detect_laps(df)
    
    if len(lap_starts) < 2:
        return
    
    segment_times = {}
    
    for lap_idx in range(len(lap_starts)):
        start_idx = lap_starts[lap_idx]
        end_idx = lap_starts[lap_idx + 1] if lap_idx + 1 < len(lap_starts) else len(df)
        lap_df = df.iloc[start_idx:end_idx]
        
        for seg_id in lap_df['segment_id'].unique():
            seg_data = lap_df[lap_df['segment_id'] == seg_id]
            if len(seg_data) > 0:
                seg_time = seg_data['timestamp'].iloc[-1] - seg_data['timestamp'].iloc[0]
                if seg_id not in segment_times:
                    segment_times[seg_id] = []
                segment_times[seg_id].append(seg_time)
    
    if len(segment_times) == 0:
        return
    
    seg_ids = sorted(segment_times.keys())
    max_laps = max(len(times) for times in segment_times.values())
    matrix = np.zeros((len(seg_ids), max_laps))
    
    for i, seg_id in enumerate(seg_ids):
        for j in range(len(segment_times[seg_id])):
            matrix[i, j] = segment_times[seg_id][j]
    
    plt.figure(figsize=(10, 8))
    sns.heatmap(matrix, annot=True, fmt='.2f', cmap='RdYlGn_r',
                xticklabels=[f'Lap {i+1}' for i in range(max_laps)],
                yticklabels=[f'Segment {seg}' for seg in seg_ids],
                cbar_kws={'label': 'Time (s)'})
    
    plt.title('Segment Performance Heatmap - Learning Effectiveness', fontsize=15, fontweight='bold')
    plt.xlabel('Lap Number', fontsize=13, fontweight='bold')
    plt.ylabel('Segment ID', fontsize=13, fontweight='bold')
    
    output_file = Path(output_dir) / 'segment_improvement_heatmap.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.close()

def plot_speed_profile_per_segment(df, output_dir):
    if df is None or len(df) == 0:
        return
    
    segments = df['segment_id'].unique()
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    axes = axes.flatten()
    
    for idx, seg_id in enumerate(sorted(segments)[:4]):
        seg_data = df[df['segment_id'] == seg_id]
        
        ax = axes[idx]
        ax.plot(seg_data['vel_linear'], linewidth=1.5, color='#3498db', alpha=0.7)
        ax.axhline(seg_data['vel_linear'].mean(), color='red', linestyle='--', 
                   label=f'Avg: {seg_data["vel_linear"].mean():.2f} m/s')
        ax.fill_between(range(len(seg_data)), seg_data['vel_linear'], alpha=0.2)
        
        ax.set_title(f'Segment {seg_id} Speed Profile', fontsize=12, fontweight='bold')
        ax.set_xlabel('Data Point', fontsize=10)
        ax.set_ylabel('Speed (m/s)', fontsize=10)
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_file = Path(output_dir) / 'speed_profile_per_segment.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.close()

def plot_summary_statistics(df, output_dir):
    if df is None or len(df) == 0:
        return
    
    lap_starts = detect_laps(df)
    
    lap_times = []
    for i in range(len(lap_starts) - 1):
        start_idx = lap_starts[i]
        end_idx = lap_starts[i + 1]
        lap_time = df['timestamp'].iloc[end_idx] - df['timestamp'].iloc[start_idx]
        lap_times.append(lap_time)
    
    if len(lap_starts) > 0:
        start_idx = lap_starts[-1]
        lap_time = df['timestamp'].iloc[-1] - df['timestamp'].iloc[start_idx]
        lap_times.append(lap_time)
    
    total_laps = len(lap_times)
    best_lap = min(lap_times) if lap_times else 0
    avg_speed = df['vel_linear'].mean()
    min_clearance = min(df['left_dist'].min(), df['right_dist'].min())
    improvement = ((lap_times[0] - lap_times[-1]) / lap_times[0] * 100) if len(lap_times) > 1 else 0
    
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.axis('off')
    
    stats_text = f"""
    PERFORMANCE SUMMARY
    {'='*40}
    
    Total Laps:           {total_laps}
    Segments:             {len(df['segment_id'].unique())}
    
    Best Lap Time:        {best_lap:.1f}s
    First Lap:            {lap_times[0]:.1f}s
    Last Lap:             {lap_times[-1]:.1f}s
    Improvement:          {improvement:.1f}%
    
    Average Speed:        {avg_speed:.2f} m/s
    Max Speed:            {df['vel_linear'].max():.2f} m/s
    
    Min Clearance:        {min_clearance:.2f}m
    Avg Left Distance:    {df['left_dist'].mean():.2f}m
    Avg Right Distance:   {df['right_dist'].mean():.2f}m
    {'='*40}
    """
    
    ax.text(0.5, 0.5, stats_text, transform=ax.transAxes,
            fontsize=12, verticalalignment='center', horizontalalignment='center',
            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8),
            family='monospace', fontweight='bold')
    
    output_file = Path(output_dir) / 'summary_statistics.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.close()

def plot_learning_rate(df, output_dir):
    if df is None or len(df) == 0:
        return
    
    lap_starts = detect_laps(df)
    
    if len(lap_starts) < 2:
        return
    
    segment_times = {}
    
    for lap_idx in range(len(lap_starts)):
        start_idx = lap_starts[lap_idx]
        end_idx = lap_starts[lap_idx + 1] if lap_idx + 1 < len(lap_starts) else len(df)
        lap_df = df.iloc[start_idx:end_idx]
        
        for seg_id in lap_df['segment_id'].unique():
            seg_data = lap_df[lap_df['segment_id'] == seg_id]
            if len(seg_data) > 0:
                seg_time = seg_data['timestamp'].iloc[-1] - seg_data['timestamp'].iloc[0]
                if seg_id not in segment_times:
                    segment_times[seg_id] = []
                segment_times[seg_id].append(seg_time)
    
    plt.figure(figsize=(12, 7))
    
    seg_ids = sorted(segment_times.keys())
    for seg_id in seg_ids:
        times = segment_times[seg_id]
        if len(times) > 1:
            laps = list(range(1, len(times) + 1))
            improvement = [(times[0] - t) / times[0] * 100 for t in times]
            plt.plot(laps, improvement, marker='o', linewidth=2, label=f'Segment {seg_id}', alpha=0.7)
    
    plt.xlabel('Lap Number', fontsize=13, fontweight='bold')
    plt.ylabel('Improvement (%)', fontsize=13, fontweight='bold')
    plt.title('Learning Rate per Segment - Improvement Over Laps', fontsize=15, fontweight='bold')
    plt.legend(fontsize=9, ncol=2)
    plt.grid(True, alpha=0.3)
    plt.axhline(y=0, color='red', linestyle='--', alpha=0.5)
    
    output_file = Path(output_dir) / 'learning_rate_curve.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.close()

def plot_safety_distribution(df, output_dir):
    if df is None or len(df) == 0:
        return
    
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    
    distances = {
        'Left': df['left_dist'],
        'Right': df['right_dist'],
        'Front': df['front_dist']
    }
    
    colors = ['#3498db', '#e74c3c', '#2ecc71']
    
    for idx, (name, data) in enumerate(distances.items()):
        ax = axes[idx]
        ax.hist(data, bins=30, color=colors[idx], alpha=0.7, edgecolor='black')
        ax.axvline(data.mean(), color='red', linestyle='--', linewidth=2, label=f'Mean: {data.mean():.2f}m')
        ax.axvline(2.0, color='orange', linestyle='--', linewidth=2, label='Safety Threshold')
        
        ax.set_xlabel('Distance (m)', fontsize=11)
        ax.set_ylabel('Frequency', fontsize=11)
        ax.set_title(f'{name} Distance Distribution', fontsize=12, fontweight='bold')
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    output_file = Path(output_dir) / 'safety_distribution.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.close()


def main():
    if len(sys.argv) < 3:
        sys.exit(1)
    
    data_dir = sys.argv[1]
    session_id = sys.argv[2]
    output_dir = Path(data_dir) / "images"
    output_dir.mkdir(exist_ok=True)
    
    df = load_racing_data(data_dir, session_id)
    
    if df is None:
        sys.exit(1)
    
    plot_lap_times(df, output_dir)
    plot_segment_times(df, output_dir)
    plot_segment_improvement_heatmap(df, output_dir)
    plot_speed_profile_per_segment(df, output_dir)
    plot_summary_statistics(df, output_dir)
    plot_learning_rate(df, output_dir)
    plot_safety_distribution(df, output_dir)

if __name__ == "__main__":
    main()
