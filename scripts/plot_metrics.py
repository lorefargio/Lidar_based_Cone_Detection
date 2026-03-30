import json
import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

# Configure Seaborn for professional-grade charts
sns.set_theme(style="whitegrid", context="paper", font_scale=1.2)

# Use log_profiler in the current directory as default
LOG_DIR = os.getenv("PERCEPTION_LOG_DIR", "log_profiler")
FIGURES_DIR = os.path.join(LOG_DIR, "figures")

def load_data():
    """
    Loads all JSON profile files from the log directory and combines them into a single DataFrame.
    """
    all_data = []
    
    json_files = glob.glob(os.path.join(LOG_DIR, "profiler_*.json"))
    
    if not json_files:
        print(f"Error: No JSON files found in {LOG_DIR}.")
        return None

    for file in json_files:
        with open(file, 'r') as f:
            data = json.load(f)
            algo_full_name = data['metadata']['algorithm']
            
            df = pd.DataFrame(data['raw_frames'])
            df['algorithm'] = algo_full_name
            df['total_ms'] = df['total_ms'].astype(float)
            all_data.append(df)
            
    return pd.concat(all_data, ignore_index=True)

def print_statistics(df):
    """
    Calculates and prints summary statistics for each algorithm's total latency.
    """
    print("\n" + "="*60)
    print(" PERFORMANCE STATISTICS (TOTAL LATENCY)")
    print("="*60)
    
    stats_list = []
    
    for algo in df['algorithm'].unique():
        algo_df = df[df['algorithm'] == algo]['total_ms']
        
        stats = {
            'Algorithm': algo,
            'Mean (ms)': algo_df.mean(),
            'Median (ms)': algo_df.median(),
            'P95 (ms)': np.percentile(algo_df, 95),
            'P99 (ms)': np.percentile(algo_df, 99),
            'Max (ms)': algo_df.max(),
            'Drop > 50ms (%)': (len(algo_df[algo_df > 50]) / len(algo_df)) * 100
        }
        stats_list.append(stats)
        
    stats_df = pd.DataFrame(stats_list)
    print(stats_df.to_string(index=False, float_format=lambda x: f"{x:.2f}"))
    print("="*60 + "\n")

def plot_boxplots(df):
    """
    Generates a boxplot showing the distribution of total latency per algorithm.
    """
    plt.figure(figsize=(10, 6))
    ax = sns.boxplot(x="algorithm", y="total_ms", data=df, showfliers=True, 
                     palette="Set2", width=0.5, linewidth=1.5)
    
    # 20Hz real-time budget line (50ms)
    ax.axhline(50, color='red', linestyle='--', linewidth=2, label='20Hz Budget (50ms)')
    
    plt.title("Latency Distribution by Clustering Algorithm", fontsize=14, pad=15)
    plt.ylabel("Total Execution Time (ms)", fontsize=12)
    plt.xlabel("Algorithm", fontsize=12)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(FIGURES_DIR, "latency_boxplot.png"), dpi=300)
    plt.close()

def generate_phase_plots(df, phase_name, output_subdir):
    """
    Helper to generate all three standard plots for a specific subset of data.
    """
    if df.empty:
        return

    phase_dir = os.path.join(FIGURES_DIR, output_subdir)
    os.makedirs(phase_dir, exist_ok=True)
    
    print(f"--- Generating plots for: {phase_name} ---")
    
    # 1. Boxplot
    plt.figure(figsize=(10, 6))
    sns.boxplot(x="algorithm", y="total_ms", data=df, palette="Set2")
    plt.axhline(50, color='red', linestyle='--', label='20Hz Budget')
    plt.title(f"Latency Distribution: {phase_name}", fontsize=14)
    plt.ylabel("Total Time (ms)")
    plt.tight_layout()
    plt.savefig(os.path.join(phase_dir, "latency_boxplot.png"), dpi=300)
    plt.close()

    # 2. Stacked Bar
    phases = ['conversion_ms', 'deskewing_ms', 'ground_removal_ms', 'clustering_ms', 'merging_ms', 'estimation_ms', 'duplicate_ms']
    available = [p for p in phases if p in df.columns]
    avg_times = df.groupby('algorithm')[available].mean().reset_index()
    
    fig, ax = plt.subplots(figsize=(12, 7))
    bottom = np.zeros(len(avg_times))
    colors = sns.color_palette("husl", len(available))
    for col, color in zip(available, colors):
        label = col.replace('_ms', '').replace('_', ' ').capitalize()
        ax.bar(avg_times['algorithm'], avg_times[col], bottom=bottom, label=label, color=color)
        bottom += avg_times[col].values
    plt.title(f"Execution Breakdown: {phase_name}", fontsize=14)
    plt.ylabel("Time (ms)")
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.tight_layout()
    plt.savefig(os.path.join(phase_dir, "latency_breakdown.png"), dpi=300)
    plt.close()

    # 3. Stability
    plt.figure(figsize=(12, 6))
    sns.lineplot(data=df, x="frame_id", y="cones_detected", hue="algorithm", alpha=0.7)
    plt.title(f"Detection Stability: {phase_name}", fontsize=14)
    plt.tight_layout()
    plt.savefig(os.path.join(phase_dir, "cones_stability.png"), dpi=300)
    plt.close()

def main():
    os.makedirs(FIGURES_DIR, exist_ok=True)
    df = load_data()
    if df is None: return

    # Split data based on naming conventions from run_benchmarks.sh
    # Clustering Phase: grid_slope_based, euclidean_slope_based, etc.
    clustering_df = df[df['algorithm'].str.contains('_slope_based')].copy()
    if not clustering_df.empty:
        clustering_df['algorithm'] = clustering_df['algorithm'].str.replace('_slope_based', '').str.upper()
        generate_phase_plots(clustering_df, "Clustering Algorithms (Fixed Ground: Slope)", "clustering_comparison")

    # Ground Phase: grid_bin_based, grid_patchworkpp, etc.
    ground_df = df[df['algorithm'].str.startswith('grid_')].copy()
    if not ground_df.empty:
        ground_df['algorithm'] = ground_df['algorithm'].str.replace('grid_', '').str.upper()
        generate_phase_plots(ground_df, "Ground Removal Algorithms (Fixed Clusterer: Grid)", "ground_comparison")

    print(f"\nAll charts generated in: {FIGURES_DIR}")

if __name__ == "__main__":
    main()
