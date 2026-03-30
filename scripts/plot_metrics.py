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
    Returns:
        pd.DataFrame: Combined metrics for all algorithms.
        dict: Metadata for each algorithm session.
    """
    all_data = []
    metadata = {}
    
    json_files = glob.glob(os.path.join(LOG_DIR, "profiler_*.json"))
    
    if not json_files:
        print(f"Error: No JSON files found in {LOG_DIR}.")
        return None, None

    for file in json_files:
        with open(file, 'r') as f:
            data = json.load(f)
            algo = data['metadata']['algorithm']
            metadata[algo] = data['metadata']
            
            df = pd.DataFrame(data['raw_frames'])
            df['algorithm'] = algo.capitalize()
            df['total_ms'] = df['total_ms'].astype(float)
            all_data.append(df)
            
    return pd.concat(all_data, ignore_index=True), metadata

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

def plot_stacked_bar(df):
    """
    Generates a stacked bar chart showing the breakdown of time spent in each pipeline phase.
    """
    # Define all phases to be plotted
    phases = [
        'conversion_ms', 'deskewing_ms', 'ground_removal_ms', 
        'clustering_ms', 'merging_ms', 'estimation_ms', 'duplicate_ms'
    ]
    
    # Filter only available phases in the dataframe
    available_phases = [p for p in phases if p in df.columns]
    
    # Calculate average time per phase per algorithm
    avg_times = df.groupby('algorithm')[available_phases].mean().reset_index()
    
    fig, ax = plt.subplots(figsize=(12, 7))
    
    bottom = np.zeros(len(avg_times))
    # Use a larger color palette for more phases
    colors = sns.color_palette("husl", len(available_phases))
    
    for col, color in zip(available_phases, colors):
        label = col.replace('_ms', '').replace('_', ' ').capitalize()
        ax.bar(avg_times['algorithm'], avg_times[col], bottom=bottom, label=label, color=color, width=0.6)
        bottom += avg_times[col].values
        
    plt.title("Mean Execution Time Breakdown (Full Pipeline)", fontsize=14, pad=15)
    plt.ylabel("Time (ms)", fontsize=12)
    plt.xlabel("Algorithm", fontsize=12)
    plt.legend(title="Pipeline Phases", bbox_to_anchor=(1.05, 1), loc='upper left')
    
    # Target 20Hz limit line
    ax.axhline(50, color='red', linestyle=':', alpha=0.5, label='20Hz Budget')
    
    plt.tight_layout()
    plt.savefig(os.path.join(FIGURES_DIR, "latency_breakdown_stacked.png"), dpi=300)
    plt.close()

def plot_cones_stability(df):
    """
    Plots the number of cones detected frame-by-frame for each algorithm.
    """
    plt.figure(figsize=(12, 6))
    sns.lineplot(data=df, x="frame_id", y="cones_detected", hue="algorithm", alpha=0.7)
    
    plt.title("Cone Detection Stability (Frame-by-Frame)", fontsize=14, pad=15)
    plt.ylabel("Number of Detected Cones", fontsize=12)
    plt.xlabel("Frame ID", fontsize=12)
    plt.tight_layout()
    plt.savefig(os.path.join(FIGURES_DIR, "cones_stability.png"), dpi=300)
    plt.close()

def main():
    """
    Main entry point for the analysis script.
    """
    os.makedirs(FIGURES_DIR, exist_ok=True)
    print("Loading profiling data...")
    df, metadata = load_data()
    
    if df is not None:
        print_statistics(df)
        print(f"Generating charts in {FIGURES_DIR} ...")
        plot_boxplots(df)
        plot_stacked_bar(df)
        plot_cones_stability(df)
        print("Analysis and chart generation completed successfully.")

if __name__ == "__main__":
    main()
