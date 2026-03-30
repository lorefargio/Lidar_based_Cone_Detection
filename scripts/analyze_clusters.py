import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os
import glob
import numpy as np

# Configure Seaborn for professional-grade charts
sns.set_theme(style="whitegrid", context="paper", font_scale=1.2)

LOG_DIR = os.getenv("PERCEPTION_LOG_DIR", "log_profiler")
FIGURES_DIR = os.path.join(LOG_DIR, "figures", "cluster_analysis")

def load_data():
    """
    Loads all CSV cluster logs from the log directory.
    """
    csv_files = glob.glob(os.path.join(LOG_DIR, "clusters_*.csv"))
    if not csv_files:
        print(f"No cluster logs found in {LOG_DIR}")
        return None
    
    dataframes = []
    for f in csv_files:
        try:
            df = pd.read_csv(f)
            algo = os.path.basename(f).replace("clusters_", "").replace(".csv", "")
            df['algorithm'] = algo.capitalize()
            dataframes.append(df)
        except Exception as e:
            print(f"Error loading {f}: {e}")
    
    if not dataframes:
        return None
        
    return pd.concat(dataframes, ignore_index=True)

def create_distance_bins(df):
    """
    Creates discrete distance ranges for analysis.
    """
    # Define bins: 0-5m, 5-10m, 10-15m, 15-20m, 20-25m
    bins = [0, 5, 10, 15, 20, 25]
    labels = ['0-5m', '5-10m', '10-15m', '15-20m', '20-25m']
    df['distance_bin'] = pd.cut(df['range'], bins=bins, labels=labels, include_lowest=True)
    return df

def plot_distance_analysis(df):
    """
    Plots how features change with distance using boxplots.
    """
    features = [
        'point_count', 'height', 'width_max', 'aspect_ratio', 
        'linearity', 'planarity', 'scattering', 'verticality'
    ]
    
    for feat in features:
        plt.figure(figsize=(12, 7))
        # Use boxplot to show distribution per distance bin
        sns.boxplot(data=df, x='distance_bin', y=feat, hue='algorithm', palette="muted")
        
        plt.title(f"Distribution of {feat.replace('_', ' ').capitalize()} vs Distance Range")
        plt.xlabel("Distance Range (m)")
        plt.ylabel(feat.replace('_', ' ').capitalize())
        plt.legend(title="Algorithm", bbox_to_anchor=(1.05, 1), loc='upper left')
        plt.tight_layout()
        plt.savefig(os.path.join(FIGURES_DIR, f"dist_bin_{feat}.png"), dpi=300)
        plt.close()

def plot_joint_trends(df):
    """
    Plots trends of features vs continuous distance.
    """
    plt.figure(figsize=(10, 6))
    sns.regplot(data=df, x='range', y='point_count', scatter_kws={'alpha':0.3}, line_kws={'color':'red'})
    plt.title("Point Count Decay vs Range (Trend Analysis)")
    plt.xlabel("Range (m)")
    plt.ylabel("Point Count")
    plt.savefig(os.path.join(FIGURES_DIR, "trend_range_points.png"), dpi=300)
    plt.close()

def print_summary_stats(df):
    """
    Prints statistical summary per distance bin.
    """
    print("\n" + "="*80)
    print(" FEATURE SUMMARY BY DISTANCE BIN")
    print("="*80)
    
    summary = df.groupby(['algorithm', 'distance_bin'], observed=False)[['point_count', 'height', 'width_max', 'linearity']].agg(['mean', 'std']).round(3)
    print(summary)
    print("="*80 + "\n")

def main():
    os.makedirs(FIGURES_DIR, exist_ok=True)
    df = load_data()
    if df is not None:
        df = create_distance_bins(df)
        print_summary_stats(df)
        
        print(f"Analyzing {len(df)} detected cones...")
        plot_distance_analysis(df)
        plot_joint_trends(df)
        
        print(f"Analysis complete. Distance-binned figures saved in {FIGURES_DIR}")

if __name__ == "__main__":
    main()
