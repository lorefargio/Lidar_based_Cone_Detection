import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os
import glob
import numpy as np
import json

# Configure Seaborn for professional-grade charts
sns.set_theme(style="whitegrid", context="paper", font_scale=1.2)

LOG_DIR = os.getenv("PERCEPTION_LOG_DIR", "log_profiler")
FIGURES_DIR = os.path.join(LOG_DIR, "figures", "cluster_analysis")

def load_data():
    """
    Loads all CSV cluster logs and their corresponding config JSONs.
    """
    csv_files = glob.glob(os.path.join(LOG_DIR, "clusters_*.csv"))
    if not csv_files:
        print(f"No cluster logs found in {LOG_DIR}")
        return None, None
    
    dataframes = []
    configs = {}

    for f in csv_files:
        try:
            algo = os.path.basename(f).replace("clusters_", "").replace(".csv", "")
            
            # Load CSV
            df = pd.read_csv(f)
            df['algorithm'] = algo.capitalize()
            dataframes.append(df)
            
            # Load Config JSON
            config_path = os.path.join(LOG_DIR, f"config_{algo}.json")
            if os.path.exists(config_path):
                with open(config_path, 'r') as jf:
                    configs[algo.capitalize()] = json.load(jf)
            else:
                print(f"Warning: No config found for {algo}")

        except Exception as e:
            print(f"Error loading {f}: {e}")
    
    if not dataframes:
        return None, None
        
    full_df = pd.concat(dataframes, ignore_index=True)
    
    # Add status column
    if 'confidence' in full_df.columns:
        full_df['status'] = full_df['confidence'].apply(lambda x: 'Detected' if x > 0.5 else 'Rejected')
    else:
        full_df['status'] = 'Unknown'
        
    return full_df, configs

def create_distance_bins(df):
    bins = [0, 5, 10, 15, 20, 25]
    labels = ['0-5m', '5-10m', '10-15m', '15-20m', '20-25m']
    df['distance_bin'] = pd.cut(df['range'], bins=bins, labels=labels, include_lowest=True)
    return df

def plot_rejection_reasons(df):
    """
    Provides a breakdown of why clusters were rejected.
    """
    rejected = df[df['status'] == 'Rejected']
    if rejected.empty:
        return

    plt.figure(figsize=(12, 6))
    reason_counts = rejected.groupby(['algorithm', 'rejection_reason']).size().unstack(fill_value=0)
    
    reason_counts.plot(kind='bar', stacked=True, colormap='viridis', ax=plt.gca())
    
    plt.title("Rejection Reason Breakdown per Algorithm")
    plt.ylabel("Number of Clusters")
    plt.xlabel("Algorithm")
    plt.legend(title="Reason", bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.tight_layout()
    plt.savefig(os.path.join(FIGURES_DIR, "rejection_breakdown.png"), dpi=300)
    plt.close()

# Standard Dimensions (Regulation)
CONE_SMALL = {'width': 0.228, 'height': 0.325}
CONE_LARGE = {'width': 0.285, 'height': 0.505}

def plot_dimensional_analysis(df):
    """
    Plots height vs width distribution to see how clusters align with regulation sizes.
    """
    plt.figure(figsize=(10, 8))
    sns.scatterplot(data=df, x='width_max', y='height', hue='status', alpha=0.5)
    
    # Draw Regulation Targets
    plt.scatter([CONE_SMALL['width']], [CONE_SMALL['height']], color='blue', s=200, marker='X', label='Target: Small (228x325)')
    plt.scatter([CONE_LARGE['width']], [CONE_LARGE['height']], color='orange', s=200, marker='X', label='Target: Large (285x505)')
    
    plt.title("Cluster Dimensions vs Regulation Targets")
    plt.xlabel("Max Width (m)")
    plt.ylabel("Height (m)")
    plt.legend()
    plt.savefig(os.path.join(FIGURES_DIR, "dimension_scatter.png"), dpi=300)
    plt.close()

def plot_decision_boundaries(df, configs):
    """
    Visualizes boundaries using the ACTUAL parameters from the config file.
    """
    if 'verticality' not in df.columns or 'linearity' not in df.columns:
        return

    for algo, config in configs.items():
        algo_df = df[df['algorithm'] == algo]
        if algo_df.empty: continue

        plt.figure(figsize=(10, 8))
        sns.scatterplot(data=algo_df, x='linearity', y='verticality', hue='status', alpha=0.6, palette={"Detected": "green", "Rejected": "red"})
        
        # Get thresholds from config (dynamically)
        max_lin = config.get('pca_max_linearity', 0.8)
        min_vert = config.get('pca_min_verticality', 0.65)
        
        plt.axhline(min_vert, color='darkred', linestyle='--', label=f'Min Verticality ({min_vert})')
        plt.axvline(max_lin, color='darkred', linestyle='-', label=f'Max Linearity ({max_lin})')
        
        plt.title(f"Decision Boundary: {algo} (Active Thresholds)")
        plt.legend()
        plt.savefig(os.path.join(FIGURES_DIR, f"boundary_{algo.lower()}.png"), dpi=300)
        plt.close()

def suggest_parameters(df, configs):
    """
    Suggests improvements by comparing active config with actual detected distributions.
    """
    detected = df[df['status'] == 'Detected']
    if detected.empty: return

    print("\n" + "="*60)
    print(" TUNING SUGGESTIONS (DYNAMIC)")
    print("="*60)

    for algo, config in configs.items():
        algo_detected = detected[detected['algorithm'] == algo]
        if algo_detected.empty: continue

        print(f"\n>>> Algorithm: {algo}")
        
        # Check Height
        current_min_h = config.get('rule_min_height', 0.1)
        obs_min_h = algo_detected['height'].quantile(0.01)
        if obs_min_h < current_min_h * 1.1:
            print(f"  [CRITICAL] Cones are very close to your rule_min_height ({current_min_h}).")
            print(f"             Consider lowering it to {obs_min_h * 0.9:.3f} to improve distant recall.")

        # Check Linearity
        current_max_lin = config.get('pca_max_linearity', 0.8)
        obs_max_lin = algo_detected['linearity'].quantile(0.99)
        if obs_max_lin > current_max_lin * 0.9:
            print(f"  [STRICT] Your pca_max_linearity ({current_max_lin}) is cutting off valid cones.")
            print(f"           Suggest increasing to {obs_max_lin + 0.05:.2f}.")

        # Check Verticality
        current_min_vert = config.get('pca_min_verticality', 0.65)
        obs_min_vert = algo_detected['verticality'].quantile(0.01)
        if obs_min_vert < current_min_vert * 1.1:
            print(f"  [STRICT] Your pca_min_verticality ({current_min_vert}) is cutting off valid cones.")
            print(f"           Suggest lowering to {obs_min_vert - 0.05:.2f}.")

        # Check Dimensional Consistency
        obs_height_q01 = algo_detected['height'].quantile(0.01)
        obs_height_q99 = algo_detected['height'].quantile(0.99)
        obs_width_q99 = algo_detected['width_max'].quantile(0.99)
        
        if obs_height_q01 < CONE_SMALL['height'] * 0.8:
            print(f"  [GEOMETRY] Cones are being detected significantly shorter ({obs_height_q01:.3f}m) than target (0.325m).")
            print(f"             Check if your ground remover is cutting the base too much.")
        
        if obs_width_q99 > config.get('rule_max_width', 0.36) * 0.9:
             print(f"  [STRICT] Your rule_max_width ({config.get('rule_max_width')}) is very close to observed max width ({obs_width_q99:.3f}m).")
             print(f"           Lidar bloom often makes objects look wider; consider increasing to {obs_width_q99 + 0.04:.2f}.")

def main():
    os.makedirs(FIGURES_DIR, exist_ok=True)
    df, configs = load_data()
    
    if df is not None:
        df = create_distance_bins(df)
        
        print(f"Loaded {len(df)} total clusters across {len(configs)} configurations.")
        
        plot_rejection_reasons(df)
        plot_dimensional_analysis(df)
        if configs:
            plot_decision_boundaries(df, configs)
            suggest_parameters(df, configs)
        
        # Reuse violin plots from previous version but keep them simple
        features = ['height', 'linearity', 'verticality']
        for feat in features:
            plt.figure(figsize=(10, 6))
            sns.violinplot(data=df, x='distance_bin', y=feat, hue='status', split=True, palette={"Detected": "green", "Rejected": "red"})
            plt.title(f"Distribution Comparison: {feat.capitalize()}")
            plt.savefig(os.path.join(FIGURES_DIR, f"violin_{feat}.png"))
            plt.close()

        print(f"\nAnalysis complete. Figures and Dynamic Suggestions saved in {FIGURES_DIR}")

if __name__ == "__main__":
    main()
