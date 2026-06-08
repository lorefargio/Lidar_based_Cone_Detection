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
    Analyzes the statistical distribution of accepted cones (status == 'Detected')
    and prints a comprehensive tuning report with specific parameter recommendations.
    """
    detected = df[df['status'] == 'Detected']
    if detected.empty:
        print("\nNo accepted cones (status == 'Detected') found to analyze.")
        return

    # Create directory for reports
    os.makedirs(FIGURES_DIR, exist_ok=True)
    report_path = os.path.join(FIGURES_DIR, "tuning_report.txt")
    
    # We will build a text report and print/save it
    lines = []
    lines.append("=" * 80)
    lines.append("                    ACCEPTED CONES FEATURE ANALYSIS DASHBOARD")
    lines.append("=" * 80)
    lines.append(f"Total accepted cones analyzed: {len(detected)}")
    lines.append("")
    
    # 1. Overall Feature Statistics
    lines.append("-" * 80)
    lines.append(" 1. OVERALL FEATURE STATISTICS (Only for Accepted Cones)")
    lines.append("-" * 80)
    
    features = [
        ('point_count', 'Point Count', '.0f'),
        ('height', 'Height (m)', '.3f'),
        ('width_max', 'Max Width (m)', '.3f'),
        ('avg_intensity', 'Avg Intensity', '.1f'),
        ('linearity', 'Linearity (1D)', '.3f'),
        ('planarity', 'Planarity (2D)', '.3f'),
        ('scattering', 'Scattering (3D)', '.3f'),
        ('verticality', 'Verticality', '.3f'),
        ('symmetry', 'Symmetry Ratio', '.2f')
    ]
    
    header = f"{'Feature':<20s} | {'Min':<8s} | {'Q05 (5%)':<8s} | {'Mean':<8s} | {'Median':<8s} | {'Q95 (95%)':<8s} | {'Max':<8s}"
    lines.append(header)
    lines.append("-" * len(header))
    
    for col, label, fmt in features:
        if col in detected.columns:
            vals = detected[col].dropna()
            if not vals.empty:
                f_min = vals.min()
                f_q05 = vals.quantile(0.05)
                f_mean = vals.mean()
                f_med = vals.median()
                f_q95 = vals.quantile(0.95)
                f_max = vals.max()
                
                lines.append(
                    f"{label:<20s} | "
                    f"{f_min:<8{fmt}} | "
                    f"{f_q05:<8{fmt}} | "
                    f"{f_mean:<8{fmt}} | "
                    f"{f_med:<8{fmt}} | "
                    f"{f_q95:<8{fmt}} | "
                    f"{f_max:<8{fmt}}"
                )
    lines.append("")
    
    # 2. Point Count Analysis by Distance Bins
    lines.append("-" * 80)
    lines.append(" 2. POINT COUNT DISTRIBUTION BY DISTANCE")
    lines.append("-" * 80)
    lines.append("Helpful for tuning the dynamic point count threshold.")
    lines.append("")
    
    if 'distance_bin' in detected.columns:
        grouped = detected.groupby('distance_bin', observed=False)['point_count']
        lines.append(f"{'Distance Bin':<15s} | {'Count':<6s} | {'Min Pts':<8s} | {'Q05 (5%)':<8s} | {'Mean Pts':<8s} | {'Max Pts':<8s}")
        lines.append("-" * 65)
        for name, group in grouped:
            if not group.empty:
                lines.append(
                    f"{str(name):<15s} | "
                    f"{len(group):<6d} | "
                    f"{group.min():<8.0f} | "
                    f"{group.quantile(0.05):<8.0f} | "
                    f"{group.mean():<8.1f} | "
                    f"{group.max():<8.0f}"
                )
    lines.append("")
    
    # 3. Dynamic Tuning Recommendations
    lines.append("-" * 80)
    lines.append(" 3. CONCRETE PARAMETER RECOMMENDATIONS")
    lines.append("-" * 80)
    
    for algo, config in configs.items():
        algo_detected = detected[detected['algorithm'] == algo]
        if algo_detected.empty:
            continue
            
        lines.append(f"\n>>> Recommendations for Algorithm: {algo}")
        
        # Suggest min_height
        h_q01 = algo_detected['height'].quantile(0.01)
        suggested_min_h = max(0.05, h_q01 * 0.9)
        lines.append(f"  * rule_min_height:")
        lines.append(f"    - Current config  : {config.get('rule_min_height', 0.1):.3f} m")
        lines.append(f"    - Obs. 1% (Short) : {h_q01:.3f} m")
        lines.append(f"    - Recommendation  : {suggested_min_h:.3f} m (to catch short/partial scans far away)")
        
        # Suggest max_height
        h_q99 = algo_detected['height'].quantile(0.99)
        suggested_max_h = min(1.0, h_q99 * 1.1)
        lines.append(f"  * rule_max_height:")
        lines.append(f"    - Current config  : {config.get('rule_max_height', 0.8):.3f} m")
        lines.append(f"    - Obs. 99% (Tall) : {h_q99:.3f} m")
        lines.append(f"    - Recommendation  : {suggested_max_h:.3f} m (tightening from 0.80m excludes tall false positives)")
        
        # Suggest max_width
        w_q99 = algo_detected['width_max'].quantile(0.99)
        suggested_max_w = min(0.6, w_q99 * 1.1)
        lines.append(f"  * rule_max_width:")
        lines.append(f"    - Current config  : {config.get('rule_max_width', 0.5):.3f} m")
        lines.append(f"    - Obs. 99% (Wide) : {w_q99:.3f} m")
        lines.append(f"    - Recommendation  : {suggested_max_w:.3f} m (tightening from 0.50m removes double clusters / walls)")
        
        # Suggest linearity
        lin_q99 = algo_detected['linearity'].quantile(0.99)
        suggested_max_lin = min(0.95, lin_q99 + 0.02)
        lines.append(f"  * pca_max_linearity:")
        lines.append(f"    - Current config  : {config.get('pca_max_linearity', 0.8):.3f}")
        lines.append(f"    - Obs. 99% (Linear): {lin_q99:.3f}")
        lines.append(f"    - Recommendation  : {suggested_max_lin:.3f} (prevents post/pole-like objects from passing)")
        
        # Suggest verticality
        v_q01 = algo_detected['verticality'].quantile(0.01)
        suggested_min_vert = max(0.5, v_q01 - 0.02)
        lines.append(f"  * pca_min_verticality:")
        lines.append(f"    - Current config  : {config.get('pca_min_verticality', 0.65):.3f}")
        lines.append(f"    - Obs. 1% (Tilted) : {v_q01:.3f}")
        lines.append(f"    - Recommendation  : {suggested_min_vert:.3f} (increasing this rejects sloped ground residues)")
        
        # Suggest min points at 10m
        # We look at the 10-15m bin or estimate based on 10m
        pts_10_15 = algo_detected[algo_detected['distance_bin'] == '10-15m']['point_count']
        if not pts_10_15.empty:
            pts_q05 = pts_10_15.quantile(0.05)
            lines.append(f"  * rule_min_points_at_10m:")
            lines.append(f"    - Current config  : {config.get('rule_min_points_at_10m', 5)}")
            lines.append(f"    - Obs. 5% at 10-15m: {pts_q05:.0f} points")
            lines.append(f"    - Recommendation  : {max(3, int(pts_q05)):d} (prevents tiny noise clusters of 1-3 points from passing)")
            
    lines.append("\n" + "=" * 80)
    
    # Save to file
    report_content = "\n".join(lines)
    with open(report_path, "w") as rf:
        rf.write(report_content)
        
    # Print to terminal
    print(report_content)


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
        
        # Violin plots for feature distribution over distance bins
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
