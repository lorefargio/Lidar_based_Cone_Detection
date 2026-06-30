import json
import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

# Configure Seaborn for professional-grade charts
sns.set_theme(style="whitegrid", context="paper", font_scale=1.2)

LOG_DIR = os.getenv("PERCEPTION_LOG_DIR", "log_profiler")
FIGURES_DIR = os.path.join(LOG_DIR, "figures", "deskew_analysis")

def load_data():
    """
    Loads all JSON profile files from the log directory containing deskewing metrics.
    """
    all_data = []
    json_files = glob.glob(os.path.join(LOG_DIR, "profiler_*.json"))
    
    if not json_files:
        print(f"Error: No JSON files found in {LOG_DIR}.")
        return None

    for file in json_files:
        with open(file, 'r') as f:
            try:
                data = json.load(f)
                algo_full_name = data['metadata']['algorithm']
                
                raw_frames = data['raw_frames']
                if not raw_frames:
                    continue
                    
                df = pd.DataFrame(raw_frames)
                df['algorithm'] = algo_full_name
                
                # Check if deskew fields exist
                if 'deskew_avg_translation_m' not in df.columns:
                    print(f"Warning: {file} does not contain deskew metrics. Make sure you are using the updated C++ code.")
                    continue
                    
                df['deskew_avg_translation_m'] = df['deskew_avg_translation_m'].astype(float)
                df['deskew_max_translation_m'] = df['deskew_max_translation_m'].astype(float)
                all_data.append(df)
            except Exception as e:
                print(f"Error loading {file}: {e}")
            
    if not all_data:
        return None
    return pd.concat(all_data, ignore_index=True)

def plot_deskew_metrics(df):
    """
    Plots the average and max deskew correction distance over time/frames.
    """
    os.makedirs(FIGURES_DIR, exist_ok=True)
    
    for algo in df['algorithm'].unique():
        algo_df = df[df['algorithm'] == algo]
        
        plt.figure(figsize=(12, 6))
        plt.plot(algo_df['frame_id'], algo_df['deskew_max_translation_m'] * 100.0, label='Max Correction (cm)', color='#e056fd', alpha=0.8, linewidth=1.5)
        plt.plot(algo_df['frame_id'], algo_df['deskew_avg_translation_m'] * 100.0, label='Avg Correction (cm)', color='#30336b', alpha=0.9, linewidth=2)
        
        plt.title(f"Deskew Displacement Resolution over Time ({algo})", fontsize=14, pad=15)
        plt.xlabel("Frame ID", fontsize=12)
        plt.ylabel("Point Correction Distance (cm)", fontsize=12)
        plt.legend(frameon=True)
        plt.tight_layout()
        
        filename = f"deskew_displacement_{algo.lower().replace(' ', '_')}.png"
        plt.savefig(os.path.join(FIGURES_DIR, filename), dpi=300)
        plt.close()

def plot_frequency_comparison(df):
    """
    Compares the 10Hz distortion against the simulated 20Hz distortion (50% reduction).
    """
    os.makedirs(FIGURES_DIR, exist_ok=True)
    
    for algo in df['algorithm'].unique():
        algo_df = df[df['algorithm'] == algo].copy()
        
        # Calculate simulated 20Hz values (linear scaling of dt by 0.5)
        algo_df['deskew_avg_20hz_m'] = algo_df['deskew_avg_translation_m'] * 0.5
        algo_df['deskew_max_20hz_m'] = algo_df['deskew_max_translation_m'] * 0.5
        
        # Melt dataframe for seaborn plotting
        melted_df = pd.melt(algo_df, id_vars=['frame_id'], 
                            value_vars=['deskew_max_translation_m', 'deskew_max_20hz_m'],
                            var_name='frequency_group', value_name='max_displacement')
        
        melted_df['frequency'] = melted_df['frequency_group'].map({
            'deskew_max_translation_m': '10Hz (Measured)',
            'deskew_max_20hz_m': '20Hz (Simulated/Estimated)'
        })
        
        plt.figure(figsize=(10, 6))
        melted_df['max_displacement_cm'] = melted_df['max_displacement'] * 100.0
        sns.boxplot(x='frequency', y='max_displacement_cm', data=melted_df, palette="muted", width=0.5)
        plt.title(f"Max Scan Distortion Comparison: 10Hz vs 20Hz ({algo})", fontsize=14, pad=15)
        plt.xlabel("LiDAR Scan Frequency", fontsize=12)
        plt.ylabel("Maximum Scan Distortion (cm)", fontsize=12)
        plt.tight_layout()
        
        filename = f"deskew_freq_comparison_{algo.lower().replace(' ', '_')}.png"
        plt.savefig(os.path.join(FIGURES_DIR, filename), dpi=300)
        plt.close()

def print_deskew_summary(df):
    """
    Calculates and prints statistics on deskew displacement.
    """
    print("\n" + "="*70)
    print(" DESKEWING IMPACT REPORT (DISTORTION RESOLVED)")
    print("="*70)
    
    for algo in df['algorithm'].unique():
        algo_df = df[df['algorithm'] == algo]
        avg_corr = algo_df['deskew_avg_translation_m'].mean() * 100.0
        max_corr_mean = algo_df['deskew_max_translation_m'].mean() * 100.0
        absolute_max = algo_df['deskew_max_translation_m'].max() * 100.0
        
        print(f"Configuration: {algo}")
        print(f"  - Average point correction distance: {avg_corr:.3f} cm")
        print(f"  - Average max-per-frame distortion:   {max_corr_mean:.3f} cm")
        print(f"  - Absolute worst-case distortion:     {absolute_max:.3f} cm")
        print("-" * 70)
        
    print("Figures generated and saved in:")
    print(f"  {os.path.abspath(FIGURES_DIR)}")
    print("="*70 + "\n")

def main():
    df = load_data()
    if df is None:
        print("No valid deskewing data loaded. Run the perception node inside your container first.")
        return
        
    plot_deskew_metrics(df)
    plot_frequency_comparison(df)
    print_deskew_summary(df)

if __name__ == "__main__":
    main()
