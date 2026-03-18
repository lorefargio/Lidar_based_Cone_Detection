import json
import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

# Configurazione Seaborn per grafici più professionali (stile "Publication Ready")
sns.set_theme(style="whitegrid", context="paper", font_scale=1.2)

LOG_DIR = "/home/lore/lidar_ws/log_profiler"
FIGURES_DIR = os.path.join(LOG_DIR, "figures")

def load_data():
    all_data = []
    metadata = {}
    
    json_files = glob.glob(os.path.join(LOG_DIR, "profiler_*.json"))
    
    if not json_files:
        print(f"Errore: Nessun file JSON trovato in {LOG_DIR}.")
        return None, None

    for file in json_files:
        with open(file, 'r') as f:
            data = json.load(f)
            algo = data['metadata']['algorithm']
            metadata[algo] = data['metadata']
            
            df = pd.DataFrame(data['raw_frames'])
            df['algorithm'] = algo.capitalize()
            # Converti a float, nel caso il JSON abbia stringhe o int
            df['total_ms'] = df['total_ms'].astype(float)
            all_data.append(df)
            
    # Combina tutti i dataframe
    return pd.concat(all_data, ignore_index=True), metadata

def print_statistics(df):
    print("" + "="*50)
    print(" STATISTICHE DELLE PRESTAZIONI (LATENZA TOTALE)")
    print("="*50)
    
    stats_list = []
    
    for algo in df['algorithm'].unique():
        algo_df = df[df['algorithm'] == algo]['total_ms']
        
        stats = {
            'Algorithm': algo,
            'Mean (ms)': algo_df.mean(),
            'Median (ms)': algo_df.median(),
            'P90 (ms)': np.percentile(algo_df, 90),
            'P95 (ms)': np.percentile(algo_df, 95),
            'P99 (ms)': np.percentile(algo_df, 99),
            'Max (ms)': algo_df.max(),
            '> 50ms Drop (%)': (len(algo_df[algo_df > 50]) / len(algo_df)) * 100
        }
        stats_list.append(stats)
        
    stats_df = pd.DataFrame(stats_list)
    print(stats_df.to_string(index=False, float_format=lambda x: f"{x:.2f}"))
    print("="*50 + "")

def plot_boxplots(df):
    plt.figure(figsize=(10, 6))
    ax = sns.boxplot(x="algorithm", y="total_ms", data=df, showfliers=True, 
                     palette="Set2", width=0.5, linewidth=1.5)
    
    # Linea limite per 20Hz (50ms)
    ax.axhline(50, color='red', linestyle='--', linewidth=2, label='Budget 20Hz (50ms)')
    
    plt.title("Distribuzione della Latenza per Algoritmo di Clustering", fontsize=14, pad=15)
    plt.ylabel("Tempo Totale di Esecuzione (ms)", fontsize=12)
    plt.xlabel("Algoritmo", fontsize=12)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(FIGURES_DIR, "latency_boxplot.png"), dpi=300)
    plt.close()

def plot_stacked_bar(df):
    # Calcola la media per ogni fase
    avg_times = df.groupby('algorithm')[['ground_removal_ms', 'clustering_ms', 'estimation_ms']].mean().reset_index()
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Stacked bar
    bottom = np.zeros(len(avg_times))
    colors = ['#4c72b0', '#55a868', '#c44e52']
    labels = ['Ground Removal', 'Clustering', 'Estimation']
    columns = ['ground_removal_ms', 'clustering_ms', 'estimation_ms']
    
    for col, color, label in zip(columns, colors, labels):
        ax.bar(avg_times['algorithm'], avg_times[col], bottom=bottom, label=label, color=color, width=0.6)
        bottom += avg_times[col].values
        
    plt.title("Ripartizione del Tempo Medio di Esecuzione", fontsize=14, pad=15)
    plt.ylabel("Tempo (ms)", fontsize=12)
    plt.xlabel("Algoritmo", fontsize=12)
    plt.legend(title="Fasi della Pipeline")
    plt.tight_layout()
    plt.savefig(os.path.join(FIGURES_DIR, "latency_breakdown_stacked.png"), dpi=300)
    plt.close()

def plot_cones_stability(df):
    plt.figure(figsize=(12, 6))
    sns.lineplot(data=df, x="frame_id", y="cones_detected", hue="algorithm", alpha=0.7)
    
    plt.title("Stabilità Rilevamento Coni (Frame per Frame)", fontsize=14, pad=15)
    plt.ylabel("Numero di Coni Rilevati", fontsize=12)
    plt.xlabel("ID Frame", fontsize=12)
    plt.tight_layout()
    plt.savefig(os.path.join(FIGURES_DIR, "cones_stability.png"), dpi=300)
    plt.close()

def main():
    os.makedirs(FIGURES_DIR, exist_ok=True)
    print("Caricamento dati...")
    df, metadata = load_data()
    
    if df is not None:
        print_statistics(df)
        print(f"Generazione grafici in {FIGURES_DIR} ...")
        plot_boxplots(df)
        plot_stacked_bar(df)
        plot_cones_stability(df)
        print("✅ Generazione grafici completata.")

if __name__ == "__main__":
    main()