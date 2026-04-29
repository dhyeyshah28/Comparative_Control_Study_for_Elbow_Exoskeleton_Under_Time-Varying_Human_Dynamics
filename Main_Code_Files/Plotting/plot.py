#!/usr/bin/env python3
"""
Standalone Motor Feedback Plotter
Just run: python plot_motor.py
"""

import pandas as pd
import matplotlib.pyplot as plt

# ========================================
# CONFIGURATION - EDIT THESE
# ========================================
CSV_FILE = 'motor_feedback2.csv'
COLUMNS_TO_PLOT = ['p_out_rad', 'p_cmd_rad', 't_out_Nm']  # <-- Change columns here
# ========================================

def main():
    print(f"📂 Reading: {CSV_FILE}")
    print(f"📊 Plotting: {COLUMNS_TO_PLOT}")
    
    # Read CSV
    df = pd.read_csv(CSV_FILE)
    
    # Validate columns exist
    missing = [col for col in COLUMNS_TO_PLOT if col not in df.columns]
    if missing:
        print(f"❌ Error: Columns not found: {missing}")
        print(f"Available columns: {list(df.columns)}")
        return
    
    # Create figure
    plt.figure(figsize=(14, 8))
    
    # Plot each column
    for col in COLUMNS_TO_PLOT:
        plt.plot(df['sample'], df[col], linewidth=2, label=col, marker='o', markersize=3)
    
    # Formatting
    plt.xlabel('Sample Number', fontsize=14, fontweight='bold')
    plt.ylabel('Value', fontsize=14, fontweight='bold')
    plt.title(f'Motor Feedback: {", ".join(COLUMNS_TO_PLOT)}', fontsize=16, fontweight='bold')
    plt.legend(fontsize=12, loc='best')
    plt.grid(True, alpha=0.3, linestyle='--')
    plt.tight_layout()
    
    # Save and show
    output_file = 'plot_output.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"✅ Plot saved to: {output_file}")
    plt.show()

if __name__ == "__main__":
    main()