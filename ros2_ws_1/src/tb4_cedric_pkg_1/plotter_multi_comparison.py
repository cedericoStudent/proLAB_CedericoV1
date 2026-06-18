#!/usr/bin/env python3
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from ament_index_python.packages import get_package_share_directory

WINDOW_SIZE = 15  
PLOT_STRIDE = 300  # Erhöht, damit sich die Ellipsen beider EKFs nicht überlagern

def plot_cov_ellipse(x, y, p_xx, p_xy, p_yy, ax, n_std=2.0, edgecolor='red', **kwargs):
    covariance = np.array([[p_xx, p_xy], [p_xy, p_yy]], dtype=float)
    try:
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    except np.linalg.LinAlgError:
        return None

    if eigenvalues[0] < eigenvalues[1]:
        eigenvalues = np.array([eigenvalues[1], eigenvalues[0]])
        v_max = np.array([eigenvectors[0, 1], eigenvectors[1, 1]])
    else:
        eigenvalues = np.array([eigenvalues[0], eigenvalues[1]])
        v_max = np.array([eigenvectors[0, 0], eigenvectors[1, 0]])
    
    angle = np.degrees(np.arctan2(v_max[1], v_max[0]))
    width = 2 * n_std * np.sqrt(max(0.0, eigenvalues[0]))
    height = 2 * n_std * np.sqrt(max(0.0, eigenvalues[1]))
    
    ellipse = Ellipse(xy=(x, y), width=width, height=height, angle=angle,
                      edgecolor=edgecolor, facecolor='none', **kwargs)
    return ax.add_patch(ellipse)

def main():
    try:
        pkg_share_path = get_package_share_directory('tb4_cedric_pkg_1')
        dir_path = os.path.join(pkg_share_path, 'trajectories')
        
        df_gt = pd.read_csv(os.path.join(dir_path, 'multi_path_gt.csv'))
        df_kf = pd.read_csv(os.path.join(dir_path, 'multi_path_kf.csv'))
        df_ekf_std = pd.read_csv(os.path.join(dir_path, 'multi_path_ekf_std.csv'))
        df_ekf_lm = pd.read_csv(os.path.join(dir_path, 'multi_path_ekf_lm.csv'))
        df_pf = pd.read_csv(os.path.join(dir_path, 'multi_path_pf.csv'))
    except Exception as e:
        print(f"❌ Fehler beim Laden der CSVs (Schon Daten aufgenommen?): {e}")
        return

    # Foxy-Safe Rolling Average
    for df in [df_kf, df_ekf_std, df_ekf_lm, df_pf]:
        df['x_smooth'] = df['x'].rolling(window=WINDOW_SIZE, min_periods=1).mean()
        df['y_smooth'] = df['y'].rolling(window=WINDOW_SIZE, min_periods=1).mean()

    plt.figure(figsize=(15, 11))
    ax = plt.gca()

    # Ground Truth (Gazebo)
    plt.plot(df_gt['x'].to_numpy(), df_gt['y'].to_numpy(), label='Ground Truth (Gazebo)', color='black', linewidth=3, zorder=2)
    
    # Filter-Trajektorien plotten
    plt.plot(df_kf['x_smooth'].to_numpy(), df_kf['y_smooth'].to_numpy(), label='Lineares KF', color='#ff7f0e', linestyle='-.', linewidth=1.8)
    plt.plot(df_ekf_std['x_smooth'].to_numpy(), df_ekf_std['y_smooth'].to_numpy(), label='Standard EKF (Keine Landmarken)', color='crimson', linestyle=':', linewidth=1.8)
    plt.plot(df_ekf_lm['x_smooth'].to_numpy(), df_ekf_lm['y_smooth'].to_numpy(), label='EKF (Mit Landmarken-Korrektur)', color='#2ca02c', linestyle='--', linewidth=2)
    plt.plot(df_pf['x_smooth'].to_numpy(), df_pf['y_smooth'].to_numpy(), label='Partikelfilter (PF)', color='#1f77b4', linestyle='-', linewidth=1.5)

    # Daten für Ellipsen extrahieren
    std_x, std_y = df_ekf_std['x_smooth'].to_numpy(), df_ekf_std['y_smooth'].to_numpy()
    std_pxx, std_pxy, std_pyy = df_ekf_std['p_xx'].to_numpy(), df_ekf_std['p_xy'].to_numpy(), df_ekf_std['p_yy'].to_numpy()
    
    lm_x, lm_y = df_ekf_lm['x_smooth'].to_numpy(), df_ekf_lm['y_smooth'].to_numpy()
    lm_pxx, lm_pxy, lm_pyy = df_ekf_lm['p_xx'].to_numpy(), df_ekf_lm['p_xy'].to_numpy(), df_ekf_lm['p_yy'].to_numpy()

    # Standard EKF Ellipsen (Rot - werden im Verlauf der Trajektorie immer größer wegen Drift)
    for i in range(0, len(std_x), PLOT_STRIDE):
        plot_cov_ellipse(std_x[i], std_y[i], std_pxx[i], std_pxy[i], std_pyy[i], ax, edgecolor='crimson', alpha=0.35, linewidth=1)
        
    # Landmarken EKF Ellipsen (Grün - schrumpfen schlagartig bei Landmarken-Sicht!)
    for i in range(0, len(lm_x), PLOT_STRIDE):
        plot_cov_ellipse(lm_x[i], lm_y[i], lm_pxx[i], lm_pxy[i], lm_pyy[i], ax, edgecolor='#2ca02c', alpha=0.5, linewidth=1.2)

    plt.title('Systemischer Gesamtvergleich aller Zustandsschätzer (inkl. EKF-Varianten)', fontsize=14, fontweight='bold')
    plt.xlabel('X Position [m]')
    plt.ylabel('Y Position [m]')
    plt.legend(loc='best', shadow=True)
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.axis('equal')
    
    print("► Erfolg: 5-Kanal Vergleichs-Plots generiert.")
    plt.show()

if __name__ == '__main__':
    main()