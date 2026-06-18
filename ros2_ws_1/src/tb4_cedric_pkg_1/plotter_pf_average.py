#!/usr/bin/env python3
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from ament_index_python.packages import get_package_share_directory

# ==========================================
# SEITEN-PARAMETER FÜR DIE DARSTELLUNG
# ==========================================
WINDOW_SIZE = 15  # Größe des gleitenden Fensters für den Rolling Average
PLOT_STRIDE = 20  # Downsampling: Zeichnet nur jede 20. Ellipse


def plot_cov_ellipse(x, y, p_xx, p_xy, p_yy, ax, n_std=2.0, edgecolor='red', **kwargs):
    """
    Berechnet und zeichnet eine präzise 2D-Kovarianzellipse.
    Absolut FOXY-SAFE: Verhindert den 'obj[:, None]' Multi-dimensional Indexing Fehler.
    """
    covariance = np.array([[p_xx, p_xy], 
                           [p_xy, p_yy]], dtype=float)
    try:
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    except np.linalg.LinAlgError:
        return None

    # FOXY-FIX: Sortierung ohne das fehlerhafte [:, order] Slicing
    if eigenvalues[0] < eigenvalues[1]:
        eigenvalues = np.array([eigenvalues[1], eigenvalues[0]])
        # Spalten der Eigenvektoren manuell zuweisen
        v_max = np.array([eigenvectors[0, 1], eigenvectors[1, 1]])
    else:
        eigenvalues = np.array([eigenvalues[0], eigenvalues[1]])
        v_max = np.array([eigenvectors[0, 0], eigenvectors[1, 0]])
    
    # Winkel zur globalen X-Achse berechnen
    angle = np.degrees(np.arctan2(v_max[1], v_max[0]))
    
    width = 2 * n_std * np.sqrt(max(0.0, eigenvalues[0]))
    height = 2 * n_std * np.sqrt(max(0.0, eigenvalues[1]))
    
    ellipse = Ellipse(xy=(x, y), width=width, height=height, angle=angle,
                      edgecolor=edgecolor, facecolor='none', **kwargs)
    return ax.add_patch(ellipse)


def plot_combined_trajectory(gt_file, est_file):
    if not os.path.exists(gt_file) or not os.path.exists(est_file):
        raise FileNotFoundError("Eine oder beide CSV-Dateien wurden nicht gefunden!")

    try:
        data_gt = pd.read_csv(gt_file)
        data_est = pd.read_csv(est_file)
    except Exception as e:
        print(f"❌ Fehler beim Einlesen der CSV-Dateien: {e}")
        return

    # 1. ROLLING AVERAGE (Gleitender Mittelwert auf den Schätzdaten gegen das Zittern)
    data_est['x_smooth'] = data_est['x'].rolling(window=WINDOW_SIZE, min_periods=1).mean()
    data_est['y_smooth'] = data_est['y'].rolling(window=WINDOW_SIZE, min_periods=1).mean()

    # 2. SOFORTIGE NUMPY-KONVERTIERUNG (Kills all Pandas tracking issues)
    x_gt, y_gt = data_gt['x'].to_numpy(), data_gt['y'].to_numpy()
    x_est, y_est = data_est['x'].to_numpy(), data_est['y'].to_numpy()
    x_smooth = data_est['x_smooth'].to_numpy()
    y_smooth = data_est['y_smooth'].to_numpy()
    
    p_xx = data_est['p_xx'].to_numpy()
    p_xy = data_est['p_xy'].to_numpy()
    p_yy = data_est['p_yy'].to_numpy()

    # Layout: 2 Plots untereinander
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 13), gridspec_kw={'height_ratios': [1.8, 1]})
    
    # =========================================================================
    # PLOT 1: Trajektorien & Unsicherheiten
    # =========================================================================
    ax1.plot(x_gt, y_gt, label='Ground Truth (Gazebo)', color='#2ca02c', linewidth=2.5, zorder=2)
    ax1.plot(x_est, y_est, label='Filter Rohdaten (Zittrig)', color='#1f77b4', alpha=0.15, linewidth=1.0, zorder=1)
    ax1.plot(x_smooth, y_smooth, label=f'Geglättete Schätzung (Window: {WINDOW_SIZE})', color='blue', linestyle='--', linewidth=2, zorder=3)
    
    # Ellipsen zeichnen (über die glatte Bahn via Schrittweite PLOT_STRIDE)
    ellipse_labeled = False
    for i in range(0, len(x_smooth), PLOT_STRIDE):
        lbl = '$2\sigma$ Unsicherheitsellipse' if not ellipse_labeled else ""
        plot_cov_ellipse(x_smooth[i], y_smooth[i], p_xx[i], p_xy[i], p_yy[i], 
                         ax1, n_std=2.0, edgecolor='#d62728', alpha=0.5, linewidth=1.2, label=lbl)
        ellipse_labeled = True

    ax1.scatter(x_smooth[0], y_smooth[0], color='blue', s=120, label='Start', zorder=5)
    ax1.scatter(x_smooth[-1], y_smooth[-1], color='black', marker='X', s=140, label='Ende', zorder=5)
    
    ax1.set_title('Geglättete Schätztrajektorie mit Kovarianzellipsen', fontsize=14, fontweight='bold')
    ax1.set_xlabel('X Position [m]', fontsize=11)
    ax1.set_ylabel('Y Position [m]', fontsize=11)
    ax1.legend(loc='best', shadow=True)
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.axis('equal')

    # =========================================================================
    # PLOT 2: Kovarianz-Evolution
    # =========================================================================
    indices = np.arange(0, len(p_xx), 1)
    ax2.plot(indices, p_xx, label='Varianz X ($P_{xx}$)', color='#ff7f0e', linewidth=2)
    ax2.plot(indices, p_yy, label='Varianz Y ($P_{yy}$)', color='#9467bd', linewidth=2, linestyle=':')
    ax2.plot(indices, p_xy, label='Kreuzkovarianz XY ($P_{xy}$)', color='#7f7f7f', linewidth=1.5, alpha=0.7)

    ax2.set_title('Verlauf der mathematischen Filterunsicherheiten', fontsize=14, fontweight='bold')
    ax2.set_xlabel('Messschritt (Zeitschritt-Index)', fontsize=11)
    ax2.set_ylabel('Varianzwert $[m^2]$', fontsize=11)
    ax2.legend(loc='upper right')
    ax2.grid(True, linestyle=':', alpha=0.6)

    plt.tight_layout()
    print("► Erfolg: Daten visualisiert.")
    plt.show()


if __name__ == "__main__":
    try:
        pkg_share_path = get_package_share_directory('tb4_cedric_pkg_1')
        file_path_gt = os.path.join(pkg_share_path, 'trajectories', 'robot_path_GT_EKF.csv')
        file_path_ekf = os.path.join(pkg_share_path, 'trajectories', 'ekf_path1.csv')
        plot_combined_trajectory(file_path_gt, file_path_ekf)
    except Exception as e:
        print(f"\n❌ Kritischer Fehler beim Ausführen: {e}")