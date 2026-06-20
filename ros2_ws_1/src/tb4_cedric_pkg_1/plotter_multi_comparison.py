#!/usr/bin/env python3
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Rectangle, Circle  # Circle hinzugefügt
from ament_index_python.packages import get_package_share_directory

WINDOW_SIZE = 15  
PLOT_STRIDE = 300  

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
        df_kf = pd.read_csv(os.path.join(dir_path, 'multi_path_ekf_std.csv'))
        df_ekf_std = pd.read_csv(os.path.join(dir_path, 'multi_path_kf.csv'))
        df_ekf_lm = pd.read_csv(os.path.join(dir_path, 'multi_path_ekf_lm.csv'))
        df_pf = pd.read_csv(os.path.join(dir_path, 'multi_path_pf.csv'))
    except Exception as e:
        print(f"❌ Fehler beim Laden der CSVs (Schon Daten aufgenommen?): {e}")
        return

    for df in [df_kf, df_ekf_std, df_ekf_lm, df_pf]:
        df['x_smooth'] = df['x'].rolling(window=WINDOW_SIZE, min_periods=1).mean()
        df['y_smooth'] = df['y'].rolling(window=WINDOW_SIZE, min_periods=1).mean()

    plt.figure(figsize=(13, 11))
    ax = plt.gca()

    # =====================================================================
    # MAẞSTABSGETREUE WÄNDE (AUS SDF)
    # =====================================================================
    wall_color = '#7f8c8d'  # Labor-Grau
    w_thick = 0.1           # Box size in SDF ist 0.1 dick
    w_len = 4.1             # Box-Länge laut SDF ist 4.1

    # Nord-Wand (Pose: 0, 2) -> Untere linke Ecke berechnen für Rectangle
    ax.add_patch(Rectangle((-w_len/2, 2.0 - w_thick/2), w_len, w_thick, color=wall_color, alpha=0.6, zorder=1))
    # Süd-Wand (Pose: 0, -2)
    ax.add_patch(Rectangle((-w_len/2, -2.0 - w_thick/2), w_len, w_thick, color=wall_color, alpha=0.6, zorder=1))
    # Ost-Wand (Pose: 2, 0)
    ax.add_patch(Rectangle((2.0 - w_thick/2, -w_len/2), w_thick, w_len, color=wall_color, alpha=0.6, zorder=1))
    # West-Wand (Pose: -2, 0)
    ax.add_patch(Rectangle((-2.0 - w_thick/2, -w_len/2), w_thick, w_len, color=wall_color, alpha=0.6, zorder=1))

    # Dummy-Label für die Wände
    plt.plot([], [], color=wall_color, alpha=0.6, linewidth=8, label='🧱 Simulations-Wände (SDF-Begrenzung)')

    # =====================================================================
    # MAẞSTABSGETREUE LANDMARKE (AUS SDF)
    # =====================================================================
    # landmark_medium: Pose -1.2, 1.2 mit Radius 0.35
    landmark_circle = Circle((-1.2, 1.2), radius=0.35, facecolor='#34495e', edgecolor='crimson', linewidth=1.5, zorder=10, label='📍 Zylinder-Landmarke (r=0.35m)')
    ax.add_patch(landmark_circle)

    # 1. Ground Truth (Gazebo)
    plt.plot(df_gt['x'].to_numpy(), df_gt['y'].to_numpy(), label='Ground Truth (Gazebo)', color='black', linewidth=3, zorder=2)
    
    # 2. Filter-Trajektorien plotten
    plt.plot(df_kf['x_smooth'].to_numpy(), df_kf['y_smooth'].to_numpy(), label='Lineares KF', color='#ff7f0e', linestyle='-.', linewidth=1.8)
    plt.plot(df_ekf_std['x_smooth'].to_numpy(), df_ekf_std['y_smooth'].to_numpy(), label='Standard EKF (Keine Landmarken)', color='crimson', linestyle=':', linewidth=1.8)
    plt.plot(df_ekf_lm['x_smooth'].to_numpy(), df_ekf_lm['y_smooth'].to_numpy(), label='EKF (Mit Landmarken-Korrektur)', color='#2ca02c', linestyle='--', linewidth=2)
    plt.plot(df_pf['x_smooth'].to_numpy(), df_pf['y_smooth'].to_numpy(), label='Partikelfilter (PF)', color='#1f77b4', linestyle='-', linewidth=1.5)

    # Ellipsen-Plotting Landmarken EKF (Grün)
    lm_x, lm_y = df_ekf_lm['x_smooth'].to_numpy(), df_ekf_lm['y_smooth'].to_numpy()
    lm_pxx, lm_pxy, lm_pyy = df_ekf_lm['p_xx'].to_numpy(), df_ekf_lm['p_xy'].to_numpy(), df_ekf_lm['p_yy'].to_numpy()
    
    if 'landmark_detected' in df_ekf_lm.columns:
        lm_detected = df_ekf_lm['landmark_detected'].to_numpy()
    else:
        lm_detected = np.zeros(len(lm_x))

    last_drawn_landmark_idx = -100  

    for i in range(0, len(lm_x)):
        is_stride_step = (i % PLOT_STRIDE == 0)
        is_fresh_landmark = (lm_detected[i] > 0.5) and (i - last_drawn_landmark_idx > 25)

        if is_stride_step or is_fresh_landmark:
            if lm_detected[i] > 0.5:
                plot_cov_ellipse(lm_x[i], lm_y[i], lm_pxx[i], lm_pxy[i], lm_pyy[i], ax, 
                                 edgecolor='darkgreen', alpha=0.8, linewidth=2.0, zorder=5)
                last_drawn_landmark_idx = i
            else:
                plot_cov_ellipse(lm_x[i], lm_y[i], lm_pxx[i], lm_pxy[i], lm_pyy[i], ax, 
                                 edgecolor='#2ca02c', alpha=0.35, linewidth=1.0, zorder=4)

    # Diagramm-Styling für wissenschaftliche Arbeiten
    plt.title('Gesamtvergleich der Zustandsschätzer', fontsize=14, fontweight='bold')
    plt.xlabel('X Position [m]', fontsize=12)
    plt.ylabel('Y Position [m]', fontsize=12)
    
    plt.plot([], [], ' ', label='-----------------------------------------')
    plt.plot([], [], color='#2ca02c', alpha=0.5, linewidth=1, label='Unsicherheit Landmarken EKF (Drift)')
    plt.plot([], [], color='darkgreen', alpha=0.9, linewidth=2, label='🎯 Landmarken-Update (Kovarianz schrumpft)')

    plt.legend(loc='best', shadow=True, fontsize=10)
    plt.grid(True, linestyle=':', alpha=0.6)
    
    # Grenzen exakt auf die echten Wände + Puffer zentrieren
    plt.xlim(-2.5, 2.5)
    plt.ylim(-2.5, 2.5)
    plt.axis('equal')
    
    print("► Erfolg: SDF-getreuer Plotter generiert. Wände und Landmarke sind mathematisch exakt ausgerichtet.")
    plt.show()

if __name__ == '__main__':
    main()