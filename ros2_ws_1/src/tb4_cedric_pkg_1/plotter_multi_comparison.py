#!/usr/bin/env python3
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from ament_index_python.packages import get_package_share_directory

WINDOW_SIZE = 15  
PLOT_STRIDE = 300  # Schrittweite für die Standard-Ellipsen (Drift-Visualisierung)

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

    # Foxy-Safe Rolling Average für glatte Trajektorien
    for df in [df_kf, df_ekf_std, df_ekf_lm, df_pf]:
        df['x_smooth'] = df['x'].rolling(window=WINDOW_SIZE, min_periods=1).mean()
        df['y_smooth'] = df['y'].rolling(window=WINDOW_SIZE, min_periods=1).mean()

    plt.figure(figsize=(15, 11))
    ax = plt.gca()

    # 1. Ground Truth (Gazebo) als fette Referenzlinie im Hintergrund
    plt.plot(df_gt['x'].to_numpy(), df_gt['y'].to_numpy(), label='Ground Truth (Gazebo)', color='black', linewidth=3, zorder=2)
    
    # 2. Filter-Trajektorien plotten
    plt.plot(df_kf['x_smooth'].to_numpy(), df_kf['y_smooth'].to_numpy(), label='Lineares KF', color='#ff7f0e', linestyle='-.', linewidth=1.8)
    plt.plot(df_ekf_std['x_smooth'].to_numpy(), df_ekf_std['y_smooth'].to_numpy(), label='Standard EKF (Keine Landmarken)', color='crimson', linestyle=':', linewidth=1.8)
    plt.plot(df_ekf_lm['x_smooth'].to_numpy(), df_ekf_lm['y_smooth'].to_numpy(), label='EKF (Mit Landmarken-Korrektur)', color='#2ca02c', linestyle='--', linewidth=2)
    plt.plot(df_pf['x_smooth'].to_numpy(), df_pf['y_smooth'].to_numpy(), label='Partikelfilter (PF)', color='#1f77b4', linestyle='-', linewidth=1.5)

    # ==========================================
    # NEU: LANDMARKE (PFIOSTEN) EINZEICHNEN
    # ==========================================
    # Ein großer, deutlicher Stern mit hoher Z-Order, damit er über den Trajektorien liegt
    plt.scatter(-1.2, 1.2, color='black', edgecolors='crimson', marker='*', s=350, linewidths=1.5, zorder=10, 
                label='📍 Reale Landmarke (Zylinder bei -1.2m, 1.2m)')

    # ==========================================
    # ELLIPSEN-PLOTTING STANDARD EKF (ROT)
    # ==========================================
    #std_x, std_y = df_ekf_std['x_smooth'].to_numpy(), df_ekf_std['y_smooth'].to_numpy()
    #std_pxx, std_pxy, std_pyy = df_ekf_std['p_xx'].to_numpy(), df_ekf_std['p_xy'].to_numpy(), df_ekf_std['p_yy'].to_numpy()
    
    #for i in range(0, len(std_x), PLOT_STRIDE):
    #    plot_cov_ellipse(std_x[i], std_y[i], std_pxx[i], std_pxy[i], std_pyy[i], ax, edgecolor='crimson', alpha=0.3, linewidth=1)
        
    # ==========================================
    # ELLIPSEN-PLOTTING LANDMARKEN EKF (GRÜN)
    # ==========================================
    lm_x, lm_y = df_ekf_lm['x_smooth'].to_numpy(), df_ekf_lm['y_smooth'].to_numpy()
    lm_pxx, lm_pxy, lm_pyy = df_ekf_lm['p_xx'].to_numpy(), df_ekf_lm['p_xy'].to_numpy(), df_ekf_lm['p_yy'].to_numpy()
    
    # Prüfen, ob die Spalte existiert (Abwärtskompatibilität)
    if 'landmark_detected' in df_ekf_lm.columns:
        lm_detected = df_ekf_lm['landmark_detected'].to_numpy()
    else:
        lm_detected = np.zeros(len(lm_x))

    last_drawn_landmark_idx = -100  # Hilfsvariable gegen optischen "Zusammenstau" bei Dauersicht

    for i in range(0, len(lm_x)):
        # Bedingung 1: Feste Schrittweite für die Standard-Ellipsen im Odometriebetrieb
        is_stride_step = (i % PLOT_STRIDE == 0)
        
        # Bedingung 2: Landmarke wurde exakt hier erkannt UND wir haben seit mindestens 25 Zeitschritten 
        # keine Landmarken-Ellipse mehr gezeichnet (verhindert fette grüne Klumpen im Plot)
        is_fresh_landmark = (lm_detected[i] > 0.5) and (i - last_drawn_landmark_idx > 25)

        if is_stride_step or is_fresh_landmark:
            if lm_detected[i] > 0.5:
                # MARKANTE dunkelgrüne, feine Ellipse für den exakten Moment des sensorischen Updates
                plot_cov_ellipse(lm_x[i], lm_y[i], lm_pxx[i], lm_pxy[i], lm_pyy[i], ax, 
                                 edgecolor='darkgreen', alpha=0.8, linewidth=2.0)
                last_drawn_landmark_idx = i
            else:
                # Normale hellgrüne Ellipse im laufenden Driftbetrieb
                plot_cov_ellipse(lm_x[i], lm_y[i], lm_pxx[i], lm_pxy[i], lm_pyy[i], ax, 
                                 edgecolor='#2ca02c', alpha=0.35, linewidth=1.0)

    # Diagramm-Styling für wissenschaftliche Arbeiten
    plt.title('Gesamtvergleich der Zustandsschätzer', fontsize=14, fontweight='bold')
    plt.xlabel('X Position [m]', fontsize=12)
    plt.ylabel('Y Position [m]', fontsize=12)
    
    # Dummy-Plots für eine saubere, erweiterte Legende hinzufügen
    plt.plot([], [], ' ', label='-----------------------------------------')
    #plt.plot([], [], color='crimson', alpha=0.5, linewidth=1, label='Unsicherheit Standard EKF (wächst)')
    plt.plot([], [], color='#2ca02c', alpha=0.5, linewidth=1, label='Unsicherheit Landmarken EKF (Drift)')
    plt.plot([], [], color='darkgreen', alpha=0.9, linewidth=2, label='🎯 Landmarken-Update (Kovarianz schrumpft)')

    plt.legend(loc='best', shadow=True, fontsize=10)
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.axis('equal')
    
    print("► Erfolg: 5-Kanal Vergleichs-Plots inklusive Landmarken-Position und dynamischer Kovarianz generiert.")
    plt.show()

if __name__ == '__main__':
    main()