#!/usr/bin/env python3
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from ament_index_python.packages import get_package_share_directory

def plot_cov_ellipse(x, y, p_xx, p_xy, p_yy, ax, n_std=2.0, edgecolor='red', **kwargs):
    """
    Berechnet und zeichnet eine präzise 2D-Kovarianzellipse.
    Verwendet die Eigenwertzerlegung der 2x2 Matrix für Orientierung und Achsenlängen.
    """
    # 2x2 Kovarianzmatrix rekonstruieren
    covariance = np.array([[p_xx, p_xy], 
                           [p_xy, p_yy]])
    
    try:
        # Eigenwerte und Eigenvektoren der symmetrischen Matrix berechnen
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    except np.linalg.LinAlgError:
        return None # Fehlervermeidung bei instabilen Systemen

    # Sortieren, damit der größte Eigenwert zuerst kommt
    order = eigenvalues.argsort()[::-1]
    eigenvalues = eigenvalues[order]
    eigenvectors = eigenvectors[:, order]
    
    # Winkel der Hauptachse zur X-Achse in Grad berechnen
    angle = np.degrees(np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]))
    
    # Durchmesser der Halbachsen berechnen (Breite/Höhe verlangen den vollen Durchmesser -> 2 *)
    # Multiplikation mit n_std (z.B. 2.0 für das 95% Konfidenzintervall / 2-Sigma)
    width = 2 * n_std * np.sqrt(max(0.0, eigenvalues[0]))
    height = 2 * n_std * np.sqrt(max(0.0, eigenvalues[1]))
    
    ellipse = Ellipse(xy=(x, y), width=width, height=height, angle=angle,
                      edgecolor=edgecolor, facecolor='none', **kwargs)
    return ax.add_patch(ellipse)

def plot_robot_data():
    # --- INTERVALL FÜR DIE ELLIPSEN-ANZEIGE ---
    # Wert verkleinern (z.B. 50) für MEHR Ellipsen, vergrößern für WENIGER Verklammerung.
    step_size = 450 
    # --------------------------------------------

    package_name = 'tb4_cedric_pkg_1'
    
    try:
        pkg_share_path = get_package_share_directory(package_name)
    except Exception as e:
        print(f"Fehler: ROS2-Package '{package_name}' nicht gefunden. Projekt gesourced?")
        return
        
    file_path_gt = os.path.join(pkg_share_path, 'trajectories', 'robot_path_GT_EKF.csv')
    file_path_ekf = os.path.join(pkg_share_path, 'trajectories', 'ekf_path1.csv')
    
    if not os.path.exists(file_path_gt) or not os.path.exists(file_path_ekf):
        print("Fehler: Mindestens eine der CSV-Dateien fehlt!")
        print(f"Gesucht unter:\n  GT:  {file_path_gt}\n  EKF: {file_path_ekf}")
        return

    # CSV laden
    data_gt = pd.read_csv(file_path_gt)
    data_ekf = pd.read_csv(file_path_ekf)
    
    if data_gt.empty or data_ekf.empty:
        print("Fehler: Eine der Trajektorien-Dateien ist komplett leer.")
        return

    # In NumPy-Arrays konvertieren
    x_gt, y_gt = data_gt['x'].to_numpy(), data_gt['y'].to_numpy()
    x_ekf, y_ekf = data_ekf['x'].to_numpy(), data_ekf['y'].to_numpy()
    p_xx = data_ekf['p_xx'].to_numpy()
    p_xy = data_ekf['p_xy'].to_numpy()
    p_yy = data_ekf['p_yy'].to_numpy()

    # Layout: 2 Plots untereinander
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 13), gridspec_kw={'height_ratios': [1.8, 1]})
    
    # =========================================================================
    # PLOT 1: TRAJEKTORIEN UND DIE ECHTEN MITDREHENDEN ELLIPSEN
    # =========================================================================
    ax1.plot(x_gt, y_gt, label='Ground Truth (Gazebo)', color='#2ca02c', linewidth=2.5, zorder=2)
    ax1.plot(x_ekf, y_ekf, label='Extended Kalman-Filter (EKF)', color='#1f77b4', linewidth=2, linestyle='--', zorder=3)
    
    # Ellipsen zeichnen
    ellipse_labeled = False
    for i in range(0, len(x_ekf), step_size):
        lbl = '$2\sigma$ Unsicherheitsellipse (mitdrehend)' if not ellipse_labeled else ""
        plot_cov_ellipse(x_ekf[i], y_ekf[i], p_xx[i], p_xy[i], p_yy[i], 
                         ax1, n_std=2.0, edgecolor='#d62728', alpha=0.6, linewidth=1.2, label=lbl)
        ellipse_labeled = True

    # Start/Ende Markierung
    ax1.scatter(x_gt[0], y_gt[0], color='blue', s=120, label='Start', zorder=5)
    ax1.scatter(x_gt[-1], y_gt[-1], color='black', marker='X', s=140, label='Ende', zorder=5)
    
    ax1.set_title('Nichtlineare EKF-Trajektorie mit richtungsorientierten Kovarianzellipsen', fontsize=14, fontweight='bold')
    ax1.set_xlabel('X Position [m]', fontsize=11)
    ax1.set_ylabel('Y Position [m]', fontsize=11)
    ax1.legend(loc='best', shadow=True)
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.axis('equal') # WICHTIG: Verhindert optische Verzerrung der Ellipsen im UI-Fenster

    # =========================================================================
    # PLOT 2: KOVARIANZ-EVOLUTION (KONTROLLE DER KREUZKOVARIANZ)
    # =========================================================================
    indices = np.arange(0, len(p_xx), 1) # Zeige vollen zeitlichen Verlauf ungestaucht
    
    ax2.plot(indices, p_xx, label='Varianz X ($P_{xx}$)', color='#ff7f0e', linewidth=2)
    ax2.plot(indices, p_yy, label='Varianz Y ($P_{yy}$)', color='#9467bd', linewidth=2, linestyle=':')
    ax2.plot(indices, p_xy, label='Kreuzkovarianz XY ($P_{xy}$)', color='#7f7f7f', linewidth=1.5, alpha=0.7)

    ax2.set_title('Verlauf der mathematischen Filterunsicherheiten (Fehler-Kovarianzen)', fontsize=14, fontweight='bold')
    ax2.set_xlabel('Messschritt (Zeitschritt-Index)', fontsize=11)
    ax2.set_ylabel('Varianzwert $[m^2]$', fontsize=11)
    ax2.legend(loc='upper right')
    ax2.grid(True, linestyle=':', alpha=0.6)

    plt.tight_layout()
    print("► Erfolg: Datenströme synchronisiert. Plots bereit!")
    plt.show()

if __name__ == "__main__":
    plot_robot_data()