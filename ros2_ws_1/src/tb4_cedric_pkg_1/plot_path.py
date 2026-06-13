import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from ament_index_python.packages import get_package_share_directory

def plot_cov_ellipse(x, y, p_xx, p_xy, p_yy, ax, n_std=2.0, edgecolor='red', **kwargs):
    """ Berechnet und zeichnet eine 2D-Kovarianzellipse """
    covariance = np.array([[p_xx, p_xy], [p_xy, p_yy]])
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    
    order = eigenvalues.argsort()[::-1]
    eigenvalues = eigenvalues[order]
    eigenvectors = eigenvectors[:, order]
    
    angle = np.degrees(np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]))
    
    width = 2 * n_std * np.sqrt(max(0, eigenvalues[0]))
    height = 2 * n_std * np.sqrt(max(0, eigenvalues[1]))
    
    ellipse = Ellipse(xy=(x, y), width=width, height=height, angle=angle,
                      edgecolor=edgecolor, facecolor='none', **kwargs)
    return ax.add_patch(ellipse)

def plot_robot_data():
    # --- KONFIGURATION ---
    # Hier kannst du einstellen, jeder wievielte Eintrag geplottet wird
    step_size = 100 
    # ---------------------

    try:
        package_name = 'tb4_cedric_pkg_1'
        pkg_share_path = get_package_share_directory(package_name)
        
        file_path_gt = os.path.join(pkg_share_path, 'trajectories', 'robot_path1.csv')
        file_path_kf = os.path.join(pkg_share_path, 'trajectories', 'kf_path1.csv')
        
        if not os.path.exists(file_path_gt) or not os.path.exists(file_path_kf):
            print("Fehler: Mindestens eine der CSV-Dateien fehlt!")
            return

        data_gt = pd.read_csv(file_path_gt)
        data_kf = pd.read_csv(file_path_kf)
        
        if data_gt.empty or data_kf.empty:
            print("Eine der CSV-Dateien ist leer!")
            return

        # Daten extrahieren
        x_gt, y_gt = data_gt['x'].to_numpy(), data_gt['y'].to_numpy()
        x_kf, y_kf = data_kf['x'].to_numpy(), data_kf['y'].to_numpy()
        p_xx = data_kf['p_xx'].to_numpy()
        p_xy = data_kf['p_xy'].to_numpy()
        p_yy = data_kf['p_yy'].to_numpy()

        # Erstelle ein ungleiches Layout: Oben die Trajektorie, unten die Kovarianz über Zeit
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 12), gridspec_kw={'height_ratios': [2, 1]})
        
        # ==========================================
        # PLOT 1: TRAJEKTORIE MIT ELLIPSEN
        # ==========================================
        ax1.plot(x_gt, y_gt, label='Ground Truth (Gazebo)', color='green', linewidth=2)
        ax1.plot(x_kf, y_kf, label='Kalman-Filter (KF)', color='blue', linewidth=2, linestyle='--')
        
        # Ellipsen basierend auf der variablen Schrittweite zeichnen
        for i in range(0, len(x_kf), step_size):
            lbl = '$2\sigma$ Unsicherheit' if i == 0 else ""
            plot_cov_ellipse(x_kf[i], y_kf[i], p_xx[i], p_xy[i], p_yy[i], 
                             ax1, n_std=2.0, edgecolor='red', alpha=0.5, linewidth=1, label=lbl)

        ax1.scatter(x_gt[0], y_gt[0], color='red', s=100, label='Start', zorder=5)
        ax1.scatter(x_gt[-1], y_gt[-1], color='black', marker='X', s=100, label='Ende', zorder=5)
        
        ax1.set_title('KF Trajektorie mit Kovarianzellipsen')
        ax1.set_xlabel('X Position [m]')
        ax1.set_ylabel('Y Position [m]')
        ax1.legend()
        ax1.grid(True, linestyle='--', alpha=0.5)
        ax1.axis('equal')

        # ==========================================
        # PLOT 2: KOVARIANZ ÜBER DIE ZEIT (Schrittweite berücksichtigt)
        # ==========================================
        # Indizes für die reduzierte Darstellung erzeugen
        indices = np.arange(0, len(p_xx), step_size)
        
        # Plot der Varianzen für x und y
        ax2.plot(indices, p_xx[indices], label='Varianz X ($P_{xx}$)', color='darkorange', linewidth=2)
        ax2.plot(indices, p_yy[indices], label='Varianz Y ($P_{yy}$)', color='purple', linewidth=2, linestyle=':')
        
        # Optional: Zeige auch die Kreuzkovarianz (sollte beim KF gegen 0 gehen)
        ax2.plot(indices, p_xy[indices], label='Kreuzkovarianz XY ($P_{xy}$)', color='gray', alpha=0.5)

        ax2.set_title(f'Entwicklung der Filterunsicherheit über die Zeit (Jeder {step_size}. Eintrag)')
        ax2.set_xlabel('Messschritt (Index)')
        ax2.set_ylabel('Varianz $[m^2]$')
        ax2.legend()
        ax2.grid(True, linestyle='--', alpha=0.5)

        plt.tight_layout()
        print("Erfolg! Beide Plots wurden generiert.")
        plt.show()

    except Exception as e:
        print(f"Ein Fehler ist aufgetreten: {e}")

if __name__ == "__main__":
    plot_robot_data()