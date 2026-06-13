import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory

def plot_robot_path():
    try:
        package_name = 'tb4_cedric_pkg_1'
        pkg_share_path = get_package_share_directory(package_name)
        
        # Pfade zu beiden Dateien auflösen
        file_path_gt = os.path.join(pkg_share_path, 'trajectories', 'robot_path1.csv')
        file_path_kf = os.path.join(pkg_share_path, 'trajectories', 'kf_path1.csv')
        
        if not os.path.exists(file_path_gt) or not os.path.exists(file_path_kf):
            print("Fehler: Mindestens eine der CSV-Dateien wurde nicht gefunden!")
            return

        # Daten einlesen
        data_gt = pd.read_csv(file_path_gt)
        data_kf = pd.read_csv(file_path_kf)
        
        if data_gt.empty or data_kf.empty:
            print("Eine der CSV-Dateien ist leer!")
            return

        # Konvertierung in NumPy Arrays
        x_gt = data_gt['x'].to_numpy()
        y_gt = data_gt['y'].to_numpy()

        x_kf = data_kf['x'].to_numpy()
        y_kf = data_kf['y'].to_numpy()

        plt.figure(figsize=(10, 8))
        
        # Beide Trajektorien plotten
        plt.plot(x_gt, y_gt, label='Ground Truth (Gazebo Odom)', color='green', linewidth=2)
        plt.plot(x_kf, y_kf, label='Kalman-Filter (KF Schätzung)', color='blue', linewidth=2, linestyle='--')
        
        # Start- und Endpunkte der Ground Truth markieren
        plt.scatter(x_gt[0], y_gt[0], color='red', s=100, label='Start (GT)', zorder=5)
        plt.scatter(x_gt[-1], y_gt[-1], color='black', marker='X', s=100, label='Ende (GT)', zorder=5)
        
        plt.title('Vergleich: Ground Truth vs. Kalman-Filter Trajektorie')
        plt.xlabel('X Position [m]')
        plt.ylabel('Y Position [m]')
        plt.legend()
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.axis('equal')
        
        print(f"Erfolg! Trajektorien geladen.")
        plt.show()

    except Exception as e:
        print(f"Ein Fehler ist aufgetreten: {e}")

if __name__ == "__main__":
    plot_robot_path()