import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np # Neu hinzufügen
from ament_index_python.packages import get_package_share_directory

def plot_robot_path():
    try:
        package_name = 'tb4_cedric_pkg_1'
        pkg_share_path = get_package_share_directory(package_name)
        file_path = os.path.join(pkg_share_path, 'trajectories', 'robot_path1.csv')
        
        if not os.path.exists(file_path):
            print(f"Fehler: Datei nicht gefunden unter: {file_path}")
            return

        data = pd.read_csv(file_path)
        
        if data.empty:
            print("Die CSV-Datei ist leer!")
            return

        # --- REPARATUR: Konvertierung in NumPy Arrays ---
        x_values = data['x'].to_numpy()
        y_values = data['y'].to_numpy()

        plt.figure(figsize=(10, 8))
        
        # Jetzt nutzen wir die reinen NumPy-Arrays für den Plot
        plt.plot(x_values, y_values, label='Ground Truth (Gazebo Odom)', color='green', linewidth=2)
        
        # Start- und Endpunkte ebenfalls über NumPy-Indizierung
        plt.scatter(x_values[0], y_values[0], color='red', s=100, label='Start', zorder=5)
        plt.scatter(x_values[-1], y_values[-1], color='black', marker='X', s=100, label='Ende', zorder=5)
        
        plt.title('Aufgezeichnete Roboter-Trajektorie (Ground Truth)')
        plt.xlabel('X Position [m]')
        plt.ylabel('Y Position [m]')
        plt.legend()
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.axis('equal')
        
        print(f"Erfolg! Plot wird geladen aus: {file_path}")
        plt.show()

    except Exception as e:
        print(f"Ein Fehler ist aufgetreten: {e}")

if __name__ == "__main__":
    plot_robot_path()