import pandas as pd
import matplotlib.pyplot as plt
import os

def plot_robot_path():
    file_path = 'trajectories/robot_path.csv'
    
    if not os.path.exists(file_path):
        print(f"Fehler: {file_path} nicht gefunden!")
        return

    data = pd.read_csv(file_path)
    
    plt.figure(figsize=(10, 8))
    plt.plot(data['x'], data['y'], label='Ground Truth (Gazebo Odom)', color='green', linewidth=2)
    plt.scatter(data['x'].iloc[0], data['y'].iloc[0], color='red', label='Start') # Startpunkt
    
    plt.title('Aufgezeichnete Roboter-Trajektorie')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.legend()
    plt.grid(True, linestyle='--')
    plt.axis('equal')
    plt.show()

if __name__ == "__main__":
    plot_robot_path()