#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

struct PoseData {
    double x;
    double y;
};

// Robustes Einlesen ohne hängende Zeitstempel-Konvertierungen
std::vector<PoseData> read_csv(const std::string& filepath) {
    std::vector<PoseData> data;
    std::ifstream file(filepath);
    if (!file.is_open()) {
        return data;
    }

    std::string line;
    // Header-Zeile überspringen (z.B. x,y,p_xx...)
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string item;
        std::vector<std::string> row;

        while (std::getline(ss, item, ',')) {
            row.push_back(item);
        }

        // Wir brauchen mindestens 2 Spalten (x und y)
        if (row.size() < 2) continue;

        try {
            PoseData pose;
            // EGAL was dahinter kommt: Index 0 ist IMMER x, Index 1 ist IMMER y
            pose.x = std::stod(row[0]);
            pose.y = std::stod(row[1]);
            data.push_back(pose);
        } catch (...) {
            // Ignoriere fehlerhafte Zeilen am Datei-Ende
            continue;
        }
    }
    return data;
}

void calculate_metrics(const std::string& name, const std::vector<PoseData>& filter_data, const std::vector<PoseData>& gt_data) {
    if (filter_data.empty() || gt_data.empty()) {
        std::cout << std::left << std::setw(35) << name << " | ⚠️ Keine Daten vorhanden." << std::endl;
        return;
    }

    double sum_sq_error = 0.0;
    // Wir vergleichen bis zur maximal verfügbaren Länge der kürzeren Liste
    size_t compare_points = std::min(filter_data.size(), gt_data.size());

    for (size_t i = 0; i < compare_points; ++i) {
        double err_x = filter_data[i].x - gt_data[i].x;
        double err_y = filter_data[i].y - gt_data[i].y;
        sum_sq_error += (err_x * err_x + err_y * err_y);
    }

    double rmse = std::sqrt(sum_sq_error / compare_points);

    // Absolute Abweichung der jeweils letzten Position
    double final_err_x = filter_data.back().x - gt_data.back().x;
    double final_err_y = filter_data.back().y - gt_data.back().y;
    double final_distance = std::sqrt(final_err_x * final_err_x + final_err_y * final_err_y);

    std::cout << std::left << std::setw(35) << name 
              << " | RMSE: " << std::fixed << std::setprecision(4) << std::setw(8) << rmse << " m"
              << " | Finaler Abs. Fehler: " << std::setw(8) << final_distance << " m" << std::endl;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    std::string pkg_share_path;
    try {
        pkg_share_path = ament_index_cpp::get_package_share_directory("tb4_cedric_pkg_1");
    } catch (const std::exception& e) {
        std::cerr << "❌ Package nicht gefunden: " << e.what() << std::endl;
        return 1;
    }

    std::string dir_path = pkg_share_path + "/trajectories/";

    std::cout << "\n=========================================================================" << std::endl;
    std::cout << "📊 STARTE EVALUIERUNG DER ZUSTANDSSCHÄTZER" << std::endl;
    std::cout << "📂 Pfad: " << dir_path << std::endl;
    std::cout << "=========================================================================\n" << std::endl;

    auto df_gt       = read_csv(dir_path + "multi_path_gt.csv");
    auto df_kf       = read_csv(dir_path + "multi_path_ekf_std.csv");  
    auto df_ekf_std  = read_csv(dir_path + "multi_path_kf.csv");       
    auto df_ekf_lm   = read_csv(dir_path + "multi_path_ekf_lm.csv");
    auto df_pf       = read_csv(dir_path + "multi_path_pf.csv");

    if (df_gt.empty()) {
        std::cerr << "❌ Fehler: Ground-Truth-Datei (" << dir_path << "multi_path_gt.csv) konnte nicht geparsed werden!" << std::endl;
        return 1;
    }

    calculate_metrics("Lineares Kalman Filter (KF)", df_kf, df_gt);
    calculate_metrics("Standard EKF (Ohne Landmarken)", df_ekf_std, df_gt);
    calculate_metrics("EKF (Mit Landmarken-Korrektur)", df_ekf_lm, df_gt);
    calculate_metrics("Partikelfilter (PF)", df_pf, df_gt);

    std::cout << "\n=========================================================================" << std::endl;
    
    rclcpp::shutdown();
    return 0;
}