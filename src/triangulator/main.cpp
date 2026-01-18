
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <chrono>

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std::chrono;

const double FX = 377.535257164;
const double FY = 377.209841379;
const double CX = 328.193371286;
const double CY = 240.426878936;

struct CameraPose {
    Eigen::Matrix4d T_wc, T_cw;
    Eigen::Vector3d center;
    
    void SetFromTUM(double tx, double ty, double tz, double qx, double qy, double qz, double qw) {
        Eigen::Quaterniond q(qw, qx, qy, qz);
        q.normalize();
        T_wc = Eigen::Matrix4d::Identity();
        T_wc.block<3,3>(0,0) = q.toRotationMatrix();
        T_wc.block<3,1>(0,3) = Eigen::Vector3d(tx, ty, tz);
        T_cw = T_wc.inverse();
        center = T_wc.block<3,1>(0,3);
    }
};

struct Observation {
    int kf_idx;
    Eigen::Vector2d pixel;
    double depth_measured;
    double depth_projected;
    double weight;
    double confidence;
    double pose_change;
};

struct Point3D {
    int id;
    Eigen::Vector3d position_original;
    Eigen::Vector3d position_target;
    std::vector<Observation> observations;
    bool is_anchor_input;
    bool is_anchor_output;
};

class LindstromTriangulator {
private:
    std::map<int, CameraPose> poses_;
    std::map<int, Point3D> points_;
    std::map<std::string, double> timings_;
    
    // Parameters
    double displacement_scale_ = 1.0;
    double displacement_offset_ = 0.0;
    
    // Anchor mode:
    // 0 = ignore input anchor flag, treat all as moving points
    // 1 = use input anchor flag
    // 2 = re-determine anchor based on triangulated displacement
    int anchor_mode_ = 0;
    double anchor_threshold_ = 0.03;
    
    void tic(const std::string& name) {
        timings_[name] = duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
    }
    
    void toc(const std::string& name) {
        double start = timings_[name];
        double end = duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
        timings_[name] = (end - start) / 1000.0;
    }
    
public:
    void SetDisplacementParams(double scale, double offset_mm = 0) {
        displacement_scale_ = scale;
        displacement_offset_ = offset_mm / 1000.0;
    }
    
    void SetAnchorMode(int mode, double threshold_mm = 30) {
        anchor_mode_ = mode;
        anchor_threshold_ = threshold_mm / 1000.0;
    }
    
    bool LoadPoses(const std::string& file) {
        tic("1. Load Poses");
        
        std::ifstream f(file);
        if (!f) return false;
        
        std::string line;
        int id = 0;
        while (std::getline(f, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::istringstream iss(line);
            double ts, tx, ty, tz, qx, qy, qz, qw;
            if (iss >> ts >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
                CameraPose p;
                p.SetFromTUM(tx, ty, tz, qx, qy, qz, qw);
                poses_[id++] = p;
            }
        }
        
        toc("1. Load Poses");
        std::cout << "  Loaded " << poses_.size() << " poses" << std::endl;
        return true;
    }
    
    bool LoadPoints(const std::string& file) {
        tic("2. Load Points");
        
        std::ifstream f(file);
        if (!f) return false;
        
        std::string line;
        int input_anchors = 0, input_moving = 0;
        
        while (std::getline(f, line)) {
            if (line.empty() || line[0] == '#') continue;
            
            if (line[0] != ' ' && line[0] != '\t') {
                std::istringstream iss(line);
                int id, num_obs, is_anchor;
                double x, y, z;
                
                if (iss >> id >> x >> y >> z >> num_obs >> is_anchor) {
                    Point3D pt;
                    pt.id = id;
                    pt.position_original = Eigen::Vector3d(x, y, z);
                    pt.is_anchor_input = (is_anchor == 1);
                    pt.is_anchor_output = false;
                    
                    if (pt.is_anchor_input) input_anchors++; else input_moving++;
                    
                    for (int i = 0; i < num_obs; ++i) {
                        if (std::getline(f, line)) {
                            size_t first = line.find_first_not_of(" \t");
                            if (first != std::string::npos) line = line.substr(first);
                            
                            std::istringstream obs_iss(line);
                            Observation obs;
                            double dist;
                            
                            if (obs_iss >> obs.kf_idx >> obs.pixel[0] >> obs.pixel[1]
                                >> obs.depth_measured >> obs.depth_projected
                                >> obs.weight >> obs.confidence >> obs.pose_change >> dist) {
                                pt.observations.push_back(obs);
                            }
                        }
                    }
                    
                    if (!pt.observations.empty()) {
                        points_[id] = pt;
                    }
                }
            }
        }
        
        toc("2. Load Points");
        std::cout << "  Loaded " << points_.size() << " points" << std::endl;
        std::cout << "  Input: anchors=" << input_anchors << ", moving=" << input_moving << std::endl;
        return true;
    }
    
    Eigen::Vector3d Triangulate(const Point3D& point) {
        std::vector<Eigen::Vector3d> points_world;
        std::vector<double> weights;
        
        for (const auto& obs : point.observations) {
            auto it = poses_.find(obs.kf_idx);
            if (it == poses_.end()) continue;
            
            const CameraPose& pose = it->second;
            
            double z = obs.depth_measured;
            double x = (obs.pixel[0] - CX) * z / FX;
            double y = (obs.pixel[1] - CY) * z / FY;
            Eigen::Vector3d P_cam(x, y, z);
            
            Eigen::Vector3d P_world = pose.T_wc.block<3,3>(0,0) * P_cam + pose.center;
            
            double w = obs.weight * obs.confidence;
            w *= 1.0 / (1.0 + z / 3.0);
            w *= (1.0 + std::min(obs.pose_change * 15.0, 3.0));
            
            points_world.push_back(P_world);
            weights.push_back(w);
        }
        
        if (points_world.empty()) return point.position_original;
        if (points_world.size() == 1) return points_world[0];
        
        // Weighted average
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        double total_w = 0;
        for (size_t i = 0; i < points_world.size(); ++i) {
            mean += weights[i] * points_world[i];
            total_w += weights[i];
        }
        mean /= total_w;
        
        // Outlier rejection
        if (points_world.size() >= 3) {
            std::vector<double> distances;
            for (const auto& p : points_world) {
                distances.push_back((p - mean).norm());
            }
            
            std::vector<double> sorted_d = distances;
            std::sort(sorted_d.begin(), sorted_d.end());
            double median_d = sorted_d[sorted_d.size() / 2];
            double threshold = std::max(0.02, median_d * 2.5);
            
            Eigen::Vector3d filtered_mean = Eigen::Vector3d::Zero();
            double filtered_w = 0;
            int kept = 0;
            
            for (size_t i = 0; i < points_world.size(); ++i) {
                if (distances[i] < threshold) {
                    filtered_mean += weights[i] * points_world[i];
                    filtered_w += weights[i];
                    kept++;
                }
            }
            
            if (kept >= 1 && filtered_w > 0) {
                return filtered_mean / filtered_w;
            }
        }
        
        return mean;
    }
    
    void Process() {
        tic("3. Triangulation");
        
        std::vector<double> all_movements;
        
        // Step 1: Triangulate all points and compute displacement
        for (auto& kv : points_) {
            Point3D& pt = kv.second;
            
            Eigen::Vector3d triangulated = Triangulate(pt);
            Eigen::Vector3d displacement = triangulated - pt.position_original;
            double disp_norm = displacement.norm();
            
            // Apply scale and offset
            if (disp_norm > 0.0001) {
                double new_disp = disp_norm * displacement_scale_ + displacement_offset_;
                if (new_disp < 0) new_disp = 0;
                pt.position_target = pt.position_original + displacement.normalized() * new_disp;
            } else {
                pt.position_target = pt.position_original;
            }
            
            all_movements.push_back((pt.position_target - pt.position_original).norm());
        }
        
        // Step 2: Determine anchor flag based on anchor_mode
        int output_anchors = 0, output_moving = 0;
        
        for (auto& kv : points_) {
            Point3D& pt = kv.second;
            double disp = (pt.position_target - pt.position_original).norm();
            
            switch (anchor_mode_) {
                case 0:
                    // All treated as moving points
                    pt.is_anchor_output = false;
                    break;
                    
                case 1:
                    // Use input anchor flag
                    pt.is_anchor_output = pt.is_anchor_input;
                    break;
                    
                case 2:
                    // Re-determine based on triangulated displacement
                    pt.is_anchor_output = (disp < anchor_threshold_);
                    break;
                    
                default:
                    pt.is_anchor_output = false;
            }
            
            if (pt.is_anchor_output) output_anchors++;
            else output_moving++;
        }
        
        toc("3. Triangulation");
        
        // Statistics
        std::cout << "\n=== Displacement Statistics ===" << std::endl;
        if (!all_movements.empty()) {
            std::sort(all_movements.begin(), all_movements.end());
            double avg = std::accumulate(all_movements.begin(), all_movements.end(), 0.0) / all_movements.size();
            std::cout << "  All points (" << all_movements.size() << "):" << std::endl;
            std::cout << "    avg=" << avg*1000 << "mm, median=" << all_movements[all_movements.size()/2]*1000 
                      << "mm, max=" << all_movements.back()*1000 << "mm" << std::endl;
        }
        
        std::cout << "\n=== Output Anchor Assignment ===" << std::endl;
        std::cout << "  Mode: " << anchor_mode_;
        switch (anchor_mode_) {
            case 0: std::cout << " (all moving)"; break;
            case 1: std::cout << " (use input)"; break;
            case 2: std::cout << " (by threshold " << anchor_threshold_*1000 << "mm)"; break;
        }
        std::cout << std::endl;
        std::cout << "  Output: anchors=" << output_anchors << ", moving=" << output_moving << std::endl;
    }
    
    void Evaluate() {
        tic("4. Evaluation");
        
        std::vector<double> reproj_errors, depth_errors;
        
        for (const auto& kv : points_) {
            const Point3D& pt = kv.second;
            
            for (const auto& obs : pt.observations) {
                auto it = poses_.find(obs.kf_idx);
                if (it == poses_.end()) continue;
                
                Eigen::Vector4d P_w;
                P_w << pt.position_target, 1.0;
                Eigen::Vector4d P_c = it->second.T_cw * P_w;
                
                if (P_c[2] <= 0.01) continue;
                
                double u = FX * (P_c[0] / P_c[2]) + CX;
                double v = FY * (P_c[1] / P_c[2]) + CY;
                
                reproj_errors.push_back(std::sqrt(std::pow(u - obs.pixel[0], 2) + std::pow(v - obs.pixel[1], 2)));
                depth_errors.push_back(std::abs(P_c[2] - obs.depth_measured));
            }
        }
        
        toc("4. Evaluation");
        
        std::cout << "\n=== Evaluation ===" << std::endl;
        if (!reproj_errors.empty()) {
            std::sort(reproj_errors.begin(), reproj_errors.end());
            double avg = std::accumulate(reproj_errors.begin(), reproj_errors.end(), 0.0) / reproj_errors.size();
            std::cout << "  Reprojection: avg=" << avg << "px, median=" << reproj_errors[reproj_errors.size()/2] << "px" << std::endl;
        }
        if (!depth_errors.empty()) {
            std::sort(depth_errors.begin(), depth_errors.end());
            double avg = std::accumulate(depth_errors.begin(), depth_errors.end(), 0.0) / depth_errors.size();
            std::cout << "  Depth: avg=" << avg*1000 << "mm, median=" << depth_errors[depth_errors.size()/2]*1000 << "mm" << std::endl;
        }
    }
    
    void Save(const std::string& file) {
        tic("5. Save");
        
        std::ofstream f(file);
        f << std::fixed << std::setprecision(6);
        f << "# Lindstrom Triangulation\n";
        f << "# disp_scale=" << displacement_scale_ << ", disp_offset=" << displacement_offset_*1000 << "mm\n";
        f << "# anchor_mode=" << anchor_mode_ << ", anchor_threshold=" << anchor_threshold_*1000 << "mm\n";
        f << "# id x_orig y_orig z_orig x_target y_target z_target movement_mm is_anchor\n";
        
        for (const auto& kv : points_) {
            const Point3D& pt = kv.second;
            double m = (pt.position_target - pt.position_original).norm();
            
            f << pt.id << " "
              << pt.position_original[0] << " " << pt.position_original[1] << " " << pt.position_original[2] << " "
              << pt.position_target[0] << " " << pt.position_target[1] << " " << pt.position_target[2] << " "
              << m * 1000 << " " << (pt.is_anchor_output ? 1 : 0) << "\n";
        }
        
        toc("5. Save");
        std::cout << "\n  Saved: " << file << std::endl;
    }
    
    void PrintTiming() {
        std::cout << "\n============================================" << std::endl;
        std::cout << "Timing Report" << std::endl;
        std::cout << "============================================" << std::endl;
        
        double total = 0;
        std::vector<std::pair<std::string, double>> sorted_timings(timings_.begin(), timings_.end());
        std::sort(sorted_timings.begin(), sorted_timings.end());
        
        for (const auto& kv : sorted_timings) {
            std::cout << "  " << std::setw(22) << std::left << kv.first 
                      << ": " << std::fixed << std::setprecision(2) << kv.second << " ms" << std::endl;
            total += kv.second;
        }
        std::cout << "  " << std::string(35, '-') << std::endl;
        std::cout << "  " << std::setw(22) << std::left << "TOTAL" << ": " << total << " ms" << std::endl;
    }
    
    bool Run(const std::string& poses_file, const std::string& data_file, const std::string& output_file) {
        std::cout << "============================================" << std::endl;
        std::cout << "Lindstrom Triangulator" << std::endl;
        std::cout << "============================================" << std::endl;
        std::cout << "  Displacement scale:  " << displacement_scale_ << " (" << (displacement_scale_-1)*100 << "%)" << std::endl;
        std::cout << "  Displacement offset: " << displacement_offset_*1000 << " mm" << std::endl;
        std::cout << "  Anchor mode:         " << anchor_mode_;
        switch (anchor_mode_) {
            case 0: std::cout << " (all -> moving)"; break;
            case 1: std::cout << " (use input)"; break;
            case 2: std::cout << " (by threshold)"; break;
        }
        std::cout << std::endl;
        if (anchor_mode_ == 2) {
            std::cout << "  Anchor threshold:    " << anchor_threshold_*1000 << " mm" << std::endl;
        }
        std::cout << "============================================" << std::endl;
        
        std::cout << "\n=== Loading ===" << std::endl;
        if (!LoadPoses(poses_file)) return false;
        if (!LoadPoints(data_file)) return false;
        
        std::cout << "\n=== Processing ===" << std::endl;
        Process();
        Evaluate();
        Save(output_file);
        PrintTiming();
        
        return true;
    }
};

int main(int argc, char** argv) {
    std::string root = "/Datasets/Work_Data/ACL_OneLoop20251220_214841/";
    std::string loop = root + "loop_1_1670449626.805310/";
    
    // Command line arguments:
    // ./program [disp_scale] [disp_offset_mm] [anchor_mode] [anchor_threshold_mm]
    //
    // anchor_mode:
    //   0 = all treated as moving points, ARAP does not distinguish
    //   1 = use ray_caster input anchor flag
    //   2 = re-determine anchor based on triangulated displacement
    //
    // Examples:
    //   ./program                     # default: scale=1.0, offset=0, mode=0
    //   ./program 1.0 0 0             # all as moving points
    //   ./program 1.01 0 0            # scale displacement by 1%, all moving
    //   ./program 1.0 0 1             # use input anchor flag
    //   ./program 1.0 0 2 30          # re-determine anchor with 30mm threshold
    
    double disp_scale = 1.0;
    double disp_offset = 0;
    int anchor_mode = 0;
    double anchor_threshold = 30;
    
    if (argc > 1) disp_scale = std::atof(argv[1]);
    if (argc > 2) disp_offset = std::atof(argv[2]);
    if (argc > 3) anchor_mode = std::atoi(argv[3]);
    if (argc > 4) anchor_threshold = std::atof(argv[4]);
    
    LindstromTriangulator tri;
    tri.SetDisplacementParams(disp_scale, disp_offset);
    tri.SetAnchorMode(anchor_mode, anchor_threshold);
    
    tri.Run(
        loop + "post/standard_trajectory_with_loop.txt",
        root + "Output/optimization_data.txt",
        root + "Output/optimized_points_final.txt"
    );
    
    std::cout << "\n============================================" << std::endl;
    std::cout << "Complete!" << std::endl;
    std::cout << "============================================" << std::endl;
    
    return 0;
}
