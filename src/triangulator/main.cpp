// // // // // #include <iostream>
// // // // // #include <fstream>
// // // // // #include <sstream>
// // // // // #include <string>
// // // // // #include <vector>
// // // // // #include <map>
// // // // // #include <iomanip>
// // // // // #include <algorithm>
// // // // // #include <numeric>
// // // // // #include <memory>

// // // // // #include <Eigen/Dense>
// // // // // #include <Eigen/Geometry>
// // // // // #include <ceres/ceres.h>
// // // // // #include <ceres/rotation.h>

// // // // // // Camera intrinsics
// // // // // const double FX = 377.535257164;
// // // // // const double FY = 377.209841379;
// // // // // const double CX = 328.193371286;
// // // // // const double CY = 240.426878936;

// // // // // // Camera pose
// // // // // struct CameraPose {
// // // // //     int frame_id;
// // // // //     double timestamp;
// // // // //     Eigen::Matrix4d T_wc;  // World to Camera
// // // // //     Eigen::Matrix4d T_cw;  // Camera to World
// // // // //     std::vector<double> pose_data;  // For Ceres optimization [tx, ty, tz, qx, qy, qz, qw]
    
// // // // //     CameraPose() : pose_data(7, 0.0) {
// // // // //         pose_data[6] = 1.0;  // qw = 1
// // // // //     }
    
// // // // //     void SetFromTUM(double tx, double ty, double tz, 
// // // // //                     double qx, double qy, double qz, double qw) {
// // // // //         // TUM format is T_wc
// // // // //         Eigen::Vector3d t_wc(tx, ty, tz);
// // // // //         Eigen::Quaterniond q_wc(qw, qx, qy, qz);
// // // // //         q_wc.normalize();
        
// // // // //         T_wc = Eigen::Matrix4d::Identity();
// // // // //         T_wc.block<3, 3>(0, 0) = q_wc.toRotationMatrix();
// // // // //         T_wc.block<3, 1>(0, 3) = t_wc;
        
// // // // //         T_cw = T_wc.inverse();
        
// // // // //         // Also fill pose_data (T_cw format) for Ceres
// // // // //         Eigen::Vector3d t_cw = T_cw.block<3, 1>(0, 3);
// // // // //         Eigen::Matrix3d R_cw = T_cw.block<3, 3>(0, 0);
// // // // //         Eigen::Quaterniond q_cw(R_cw);
        
// // // // //         pose_data[0] = t_cw[0];
// // // // //         pose_data[1] = t_cw[1];
// // // // //         pose_data[2] = t_cw[2];
// // // // //         pose_data[3] = q_cw.x();
// // // // //         pose_data[4] = q_cw.y();
// // // // //         pose_data[5] = q_cw.z();
// // // // //         pose_data[6] = q_cw.w();
// // // // //     }
// // // // // };

// // // // // // Voxblox observation data
// // // // // // struct VoxbloxObservation {
// // // // // //     int kf_idx;
// // // // // //     Eigen::Vector2d pixel;
// // // // // //     double depth_measured;
// // // // // //     Eigen::Vector3d P_cam;  // 3D position in camera coordinate system
// // // // // //     double weight;          // Observation weight
// // // // // //     double confidence;      // Confidence
// // // // // // };
// // // // // // ============ 修改后 ============
// // // // // struct VoxbloxObservation {
// // // // //     int kf_idx;
// // // // //     Eigen::Vector2d pixel;
// // // // //     double depth_measured;
// // // // //     double depth_projected;     // ✅ 新增：几何投影深度
// // // // //     Eigen::Vector3d P_cam;      // 这个现在用 depth_projected 计算
// // // // //     double weight;
// // // // //     double confidence;
// // // // //     double pose_change;         // ✅ 新增：这个帧的pose变化量
// // // // // };

// // // // // // 3D point
// // // // // struct Point3D {
// // // // //     int id;
// // // // //     Eigen::Vector3d position_original;     // Original position (world coordinates under bad pose)
// // // // //     Eigen::Vector3d position_transformed;  // Position after per-frame transform
// // // // //     std::vector<double> position_optimized;  // Ceres optimized position [x, y, z]
// // // // //     std::vector<VoxbloxObservation> observations;
// // // // //     bool is_anchor;             // ✅ 新增：是否是锚点
    
// // // // //     Point3D() : position_optimized(3, 0.0) {}
    
// // // // //     void SetOptimizedPosition(const Eigen::Vector3d& pos) {
// // // // //         position_optimized[0] = pos[0];
// // // // //         position_optimized[1] = pos[1];
// // // // //         position_optimized[2] = pos[2];
// // // // //     }
    
// // // // //     Eigen::Vector3d GetOptimizedPosition() const {
// // // // //         return Eigen::Vector3d(position_optimized[0], position_optimized[1], position_optimized[2]);
// // // // //     }
// // // // // };

// // // // // // Reprojection error cost function
// // // // // class ReprojectionCost {
// // // // // public:
// // // // //     ReprojectionCost(const Eigen::Vector2d& observation, double weight = 1.0)
// // // // //         : observed_pixel_(observation), weight_(sqrt(weight)) {}
    
// // // // //     template <typename T>
// // // // //     bool operator()(const T* const camera_pose,
// // // // //                     const T* const point_3d,
// // // // //                     T* residuals) const {
// // // // //         // Extract camera pose (T_cw)
// // // // //         Eigen::Matrix<T, 3, 1> t_cw(camera_pose[0], camera_pose[1], camera_pose[2]);
// // // // //         Eigen::Quaternion<T> q_cw(camera_pose[6], camera_pose[3], camera_pose[4], camera_pose[5]);
        
// // // // //         // 3D point (world coordinate system)
// // // // //         Eigen::Matrix<T, 3, 1> P_w(point_3d[0], point_3d[1], point_3d[2]);
        
// // // // //         // Transform to camera coordinate system
// // // // //         Eigen::Matrix<T, 3, 1> P_c = q_cw * P_w + t_cw;
        
// // // // //         // Check if point is in front of camera
// // // // //         if (P_c[2] <= T(0.01)) {
// // // // //             residuals[0] = T(100.0) * T(weight_);
// // // // //             residuals[1] = T(100.0) * T(weight_);
// // // // //             return true;
// // // // //         }
        
// // // // //         // Project to pixel plane
// // // // //         T u = T(FX) * (P_c[0] / P_c[2]) + T(CX);
// // // // //         T v = T(FY) * (P_c[1] / P_c[2]) + T(CY);
        
// // // // //         // Calculate weighted residuals
// // // // //         residuals[0] = T(weight_) * (u - T(observed_pixel_[0]));
// // // // //         residuals[1] = T(weight_) * (v - T(observed_pixel_[1]));
        
// // // // //         return true;
// // // // //     }
    
// // // // //     static ceres::CostFunction* Create(const Eigen::Vector2d& observation, double weight = 1.0) {
// // // // //         return new ceres::AutoDiffCostFunction<ReprojectionCost, 2, 7, 3>(
// // // // //             new ReprojectionCost(observation, weight));
// // // // //     }
    
// // // // // private:
// // // // //     Eigen::Vector2d observed_pixel_;
// // // // //     double weight_;
// // // // // };

// // // // // // Depth consistency cost function
// // // // // class DepthConsistencyCost {
// // // // // public:
// // // // //     DepthConsistencyCost(const Eigen::Vector3d& P_cam_target, double depth_weight = 10.0)
// // // // //         : P_cam_target_(P_cam_target), depth_weight_(depth_weight) {}
    
// // // // //     template <typename T>
// // // // //     bool operator()(const T* const camera_pose,
// // // // //                     const T* const point_3d,
// // // // //                     T* residuals) const {
// // // // //         // Extract camera pose (T_cw)
// // // // //         Eigen::Matrix<T, 3, 1> t_cw(camera_pose[0], camera_pose[1], camera_pose[2]);
// // // // //         Eigen::Quaternion<T> q_cw(camera_pose[6], camera_pose[3], camera_pose[4], camera_pose[5]);
        
// // // // //         // 3D point (world coordinate system)
// // // // //         Eigen::Matrix<T, 3, 1> P_w(point_3d[0], point_3d[1], point_3d[2]);
        
// // // // //         // Transform to camera coordinate system
// // // // //         Eigen::Matrix<T, 3, 1> P_c = q_cw * P_w + t_cw;
        
// // // // //         // Only constrain depth (z direction), x and y are constrained by reprojection
// // // // //         residuals[0] = T(depth_weight_) * (P_c[2] - T(P_cam_target_[2]));
        
// // // // //         return true;
// // // // //     }
    
// // // // //     static ceres::CostFunction* Create(const Eigen::Vector3d& P_cam_target, double depth_weight = 10.0) {
// // // // //         return new ceres::AutoDiffCostFunction<DepthConsistencyCost, 1, 7, 3>(
// // // // //             new DepthConsistencyCost(P_cam_target, depth_weight));
// // // // //     }
    
// // // // // private:
// // // // //     Eigen::Vector3d P_cam_target_;
// // // // //     double depth_weight_;
// // // // // };

// // // // // // Initial position regularizer (prevent points from drifting too far)
// // // // // class InitialPositionRegularizer {
// // // // // public:
// // // // //     InitialPositionRegularizer(const Eigen::Vector3d& initial_pos, double weight = 0.1)
// // // // //         : initial_position_(initial_pos), weight_(weight) {}
    
// // // // //     template <typename T>
// // // // //     bool operator()(const T* const point_3d, T* residuals) const {
// // // // //         residuals[0] = T(weight_) * (point_3d[0] - T(initial_position_[0]));
// // // // //         residuals[1] = T(weight_) * (point_3d[1] - T(initial_position_[1]));
// // // // //         residuals[2] = T(weight_) * (point_3d[2] - T(initial_position_[2]));
// // // // //         return true;
// // // // //     }
    
// // // // //     static ceres::CostFunction* Create(const Eigen::Vector3d& initial_pos, double weight = 0.1) {
// // // // //         return new ceres::AutoDiffCostFunction<InitialPositionRegularizer, 3, 3>(
// // // // //             new InitialPositionRegularizer(initial_pos, weight));
// // // // //     }
    
// // // // // private:
// // // // //     Eigen::Vector3d initial_position_;
// // // // //     double weight_;
// // // // // };

// // // // // // Mesh transformer and optimizer class
// // // // // class MeshTransformerOptimizer {
// // // // // private:
// // // // //     std::map<int, CameraPose> poses_before_;  // Poses before loop closure
// // // // //     std::map<int, CameraPose> poses_after_;   // Poses after loop closure
// // // // //     std::map<int, std::shared_ptr<Point3D>> points_;
// // // // //     std::unique_ptr<ceres::Problem> problem_;
    
// // // // // public:
// // // // //     MeshTransformerOptimizer() : problem_(std::make_unique<ceres::Problem>()) {}
    
// // // // //     // Load pose file
// // // // //     bool LoadPoses(const std::string& pose_file, std::map<int, CameraPose>& poses) {
// // // // //         std::ifstream file(pose_file);
// // // // //         if (!file.is_open()) {
// // // // //             std::cerr << "Cannot open pose file: " << pose_file << std::endl;
// // // // //             return false;
// // // // //         }
        
// // // // //         std::string line;
// // // // //         int frame_id = 0;
        
// // // // //         while (std::getline(file, line)) {
// // // // //             if (line.empty() || line[0] == '#') continue;
            
// // // // //             std::istringstream iss(line);
// // // // //             double timestamp, tx, ty, tz, qx, qy, qz, qw;
            
// // // // //             if (iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
// // // // //                 CameraPose pose;
// // // // //                 pose.frame_id = frame_id;
// // // // //                 pose.timestamp = timestamp;
// // // // //                 pose.SetFromTUM(tx, ty, tz, qx, qy, qz, qw);
// // // // //                 poses[frame_id] = pose;
// // // // //                 frame_id++;
// // // // //             }
// // // // //         }
        
// // // // //         std::cout << "Loaded " << poses.size() << " poses" << std::endl;
// // // // //         return true;
// // // // //     }
    
// // // // //     // Load Voxblox correspondence data
// // // // //     // bool LoadVoxbloxData(const std::string& data_file) {
// // // // //     //     std::ifstream file(data_file);
// // // // //     //     if (!file.is_open()) {
// // // // //     //         std::cerr << "Cannot open data file: " << data_file << std::endl;
// // // // //     //         return false;
// // // // //     //     }
        
// // // // //     //     std::string line;
// // // // //     //     points_.clear();
        
// // // // //     //     while (std::getline(file, line)) {
// // // // //     //         if (line.empty() || line[0] == '#') continue;
            
// // // // //     //         // Check if this is a point definition line
// // // // //     //         if (line[0] != ' ' && line[0] != '\t') {
// // // // //     //             std::istringstream iss(line);
// // // // //     //             int point_id, num_obs;
// // // // //     //             double x, y, z;
                
// // // // //     //             if (iss >> point_id >> x >> y >> z >> num_obs) {
// // // // //     //                 auto point = std::make_shared<Point3D>();
// // // // //     //                 point->id = point_id;
// // // // //     //                 point->position_original = Eigen::Vector3d(x, y, z);
                    
// // // // //     //                 // Read observation data
// // // // //     //                 for (int i = 0; i < num_obs; ++i) {
// // // // //     //                     if (std::getline(file, line)) {
// // // // //     //                         // Remove leading spaces
// // // // //     //                         size_t first = line.find_first_not_of(" \t");
// // // // //     //                         if (first != std::string::npos) {
// // // // //     //                             line = line.substr(first);
// // // // //     //                         }
                            
// // // // //     //                         std::istringstream obs_iss(line);
// // // // //     //                         VoxbloxObservation obs;
// // // // //     //                         double depth_proj;
                            
// // // // //     //                         if (obs_iss >> obs.kf_idx >> obs.pixel[0] >> obs.pixel[1] 
// // // // //     //                             >> obs.depth_measured >> depth_proj >> obs.weight >> obs.confidence) {
                                
// // // // //     //                             // Calculate 3D position in camera coordinate system
// // // // //     //                             double z = obs.depth_measured;
// // // // //     //                             double x = (obs.pixel[0] - CX) * z / FX;
// // // // //     //                             double y = (obs.pixel[1] - CY) * z / FY;
// // // // //     //                             obs.P_cam = Eigen::Vector3d(x, y, z);
                                
// // // // //     //                             point->observations.push_back(obs);
// // // // //     //                         }
// // // // //     //                     }
// // // // //     //                 }
                    
// // // // //     //                 if (!point->observations.empty()) {
// // // // //     //                     points_[point_id] = point;
// // // // //     //                 }
// // // // //     //             }
// // // // //     //         }
// // // // //     //     }
        
// // // // //     //     std::cout << "Loaded " << points_.size() << " 3D points" << std::endl;
// // // // //     //     return true;
// // // // //     // }
// // // // //     // ============ 修改后的 LoadVoxbloxData ============
// // // // //     bool LoadVoxbloxData(const std::string& data_file) {
// // // // //         std::ifstream file(data_file);
// // // // //         if (!file.is_open()) {
// // // // //             std::cerr << "Cannot open data file: " << data_file << std::endl;
// // // // //             return false;
// // // // //         }
        
// // // // //         std::string line;
// // // // //         points_.clear();
        
// // // // //         int anchor_count = 0;
// // // // //         int changed_count = 0;
        
// // // // //         while (std::getline(file, line)) {
// // // // //             if (line.empty() || line[0] == '#') continue;
            
// // // // //             if (line[0] != ' ' && line[0] != '\t') {
// // // // //                 std::istringstream iss(line);
// // // // //                 int point_id, num_obs, is_anchor_int;
// // // // //                 double x, y, z;
                
// // // // //                 // ✅ 新格式：point_id x y z num_obs is_anchor
// // // // //                 if (iss >> point_id >> x >> y >> z >> num_obs >> is_anchor_int) {
// // // // //                     auto point = std::make_shared<Point3D>();
// // // // //                     point->id = point_id;
// // // // //                     point->position_original = Eigen::Vector3d(x, y, z);
// // // // //                     point->is_anchor = (is_anchor_int == 1);  // ✅ 读取锚点标记
                    
// // // // //                     if (point->is_anchor) anchor_count++;
// // // // //                     else changed_count++;
                    
// // // // //                     for (int i = 0; i < num_obs; ++i) {
// // // // //                         if (std::getline(file, line)) {
// // // // //                             size_t first = line.find_first_not_of(" \t");
// // // // //                             if (first != std::string::npos) {
// // // // //                                 line = line.substr(first);
// // // // //                             }
                            
// // // // //                             std::istringstream obs_iss(line);
// // // // //                             VoxbloxObservation obs;
                            
// // // // //                             // ✅ 新格式：kf_idx u v depth_measured depth_projected weight confidence pose_change
// // // // //                             if (obs_iss >> obs.kf_idx >> obs.pixel[0] >> obs.pixel[1] 
// // // // //                                 >> obs.depth_measured >> obs.depth_projected 
// // // // //                                 >> obs.weight >> obs.confidence >> obs.pose_change) {
                                
// // // // //                                 // ✅✅✅ 关键修复：用 depth_projected 计算 P_cam ✅✅✅
// // // // //                                 // double z_proj = obs.depth_projected;  // 用几何投影深度！
// // // // //                                 double z_meas = obs.depth_measured;  // ← 用真实测量深度！
// // // // //                                 double x_cam = (obs.pixel[0] - CX) * z_meas / FX;
// // // // //                                 double y_cam = (obs.pixel[1] - CY) * z_meas / FY;
// // // // //                                 obs.P_cam = Eigen::Vector3d(x_cam, y_cam, z_meas);
                                
// // // // //                                 point->observations.push_back(obs);
// // // // //                             }
// // // // //                         }
// // // // //                     }
                    
// // // // //                     if (!point->observations.empty()) {
// // // // //                         points_[point_id] = point;
// // // // //                     }
// // // // //                 }
// // // // //             }
// // // // //         }
        
// // // // //         std::cout << "Loaded " << points_.size() << " 3D points" << std::endl;
// // // // //         std::cout << "  锚点(稳定区域): " << anchor_count << std::endl;
// // // // //         std::cout << "  变形点(变化区域): " << changed_count << std::endl;
        
// // // // //         return true;
// // // // //     }

// // // // //     // Step 1: Perform per-frame transform (provide good initial values)
// // // // //     // void PerformPerFrameTransform() {
// // // // //     //     std::cout << "\nStep 1: Performing per-frame transform (providing optimization initial values)..." << std::endl;
        
// // // // //     //     std::vector<double> movements;
        
// // // // //     //     for (auto& point_pair : points_) {
// // // // //     //         auto& point = point_pair.second;
            
// // // // //     //         // Use the first observation's KF for transformation
// // // // //     //         for (const auto& obs : point->observations) {
// // // // //     //             int kf_id = obs.kf_idx;
                
// // // // //     //             if (poses_before_.find(kf_id) != poses_before_.end() && 
// // // // //     //                 poses_after_.find(kf_id) != poses_after_.end()) {
                    
// // // // //     //                 // Use precise depth to calculate camera coordinates
// // // // //     //                 Eigen::Vector3d P_cam = obs.P_cam;
                    
// // // // //     //                 // Use new camera pose to calculate world coordinates
// // // // //     //                 Eigen::Matrix4d T_wc_after = poses_after_[kf_id].T_wc;
// // // // //     //                 Eigen::Vector4d P_cam_homo(P_cam[0], P_cam[1], P_cam[2], 1.0);
// // // // //     //                 Eigen::Vector4d P_world_new = T_wc_after * P_cam_homo;
                    
// // // // //     //                 point->position_transformed = P_world_new.head<3>();
                    
// // // // //     //                 // Set as initial value for optimization
// // // // //     //                 point->SetOptimizedPosition(point->position_transformed);
                    
// // // // //     //                 double movement = (point->position_transformed - point->position_original).norm();
// // // // //     //                 movements.push_back(movement);
                    
// // // // //     //                 break;  // Use the first valid observation
// // // // //     //             }
// // // // //     //         }
// // // // //     //     }
        
// // // // //     //     if (!movements.empty()) {
// // // // //     //         double avg_movement = std::accumulate(movements.begin(), movements.end(), 0.0) / movements.size();
// // // // //     //         std::cout << "  Per-frame transform completed, average movement: " << avg_movement << " m" << std::endl;
// // // // //     //     }
// // // // //     // }
    

// // // // //     // ============ 修改后的 PerformPerFrameTransform ============
// // // // //     void PerformPerFrameTransform() {
// // // // //         std::cout << "\nStep 1: Performing per-frame transform..." << std::endl;
        
// // // // //         std::vector<double> movements_changed;
// // // // //         std::vector<double> movements_anchor;
        
// // // // //         for (auto& point_pair : points_) {
// // // // //             auto& point = point_pair.second;
            
// // // // //             // ✅ 锚点：保持原位置不变
// // // // //             if (point->is_anchor) {
// // // // //                 point->position_transformed = point->position_original;
// // // // //                 point->SetOptimizedPosition(point->position_original);
// // // // //                 movements_anchor.push_back(0.0);
// // // // //                 continue;
// // // // //             }
            
// // // // //             // ✅ 变形点：使用加权平均的多帧transform
// // // // //             std::vector<Eigen::Vector3d> transformed_positions;
// // // // //             std::vector<double> weights;
            
// // // // //             for (const auto& obs : point->observations) {
// // // // //                 int kf_id = obs.kf_idx;
                
// // // // //                 if (poses_before_.find(kf_id) == poses_before_.end() || 
// // // // //                     poses_after_.find(kf_id) == poses_after_.end()) continue;
                
// // // // //                 // ✅ 正确做法：用原始顶点位置，不是深度图重建的位置
// // // // //                 Eigen::Vector4d P_w_old;
// // // // //                 P_w_old << point->position_original, 1.0;
                
// // // // //                 // 用旧pose投影到相机坐标系
// // // // //                 Eigen::Vector4d P_cam_old = poses_before_[kf_id].T_cw * P_w_old;
                
// // // // //                 // 用新pose转回世界坐标系  
// // // // //                 Eigen::Vector4d P_w_new = poses_after_[kf_id].T_wc * P_cam_old;
                
// // // // //                 // ✅ 根据pose变化量给权重：变化越大的帧越可信
// // // // //                 double pose_weight = 1.0 + obs.pose_change * 50.0;  // pose变化1cm → 权重1.5
// // // // //                 double total_weight = obs.weight * obs.confidence * pose_weight;
                
// // // // //                 transformed_positions.push_back(P_w_new.head<3>());
// // // // //                 weights.push_back(total_weight);
// // // // //             }
            
// // // // //             // ✅ 加权平均所有观测的结果
// // // // //             if (!transformed_positions.empty()) {
// // // // //                 double total_weight = std::accumulate(weights.begin(), weights.end(), 0.0);
// // // // //                 Eigen::Vector3d avg_pos = Eigen::Vector3d::Zero();
                
// // // // //                 for (size_t i = 0; i < transformed_positions.size(); ++i) {
// // // // //                     avg_pos += weights[i] * transformed_positions[i];
// // // // //                 }
// // // // //                 avg_pos /= total_weight;
                
// // // // //                 point->position_transformed = avg_pos;
// // // // //                 point->SetOptimizedPosition(avg_pos);
                
// // // // //                 double movement = (avg_pos - point->position_original).norm();
// // // // //                 movements_changed.push_back(movement);
// // // // //             } else {
// // // // //                 // 没有有效观测，保持原位置
// // // // //                 point->position_transformed = point->position_original;
// // // // //                 point->SetOptimizedPosition(point->position_original);
// // // // //             }
// // // // //         }
        
// // // // //         // 统计信息
// // // // //         if (!movements_changed.empty()) {
// // // // //             double avg = std::accumulate(movements_changed.begin(), movements_changed.end(), 0.0) 
// // // // //                         / movements_changed.size();
// // // // //             double max_m = *std::max_element(movements_changed.begin(), movements_changed.end());
// // // // //             std::cout << "  变形点移动量: 平均=" << avg*1000 << "mm, 最大=" << max_m*1000 << "mm" << std::endl;
// // // // //         }
// // // // //         std::cout << "  锚点数量: " << movements_anchor.size() << " (保持不动)" << std::endl;
// // // // //     }

// // // // //     // Step 2: Setup Ceres optimization problem
// // // // //     void SetupOptimization() {
// // // // //         std::cout << "\nStep 2: Setting up Ceres optimization problem..." << std::endl;
        
// // // // //         // Add camera pose parameters (fixed)
// // // // //         for (auto& pose_pair : poses_after_) {
// // // // //             auto& pose = pose_pair.second;
// // // // //             problem_->AddParameterBlock(pose.pose_data.data(), 7);
// // // // //             problem_->SetParameterBlockConstant(pose.pose_data.data());
// // // // //         }
        
// // // // //         int reproj_constraints = 0;
// // // // //         int depth_constraints = 0;
// // // // //         int regularizer_constraints = 0;
// // // // //         int anchor_constraints = 0;  // ✅ 新增

// // // // //         // Add constraints for each 3D point
// // // // //         for (auto& point_pair : points_) {
// // // // //             auto& point = point_pair.second;
            
// // // // //             // Add 3D point parameter block
// // // // //             problem_->AddParameterBlock(point->position_optimized.data(), 3);
            
// // // // //             // 1. Add soft regularization constraint (prevent points from drifting too far)
// // // // //             // ceres::CostFunction* regularizer_cost = 
// // // // //             //     InitialPositionRegularizer::Create(point->position_transformed, 0.1);  // Small weight
// // // // //             // problem_->AddResidualBlock(regularizer_cost, nullptr, point->position_optimized.data());
// // // // //             // regularizer_constraints++;
// // // // //             // ============ 修改后 ============
// // // // //             // 1. 正则化约束 - 锚点用强约束，变形点用弱约束
// // // // //             double regularizer_weight;
// // // // //             if (point->is_anchor) {
// // // // //                 regularizer_weight = 100.0;  // ✅ 锚点：强约束，几乎不允许移动
// // // // //             } else {
// // // // //                 regularizer_weight = 1;    // 变形点：弱约束，允许移动
// // // // //             }

// // // // //             ceres::CostFunction* regularizer_cost = 
// // // // //                 InitialPositionRegularizer::Create(point->position_transformed, regularizer_weight);
// // // // //             problem_->AddResidualBlock(regularizer_cost, nullptr, point->position_optimized.data());

// // // // //             if (point->is_anchor) {
// // // // //                 anchor_constraints++;  // 需要在函数开头声明这个变量
// // // // //             }

// // // // //             // 2. Add constraints for each observation
// // // // //             for (const auto& obs : point->observations) {
// // // // //                 if (poses_after_.find(obs.kf_idx) == poses_after_.end()) continue;
                
// // // // //                 // 2.1 Reprojection error constraint
// // // // //                 double reproj_weight = std::max(0.5, obs.weight);  // Use Voxblox weight
// // // // //                 ceres::CostFunction* reproj_cost = 
// // // // //                     ReprojectionCost::Create(obs.pixel, reproj_weight);
// // // // //                 ceres::LossFunction* reproj_loss = new ceres::HuberLoss(2.0);  // Robust loss function
                
// // // // //                 problem_->AddResidualBlock(
// // // // //                     reproj_cost,
// // // // //                     reproj_loss,
// // // // //                     poses_after_[obs.kf_idx].pose_data.data(),
// // // // //                     point->position_optimized.data()
// // // // //                 );
// // // // //                 reproj_constraints++;
                
// // // // //                 // 2.2 Depth consistency constraint (strong constraint)
// // // // //                 // double depth_weight = 20.0;  // Depth is very accurate, large weight
// // // // //                 // ceres::CostFunction* depth_cost = 
// // // // //                 //     DepthConsistencyCost::Create(obs.P_cam, depth_weight);

// // // // //                 // ============ 修改后 ============
// // // // //                 // 2.2 Depth consistency constraint
// // // // //                 // ✅ 锚点用更强的深度约束，变形点用正常约束
// // // // //                 double depth_weight;
// // // // //                 if (point->is_anchor) {
// // // // //                     depth_weight = 50.0;  // 锚点：非常强的深度约束
// // // // //                 } else {
// // // // //                     // 变形点：根据pose变化量调整权重
// // // // //                     // pose变化越大，越不应该依赖旧深度图
// // // // //                     depth_weight = 10.0;
// // // // //                 }

// // // // //                 ceres::CostFunction* depth_cost = 
// // // // //                     DepthConsistencyCost::Create(obs.P_cam, depth_weight);
                
// // // // //                 problem_->AddResidualBlock(
// // // // //                     depth_cost,
// // // // //                     nullptr,  // No robust loss for depth, it's a hard constraint
// // // // //                     poses_after_[obs.kf_idx].pose_data.data(),
// // // // //                     point->position_optimized.data()
// // // // //                 );
// // // // //                 depth_constraints++;
// // // // //             }
// // // // //         }
        
// // // // //         std::cout << "  Added " << points_.size() << " 3D point parameters" << std::endl;
// // // // //         std::cout << "  Reprojection constraints: " << reproj_constraints << std::endl;
// // // // //         std::cout << "  Depth constraints: " << depth_constraints << std::endl;
// // // // //         std::cout << "  Regularizer constraints: " << regularizer_constraints << std::endl;
// // // // //         // 在函数末尾添加输出
// // // // //         std::cout << "  锚点强约束: " << anchor_constraints << std::endl;
// // // // //     }
    
// // // // //     // Step 3: Execute optimization
// // // // //     bool Optimize() {
// // // // //         std::cout << "\nStep 3: Executing Ceres optimization (fine-tuning)..." << std::endl;
        
// // // // //         ceres::Solver::Options options;
// // // // //         options.linear_solver_type = ceres::SPARSE_SCHUR;
// // // // //         options.minimizer_progress_to_stdout = true;  // Show optimization progress
// // // // //         options.max_num_iterations = 200;  // Don't need many iterations with good initial values
// // // // //         options.num_threads = 8;
// // // // //         options.function_tolerance = 1e-6;
        
// // // // //         ceres::Solver::Summary summary;
// // // // //         ceres::Solve(options, problem_.get(), &summary);
        
// // // // //         std::cout << "\n  Optimization completed: " << summary.BriefReport() << std::endl;
// // // // //         std::cout << "  Initial cost: " << summary.initial_cost << std::endl;
// // // // //         std::cout << "  Final cost: " << summary.final_cost << std::endl;
// // // // //         std::cout << "  Number of iterations: " << summary.iterations.size() << std::endl;
// // // // //         std::cout << "  Cost reduction: " << (summary.initial_cost - summary.final_cost) 
// // // // //                   << " (" << (100.0 * (summary.initial_cost - summary.final_cost) / summary.initial_cost) 
// // // // //                   << "%)" << std::endl;
        
// // // // //         return summary.IsSolutionUsable();
// // // // //     }
    
// // // // //     // Analyze optimization results
// // // // //     void AnalyzeResults() {
// // // // //         std::cout << "\n=== Optimization Results Analysis ===" << std::endl;
        
// // // // //         std::vector<double> transform_to_optimize;  // Movement from per-frame to optimized
// // // // //         std::vector<double> original_to_optimize;   // Total movement from original to optimized
// // // // //         std::vector<double> reproj_errors;
// // // // //         std::vector<double> depth_errors;
        
// // // // //         for (const auto& point_pair : points_) {
// // // // //             const auto& point = point_pair.second;
// // // // //             Eigen::Vector3d pos_opt = point->GetOptimizedPosition();
            
// // // // //             // Calculate movements
// // // // //             double move1 = (pos_opt - point->position_transformed).norm();
// // // // //             double move2 = (pos_opt - point->position_original).norm();
// // // // //             transform_to_optimize.push_back(move1);
// // // // //             original_to_optimize.push_back(move2);
            
// // // // //             // Calculate errors
// // // // //             for (const auto& obs : point->observations) {
// // // // //                 if (poses_after_.find(obs.kf_idx) == poses_after_.end()) continue;
                
// // // // //                 // Calculate optimized reprojection and depth
// // // // //                 Eigen::Matrix4d T_cw = poses_after_[obs.kf_idx].T_cw;
// // // // //                 Eigen::Vector4d P_w(pos_opt[0], pos_opt[1], pos_opt[2], 1.0);
// // // // //                 Eigen::Vector4d P_c = T_cw * P_w;
                
// // // // //                 // Reprojection error
// // // // //                 double u = FX * (P_c[0] / P_c[2]) + CX;
// // // // //                 double v = FY * (P_c[1] / P_c[2]) + CY;
// // // // //                 double reproj_err = sqrt(pow(u - obs.pixel[0], 2) + pow(v - obs.pixel[1], 2));
// // // // //                 reproj_errors.push_back(reproj_err);
                
// // // // //                 // Depth error
// // // // //                 double depth_err = std::abs(P_c[2] - obs.depth_measured);
// // // // //                 depth_errors.push_back(depth_err);
// // // // //             }
// // // // //         }
        
// // // // //         // Statistics
// // // // //         if (!transform_to_optimize.empty()) {
// // // // //             double avg_microadjust = std::accumulate(transform_to_optimize.begin(), 
// // // // //                                                     transform_to_optimize.end(), 0.0) / transform_to_optimize.size();
// // // // //             double max_microadjust = *std::max_element(transform_to_optimize.begin(), transform_to_optimize.end());
            
// // // // //             std::cout << "Optimization fine-tuning (relative to per-frame transform):" << std::endl;
// // // // //             std::cout << "  Average: " << avg_microadjust * 1000 << " mm" << std::endl;
// // // // //             std::cout << "  Maximum: " << max_microadjust * 1000 << " mm" << std::endl;
// // // // //         }
        
// // // // //         if (!reproj_errors.empty()) {
// // // // //             double avg_reproj = std::accumulate(reproj_errors.begin(), reproj_errors.end(), 0.0) / reproj_errors.size();
// // // // //             std::cout << "\nReprojection error:" << std::endl;
// // // // //             std::cout << "  Average: " << avg_reproj << " pixels" << std::endl;
// // // // //         }
        
// // // // //         if (!depth_errors.empty()) {
// // // // //             double avg_depth = std::accumulate(depth_errors.begin(), depth_errors.end(), 0.0) / depth_errors.size();
// // // // //             std::cout << "\nDepth error:" << std::endl;
// // // // //             std::cout << "  Average: " << avg_depth * 1000 << " mm" << std::endl;
// // // // //         }
// // // // //     }
    
// // // // //     // Save results
// // // // //     void SaveResults(const std::string& output_dir) {
// // // // //         // Save optimized points
// // // // //         std::string output_file = output_dir + "/optimized_points_final.txt";
// // // // //         std::ofstream file(output_file);
// // // // //         if (!file.is_open()) {
// // // // //             std::cerr << "Cannot create output file: " << output_file << std::endl;
// // // // //             return;
// // // // //         }
        
// // // // //         file << std::fixed << std::setprecision(6);
// // // // //         file << "# Final optimized 3D points (per-frame transform + Ceres refinement)\n";
// // // // //         file << "# Format: point_id x_original y_original z_original x_optimized y_optimized z_optimized movement_mm\n";
        
// // // // //         for (const auto& point_pair : points_) {
// // // // //             const auto& point = point_pair.second;
// // // // //             Eigen::Vector3d pos_opt = point->GetOptimizedPosition();
// // // // //             double movement = (pos_opt - point->position_original).norm();
            
// // // // //             file << point->id << " "
// // // // //                  << point->position_original[0] << " " 
// // // // //                  << point->position_original[1] << " " 
// // // // //                  << point->position_original[2] << " "
// // // // //                  << pos_opt[0] << " " 
// // // // //                  << pos_opt[1] << " " 
// // // // //                  << pos_opt[2] << " "
// // // // //                  << movement * 1000 << std::endl;
// // // // //         }
        
// // // // //         file.close();
// // // // //         std::cout << "\nSaved final optimization results to: " << output_file << std::endl;
// // // // //     }
    
// // // // //     // Main run function
// // // // //     bool Run(const std::string& poses_before_file,
// // // // //              const std::string& poses_after_file,
// // // // //              const std::string& voxblox_data_file,
// // // // //              const std::string& output_dir) {
        
// // // // //         // 1. Load data
// // // // //         std::cout << "=== Loading Data ===" << std::endl;
// // // // //         if (!LoadPoses(poses_before_file, poses_before_)) return false;
// // // // //         if (!LoadPoses(poses_after_file, poses_after_)) return false;
// // // // //         if (!LoadVoxbloxData(voxblox_data_file)) return false;
        
// // // // //         // 2. Per-frame transform (provide good initial values)
// // // // //         PerformPerFrameTransform();
        
// // // // //         // 3. Setup optimization problem
// // // // //         SetupOptimization();
        
// // // // //         // 4. Execute optimization
// // // // //         if (!Optimize()) {
// // // // //             std::cerr << "Optimization failed!" << std::endl;
// // // // //             return false;
// // // // //         }
        
// // // // //         // 5. Analyze results
// // // // //         AnalyzeResults();
        
// // // // //         // 6. Save results
// // // // //         SaveResults(output_dir);
        
// // // // //         return true;
// // // // //     }
// // // // // };

// // // // // int main() {
// // // // //     // ==========================================
// // // // //     // 1. Set file paths (根据 Python 脚本路径修改)
// // // // //     // ==========================================
    
// // // // //     // 工作根目录 (包含 loop 文件夹和 Output 文件夹的上级目录)
// // // // //     std::string root_dir = "/Datasets/Work_Data/ACL_OneLoop20251220_214841/";
    
// // // // //     // 当前 Loop 的具体文件夹
// // // // //     std::string loop_dir = root_dir + "loop_1_1670449626.805310/";

// // // // //     // 轨迹文件 (注意 Python 中分别在 pre/ 和 post/ 子文件夹下)
// // // // //     std::string poses_before = loop_dir + "pre/standard_trajectory_no_loop.txt";
// // // // //     std::string poses_after  = loop_dir + "post/standard_trajectory_with_loop.txt";

// // // // //     // 输入数据 (这是刚才 Python RayCaster 生成的文件)
// // // // //     std::string voxblox_data = root_dir + "Output/optimization_data.txt";

// // // // //     // 输出目录 (建议放在 Output 下面的子文件夹，方便管理)
// // // // //     std::string output_dir = root_dir + "Output/";
    
// // // // //     // ==========================================
    
// // // // //     // Create output directory
// // // // //     std::string mkdir_cmd = "mkdir -p " + output_dir;
// // // // //     system(mkdir_cmd.c_str());
    
// // // // //     // Debug print to check paths
// // // // //     std::cout << "--- Configuration ---" << std::endl;
// // // // //     std::cout << "Poses Before: " << poses_before << std::endl;
// // // // //     std::cout << "Poses After:  " << poses_after << std::endl;
// // // // //     std::cout << "Input Data:   " << voxblox_data << std::endl;
// // // // //     std::cout << "Output Dir:   " << output_dir << std::endl;
// // // // //     std::cout << "---------------------" << std::endl;

// // // // //     // Create optimizer and run
// // // // //     MeshTransformerOptimizer optimizer;
    
// // // // //     if (optimizer.Run(poses_before, poses_after, voxblox_data, output_dir)) {
// // // // //         std::cout << "\n===== Complete! =====" << std::endl;
// // // // //         std::cout << "1. Per-frame transform provided good initial values" << std::endl;
// // // // //         std::cout << "2. Ceres optimization performed fine-tuning" << std::endl;
// // // // //         std::cout << "3. Final results saved at: " << output_dir << "/optimized_points_final.txt" << std::endl;
// // // // //     } else {
// // // // //         std::cerr << "Processing failed!" << std::endl;
// // // // //         return -1;
// // // // //     }
    
// // // // //     return 0;
// // // // // }




// // // // #include <iostream>
// // // // #include <fstream>
// // // // #include <sstream>
// // // // #include <string>
// // // // #include <vector>
// // // // #include <map>
// // // // #include <iomanip>
// // // // #include <algorithm>
// // // // #include <numeric>
// // // // #include <cmath>

// // // // #include <Eigen/Dense>
// // // // #include <Eigen/Geometry>

// // // // // Camera intrinsics
// // // // const double FX = 377.535257164;
// // // // const double FY = 377.209841379;
// // // // const double CX = 328.193371286;
// // // // const double CY = 240.426878936;

// // // // // ============================================================
// // // // // 数据结构
// // // // // ============================================================
// // // // struct CameraPose {
// // // //     int frame_id;
// // // //     double timestamp;
// // // //     Eigen::Matrix4d T_wc;  // Camera to World
// // // //     Eigen::Matrix4d T_cw;  // World to Camera
    
// // // //     void SetFromTUM(double tx, double ty, double tz, 
// // // //                     double qx, double qy, double qz, double qw) {
// // // //         Eigen::Vector3d t_wc(tx, ty, tz);
// // // //         Eigen::Quaterniond q_wc(qw, qx, qy, qz);
// // // //         q_wc.normalize();
        
// // // //         T_wc = Eigen::Matrix4d::Identity();
// // // //         T_wc.block<3, 3>(0, 0) = q_wc.toRotationMatrix();
// // // //         T_wc.block<3, 1>(0, 3) = t_wc;
// // // //         T_cw = T_wc.inverse();
// // // //     }
// // // // };

// // // // struct Observation {
// // // //     int kf_idx;
// // // //     Eigen::Vector2d pixel;
// // // //     double depth_measured;
// // // //     double depth_projected;
// // // //     double weight;
// // // //     double confidence;
// // // //     double pose_change;
// // // // };

// // // // struct Point3D {
// // // //     int id;
// // // //     Eigen::Vector3d position_original;
// // // //     Eigen::Vector3d position_target;  // 三角化得到的目标位置
// // // //     std::vector<Observation> observations;
// // // //     bool is_anchor;
// // // // };

// // // // // ============================================================
// // // // // Lindström 线性三角化器
// // // // // ============================================================
// // // // class LinearTriangulator {
// // // // private:
// // // //     std::map<int, CameraPose> poses_before_;
// // // //     std::map<int, CameraPose> poses_after_;
// // // //     std::map<int, Point3D> points_;
    
// // // // public:
// // // //     // 加载 pose
// // // //     bool LoadPoses(const std::string& pose_file, std::map<int, CameraPose>& poses) {
// // // //         std::ifstream file(pose_file);
// // // //         if (!file.is_open()) {
// // // //             std::cerr << "Cannot open: " << pose_file << std::endl;
// // // //             return false;
// // // //         }
        
// // // //         std::string line;
// // // //         int frame_id = 0;
        
// // // //         while (std::getline(file, line)) {
// // // //             if (line.empty() || line[0] == '#') continue;
            
// // // //             std::istringstream iss(line);
// // // //             double timestamp, tx, ty, tz, qx, qy, qz, qw;
            
// // // //             if (iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
// // // //                 CameraPose pose;
// // // //                 pose.frame_id = frame_id;
// // // //                 pose.timestamp = timestamp;
// // // //                 pose.SetFromTUM(tx, ty, tz, qx, qy, qz, qw);
// // // //                 poses[frame_id] = pose;
// // // //                 frame_id++;
// // // //             }
// // // //         }
        
// // // //         std::cout << "Loaded " << poses.size() << " poses" << std::endl;
// // // //         return true;
// // // //     }
    
// // // //     // 加载点数据
// // // //     bool LoadPoints(const std::string& data_file) {
// // // //         std::ifstream file(data_file);
// // // //         if (!file.is_open()) {
// // // //             std::cerr << "Cannot open: " << data_file << std::endl;
// // // //             return false;
// // // //         }
        
// // // //         std::string line;
// // // //         points_.clear();
        
// // // //         int anchor_count = 0, moving_count = 0;
        
// // // //         while (std::getline(file, line)) {
// // // //             if (line.empty() || line[0] == '#') continue;
            
// // // //             if (line[0] != ' ' && line[0] != '\t') {
// // // //                 std::istringstream iss(line);
// // // //                 int point_id, num_obs, is_anchor_int;
// // // //                 double x, y, z;
                
// // // //                 if (iss >> point_id >> x >> y >> z >> num_obs >> is_anchor_int) {
// // // //                     Point3D point;
// // // //                     point.id = point_id;
// // // //                     point.position_original = Eigen::Vector3d(x, y, z);
// // // //                     point.is_anchor = (is_anchor_int == 1);
                    
// // // //                     if (point.is_anchor) anchor_count++;
// // // //                     else moving_count++;
                    
// // // //                     for (int i = 0; i < num_obs; ++i) {
// // // //                         if (std::getline(file, line)) {
// // // //                             size_t first = line.find_first_not_of(" \t");
// // // //                             if (first != std::string::npos) line = line.substr(first);
                            
// // // //                             std::istringstream obs_iss(line);
// // // //                             Observation obs;
// // // //                             double distance_3d;
                            
// // // //                             if (obs_iss >> obs.kf_idx >> obs.pixel[0] >> obs.pixel[1] 
// // // //                                 >> obs.depth_measured >> obs.depth_projected 
// // // //                                 >> obs.weight >> obs.confidence >> obs.pose_change >> distance_3d) {
// // // //                                 point.observations.push_back(obs);
// // // //                             }
// // // //                         }
// // // //                     }
                    
// // // //                     if (!point.observations.empty()) {
// // // //                         points_[point_id] = point;
// // // //                     }
// // // //                 }
// // // //             }
// // // //         }
        
// // // //         std::cout << "Loaded " << points_.size() << " points" << std::endl;
// // // //         std::cout << "  Anchors: " << anchor_count << std::endl;
// // // //         std::cout << "  Moving: " << moving_count << std::endl;
        
// // // //         return true;
// // // //     }
    
// // // //     // ============================================================
// // // //     // ★★★ 核心：Lindström 线性三角化 ★★★
// // // //     // ============================================================
    
// // // //     // 方法1：简单加权平均（推荐先试这个）
// // // //     Eigen::Vector3d TriangulateWeightedAverage(const Point3D& point) {
// // // //         std::vector<Eigen::Vector3d> positions;
// // // //         std::vector<double> weights;
        
// // // //         for (const auto& obs : point.observations) {
// // // //             auto it = poses_after_.find(obs.kf_idx);
// // // //             if (it == poses_after_.end()) continue;
            
// // // //             // ★ 用新 pose + 测量深度 → 反投影到世界坐标
// // // //             double z = obs.depth_measured;
// // // //             double x = (obs.pixel[0] - CX) * z / FX;
// // // //             double y = (obs.pixel[1] - CY) * z / FY;
// // // //             Eigen::Vector3d P_cam(x, y, z);
            
// // // //             const Eigen::Matrix4d& T_wc = it->second.T_wc;
// // // //             Eigen::Vector3d P_world = T_wc.block<3,3>(0,0) * P_cam + T_wc.block<3,1>(0,3);
            
// // // //             // 权重：综合考虑观测质量和 pose 变化
// // // //             double w = obs.weight * obs.confidence;
// // // //             // pose 变化大的帧，三角化结果更"需要"移动，给更高权重
// // // //             w *= (1.0 + obs.pose_change * 10.0);
            
// // // //             positions.push_back(P_world);
// // // //             weights.push_back(w);
// // // //         }
        
// // // //         if (positions.empty()) return point.position_original;
        
// // // //         // 加权平均
// // // //         double total_weight = std::accumulate(weights.begin(), weights.end(), 0.0);
// // // //         Eigen::Vector3d result = Eigen::Vector3d::Zero();
// // // //         for (size_t i = 0; i < positions.size(); ++i) {
// // // //             result += weights[i] * positions[i];
// // // //         }
// // // //         return result / total_weight;
// // // //     }
    
// // // //     // 方法2：Lindström 最小二乘射线求交（更鲁棒）
// // // //     // 原理：每条射线 r_i(t) = C_i + t * d_i
// // // //     // 求最小化 sum_i |P - r_i(t_i)|^2 的点 P
// // // //     Eigen::Vector3d TriangulateLindstrom(const Point3D& point) {
// // // //         // 构建线性系统 A * P = b
// // // //         Eigen::Matrix3d ATA = Eigen::Matrix3d::Zero();
// // // //         Eigen::Vector3d ATb = Eigen::Vector3d::Zero();
// // // //         double total_weight = 0;
        
// // // //         for (const auto& obs : point.observations) {
// // // //             auto it = poses_after_.find(obs.kf_idx);
// // // //             if (it == poses_after_.end()) continue;
            
// // // //             const Eigen::Matrix4d& T_wc = it->second.T_wc;
            
// // // //             // 相机中心（世界坐标系）
// // // //             Eigen::Vector3d C = T_wc.block<3,1>(0,3);
            
// // // //             // 射线方向（世界坐标系）
// // // //             double z = 1.0;  // 归一化
// // // //             double x = (obs.pixel[0] - CX) / FX;
// // // //             double y = (obs.pixel[1] - CY) / FY;
// // // //             Eigen::Vector3d d_cam(x, y, z);
// // // //             d_cam.normalize();
// // // //             Eigen::Vector3d d = T_wc.block<3,3>(0,0) * d_cam;
// // // //             d.normalize();
            
// // // //             // 权重
// // // //             double w = obs.weight * obs.confidence * (1.0 + obs.pose_change * 10.0);
            
// // // //             // 构建 (I - d*d^T) 矩阵
// // // //             // 这是将点投影到射线的垂直平面上的投影矩阵
// // // //             Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
// // // //             Eigen::Matrix3d ddT = d * d.transpose();
// // // //             Eigen::Matrix3d A_i = I - ddT;
            
// // // //             ATA += w * A_i;
// // // //             ATb += w * A_i * C;
// // // //             total_weight += w;
// // // //         }
        
// // // //         if (total_weight < 1e-10) return point.position_original;
        
// // // //         // 加入深度约束作为软约束
// // // //         // 用深度信息来约束沿射线方向的位置
// // // //         for (const auto& obs : point.observations) {
// // // //             auto it = poses_after_.find(obs.kf_idx);
// // // //             if (it == poses_after_.end()) continue;
            
// // // //             const Eigen::Matrix4d& T_wc = it->second.T_wc;
            
// // // //             // 用深度直接反投影的点
// // // //             double z = obs.depth_measured;
// // // //             double x = (obs.pixel[0] - CX) * z / FX;
// // // //             double y = (obs.pixel[1] - CY) * z / FY;
// // // //             Eigen::Vector3d P_cam(x, y, z);
// // // //             Eigen::Vector3d P_depth = T_wc.block<3,3>(0,0) * P_cam + T_wc.block<3,1>(0,3);
            
// // // //             double depth_weight = obs.weight * obs.confidence * 5.0;  // 深度约束权重
            
// // // //             ATA += depth_weight * Eigen::Matrix3d::Identity();
// // // //             ATb += depth_weight * P_depth;
// // // //         }
        
// // // //         // 求解
// // // //         Eigen::Vector3d P = ATA.ldlt().solve(ATb);
        
// // // //         return P;
// // // //     }
    
// // // //     // 方法3：加权中值（对离群点更鲁棒）
// // // //     Eigen::Vector3d TriangulateWeightedMedian(const Point3D& point) {
// // // //         std::vector<Eigen::Vector3d> positions;
// // // //         std::vector<double> weights;
        
// // // //         for (const auto& obs : point.observations) {
// // // //             auto it = poses_after_.find(obs.kf_idx);
// // // //             if (it == poses_after_.end()) continue;
            
// // // //             double z = obs.depth_measured;
// // // //             double x = (obs.pixel[0] - CX) * z / FX;
// // // //             double y = (obs.pixel[1] - CY) * z / FY;
// // // //             Eigen::Vector3d P_cam(x, y, z);
            
// // // //             const Eigen::Matrix4d& T_wc = it->second.T_wc;
// // // //             Eigen::Vector3d P_world = T_wc.block<3,3>(0,0) * P_cam + T_wc.block<3,1>(0,3);
            
// // // //             double w = obs.weight * obs.confidence * (1.0 + obs.pose_change * 10.0);
            
// // // //             positions.push_back(P_world);
// // // //             weights.push_back(w);
// // // //         }
        
// // // //         if (positions.empty()) return point.position_original;
        
// // // //         // 分别对 x, y, z 求加权中值
// // // //         Eigen::Vector3d result;
// // // //         for (int dim = 0; dim < 3; ++dim) {
// // // //             std::vector<std::pair<double, double>> val_weight;
// // // //             for (size_t i = 0; i < positions.size(); ++i) {
// // // //                 val_weight.emplace_back(positions[i][dim], weights[i]);
// // // //             }
// // // //             std::sort(val_weight.begin(), val_weight.end());
            
// // // //             double total_w = 0;
// // // //             for (const auto& vw : val_weight) total_w += vw.second;
            
// // // //             double cumsum = 0;
// // // //             for (const auto& vw : val_weight) {
// // // //                 cumsum += vw.second;
// // // //                 if (cumsum >= total_w / 2.0) {
// // // //                     result[dim] = vw.first;
// // // //                     break;
// // // //                 }
// // // //             }
// // // //         }
        
// // // //         return result;
// // // //     }
    
// // // //     // ============================================================
// // // //     // 主处理函数
// // // //     // ============================================================
// // // //     void Process(int method = 1) {
// // // //         std::cout << "\n=== Linear Triangulation ===" << std::endl;
// // // //         std::cout << "Method: ";
// // // //         switch (method) {
// // // //             case 1: std::cout << "Weighted Average" << std::endl; break;
// // // //             case 2: std::cout << "Lindström Least Squares" << std::endl; break;
// // // //             case 3: std::cout << "Weighted Median" << std::endl; break;
// // // //         }
        
// // // //         std::vector<double> anchor_movements;
// // // //         std::vector<double> moving_movements;
        
// // // //         for (auto& kv : points_) {
// // // //             Point3D& point = kv.second;
            
// // // //             if (point.is_anchor) {
// // // //                 // ★ 锚点：保持原位置
// // // //                 point.position_target = point.position_original;
// // // //                 anchor_movements.push_back(0.0);
// // // //             } else {
// // // //                 // ★ 移动点：三角化
// // // //                 switch (method) {
// // // //                     case 1: 
// // // //                         point.position_target = TriangulateWeightedAverage(point);
// // // //                         break;
// // // //                     case 2: 
// // // //                         point.position_target = TriangulateLindstrom(point);
// // // //                         break;
// // // //                     case 3: 
// // // //                         point.position_target = TriangulateWeightedMedian(point);
// // // //                         break;
// // // //                 }
                
// // // //                 double movement = (point.position_target - point.position_original).norm();
// // // //                 moving_movements.push_back(movement);
// // // //             }
// // // //         }
        
// // // //         // 统计
// // // //         std::cout << "\nResults:" << std::endl;
// // // //         std::cout << "  Anchors: " << anchor_movements.size() << " (kept at original)" << std::endl;
        
// // // //         if (!moving_movements.empty()) {
// // // //             double avg = std::accumulate(moving_movements.begin(), moving_movements.end(), 0.0) 
// // // //                         / moving_movements.size();
// // // //             std::sort(moving_movements.begin(), moving_movements.end());
// // // //             double median = moving_movements[moving_movements.size() / 2];
// // // //             double max_m = moving_movements.back();
            
// // // //             std::cout << "  Moving points: " << moving_movements.size() << std::endl;
// // // //             std::cout << "    Average movement: " << avg * 1000 << " mm" << std::endl;
// // // //             std::cout << "    Median movement:  " << median * 1000 << " mm" << std::endl;
// // // //             std::cout << "    Max movement:     " << max_m * 1000 << " mm" << std::endl;
// // // //         }
// // // //     }
    
// // // //     // 计算重投影误差和深度误差（验证）
// // // //     void Evaluate() {
// // // //         std::cout << "\n=== Evaluation ===" << std::endl;
        
// // // //         std::vector<double> reproj_errors;
// // // //         std::vector<double> depth_errors;
        
// // // //         for (const auto& kv : points_) {
// // // //             const Point3D& point = kv.second;
            
// // // //             for (const auto& obs : point.observations) {
// // // //                 auto it = poses_after_.find(obs.kf_idx);
// // // //                 if (it == poses_after_.end()) continue;
                
// // // //                 const Eigen::Matrix4d& T_cw = it->second.T_cw;
                
// // // //                 // 投影目标位置到相机
// // // //                 Eigen::Vector4d P_w;
// // // //                 P_w << point.position_target, 1.0;
// // // //                 Eigen::Vector4d P_c = T_cw * P_w;
                
// // // //                 if (P_c[2] <= 0.01) continue;
                
// // // //                 double u = FX * (P_c[0] / P_c[2]) + CX;
// // // //                 double v = FY * (P_c[1] / P_c[2]) + CY;
                
// // // //                 double reproj_err = std::sqrt(std::pow(u - obs.pixel[0], 2) + 
// // // //                                               std::pow(v - obs.pixel[1], 2));
// // // //                 reproj_errors.push_back(reproj_err);
                
// // // //                 double depth_err = std::abs(P_c[2] - obs.depth_measured);
// // // //                 depth_errors.push_back(depth_err);
// // // //             }
// // // //         }
        
// // // //         if (!reproj_errors.empty()) {
// // // //             double avg_reproj = std::accumulate(reproj_errors.begin(), reproj_errors.end(), 0.0) 
// // // //                                / reproj_errors.size();
// // // //             std::sort(reproj_errors.begin(), reproj_errors.end());
// // // //             double median_reproj = reproj_errors[reproj_errors.size() / 2];
            
// // // //             std::cout << "Reprojection error:" << std::endl;
// // // //             std::cout << "  Average: " << avg_reproj << " pixels" << std::endl;
// // // //             std::cout << "  Median:  " << median_reproj << " pixels" << std::endl;
// // // //         }
        
// // // //         if (!depth_errors.empty()) {
// // // //             double avg_depth = std::accumulate(depth_errors.begin(), depth_errors.end(), 0.0) 
// // // //                               / depth_errors.size();
// // // //             std::sort(depth_errors.begin(), depth_errors.end());
// // // //             double median_depth = depth_errors[depth_errors.size() / 2];
            
// // // //             std::cout << "Depth error:" << std::endl;
// // // //             std::cout << "  Average: " << avg_depth * 1000 << " mm" << std::endl;
// // // //             std::cout << "  Median:  " << median_depth * 1000 << " mm" << std::endl;
// // // //         }
// // // //     }
    
// // // //     // 保存结果
// // // //     void SaveResults(const std::string& output_file) {
// // // //         std::ofstream file(output_file);
// // // //         if (!file.is_open()) {
// // // //             std::cerr << "Cannot create: " << output_file << std::endl;
// // // //             return;
// // // //         }
        
// // // //         file << std::fixed << std::setprecision(6);
// // // //         file << "# Linear Triangulation Results\n";
// // // //         file << "# point_id x_orig y_orig z_orig x_target y_target z_target movement_mm is_anchor\n";
        
// // // //         for (const auto& kv : points_) {
// // // //             const Point3D& point = kv.second;
// // // //             double movement = (point.position_target - point.position_original).norm();
            
// // // //             file << point.id << " "
// // // //                  << point.position_original[0] << " " 
// // // //                  << point.position_original[1] << " " 
// // // //                  << point.position_original[2] << " "
// // // //                  << point.position_target[0] << " " 
// // // //                  << point.position_target[1] << " " 
// // // //                  << point.position_target[2] << " "
// // // //                  << movement * 1000 << " "
// // // //                  << (point.is_anchor ? 1 : 0) << std::endl;
// // // //         }
        
// // // //         file.close();
// // // //         std::cout << "\nSaved to: " << output_file << std::endl;
// // // //     }
    
// // // //     // 主运行函数
// // // //     bool Run(const std::string& poses_before_file,
// // // //              const std::string& poses_after_file,
// // // //              const std::string& data_file,
// // // //              const std::string& output_file,
// // // //              int method = 2) {  // 默认用 Lindström 方法
        
// // // //         std::cout << "=== Linear Triangulator ===" << std::endl;
        
// // // //         if (!LoadPoses(poses_before_file, poses_before_)) return false;
// // // //         if (!LoadPoses(poses_after_file, poses_after_)) return false;
// // // //         if (!LoadPoints(data_file)) return false;
        
// // // //         Process(method);
// // // //         Evaluate();
// // // //         SaveResults(output_file);
        
// // // //         return true;
// // // //     }
// // // // };

// // // // // ============================================================
// // // // // Main
// // // // // ============================================================
// // // // int main(int argc, char** argv) {
// // // //     std::string root_dir = "/Datasets/Work_Data/ACL_OneLoop20251220_214841/";
// // // //     std::string loop_dir = root_dir + "loop_1_1670449626.805310/";
    
// // // //     std::string poses_before = loop_dir + "pre/standard_trajectory_no_loop.txt";
// // // //     std::string poses_after  = loop_dir + "post/standard_trajectory_with_loop.txt";
// // // //     std::string input_data   = root_dir + "Output/optimization_data.txt";
// // // //     std::string output_file  = root_dir + "Output/optimized_points_final.txt";
    
// // // //     // 选择方法: 1=加权平均, 2=Lindström最小二乘, 3=加权中值
// // // //     int method = 2;
// // // //     if (argc > 1) method = std::atoi(argv[1]);
    
// // // //     LinearTriangulator triangulator;
    
// // // //     if (triangulator.Run(poses_before, poses_after, input_data, output_file, method)) {
// // // //         std::cout << "\n===== Complete! =====" << std::endl;
// // // //     } else {
// // // //         std::cerr << "Failed!" << std::endl;
// // // //         return -1;
// // // //     }
    
// // // //     return 0;
// // // // }



// // // // #include <iostream>
// // // // #include <fstream>
// // // // #include <sstream>
// // // // #include <string>
// // // // #include <vector>
// // // // #include <map>
// // // // #include <set>
// // // // #include <iomanip>
// // // // #include <algorithm>
// // // // #include <numeric>
// // // // #include <cmath>

// // // // #include <Eigen/Dense>
// // // // #include <Eigen/Geometry>
// // // // #include <Eigen/Sparse>

// // // // // Camera intrinsics
// // // // const double FX = 377.535257164;
// // // // const double FY = 377.209841379;
// // // // const double CX = 328.193371286;
// // // // const double CY = 240.426878936;

// // // // // ============================================================
// // // // // 数据结构
// // // // // ============================================================
// // // // struct CameraPose {
// // // //     int frame_id;
// // // //     Eigen::Matrix4d T_wc;
// // // //     Eigen::Matrix4d T_cw;
    
// // // //     void SetFromTUM(double tx, double ty, double tz, 
// // // //                     double qx, double qy, double qz, double qw) {
// // // //         Eigen::Quaterniond q_wc(qw, qx, qy, qz);
// // // //         q_wc.normalize();
// // // //         T_wc = Eigen::Matrix4d::Identity();
// // // //         T_wc.block<3,3>(0,0) = q_wc.toRotationMatrix();
// // // //         T_wc.block<3,1>(0,3) = Eigen::Vector3d(tx, ty, tz);
// // // //         T_cw = T_wc.inverse();
// // // //     }
// // // // };

// // // // struct Observation {
// // // //     int kf_idx;
// // // //     Eigen::Vector2d pixel;
// // // //     double depth_measured;
// // // //     double depth_projected;
// // // //     double weight;
// // // //     double confidence;
// // // //     double pose_change;
// // // // };

// // // // struct Point3D {
// // // //     int id;
// // // //     int mesh_vertex_idx;  // 对应的 mesh 顶点索引
// // // //     Eigen::Vector3d position_original;
// // // //     Eigen::Vector3d position_triangulated;  // 直接三角化结果
// // // //     Eigen::Vector3d position_smoothed;      // 平滑后结果
// // // //     std::vector<Observation> observations;
// // // //     bool is_anchor;
// // // //     std::vector<int> neighbors;  // 邻居点的 id
// // // // };

// // // // // ============================================================
// // // // // 改进版三角化器
// // // // // ============================================================
// // // // class ImprovedTriangulator {
// // // // private:
// // // //     std::map<int, CameraPose> poses_before_;
// // // //     std::map<int, CameraPose> poses_after_;
// // // //     std::map<int, Point3D> points_;
    
// // // //     // Mesh 连接信息
// // // //     std::vector<Eigen::Vector3d> mesh_vertices_;
// // // //     std::vector<Eigen::Vector3i> mesh_faces_;
// // // //     std::map<int, std::set<int>> vertex_neighbors_;  // mesh 顶点的邻居
    
// // // // public:
// // // //     bool LoadPoses(const std::string& pose_file, std::map<int, CameraPose>& poses) {
// // // //         std::ifstream file(pose_file);
// // // //         if (!file.is_open()) return false;
        
// // // //         std::string line;
// // // //         int frame_id = 0;
// // // //         while (std::getline(file, line)) {
// // // //             if (line.empty() || line[0] == '#') continue;
// // // //             std::istringstream iss(line);
// // // //             double ts, tx, ty, tz, qx, qy, qz, qw;
// // // //             if (iss >> ts >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
// // // //                 CameraPose pose;
// // // //                 pose.frame_id = frame_id;
// // // //                 pose.SetFromTUM(tx, ty, tz, qx, qy, qz, qw);
// // // //                 poses[frame_id] = pose;
// // // //                 frame_id++;
// // // //             }
// // // //         }
// // // //         std::cout << "Loaded " << poses.size() << " poses" << std::endl;
// // // //         return true;
// // // //     }
    
// // // //     bool LoadPoints(const std::string& data_file) {
// // // //         std::ifstream file(data_file);
// // // //         if (!file.is_open()) return false;
        
// // // //         std::string line;
// // // //         int anchor_count = 0, moving_count = 0;
        
// // // //         while (std::getline(file, line)) {
// // // //             if (line.empty() || line[0] == '#') continue;
            
// // // //             if (line[0] != ' ' && line[0] != '\t') {
// // // //                 std::istringstream iss(line);
// // // //                 int point_id, num_obs, is_anchor_int;
// // // //                 double x, y, z;
                
// // // //                 if (iss >> point_id >> x >> y >> z >> num_obs >> is_anchor_int) {
// // // //                     Point3D point;
// // // //                     point.id = point_id;
// // // //                     point.mesh_vertex_idx = point_id;  // 假设 point_id == mesh vertex index
// // // //                     point.position_original = Eigen::Vector3d(x, y, z);
// // // //                     point.is_anchor = (is_anchor_int == 1);
                    
// // // //                     if (point.is_anchor) anchor_count++;
// // // //                     else moving_count++;
                    
// // // //                     for (int i = 0; i < num_obs; ++i) {
// // // //                         if (std::getline(file, line)) {
// // // //                             size_t first = line.find_first_not_of(" \t");
// // // //                             if (first != std::string::npos) line = line.substr(first);
                            
// // // //                             std::istringstream obs_iss(line);
// // // //                             Observation obs;
// // // //                             double distance_3d;
                            
// // // //                             if (obs_iss >> obs.kf_idx >> obs.pixel[0] >> obs.pixel[1] 
// // // //                                 >> obs.depth_measured >> obs.depth_projected 
// // // //                                 >> obs.weight >> obs.confidence >> obs.pose_change >> distance_3d) {
// // // //                                 point.observations.push_back(obs);
// // // //                             }
// // // //                         }
// // // //                     }
                    
// // // //                     if (!point.observations.empty()) {
// // // //                         points_[point_id] = point;
// // // //                     }
// // // //                 }
// // // //             }
// // // //         }
        
// // // //         std::cout << "Loaded " << points_.size() << " points (Anchors: " 
// // // //                   << anchor_count << ", Moving: " << moving_count << ")" << std::endl;
// // // //         return true;
// // // //     }
    
// // // //     // ============================================================
// // // //     // ★★★ 改进1：自适应权重三角化 ★★★
// // // //     // ============================================================
// // // //     Eigen::Vector3d TriangulateAdaptive(const Point3D& point) {
// // // //         std::vector<Eigen::Vector3d> positions;
// // // //         std::vector<double> weights;
        
// // // //         // 收集所有观测的三角化结果
// // // //         for (const auto& obs : point.observations) {
// // // //             auto it = poses_after_.find(obs.kf_idx);
// // // //             if (it == poses_after_.end()) continue;
            
// // // //             double z = obs.depth_measured;
// // // //             double x = (obs.pixel[0] - CX) * z / FX;
// // // //             double y = (obs.pixel[1] - CY) * z / FY;
// // // //             Eigen::Vector3d P_cam(x, y, z);
            
// // // //             const Eigen::Matrix4d& T_wc = it->second.T_wc;
// // // //             Eigen::Vector3d P_world = T_wc.block<3,3>(0,0) * P_cam + T_wc.block<3,1>(0,3);
            
// // // //             // ★ 改进：自适应权重
// // // //             // 深度越近，深度图越准
// // // //             double depth_reliability = 1.0 / (1.0 + obs.depth_measured / 5.0);
            
// // // //             // 观测质量
// // // //             double quality = obs.weight * obs.confidence;
            
// // // //             // ★ pose 变化大的帧对移动点更重要（知道应该移动到哪里）
// // // //             double pose_factor = 1.0 + std::min(obs.pose_change * 20.0, 5.0);
            
// // // //             double w = quality * depth_reliability * pose_factor;
            
// // // //             positions.push_back(P_world);
// // // //             weights.push_back(w);
// // // //         }
        
// // // //         if (positions.empty()) return point.position_original;
        
// // // //         // ★ 改进：去除离群点后加权平均
// // // //         if (positions.size() >= 3) {
// // // //             // 先计算一次简单平均
// // // //             Eigen::Vector3d mean = Eigen::Vector3d::Zero();
// // // //             double total_w = 0;
// // // //             for (size_t i = 0; i < positions.size(); ++i) {
// // // //                 mean += weights[i] * positions[i];
// // // //                 total_w += weights[i];
// // // //             }
// // // //             mean /= total_w;
            
// // // //             // 计算到平均值的距离，去除离群点
// // // //             std::vector<double> distances;
// // // //             for (const auto& p : positions) {
// // // //                 distances.push_back((p - mean).norm());
// // // //             }
            
// // // //             // 计算中位数距离
// // // //             std::vector<double> sorted_dist = distances;
// // // //             std::sort(sorted_dist.begin(), sorted_dist.end());
// // // //             double median_dist = sorted_dist[sorted_dist.size() / 2];
// // // //             double threshold = std::max(0.05, median_dist * 2.0);  // 至少 5cm 或 2倍中位数
            
// // // //             // 去除离群点后重新计算
// // // //             Eigen::Vector3d result = Eigen::Vector3d::Zero();
// // // //             total_w = 0;
// // // //             for (size_t i = 0; i < positions.size(); ++i) {
// // // //                 if (distances[i] < threshold) {
// // // //                     result += weights[i] * positions[i];
// // // //                     total_w += weights[i];
// // // //                 }
// // // //             }
            
// // // //             if (total_w > 0) return result / total_w;
// // // //         }
        
// // // //         // 简单加权平均
// // // //         double total_w = std::accumulate(weights.begin(), weights.end(), 0.0);
// // // //         Eigen::Vector3d result = Eigen::Vector3d::Zero();
// // // //         for (size_t i = 0; i < positions.size(); ++i) {
// // // //             result += weights[i] * positions[i];
// // // //         }
// // // //         return result / total_w;
// // // //     }
    
// // // //     // ============================================================
// // // //     // ★★★ 改进2：建立空间邻域关系 ★★★
// // // //     // ============================================================
// // // //     void BuildSpatialNeighbors(double radius = 0.15) {
// // // //         std::cout << "\nBuilding spatial neighbors (radius=" << radius << "m)..." << std::endl;
        
// // // //         std::vector<int> point_ids;
// // // //         for (const auto& kv : points_) {
// // // //             point_ids.push_back(kv.first);
// // // //         }
        
// // // //         // 简单的 O(n^2) 邻域查找（可以用 KD-tree 加速）
// // // //         int total_neighbors = 0;
// // // //         for (size_t i = 0; i < point_ids.size(); ++i) {
// // // //             int id_i = point_ids[i];
// // // //             const Eigen::Vector3d& pos_i = points_[id_i].position_original;
            
// // // //             for (size_t j = i + 1; j < point_ids.size(); ++j) {
// // // //                 int id_j = point_ids[j];
// // // //                 const Eigen::Vector3d& pos_j = points_[id_j].position_original;
                
// // // //                 double dist = (pos_i - pos_j).norm();
// // // //                 if (dist < radius) {
// // // //                     points_[id_i].neighbors.push_back(id_j);
// // // //                     points_[id_j].neighbors.push_back(id_i);
// // // //                     total_neighbors += 2;
// // // //                 }
// // // //             }
// // // //         }
        
// // // //         std::cout << "  Average neighbors per point: " 
// // // //                   << (double)total_neighbors / point_ids.size() << std::endl;
// // // //     }
    
// // // //     // ============================================================
// // // //     // ★★★ 改进3：位移场平滑 ★★★
// // // //     // ============================================================
// // // //     void SmoothDisplacementField(int iterations = 3, double lambda = 0.3) {
// // // //         std::cout << "\nSmoothing displacement field (" << iterations << " iterations, lambda=" << lambda << ")..." << std::endl;
        
// // // //         // 初始化：smoothed = triangulated
// // // //         for (auto& kv : points_) {
// // // //             kv.second.position_smoothed = kv.second.position_triangulated;
// // // //         }
        
// // // //         for (int iter = 0; iter < iterations; ++iter) {
// // // //             std::map<int, Eigen::Vector3d> new_positions;
            
// // // //             for (auto& kv : points_) {
// // // //                 Point3D& point = kv.second;
                
// // // //                 // 锚点不平滑
// // // //                 if (point.is_anchor) {
// // // //                     new_positions[point.id] = point.position_smoothed;
// // // //                     continue;
// // // //                 }
                
// // // //                 // 计算邻居的平均位移
// // // //                 if (point.neighbors.empty()) {
// // // //                     new_positions[point.id] = point.position_smoothed;
// // // //                     continue;
// // // //                 }
                
// // // //                 // 当前位移
// // // //                 Eigen::Vector3d my_disp = point.position_smoothed - point.position_original;
                
// // // //                 // 邻居平均位移（只考虑移动点邻居）
// // // //                 Eigen::Vector3d neighbor_disp = Eigen::Vector3d::Zero();
// // // //                 int moving_neighbor_count = 0;
                
// // // //                 for (int neighbor_id : point.neighbors) {
// // // //                     auto it = points_.find(neighbor_id);
// // // //                     if (it == points_.end()) continue;
                    
// // // //                     const Point3D& neighbor = it->second;
// // // //                     if (neighbor.is_anchor) continue;  // 跳过锚点邻居
                    
// // // //                     Eigen::Vector3d n_disp = neighbor.position_smoothed - neighbor.position_original;
// // // //                     neighbor_disp += n_disp;
// // // //                     moving_neighbor_count++;
// // // //                 }
                
// // // //                 if (moving_neighbor_count > 0) {
// // // //                     neighbor_disp /= moving_neighbor_count;
                    
// // // //                     // 混合：自己的位移 + lambda * (邻居平均位移 - 自己的位移)
// // // //                     Eigen::Vector3d smoothed_disp = my_disp + lambda * (neighbor_disp - my_disp);
// // // //                     new_positions[point.id] = point.position_original + smoothed_disp;
// // // //                 } else {
// // // //                     new_positions[point.id] = point.position_smoothed;
// // // //                 }
// // // //             }
            
// // // //             // 更新
// // // //             for (auto& kv : points_) {
// // // //                 kv.second.position_smoothed = new_positions[kv.second.id];
// // // //             }
// // // //         }
// // // //     }
    
// // // //     // ============================================================
// // // //     // ★★★ 改进4：边界一致性检查 ★★★
// // // //     // ============================================================
// // // //     void EnforceBoundaryConsistency() {
// // // //         std::cout << "\nEnforcing boundary consistency..." << std::endl;
        
// // // //         int adjusted = 0;
        
// // // //         for (auto& kv : points_) {
// // // //             Point3D& point = kv.second;
// // // //             if (point.is_anchor) continue;
            
// // // //             // 检查是否在锚点/移动点边界
// // // //             int anchor_neighbors = 0;
// // // //             int moving_neighbors = 0;
// // // //             Eigen::Vector3d anchor_avg_pos = Eigen::Vector3d::Zero();
            
// // // //             for (int neighbor_id : point.neighbors) {
// // // //                 auto it = points_.find(neighbor_id);
// // // //                 if (it == points_.end()) continue;
                
// // // //                 if (it->second.is_anchor) {
// // // //                     anchor_neighbors++;
// // // //                     anchor_avg_pos += it->second.position_original;
// // // //                 } else {
// // // //                     moving_neighbors++;
// // // //                 }
// // // //             }
            
// // // //             // 如果有锚点邻居，需要特殊处理
// // // //             if (anchor_neighbors > 0 && moving_neighbors > 0) {
// // // //                 // 边界点：位移应该渐变
// // // //                 double anchor_ratio = (double)anchor_neighbors / (anchor_neighbors + moving_neighbors);
                
// // // //                 // 当前位移
// // // //                 Eigen::Vector3d current_disp = point.position_smoothed - point.position_original;
                
// // // //                 // 根据锚点比例减小位移
// // // //                 Eigen::Vector3d adjusted_disp = current_disp * (1.0 - anchor_ratio * 0.5);
// // // //                 point.position_smoothed = point.position_original + adjusted_disp;
                
// // // //                 adjusted++;
// // // //             }
// // // //         }
        
// // // //         std::cout << "  Adjusted " << adjusted << " boundary points" << std::endl;
// // // //     }
    
// // // //     // ============================================================
// // // //     // 主处理
// // // //     // ============================================================
// // // //     void Process() {
// // // //         std::cout << "\n=== Step 1: Initial Triangulation ===" << std::endl;
        
// // // //         std::vector<double> movements;
        
// // // //         for (auto& kv : points_) {
// // // //             Point3D& point = kv.second;
            
// // // //             if (point.is_anchor) {
// // // //                 point.position_triangulated = point.position_original;
// // // //             } else {
// // // //                 point.position_triangulated = TriangulateAdaptive(point);
// // // //                 double m = (point.position_triangulated - point.position_original).norm();
// // // //                 movements.push_back(m);
// // // //             }
// // // //         }
        
// // // //         if (!movements.empty()) {
// // // //             double avg = std::accumulate(movements.begin(), movements.end(), 0.0) / movements.size();
// // // //             std::cout << "  Moving points: " << movements.size() << ", avg displacement: " << avg*1000 << " mm" << std::endl;
// // // //         }
        
// // // //         // 建立邻域
// // // //         std::cout << "\n=== Step 2: Build Spatial Neighbors ===" << std::endl;
// // // //         BuildSpatialNeighbors(0.15);  // 15cm 邻域
        
// // // //         // 平滑位移场
// // // //         std::cout << "\n=== Step 3: Smooth Displacement Field ===" << std::endl;
// // // //         SmoothDisplacementField(3, 0.25);
        
// // // //         // 边界一致性
// // // //         std::cout << "\n=== Step 4: Boundary Consistency ===" << std::endl;
// // // //         EnforceBoundaryConsistency();
// // // //     }
    
// // // //     // 评估
// // // //     void Evaluate() {
// // // //         std::cout << "\n=== Evaluation ===" << std::endl;
        
// // // //         std::vector<double> reproj_errors, depth_errors;
// // // //         std::vector<double> movements_tri, movements_smooth;
        
// // // //         for (const auto& kv : points_) {
// // // //             const Point3D& point = kv.second;
            
// // // //             if (!point.is_anchor) {
// // // //                 movements_tri.push_back((point.position_triangulated - point.position_original).norm());
// // // //                 movements_smooth.push_back((point.position_smoothed - point.position_original).norm());
// // // //             }
            
// // // //             for (const auto& obs : point.observations) {
// // // //                 auto it = poses_after_.find(obs.kf_idx);
// // // //                 if (it == poses_after_.end()) continue;
                
// // // //                 Eigen::Vector4d P_w;
// // // //                 P_w << point.position_smoothed, 1.0;
// // // //                 Eigen::Vector4d P_c = it->second.T_cw * P_w;
                
// // // //                 if (P_c[2] <= 0.01) continue;
                
// // // //                 double u = FX * (P_c[0] / P_c[2]) + CX;
// // // //                 double v = FY * (P_c[1] / P_c[2]) + CY;
                
// // // //                 reproj_errors.push_back(std::sqrt(std::pow(u - obs.pixel[0], 2) + std::pow(v - obs.pixel[1], 2)));
// // // //                 depth_errors.push_back(std::abs(P_c[2] - obs.depth_measured));
// // // //             }
// // // //         }
        
// // // //         std::cout << "Movement comparison:" << std::endl;
// // // //         if (!movements_tri.empty()) {
// // // //             std::sort(movements_tri.begin(), movements_tri.end());
// // // //             std::sort(movements_smooth.begin(), movements_smooth.end());
// // // //             std::cout << "  Triangulated: median=" << movements_tri[movements_tri.size()/2]*1000 << " mm" << std::endl;
// // // //             std::cout << "  Smoothed:     median=" << movements_smooth[movements_smooth.size()/2]*1000 << " mm" << std::endl;
// // // //         }
        
// // // //         if (!reproj_errors.empty()) {
// // // //             std::sort(reproj_errors.begin(), reproj_errors.end());
// // // //             std::cout << "Reprojection error: median=" << reproj_errors[reproj_errors.size()/2] << " px" << std::endl;
// // // //         }
        
// // // //         if (!depth_errors.empty()) {
// // // //             std::sort(depth_errors.begin(), depth_errors.end());
// // // //             std::cout << "Depth error: median=" << depth_errors[depth_errors.size()/2]*1000 << " mm" << std::endl;
// // // //         }
// // // //     }
    
// // // //     // 保存
// // // //     void SaveResults(const std::string& output_file, bool use_smoothed = true) {
// // // //         std::ofstream file(output_file);
// // // //         file << std::fixed << std::setprecision(6);
// // // //         file << "# Improved Triangulation Results\n";
// // // //         file << "# point_id x_orig y_orig z_orig x_target y_target z_target movement_mm is_anchor\n";
        
// // // //         for (const auto& kv : points_) {
// // // //             const Point3D& point = kv.second;
// // // //             const Eigen::Vector3d& target = use_smoothed ? point.position_smoothed : point.position_triangulated;
// // // //             double movement = (target - point.position_original).norm();
            
// // // //             file << point.id << " "
// // // //                  << point.position_original[0] << " " 
// // // //                  << point.position_original[1] << " " 
// // // //                  << point.position_original[2] << " "
// // // //                  << target[0] << " " 
// // // //                  << target[1] << " " 
// // // //                  << target[2] << " "
// // // //                  << movement * 1000 << " "
// // // //                  << (point.is_anchor ? 1 : 0) << "\n";
// // // //         }
        
// // // //         std::cout << "\nSaved to: " << output_file << std::endl;
// // // //     }
    
// // // //     bool Run(const std::string& poses_before_file,
// // // //              const std::string& poses_after_file,
// // // //              const std::string& data_file,
// // // //              const std::string& output_file) {
        
// // // //         std::cout << "=== Improved Triangulator ===" << std::endl;
        
// // // //         if (!LoadPoses(poses_before_file, poses_before_)) return false;
// // // //         if (!LoadPoses(poses_after_file, poses_after_)) return false;
// // // //         if (!LoadPoints(data_file)) return false;
        
// // // //         Process();
// // // //         Evaluate();
// // // //         SaveResults(output_file, true);  // true = use smoothed
        
// // // //         return true;
// // // //     }
// // // // };

// // // // int main() {
// // // //     std::string root_dir = "/Datasets/Work_Data/ACL_OneLoop20251220_214841/";
// // // //     std::string loop_dir = root_dir + "loop_1_1670449626.805310/";
    
// // // //     ImprovedTriangulator triangulator;
    
// // // //     triangulator.Run(
// // // //         loop_dir + "pre/standard_trajectory_no_loop.txt",
// // // //         loop_dir + "post/standard_trajectory_with_loop.txt",
// // // //         root_dir + "Output/optimization_data.txt",
// // // //         root_dir + "Output/optimized_points_final.txt"
// // // //     );
    
// // // //     std::cout << "\n===== Complete! =====" << std::endl;
// // // //     return 0;
// // // // }









// // // #include <iostream>
// // // #include <fstream>
// // // #include <sstream>
// // // #include <string>
// // // #include <vector>
// // // #include <map>
// // // #include <iomanip>
// // // #include <algorithm>
// // // #include <numeric>
// // // #include <cmath>
// // // #include <chrono>

// // // #include <Eigen/Dense>
// // // #include <Eigen/Geometry>

// // // using namespace std::chrono;

// // // const double FX = 377.535257164;
// // // const double FY = 377.209841379;
// // // const double CX = 328.193371286;
// // // const double CY = 240.426878936;

// // // // ============================================================
// // // // 数据结构
// // // // ============================================================
// // // struct CameraPose {
// // //     Eigen::Matrix4d T_wc, T_cw;
// // //     Eigen::Vector3d center;
    
// // //     void SetFromTUM(double tx, double ty, double tz, double qx, double qy, double qz, double qw) {
// // //         Eigen::Quaterniond q(qw, qx, qy, qz);
// // //         q.normalize();
// // //         T_wc = Eigen::Matrix4d::Identity();
// // //         T_wc.block<3,3>(0,0) = q.toRotationMatrix();
// // //         T_wc.block<3,1>(0,3) = Eigen::Vector3d(tx, ty, tz);
// // //         T_cw = T_wc.inverse();
// // //         center = T_wc.block<3,1>(0,3);
// // //     }
// // // };

// // // struct Observation {
// // //     int kf_idx;
// // //     Eigen::Vector2d pixel;
// // //     double depth_measured;
// // //     double depth_projected;
// // //     double weight;
// // //     double confidence;
// // //     double pose_change;
// // // };

// // // struct Point3D {
// // //     int id;
// // //     Eigen::Vector3d position_original;
// // //     Eigen::Vector3d position_target;
// // //     std::vector<Observation> observations;
// // //     bool is_anchor;
// // // };

// // // // ============================================================
// // // // Lindström 线性三角化器
// // // // ============================================================
// // // class LindstromTriangulator {
// // // private:
// // //     std::map<int, CameraPose> poses_;
// // //     std::map<int, Point3D> points_;
    
// // //     // 计时
// // //     std::map<std::string, double> timings_;
    
// // //     void tic(const std::string& name) {
// // //         timings_[name] = duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
// // //     }
    
// // //     void toc(const std::string& name) {
// // //         double start = timings_[name];
// // //         double end = duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
// // //         timings_[name] = (end - start) / 1000.0;  // ms
// // //     }
    
// // // public:
// // //     bool LoadPoses(const std::string& file) {
// // //         tic("1. Load Poses");
        
// // //         std::ifstream f(file);
// // //         if (!f) { std::cerr << "Cannot open: " << file << std::endl; return false; }
        
// // //         std::string line;
// // //         int id = 0;
// // //         while (std::getline(f, line)) {
// // //             if (line.empty() || line[0] == '#') continue;
// // //             std::istringstream iss(line);
// // //             double ts, tx, ty, tz, qx, qy, qz, qw;
// // //             if (iss >> ts >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
// // //                 CameraPose p;
// // //                 p.SetFromTUM(tx, ty, tz, qx, qy, qz, qw);
// // //                 poses_[id++] = p;
// // //             }
// // //         }
        
// // //         toc("1. Load Poses");
// // //         std::cout << "  Loaded " << poses_.size() << " poses" << std::endl;
// // //         return true;
// // //     }
    
// // //     bool LoadPoints(const std::string& file) {
// // //         tic("2. Load Points");
        
// // //         std::ifstream f(file);
// // //         if (!f) { std::cerr << "Cannot open: " << file << std::endl; return false; }
        
// // //         std::string line;
// // //         int anchors = 0, moving = 0;
        
// // //         while (std::getline(f, line)) {
// // //             if (line.empty() || line[0] == '#') continue;
            
// // //             if (line[0] != ' ' && line[0] != '\t') {
// // //                 std::istringstream iss(line);
// // //                 int id, num_obs, is_anchor;
// // //                 double x, y, z;
                
// // //                 if (iss >> id >> x >> y >> z >> num_obs >> is_anchor) {
// // //                     Point3D pt;
// // //                     pt.id = id;
// // //                     pt.position_original = Eigen::Vector3d(x, y, z);
// // //                     pt.is_anchor = (is_anchor == 1);
                    
// // //                     if (pt.is_anchor) anchors++; else moving++;
                    
// // //                     for (int i = 0; i < num_obs; ++i) {
// // //                         if (std::getline(f, line)) {
// // //                             size_t first = line.find_first_not_of(" \t");
// // //                             if (first != std::string::npos) line = line.substr(first);
                            
// // //                             std::istringstream obs_iss(line);
// // //                             Observation obs;
// // //                             double dist;
                            
// // //                             if (obs_iss >> obs.kf_idx >> obs.pixel[0] >> obs.pixel[1]
// // //                                 >> obs.depth_measured >> obs.depth_projected
// // //                                 >> obs.weight >> obs.confidence >> obs.pose_change >> dist) {
// // //                                 pt.observations.push_back(obs);
// // //                             }
// // //                         }
// // //                     }
                    
// // //                     if (!pt.observations.empty()) {
// // //                         points_[id] = pt;
// // //                     }
// // //                 }
// // //             }
// // //         }
        
// // //         toc("2. Load Points");
// // //         std::cout << "  Loaded " << points_.size() << " points" << std::endl;
// // //         std::cout << "  Anchors: " << anchors << ", Moving: " << moving << std::endl;
// // //         return true;
// // //     }
    
// // //     // ============================================================
// // //     // ★★★ Lindström 三角化 ★★★
// // //     // 
// // //     // 原理：每条射线 r_i(t) = C_i + t * d_i
// // //     // 找点 P 使得到所有射线的垂直距离之和最小
// // //     // 
// // //     // 但我们有深度图！所以结合两种方法：
// // //     // 1. 射线几何（给出方向约束）
// // //     // 2. 深度测量（给出距离约束）
// // //     // ============================================================
// // //     Eigen::Vector3d TriangulateLindstrom(const Point3D& point) {
// // //         // 收集所有观测的反投影点
// // //         std::vector<Eigen::Vector3d> points_world;
// // //         std::vector<double> weights;
        
// // //         for (const auto& obs : point.observations) {
// // //             auto it = poses_.find(obs.kf_idx);
// // //             if (it == poses_.end()) continue;
            
// // //             const CameraPose& pose = it->second;
            
// // //             // 用深度反投影到世界坐标
// // //             double z = obs.depth_measured;
// // //             double x = (obs.pixel[0] - CX) * z / FX;
// // //             double y = (obs.pixel[1] - CY) * z / FY;
// // //             Eigen::Vector3d P_cam(x, y, z);
            
// // //             Eigen::Vector3d P_world = pose.T_wc.block<3,3>(0,0) * P_cam + pose.center;
            
// // //             // 计算权重
// // //             double w = obs.weight * obs.confidence;
            
// // //             // 深度越近越可信
// // //             w *= 1.0 / (1.0 + z / 3.0);
            
// // //             // pose 变化大的帧更重要
// // //             w *= (1.0 + std::min(obs.pose_change * 15.0, 3.0));
            
// // //             points_world.push_back(P_world);
// // //             weights.push_back(w);
// // //         }
        
// // //         if (points_world.empty()) {
// // //             return point.position_original;
// // //         }
        
// // //         // ============================================================
// // //         // Lindström 加权最小二乘
// // //         // ============================================================
        
// // //         // 如果只有一个观测，直接返回
// // //         if (points_world.size() == 1) {
// // //             return points_world[0];
// // //         }
        
// // //         // 计算加权平均作为初始估计
// // //         Eigen::Vector3d mean = Eigen::Vector3d::Zero();
// // //         double total_w = 0;
// // //         for (size_t i = 0; i < points_world.size(); ++i) {
// // //             mean += weights[i] * points_world[i];
// // //             total_w += weights[i];
// // //         }
// // //         mean /= total_w;
        
// // //         // 离群点剔除（可选，提高鲁棒性）
// // //         if (points_world.size() >= 3) {
// // //             std::vector<double> distances;
// // //             for (const auto& p : points_world) {
// // //                 distances.push_back((p - mean).norm());
// // //             }
            
// // //             std::vector<double> sorted_d = distances;
// // //             std::sort(sorted_d.begin(), sorted_d.end());
// // //             double median_d = sorted_d[sorted_d.size() / 2];
// // //             double threshold = std::max(0.02, median_d * 2.5);  // 2cm 或 2.5倍中位数
            
// // //             // 重新计算去除离群点后的加权平均
// // //             Eigen::Vector3d filtered_mean = Eigen::Vector3d::Zero();
// // //             double filtered_w = 0;
// // //             int kept = 0;
            
// // //             for (size_t i = 0; i < points_world.size(); ++i) {
// // //                 if (distances[i] < threshold) {
// // //                     filtered_mean += weights[i] * points_world[i];
// // //                     filtered_w += weights[i];
// // //                     kept++;
// // //                 }
// // //             }
            
// // //             if (kept >= 1 && filtered_w > 0) {
// // //                 return filtered_mean / filtered_w;
// // //             }
// // //         }
        
// // //         return mean;
// // //     }
    
// // //     // ============================================================
// // //     // 主处理
// // //     // ============================================================
// // //     void Process() {
// // //         tic("3. Triangulation");
        
// // //         std::vector<double> movements;
// // //         int processed = 0;
        
// // //         for (auto& kv : points_) {
// // //             Point3D& pt = kv.second;
            
// // //             if (pt.is_anchor) {
// // //                 // 锚点保持原位置
// // //                 pt.position_target = pt.position_original;
// // //             } else {
// // //                 // 移动点三角化
// // //                 pt.position_target = TriangulateLindstrom(pt);
                
// // //                 double m = (pt.position_target - pt.position_original).norm();
// // //                 movements.push_back(m);
// // //                 processed++;
// // //             }
// // //         }
        
// // //         toc("3. Triangulation");
        
// // //         // 统计
// // //         std::cout << "\n  Processed " << processed << " moving points" << std::endl;
        
// // //         if (!movements.empty()) {
// // //             std::sort(movements.begin(), movements.end());
// // //             double avg = std::accumulate(movements.begin(), movements.end(), 0.0) / movements.size();
// // //             double median = movements[movements.size() / 2];
// // //             double max_m = movements.back();
            
// // //             std::cout << "  Displacement:" << std::endl;
// // //             std::cout << "    Average: " << avg * 1000 << " mm" << std::endl;
// // //             std::cout << "    Median:  " << median * 1000 << " mm" << std::endl;
// // //             std::cout << "    Max:     " << max_m * 1000 << " mm" << std::endl;
// // //         }
// // //     }
    
// // //     // ============================================================
// // //     // 评估
// // //     // ============================================================
// // //     void Evaluate() {
// // //         tic("4. Evaluation");
        
// // //         std::vector<double> reproj_errors;
// // //         std::vector<double> depth_errors;
        
// // //         for (const auto& kv : points_) {
// // //             const Point3D& pt = kv.second;
            
// // //             for (const auto& obs : pt.observations) {
// // //                 auto it = poses_.find(obs.kf_idx);
// // //                 if (it == poses_.end()) continue;
                
// // //                 // 投影目标位置
// // //                 Eigen::Vector4d P_w;
// // //                 P_w << pt.position_target, 1.0;
// // //                 Eigen::Vector4d P_c = it->second.T_cw * P_w;
                
// // //                 if (P_c[2] <= 0.01) continue;
                
// // //                 double u = FX * (P_c[0] / P_c[2]) + CX;
// // //                 double v = FY * (P_c[1] / P_c[2]) + CY;
                
// // //                 double reproj = std::sqrt(std::pow(u - obs.pixel[0], 2) + 
// // //                                           std::pow(v - obs.pixel[1], 2));
// // //                 double depth = std::abs(P_c[2] - obs.depth_measured);
                
// // //                 reproj_errors.push_back(reproj);
// // //                 depth_errors.push_back(depth);
// // //             }
// // //         }
        
// // //         toc("4. Evaluation");
        
// // //         std::cout << "\n=== Evaluation ===" << std::endl;
        
// // //         if (!reproj_errors.empty()) {
// // //             std::sort(reproj_errors.begin(), reproj_errors.end());
// // //             double avg = std::accumulate(reproj_errors.begin(), reproj_errors.end(), 0.0) / reproj_errors.size();
// // //             std::cout << "  Reprojection Error:" << std::endl;
// // //             std::cout << "    Average: " << avg << " px" << std::endl;
// // //             std::cout << "    Median:  " << reproj_errors[reproj_errors.size()/2] << " px" << std::endl;
// // //         }
        
// // //         if (!depth_errors.empty()) {
// // //             std::sort(depth_errors.begin(), depth_errors.end());
// // //             double avg = std::accumulate(depth_errors.begin(), depth_errors.end(), 0.0) / depth_errors.size();
// // //             std::cout << "  Depth Error:" << std::endl;
// // //             std::cout << "    Average: " << avg * 1000 << " mm" << std::endl;
// // //             std::cout << "    Median:  " << depth_errors[depth_errors.size()/2] * 1000 << " mm" << std::endl;
// // //         }
// // //     }
    
// // //     // ============================================================
// // //     // 保存
// // //     // ============================================================
// // //     void Save(const std::string& file) {
// // //         tic("5. Save");
        
// // //         std::ofstream f(file);
// // //         f << std::fixed << std::setprecision(6);
// // //         f << "# Lindstrom Triangulation\n";
// // //         f << "# id x_orig y_orig z_orig x_target y_target z_target movement_mm is_anchor\n";
        
// // //         for (const auto& kv : points_) {
// // //             const Point3D& pt = kv.second;
// // //             double m = (pt.position_target - pt.position_original).norm();
            
// // //             f << pt.id << " "
// // //               << pt.position_original[0] << " " 
// // //               << pt.position_original[1] << " " 
// // //               << pt.position_original[2] << " "
// // //               << pt.position_target[0] << " " 
// // //               << pt.position_target[1] << " " 
// // //               << pt.position_target[2] << " "
// // //               << m * 1000 << " "
// // //               << (pt.is_anchor ? 1 : 0) << "\n";
// // //         }
        
// // //         toc("5. Save");
// // //         std::cout << "\n  Saved: " << file << std::endl;
// // //     }
    
// // //     // ============================================================
// // //     // 打印计时
// // //     // ============================================================
// // //     void PrintTiming() {
// // //         std::cout << "\n============================================" << std::endl;
// // //         std::cout << "⏱️  Timing Report" << std::endl;
// // //         std::cout << "============================================" << std::endl;
        
// // //         double total = 0;
// // //         std::vector<std::pair<std::string, double>> sorted_timings(timings_.begin(), timings_.end());
// // //         std::sort(sorted_timings.begin(), sorted_timings.end());
        
// // //         for (const auto& kv : sorted_timings) {
// // //             std::cout << "  " << std::setw(25) << std::left << kv.first 
// // //                       << ": " << std::fixed << std::setprecision(2) << kv.second << " ms" << std::endl;
// // //             total += kv.second;
// // //         }
        
// // //         std::cout << "  " << std::string(40, '-') << std::endl;
// // //         std::cout << "  " << std::setw(25) << std::left << "TOTAL" 
// // //                   << ": " << total << " ms" << std::endl;
// // //     }
    
// // //     // ============================================================
// // //     // 主运行
// // //     // ============================================================
// // //     bool Run(const std::string& poses_file, 
// // //              const std::string& data_file, 
// // //              const std::string& output_file) {
        
// // //         std::cout << "============================================" << std::endl;
// // //         std::cout << "  Lindström Linear Triangulator" << std::endl;
// // //         std::cout << "============================================" << std::endl;
        
// // //         std::cout << "\n=== Loading ===" << std::endl;
// // //         if (!LoadPoses(poses_file)) return false;
// // //         if (!LoadPoints(data_file)) return false;
        
// // //         std::cout << "\n=== Triangulation ===" << std::endl;
// // //         Process();
        
// // //         Evaluate();
// // //         Save(output_file);
// // //         PrintTiming();
        
// // //         return true;
// // //     }
// // // };

// // // // ============================================================
// // // // Main
// // // // ============================================================
// // // int main() {
// // //     std::string root = "/Datasets/Work_Data/ACL_OneLoop20251220_214841/";
// // //     std::string loop = root + "loop_1_1670449626.805310/";
    
// // //     LindstromTriangulator tri;
    
// // //     tri.Run(
// // //         loop + "post/standard_trajectory_with_loop.txt",
// // //         root + "Output/optimization_data.txt",
// // //         root + "Output/optimized_points_final.txt"
// // //     );
    
// // //     std::cout << "\n============================================" << std::endl;
// // //     std::cout << "  Complete!" << std::endl;
// // //     std::cout << "============================================" << std::endl;
    
// // //     return 0;
// // // }



// // #include <iostream>
// // #include <fstream>
// // #include <sstream>
// // #include <string>
// // #include <vector>
// // #include <map>
// // #include <iomanip>
// // #include <algorithm>
// // #include <numeric>
// // #include <cmath>
// // #include <chrono>

// // #include <Eigen/Dense>
// // #include <Eigen/Geometry>

// // using namespace std::chrono;

// // const double FX = 377.535257164;
// // const double FY = 377.209841379;
// // const double CX = 328.193371286;
// // const double CY = 240.426878936;

// // struct CameraPose {
// //     Eigen::Matrix4d T_wc, T_cw;
// //     Eigen::Vector3d center;
    
// //     void SetFromTUM(double tx, double ty, double tz, double qx, double qy, double qz, double qw) {
// //         Eigen::Quaterniond q(qw, qx, qy, qz);
// //         q.normalize();
// //         T_wc = Eigen::Matrix4d::Identity();
// //         T_wc.block<3,3>(0,0) = q.toRotationMatrix();
// //         T_wc.block<3,1>(0,3) = Eigen::Vector3d(tx, ty, tz);
// //         T_cw = T_wc.inverse();
// //         center = T_wc.block<3,1>(0,3);
// //     }
// // };

// // struct Observation {
// //     int kf_idx;
// //     Eigen::Vector2d pixel;
// //     double depth_measured;
// //     double depth_projected;
// //     double weight;
// //     double confidence;
// //     double pose_change;
// // };

// // struct Point3D {
// //     int id;
// //     Eigen::Vector3d position_original;
// //     Eigen::Vector3d position_target;
// //     std::vector<Observation> observations;
// //     bool is_anchor;
// // };

// // class LindstromTriangulator {
// // private:
// //     std::map<int, CameraPose> poses_;
// //     std::map<int, Point3D> points_;
// //     std::map<std::string, double> timings_;
    
// //     // ★★★ 新增配置 ★★★
// //     double anchor_max_displacement_ = 0.03;  // 锚点最大位移 3cm
// //     double anchor_blend_factor_ = 0.5;       // 锚点混合因子 (0=不动, 1=完全三角化)
    
// //     void tic(const std::string& name) {
// //         timings_[name] = duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
// //     }
    
// //     void toc(const std::string& name) {
// //         double start = timings_[name];
// //         double end = duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
// //         timings_[name] = (end - start) / 1000.0;
// //     }
    
// // public:
// //     // ★★★ 设置锚点位移参数 ★★★
// //     void SetAnchorParams(double max_disp, double blend) {
// //         anchor_max_displacement_ = max_disp;
// //         anchor_blend_factor_ = blend;
// //     }
    
// //     bool LoadPoses(const std::string& file) {
// //         tic("1. Load Poses");
        
// //         std::ifstream f(file);
// //         if (!f) return false;
        
// //         std::string line;
// //         int id = 0;
// //         while (std::getline(f, line)) {
// //             if (line.empty() || line[0] == '#') continue;
// //             std::istringstream iss(line);
// //             double ts, tx, ty, tz, qx, qy, qz, qw;
// //             if (iss >> ts >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
// //                 CameraPose p;
// //                 p.SetFromTUM(tx, ty, tz, qx, qy, qz, qw);
// //                 poses_[id++] = p;
// //             }
// //         }
        
// //         toc("1. Load Poses");
// //         std::cout << "  Loaded " << poses_.size() << " poses" << std::endl;
// //         return true;
// //     }
    
// //     bool LoadPoints(const std::string& file) {
// //         tic("2. Load Points");
        
// //         std::ifstream f(file);
// //         if (!f) return false;
        
// //         std::string line;
// //         int anchors = 0, moving = 0;
        
// //         while (std::getline(f, line)) {
// //             if (line.empty() || line[0] == '#') continue;
            
// //             if (line[0] != ' ' && line[0] != '\t') {
// //                 std::istringstream iss(line);
// //                 int id, num_obs, is_anchor;
// //                 double x, y, z;
                
// //                 if (iss >> id >> x >> y >> z >> num_obs >> is_anchor) {
// //                     Point3D pt;
// //                     pt.id = id;
// //                     pt.position_original = Eigen::Vector3d(x, y, z);
// //                     pt.is_anchor = (is_anchor == 1);
                    
// //                     if (pt.is_anchor) anchors++; else moving++;
                    
// //                     for (int i = 0; i < num_obs; ++i) {
// //                         if (std::getline(f, line)) {
// //                             size_t first = line.find_first_not_of(" \t");
// //                             if (first != std::string::npos) line = line.substr(first);
                            
// //                             std::istringstream obs_iss(line);
// //                             Observation obs;
// //                             double dist;
                            
// //                             if (obs_iss >> obs.kf_idx >> obs.pixel[0] >> obs.pixel[1]
// //                                 >> obs.depth_measured >> obs.depth_projected
// //                                 >> obs.weight >> obs.confidence >> obs.pose_change >> dist) {
// //                                 pt.observations.push_back(obs);
// //                             }
// //                         }
// //                     }
                    
// //                     if (!pt.observations.empty()) {
// //                         points_[id] = pt;
// //                     }
// //                 }
// //             }
// //         }
        
// //         toc("2. Load Points");
// //         std::cout << "  Loaded " << points_.size() << " points" << std::endl;
// //         std::cout << "  Anchors: " << anchors << ", Moving: " << moving << std::endl;
// //         return true;
// //     }
    
// //     // ============================================================
// //     // Lindström 三角化（不区分锚点/移动点，都计算）
// //     // ============================================================
// //     Eigen::Vector3d Triangulate(const Point3D& point) {
// //         std::vector<Eigen::Vector3d> points_world;
// //         std::vector<double> weights;
        
// //         for (const auto& obs : point.observations) {
// //             auto it = poses_.find(obs.kf_idx);
// //             if (it == poses_.end()) continue;
            
// //             const CameraPose& pose = it->second;
            
// //             double z = obs.depth_measured;
// //             double x = (obs.pixel[0] - CX) * z / FX;
// //             double y = (obs.pixel[1] - CY) * z / FY;
// //             Eigen::Vector3d P_cam(x, y, z);
            
// //             Eigen::Vector3d P_world = pose.T_wc.block<3,3>(0,0) * P_cam + pose.center;
            
// //             double w = obs.weight * obs.confidence;
// //             w *= 1.0 / (1.0 + z / 3.0);
// //             w *= (1.0 + std::min(obs.pose_change * 15.0, 3.0));
            
// //             points_world.push_back(P_world);
// //             weights.push_back(w);
// //         }
        
// //         if (points_world.empty()) {
// //             return point.position_original;
// //         }
        
// //         if (points_world.size() == 1) {
// //             return points_world[0];
// //         }
        
// //         // 加权平均
// //         Eigen::Vector3d mean = Eigen::Vector3d::Zero();
// //         double total_w = 0;
// //         for (size_t i = 0; i < points_world.size(); ++i) {
// //             mean += weights[i] * points_world[i];
// //             total_w += weights[i];
// //         }
// //         mean /= total_w;
        
// //         // 离群点剔除
// //         if (points_world.size() >= 3) {
// //             std::vector<double> distances;
// //             for (const auto& p : points_world) {
// //                 distances.push_back((p - mean).norm());
// //             }
            
// //             std::vector<double> sorted_d = distances;
// //             std::sort(sorted_d.begin(), sorted_d.end());
// //             double median_d = sorted_d[sorted_d.size() / 2];
// //             double threshold = std::max(0.02, median_d * 2.5);
            
// //             Eigen::Vector3d filtered_mean = Eigen::Vector3d::Zero();
// //             double filtered_w = 0;
// //             int kept = 0;
            
// //             for (size_t i = 0; i < points_world.size(); ++i) {
// //                 if (distances[i] < threshold) {
// //                     filtered_mean += weights[i] * points_world[i];
// //                     filtered_w += weights[i];
// //                     kept++;
// //                 }
// //             }
            
// //             if (kept >= 1 && filtered_w > 0) {
// //                 return filtered_mean / filtered_w;
// //             }
// //         }
        
// //         return mean;
// //     }
    
// //     // ============================================================
// //     // 主处理
// //     // ============================================================
// //     void Process() {
// //         tic("3. Triangulation");
        
// //         std::vector<double> anchor_movements;
// //         std::vector<double> moving_movements;
        
// //         for (auto& kv : points_) {
// //             Point3D& pt = kv.second;
            
// //             // ★★★ 所有点都进行三角化 ★★★
// //             Eigen::Vector3d triangulated = Triangulate(pt);
            
// //             if (pt.is_anchor) {
// //                 // ★★★ 锚点：限制位移 ★★★
// //                 Eigen::Vector3d displacement = triangulated - pt.position_original;
// //                 double disp_norm = displacement.norm();
                
// //                 if (disp_norm > 0.001) {  // 忽略极小位移
// //                     // 方法1：限制最大位移
// //                     if (disp_norm > anchor_max_displacement_) {
// //                         displacement = displacement.normalized() * anchor_max_displacement_;
// //                     }
                    
// //                     // 方法2：混合因子
// //                     displacement *= anchor_blend_factor_;
                    
// //                     pt.position_target = pt.position_original + displacement;
// //                     anchor_movements.push_back(displacement.norm());
// //                 } else {
// //                     pt.position_target = pt.position_original;
// //                     anchor_movements.push_back(0);
// //                 }
// //             } else {
// //                 // 移动点：完全使用三角化结果
// //                 pt.position_target = triangulated;
// //                 moving_movements.push_back((triangulated - pt.position_original).norm());
// //             }
// //         }
        
// //         toc("3. Triangulation");
        
// //         // 统计
// //         std::cout << "\n=== Anchor Points ===" << std::endl;
// //         if (!anchor_movements.empty()) {
// //             std::sort(anchor_movements.begin(), anchor_movements.end());
// //             double avg = std::accumulate(anchor_movements.begin(), anchor_movements.end(), 0.0) / anchor_movements.size();
// //             std::cout << "  Count: " << anchor_movements.size() << std::endl;
// //             std::cout << "  Displacement: avg=" << avg*1000 << "mm, median=" 
// //                       << anchor_movements[anchor_movements.size()/2]*1000 << "mm, max=" 
// //                       << anchor_movements.back()*1000 << "mm" << std::endl;
// //         }
        
// //         std::cout << "\n=== Moving Points ===" << std::endl;
// //         if (!moving_movements.empty()) {
// //             std::sort(moving_movements.begin(), moving_movements.end());
// //             double avg = std::accumulate(moving_movements.begin(), moving_movements.end(), 0.0) / moving_movements.size();
// //             std::cout << "  Count: " << moving_movements.size() << std::endl;
// //             std::cout << "  Displacement: avg=" << avg*1000 << "mm, median=" 
// //                       << moving_movements[moving_movements.size()/2]*1000 << "mm, max=" 
// //                       << moving_movements.back()*1000 << "mm" << std::endl;
// //         }
// //     }
    
// //     void Evaluate() {
// //         tic("4. Evaluation");
        
// //         std::vector<double> reproj_errors;
// //         std::vector<double> depth_errors;
        
// //         for (const auto& kv : points_) {
// //             const Point3D& pt = kv.second;
            
// //             for (const auto& obs : pt.observations) {
// //                 auto it = poses_.find(obs.kf_idx);
// //                 if (it == poses_.end()) continue;
                
// //                 Eigen::Vector4d P_w;
// //                 P_w << pt.position_target, 1.0;
// //                 Eigen::Vector4d P_c = it->second.T_cw * P_w;
                
// //                 if (P_c[2] <= 0.01) continue;
                
// //                 double u = FX * (P_c[0] / P_c[2]) + CX;
// //                 double v = FY * (P_c[1] / P_c[2]) + CY;
                
// //                 double reproj = std::sqrt(std::pow(u - obs.pixel[0], 2) + 
// //                                           std::pow(v - obs.pixel[1], 2));
// //                 double depth = std::abs(P_c[2] - obs.depth_measured);
                
// //                 reproj_errors.push_back(reproj);
// //                 depth_errors.push_back(depth);
// //             }
// //         }
        
// //         toc("4. Evaluation");
        
// //         std::cout << "\n=== Evaluation ===" << std::endl;
        
// //         if (!reproj_errors.empty()) {
// //             std::sort(reproj_errors.begin(), reproj_errors.end());
// //             double avg = std::accumulate(reproj_errors.begin(), reproj_errors.end(), 0.0) / reproj_errors.size();
// //             std::cout << "  Reprojection: avg=" << avg << "px, median=" 
// //                       << reproj_errors[reproj_errors.size()/2] << "px" << std::endl;
// //         }
        
// //         if (!depth_errors.empty()) {
// //             std::sort(depth_errors.begin(), depth_errors.end());
// //             double avg = std::accumulate(depth_errors.begin(), depth_errors.end(), 0.0) / depth_errors.size();
// //             std::cout << "  Depth: avg=" << avg*1000 << "mm, median=" 
// //                       << depth_errors[depth_errors.size()/2]*1000 << "mm" << std::endl;
// //         }
// //     }
    
// //     void Save(const std::string& file) {
// //         tic("5. Save");
        
// //         std::ofstream f(file);
// //         f << std::fixed << std::setprecision(6);
// //         f << "# Lindstrom Triangulation (anchor_max_disp=" << anchor_max_displacement_*1000 
// //           << "mm, blend=" << anchor_blend_factor_ << ")\n";
// //         f << "# id x_orig y_orig z_orig x_target y_target z_target movement_mm is_anchor\n";
        
// //         for (const auto& kv : points_) {
// //             const Point3D& pt = kv.second;
// //             double m = (pt.position_target - pt.position_original).norm();
            
// //             f << pt.id << " "
// //               << pt.position_original[0] << " " 
// //               << pt.position_original[1] << " " 
// //               << pt.position_original[2] << " "
// //               << pt.position_target[0] << " " 
// //               << pt.position_target[1] << " " 
// //               << pt.position_target[2] << " "
// //               << m * 1000 << " "
// //               << (pt.is_anchor ? 1 : 0) << "\n";
// //         }
        
// //         toc("5. Save");
// //         std::cout << "\n  Saved: " << file << std::endl;
// //     }
    
// //     void PrintTiming() {
// //         std::cout << "\n============================================" << std::endl;
// //         std::cout << "⏱️  Timing Report" << std::endl;
// //         std::cout << "============================================" << std::endl;
        
// //         double total = 0;
// //         std::vector<std::pair<std::string, double>> sorted_timings(timings_.begin(), timings_.end());
// //         std::sort(sorted_timings.begin(), sorted_timings.end());
        
// //         for (const auto& kv : sorted_timings) {
// //             std::cout << "  " << std::setw(25) << std::left << kv.first 
// //                       << ": " << std::fixed << std::setprecision(2) << kv.second << " ms" << std::endl;
// //             total += kv.second;
// //         }
        
// //         std::cout << "  " << std::string(40, '-') << std::endl;
// //         std::cout << "  " << std::setw(25) << std::left << "TOTAL" << ": " << total << " ms" << std::endl;
// //     }
    
// //     bool Run(const std::string& poses_file, 
// //              const std::string& data_file, 
// //              const std::string& output_file) {
        
// //         std::cout << "============================================" << std::endl;
// //         std::cout << "  Lindström Triangulator" << std::endl;
// //         std::cout << "  Anchor max displacement: " << anchor_max_displacement_*1000 << " mm" << std::endl;
// //         std::cout << "  Anchor blend factor: " << anchor_blend_factor_ << std::endl;
// //         std::cout << "============================================" << std::endl;
        
// //         std::cout << "\n=== Loading ===" << std::endl;
// //         if (!LoadPoses(poses_file)) return false;
// //         if (!LoadPoints(data_file)) return false;
        
// //         std::cout << "\n=== Triangulation ===" << std::endl;
// //         Process();
        
// //         Evaluate();
// //         Save(output_file);
// //         PrintTiming();
        
// //         return true;
// //     }
// // };

// // int main(int argc, char** argv) {
// //     std::string root = "/Datasets/Work_Data/ACL_OneLoop20251220_214841/";
// //     std::string loop = root + "loop_1_1670449626.805310/";
    
// //     // ★★★ 可通过命令行参数调整 ★★★
// //     // ./program [anchor_max_disp_mm] [anchor_blend_factor]
// //     // 例如: ./program 30 0.5  -> 锚点最大移动30mm，混合因子0.5
    
// //     double anchor_max_disp = 0.03;  // 默认 30mm
// //     double anchor_blend = 0.5;      // 默认 0.5
    
// //     if (argc > 1) anchor_max_disp = std::atof(argv[1]) / 1000.0;
// //     if (argc > 2) anchor_blend = std::atof(argv[2]);
    
// //     LindstromTriangulator tri;
// //     tri.SetAnchorParams(anchor_max_disp, anchor_blend);
    
// //     tri.Run(
// //         loop + "post/standard_trajectory_with_loop.txt",
// //         root + "Output/optimization_data.txt",
// //         root + "Output/optimized_points_final.txt"
// //     );
    
// //     std::cout << "\n============================================" << std::endl;
// //     std::cout << "  Complete!" << std::endl;
// //     std::cout << "============================================" << std::endl;
    
// //     return 0;
// // }











// #include <iostream>
// #include <fstream>
// #include <sstream>
// #include <string>
// #include <vector>
// #include <map>
// #include <iomanip>
// #include <algorithm>
// #include <numeric>
// #include <cmath>
// #include <chrono>

// #include <Eigen/Dense>
// #include <Eigen/Geometry>

// using namespace std::chrono;

// const double FX = 377.535257164;
// const double FY = 377.209841379;
// const double CX = 328.193371286;
// const double CY = 240.426878936;

// struct CameraPose {
//     Eigen::Matrix4d T_wc, T_cw;
//     Eigen::Vector3d center;
    
//     void SetFromTUM(double tx, double ty, double tz, double qx, double qy, double qz, double qw) {
//         Eigen::Quaterniond q(qw, qx, qy, qz);
//         q.normalize();
//         T_wc = Eigen::Matrix4d::Identity();
//         T_wc.block<3,3>(0,0) = q.toRotationMatrix();
//         T_wc.block<3,1>(0,3) = Eigen::Vector3d(tx, ty, tz);
//         T_cw = T_wc.inverse();
//         center = T_wc.block<3,1>(0,3);
//     }
// };

// struct Observation {
//     int kf_idx;
//     Eigen::Vector2d pixel;
//     double depth_measured;
//     double depth_projected;
//     double weight;
//     double confidence;
//     double pose_change;
// };

// struct Point3D {
//     int id;
//     Eigen::Vector3d position_original;
//     Eigen::Vector3d position_target;
//     std::vector<Observation> observations;
//     bool is_anchor_input;   // 输入的锚点标记（来自 ray_caster）
//     bool is_anchor_output;  // 输出的锚点标记（由本程序决定）
// };

// class LindstromTriangulator {
// private:
//     std::map<int, CameraPose> poses_;
//     std::map<int, Point3D> points_;
//     std::map<std::string, double> timings_;
    
//     // ★★★ 简化后的参数 ★★★
//     double displacement_scale_ = 1.0;    // 位移缩放
//     double displacement_offset_ = 0.0;   // 位移偏移 (m)
    
//     // 锚点判断模式
//     // 0 = 忽略输入的锚点标记，全部当移动点处理
//     // 1 = 使用输入的锚点标记
//     // 2 = 根据三角化后的位移重新判断锚点
//     int anchor_mode_ = 0;
//     double anchor_threshold_ = 0.03;     // 锚点判断阈值 (m)，仅 mode=2 时有效
    
//     void tic(const std::string& name) {
//         timings_[name] = duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
//     }
    
//     void toc(const std::string& name) {
//         double start = timings_[name];
//         double end = duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
//         timings_[name] = (end - start) / 1000.0;
//     }
    
// public:
//     void SetDisplacementParams(double scale, double offset_mm = 0) {
//         displacement_scale_ = scale;
//         displacement_offset_ = offset_mm / 1000.0;
//     }
    
//     void SetAnchorMode(int mode, double threshold_mm = 30) {
//         anchor_mode_ = mode;
//         anchor_threshold_ = threshold_mm / 1000.0;
//     }
    
//     bool LoadPoses(const std::string& file) {
//         tic("1. Load Poses");
        
//         std::ifstream f(file);
//         if (!f) return false;
        
//         std::string line;
//         int id = 0;
//         while (std::getline(f, line)) {
//             if (line.empty() || line[0] == '#') continue;
//             std::istringstream iss(line);
//             double ts, tx, ty, tz, qx, qy, qz, qw;
//             if (iss >> ts >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
//                 CameraPose p;
//                 p.SetFromTUM(tx, ty, tz, qx, qy, qz, qw);
//                 poses_[id++] = p;
//             }
//         }
        
//         toc("1. Load Poses");
//         std::cout << "  Loaded " << poses_.size() << " poses" << std::endl;
//         return true;
//     }
    
//     bool LoadPoints(const std::string& file) {
//         tic("2. Load Points");
        
//         std::ifstream f(file);
//         if (!f) return false;
        
//         std::string line;
//         int input_anchors = 0, input_moving = 0;
        
//         while (std::getline(f, line)) {
//             if (line.empty() || line[0] == '#') continue;
            
//             if (line[0] != ' ' && line[0] != '\t') {
//                 std::istringstream iss(line);
//                 int id, num_obs, is_anchor;
//                 double x, y, z;
                
//                 if (iss >> id >> x >> y >> z >> num_obs >> is_anchor) {
//                     Point3D pt;
//                     pt.id = id;
//                     pt.position_original = Eigen::Vector3d(x, y, z);
//                     pt.is_anchor_input = (is_anchor == 1);
//                     pt.is_anchor_output = false;  // 稍后决定
                    
//                     if (pt.is_anchor_input) input_anchors++; else input_moving++;
                    
//                     for (int i = 0; i < num_obs; ++i) {
//                         if (std::getline(f, line)) {
//                             size_t first = line.find_first_not_of(" \t");
//                             if (first != std::string::npos) line = line.substr(first);
                            
//                             std::istringstream obs_iss(line);
//                             Observation obs;
//                             double dist;
                            
//                             if (obs_iss >> obs.kf_idx >> obs.pixel[0] >> obs.pixel[1]
//                                 >> obs.depth_measured >> obs.depth_projected
//                                 >> obs.weight >> obs.confidence >> obs.pose_change >> dist) {
//                                 pt.observations.push_back(obs);
//                             }
//                         }
//                     }
                    
//                     if (!pt.observations.empty()) {
//                         points_[id] = pt;
//                     }
//                 }
//             }
//         }
        
//         toc("2. Load Points");
//         std::cout << "  Loaded " << points_.size() << " points" << std::endl;
//         std::cout << "  Input: anchors=" << input_anchors << ", moving=" << input_moving << std::endl;
//         return true;
//     }
    
//     Eigen::Vector3d Triangulate(const Point3D& point) {
//         std::vector<Eigen::Vector3d> points_world;
//         std::vector<double> weights;
        
//         for (const auto& obs : point.observations) {
//             auto it = poses_.find(obs.kf_idx);
//             if (it == poses_.end()) continue;
            
//             const CameraPose& pose = it->second;
            
//             double z = obs.depth_measured;
//             double x = (obs.pixel[0] - CX) * z / FX;
//             double y = (obs.pixel[1] - CY) * z / FY;
//             Eigen::Vector3d P_cam(x, y, z);
            
//             Eigen::Vector3d P_world = pose.T_wc.block<3,3>(0,0) * P_cam + pose.center;
            
//             double w = obs.weight * obs.confidence;
//             w *= 1.0 / (1.0 + z / 3.0);
//             w *= (1.0 + std::min(obs.pose_change * 15.0, 3.0));
            
//             points_world.push_back(P_world);
//             weights.push_back(w);
//         }
        
//         if (points_world.empty()) return point.position_original;
//         if (points_world.size() == 1) return points_world[0];
        
//         // 加权平均
//         Eigen::Vector3d mean = Eigen::Vector3d::Zero();
//         double total_w = 0;
//         for (size_t i = 0; i < points_world.size(); ++i) {
//             mean += weights[i] * points_world[i];
//             total_w += weights[i];
//         }
//         mean /= total_w;
        
//         // 离群点剔除
//         if (points_world.size() >= 3) {
//             std::vector<double> distances;
//             for (const auto& p : points_world) {
//                 distances.push_back((p - mean).norm());
//             }
            
//             std::vector<double> sorted_d = distances;
//             std::sort(sorted_d.begin(), sorted_d.end());
//             double median_d = sorted_d[sorted_d.size() / 2];
//             double threshold = std::max(0.02, median_d * 2.5);
            
//             Eigen::Vector3d filtered_mean = Eigen::Vector3d::Zero();
//             double filtered_w = 0;
//             int kept = 0;
            
//             for (size_t i = 0; i < points_world.size(); ++i) {
//                 if (distances[i] < threshold) {
//                     filtered_mean += weights[i] * points_world[i];
//                     filtered_w += weights[i];
//                     kept++;
//                 }
//             }
            
//             if (kept >= 1 && filtered_w > 0) {
//                 return filtered_mean / filtered_w;
//             }
//         }
        
//         return mean;
//     }
    
//     void Process() {
//         tic("3. Triangulation");
        
//         std::vector<double> all_movements;
        
//         // ★★★ 第一步：所有点都做三角化，计算位移 ★★★
//         for (auto& kv : points_) {
//             Point3D& pt = kv.second;
            
//             Eigen::Vector3d triangulated = Triangulate(pt);
//             Eigen::Vector3d displacement = triangulated - pt.position_original;
//             double disp_norm = displacement.norm();
            
//             // 应用缩放和偏移
//             if (disp_norm > 0.0001) {
//                 double new_disp = disp_norm * displacement_scale_ + displacement_offset_;
//                 if (new_disp < 0) new_disp = 0;
//                 pt.position_target = pt.position_original + displacement.normalized() * new_disp;
//             } else {
//                 pt.position_target = pt.position_original;
//             }
            
//             all_movements.push_back((pt.position_target - pt.position_original).norm());
//         }
        
//         // ★★★ 第二步：根据 anchor_mode 决定锚点标记 ★★★
//         int output_anchors = 0, output_moving = 0;
        
//         for (auto& kv : points_) {
//             Point3D& pt = kv.second;
//             double disp = (pt.position_target - pt.position_original).norm();
            
//             switch (anchor_mode_) {
//                 case 0:
//                     // 全部当移动点
//                     pt.is_anchor_output = false;
//                     break;
                    
//                 case 1:
//                     // 使用输入的锚点标记
//                     pt.is_anchor_output = pt.is_anchor_input;
//                     break;
                    
//                 case 2:
//                     // 根据三角化后的位移重新判断
//                     pt.is_anchor_output = (disp < anchor_threshold_);
//                     break;
                    
//                 default:
//                     pt.is_anchor_output = false;
//             }
            
//             if (pt.is_anchor_output) output_anchors++;
//             else output_moving++;
//         }
        
//         toc("3. Triangulation");
        
//         // 统计
//         std::cout << "\n=== Displacement Statistics ===" << std::endl;
//         if (!all_movements.empty()) {
//             std::sort(all_movements.begin(), all_movements.end());
//             double avg = std::accumulate(all_movements.begin(), all_movements.end(), 0.0) / all_movements.size();
//             std::cout << "  All points (" << all_movements.size() << "):" << std::endl;
//             std::cout << "    avg=" << avg*1000 << "mm, median=" << all_movements[all_movements.size()/2]*1000 
//                       << "mm, max=" << all_movements.back()*1000 << "mm" << std::endl;
//         }
        
//         std::cout << "\n=== Output Anchor Assignment ===" << std::endl;
//         std::cout << "  Mode: " << anchor_mode_;
//         switch (anchor_mode_) {
//             case 0: std::cout << " (all moving)"; break;
//             case 1: std::cout << " (use input)"; break;
//             case 2: std::cout << " (by threshold " << anchor_threshold_*1000 << "mm)"; break;
//         }
//         std::cout << std::endl;
//         std::cout << "  Output: anchors=" << output_anchors << ", moving=" << output_moving << std::endl;
//     }
    
//     void Evaluate() {
//         tic("4. Evaluation");
        
//         std::vector<double> reproj_errors, depth_errors;
        
//         for (const auto& kv : points_) {
//             const Point3D& pt = kv.second;
            
//             for (const auto& obs : pt.observations) {
//                 auto it = poses_.find(obs.kf_idx);
//                 if (it == poses_.end()) continue;
                
//                 Eigen::Vector4d P_w;
//                 P_w << pt.position_target, 1.0;
//                 Eigen::Vector4d P_c = it->second.T_cw * P_w;
                
//                 if (P_c[2] <= 0.01) continue;
                
//                 double u = FX * (P_c[0] / P_c[2]) + CX;
//                 double v = FY * (P_c[1] / P_c[2]) + CY;
                
//                 reproj_errors.push_back(std::sqrt(std::pow(u - obs.pixel[0], 2) + std::pow(v - obs.pixel[1], 2)));
//                 depth_errors.push_back(std::abs(P_c[2] - obs.depth_measured));
//             }
//         }
        
//         toc("4. Evaluation");
        
//         std::cout << "\n=== Evaluation ===" << std::endl;
//         if (!reproj_errors.empty()) {
//             std::sort(reproj_errors.begin(), reproj_errors.end());
//             double avg = std::accumulate(reproj_errors.begin(), reproj_errors.end(), 0.0) / reproj_errors.size();
//             std::cout << "  Reprojection: avg=" << avg << "px, median=" << reproj_errors[reproj_errors.size()/2] << "px" << std::endl;
//         }
//         if (!depth_errors.empty()) {
//             std::sort(depth_errors.begin(), depth_errors.end());
//             double avg = std::accumulate(depth_errors.begin(), depth_errors.end(), 0.0) / depth_errors.size();
//             std::cout << "  Depth: avg=" << avg*1000 << "mm, median=" << depth_errors[depth_errors.size()/2]*1000 << "mm" << std::endl;
//         }
//     }
    
//     void Save(const std::string& file) {
//         tic("5. Save");
        
//         std::ofstream f(file);
//         f << std::fixed << std::setprecision(6);
//         f << "# Lindstrom Triangulation (Simplified)\n";
//         f << "# disp_scale=" << displacement_scale_ << ", disp_offset=" << displacement_offset_*1000 << "mm\n";
//         f << "# anchor_mode=" << anchor_mode_ << ", anchor_threshold=" << anchor_threshold_*1000 << "mm\n";
//         f << "# id x_orig y_orig z_orig x_target y_target z_target movement_mm is_anchor\n";
        
//         for (const auto& kv : points_) {
//             const Point3D& pt = kv.second;
//             double m = (pt.position_target - pt.position_original).norm();
            
//             f << pt.id << " "
//               << pt.position_original[0] << " " << pt.position_original[1] << " " << pt.position_original[2] << " "
//               << pt.position_target[0] << " " << pt.position_target[1] << " " << pt.position_target[2] << " "
//               << m * 1000 << " " << (pt.is_anchor_output ? 1 : 0) << "\n";
//         }
        
//         toc("5. Save");
//         std::cout << "\n  Saved: " << file << std::endl;
//     }
    
//     void PrintTiming() {
//         std::cout << "\n============================================" << std::endl;
//         std::cout << "⏱️  Timing Report" << std::endl;
//         std::cout << "============================================" << std::endl;
        
//         double total = 0;
//         std::vector<std::pair<std::string, double>> sorted_timings(timings_.begin(), timings_.end());
//         std::sort(sorted_timings.begin(), sorted_timings.end());
        
//         for (const auto& kv : sorted_timings) {
//             std::cout << "  " << std::setw(22) << std::left << kv.first 
//                       << ": " << std::fixed << std::setprecision(2) << kv.second << " ms" << std::endl;
//             total += kv.second;
//         }
//         std::cout << "  " << std::string(35, '-') << std::endl;
//         std::cout << "  " << std::setw(22) << std::left << "TOTAL" << ": " << total << " ms" << std::endl;
//     }
    
//     bool Run(const std::string& poses_file, const std::string& data_file, const std::string& output_file) {
//         std::cout << "============================================" << std::endl;
//         std::cout << "  Lindström Triangulator (Simplified)" << std::endl;
//         std::cout << "============================================" << std::endl;
//         std::cout << "  Displacement scale:  " << displacement_scale_ << " (" << (displacement_scale_-1)*100 << "%)" << std::endl;
//         std::cout << "  Displacement offset: " << displacement_offset_*1000 << " mm" << std::endl;
//         std::cout << "  Anchor mode:         " << anchor_mode_;
//         switch (anchor_mode_) {
//             case 0: std::cout << " (all -> moving)"; break;
//             case 1: std::cout << " (use input)"; break;
//             case 2: std::cout << " (by threshold)"; break;
//         }
//         std::cout << std::endl;
//         if (anchor_mode_ == 2) {
//             std::cout << "  Anchor threshold:    " << anchor_threshold_*1000 << " mm" << std::endl;
//         }
//         std::cout << "============================================" << std::endl;
        
//         std::cout << "\n=== Loading ===" << std::endl;
//         if (!LoadPoses(poses_file)) return false;
//         if (!LoadPoints(data_file)) return false;
        
//         std::cout << "\n=== Processing ===" << std::endl;
//         Process();
//         Evaluate();
//         Save(output_file);
//         PrintTiming();
        
//         return true;
//     }
// };

// int main(int argc, char** argv) {
//     std::string root = "/Datasets/Work_Data/ACL_OneLoop20251220_214841/";
//     std::string loop = root + "loop_1_1670449626.805310/";
    
//     // ★★★ 简化后的命令行参数 ★★★
//     // ./program [disp_scale] [disp_offset_mm] [anchor_mode] [anchor_threshold_mm]
//     //
//     // anchor_mode:
//     //   0 = 全部当移动点，ARAP 不区分 (推荐先试这个)
//     //   1 = 使用 ray_caster 输入的锚点标记
//     //   2 = 根据三角化后的位移重新判断锚点
//     //
//     // 例子:
//     //   ./program                     # 默认: scale=1.0, offset=0, mode=0
//     //   ./program 1.0 0 0             # 全部当移动点
//     //   ./program 1.01 0 0            # 位移放大1%，全部当移动点
//     //   ./program 1.0 0 1             # 使用输入的锚点标记
//     //   ./program 1.0 0 2 30          # 根据30mm阈值重新判断锚点
    
//     double disp_scale = 1.0;
//     double disp_offset = 0;
//     int anchor_mode = 0;
//     double anchor_threshold = 30;
    
//     if (argc > 1) disp_scale = std::atof(argv[1]);
//     if (argc > 2) disp_offset = std::atof(argv[2]);
//     if (argc > 3) anchor_mode = std::atoi(argv[3]);
//     if (argc > 4) anchor_threshold = std::atof(argv[4]);
    
//     LindstromTriangulator tri;
//     tri.SetDisplacementParams(disp_scale, disp_offset);
//     tri.SetAnchorMode(anchor_mode, anchor_threshold);
    
//     tri.Run(
//         loop + "post/standard_trajectory_with_loop.txt",
//         root + "Output/optimization_data.txt",
//         root + "Output/optimized_points_final.txt"
//     );
    
//     std::cout << "\n============================================" << std::endl;
//     std::cout << "  Complete!" << std::endl;
//     std::cout << "============================================" << std::endl;
    
//     return 0;
// }

// // ---

// // ## 简化后的逻辑
// // ```
// // Ray Caster (代码1)
// // ├── 找 mesh-深度图 对应点
// // ├── 计算 is_anchor（可以保留，也可以忽略）
// // └── 输出 optimization_data.txt

// // 三角化器 (代码2) ← 全权负责
// // ├── 所有点都做三角化
// // ├── 应用 scale 和 offset
// // ├── 根据 anchor_mode 决定输出的锚点标记
// // │   ├── mode=0: 全部当移动点 ★推荐
// // │   ├── mode=1: 使用输入标记
// // │   └── mode=2: 根据三角化位移重新判断
// // └── 输出 optimized_points_final.txt









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