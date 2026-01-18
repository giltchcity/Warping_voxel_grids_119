
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <map>
#include <set>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <iomanip>
#include <unordered_map>
#include <numeric>

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <omp.h>
#include <chrono>

#include "happly.h"

using namespace std;
using namespace Eigen;
using namespace std::chrono;

// ============================================================
// AABB & BVH for Ray-Triangle Intersection
// ============================================================
struct AABB {
    Vector3d min_pt, max_pt;
    AABB() : min_pt(Vector3d::Constant(1e30)), max_pt(Vector3d::Constant(-1e30)) {}
    void expand(const Vector3d& pt) { min_pt = min_pt.cwiseMin(pt); max_pt = max_pt.cwiseMax(pt); }
    void expand(const AABB& o) { min_pt = min_pt.cwiseMin(o.min_pt); max_pt = max_pt.cwiseMax(o.max_pt); }
    bool intersectRay(const Vector3d& origin, const Vector3d& dir_inv, double t_max) const {
        double t1 = (min_pt.x() - origin.x()) * dir_inv.x();
        double t2 = (max_pt.x() - origin.x()) * dir_inv.x();
        double tmin = std::min(t1, t2), tmax = std::max(t1, t2);
        t1 = (min_pt.y() - origin.y()) * dir_inv.y();
        t2 = (max_pt.y() - origin.y()) * dir_inv.y();
        tmin = std::max(tmin, std::min(t1, t2)); tmax = std::min(tmax, std::max(t1, t2));
        t1 = (min_pt.z() - origin.z()) * dir_inv.z();
        t2 = (max_pt.z() - origin.z()) * dir_inv.z();
        tmin = std::max(tmin, std::min(t1, t2)); tmax = std::min(tmax, std::max(t1, t2));
        return tmax >= std::max(0.0, tmin) && tmin < t_max;
    }
};

struct BVHNode {
    AABB bbox; 
    int left, right, face_start, face_count;
    BVHNode() : left(-1), right(-1), face_start(-1), face_count(0) {}
    bool isLeaf() const { return left == -1 && right == -1; }
};

class BVH {
public:
    vector<BVHNode> nodes;
    vector<int> face_indices;
    const vector<Vector3d>* vertices;
    const vector<Vector3i>* faces;
    int max_leaf_size = 4;
    
    void build(const vector<Vector3d>& verts, const vector<Vector3i>& tris) {
        vertices = &verts; 
        faces = &tris;
        if (tris.empty()) return;
        face_indices.resize(tris.size());
        iota(face_indices.begin(), face_indices.end(), 0);
        vector<AABB> boxes(tris.size());
        vector<Vector3d> centers(tris.size());
        for (size_t i = 0; i < tris.size(); ++i) {
            const auto& f = tris[i];
            boxes[i].expand(verts[f[0]]); 
            boxes[i].expand(verts[f[1]]); 
            boxes[i].expand(verts[f[2]]);
            centers[i] = (verts[f[0]] + verts[f[1]] + verts[f[2]]) / 3.0;
        }
        nodes.reserve(tris.size() * 2);
        buildRec(0, tris.size(), boxes, centers);
    }
    
    int buildRec(int start, int end, const vector<AABB>& boxes, const vector<Vector3d>& centers) {
        int idx = nodes.size(); 
        nodes.push_back(BVHNode());
        AABB bbox; 
        for (int i = start; i < end; ++i) bbox.expand(boxes[face_indices[i]]);
        nodes[idx].bbox = bbox;
        int count = end - start;
        if (count <= max_leaf_size) { 
            nodes[idx].face_start = start; 
            nodes[idx].face_count = count; 
            return idx; 
        }
        Vector3d ext = bbox.max_pt - bbox.min_pt;
        int axis = (ext.y() > ext.x()) ? 1 : 0; 
        if (ext.z() > ext[axis]) axis = 2;
        int mid = (start + end) / 2;
        nth_element(face_indices.begin() + start, face_indices.begin() + mid, face_indices.begin() + end,
                    [&](int a, int b) { return centers[a][axis] < centers[b][axis]; });
        nodes[idx].left = buildRec(start, mid, boxes, centers);
        nodes[idx].right = buildRec(mid, end, boxes, centers);
        return idx;
    }
    
    bool rayTriInt(const Vector3d& o, const Vector3d& d, const Vector3d& v0, const Vector3d& v1, const Vector3d& v2, double& t) const {
        Vector3d e1 = v1 - v0, e2 = v2 - v0, h = d.cross(e2);
        double a = e1.dot(h); 
        if (fabs(a) < 1e-8) return false;
        double f = 1.0 / a; 
        Vector3d s = o - v0;
        double u = f * s.dot(h); 
        if (u < 0 || u > 1) return false;
        Vector3d q = s.cross(e1);
        double v = f * d.dot(q); 
        if (v < 0 || u + v > 1) return false;
        t = f * e2.dot(q); 
        return t > 1e-8;
    }
    
    bool raycast(const Vector3d& o, const Vector3d& d, double max_t, int exclude, double& hit_t) const {
        if (nodes.empty()) return false;
        Vector3d inv(1.0/d.x(), 1.0/d.y(), 1.0/d.z());
        hit_t = max_t; 
        bool found = false;
        vector<int> stk; 
        stk.reserve(64); 
        stk.push_back(0);
        while (!stk.empty()) {
            int ni = stk.back(); 
            stk.pop_back();
            const auto& n = nodes[ni];
            if (!n.bbox.intersectRay(o, inv, hit_t)) continue;
            if (n.isLeaf()) {
                for (int i = 0; i < n.face_count; ++i) {
                    int fi = face_indices[n.face_start + i];
                    const auto& f = (*faces)[fi];
                    if (f[0] == exclude || f[1] == exclude || f[2] == exclude) continue;
                    double t;
                    if (rayTriInt(o, d, (*vertices)[f[0]], (*vertices)[f[1]], (*vertices)[f[2]], t))
                        if (t < hit_t) { hit_t = t; found = true; }
                }
            } else { 
                stk.push_back(n.right); 
                stk.push_back(n.left); 
            }
        }
        return found;
    }
};

// ============================================================
// Observation Data Structure
// ============================================================
struct Observation {
    int kf_idx;
    double u, v;
    double depth_measured;
    double depth_projected;
    double weight;
    double confidence;
    double distance_3d;
};

// ============================================================
// Mesh Loading
// ============================================================
void loadMesh(const string& path, vector<Vector3d>& vertices, vector<Vector3d>& normals, vector<Vector3i>& faces) {
    happly::PLYData ply(path);
    auto vx = ply.getElement("vertex").getProperty<float>("x");
    auto vy = ply.getElement("vertex").getProperty<float>("y");
    auto vz = ply.getElement("vertex").getProperty<float>("z");
    vertices.resize(vx.size());
    for (size_t i = 0; i < vx.size(); ++i) 
        vertices[i] = Vector3d(vx[i], vy[i], vz[i]);
    
    try {
        auto fi = ply.getFaceIndices<size_t>();
        for (const auto& f : fi) 
            if (f.size() >= 3) 
                faces.push_back(Vector3i(f[0], f[1], f[2]));
    } catch (...) {}
    
    // Compute vertex normals from face normals
    normals.resize(vertices.size(), Vector3d::Zero());
    for (const auto& f : faces) {
        if (f[0] >= (int)vertices.size()) continue;
        Vector3d fn = (vertices[f[1]] - vertices[f[0]]).cross(vertices[f[2]] - vertices[f[0]]);
        normals[f[0]] += fn; 
        normals[f[1]] += fn; 
        normals[f[2]] += fn;
    }
    for (auto& n : normals) { 
        double l = n.norm(); 
        if (l > 1e-10) n /= l; 
    }
}

// ============================================================
// Ray Caster - Simplified Version
// ============================================================
class RayCaster {
public:
    // Camera parameters
    double fx, fy, cx, cy;
    int width, height;
    
    // Processing parameters
    double min_depth, max_depth;
    double base_distance_threshold;
    double depth_consistency_threshold;
    double max_grazing_angle, cos_min_angle;
    int min_consecutive_observations;
    int max_frame_gap;
    int max_observations_per_point;
    double occlusion_margin;
    int search_radius;
    
    const BVH* mesh_bvh;
    map<int, cv::Mat> depth_images;
    
    // Statistics
    atomic<long long> stat_total_checked{0};
    atomic<long long> stat_passed{0};
    
    RayCaster(const map<string, double>& cam, const map<string, double>& params = {}) {
        fx = cam.at("fx"); 
        fy = cam.at("fy"); 
        cx = cam.at("cx"); 
        cy = cam.at("cy");
        width = cam.count("width") ? (int)cam.at("width") : 640;
        height = cam.count("height") ? (int)cam.at("height") : 480;
        
        min_depth = params.count("min_depth") ? params.at("min_depth") : 0.1;
        max_depth = params.count("max_depth") ? params.at("max_depth") : 10.0;
        base_distance_threshold = params.count("base_distance_threshold") ? params.at("base_distance_threshold") : 0.08;
        depth_consistency_threshold = params.count("depth_consistency_threshold") ? params.at("depth_consistency_threshold") : 0.1;
        max_grazing_angle = params.count("max_grazing_angle") ? params.at("max_grazing_angle") : 75.0;
        cos_min_angle = cos(max_grazing_angle * M_PI / 180.0);
        min_consecutive_observations = params.count("min_consecutive_observations") ? (int)params.at("min_consecutive_observations") : 1;
        max_frame_gap = params.count("max_frame_gap") ? (int)params.at("max_frame_gap") : 2;
        max_observations_per_point = params.count("max_observations_per_point") ? (int)params.at("max_observations_per_point") : 15;
        occlusion_margin = params.count("occlusion_margin") ? params.at("occlusion_margin") : 0.02;
        search_radius = params.count("search_radius") ? (int)params.at("search_radius") : 0;
        
        mesh_bvh = nullptr;
    }
    
    void setBVH(const BVH* bvh) { mesh_bvh = bvh; }
    
    // Load trajectory file (TUM format)
    tuple<map<int, Matrix4d>, vector<int>, vector<double>> loadTrajectory(const string& path) {
        map<int, Matrix4d> poses; 
        vector<int> ids; 
        vector<double> ts;
        ifstream f(path); 
        if (!f) return {poses, ids, ts};
        
        string line; 
        int idx = 0;
        while (getline(f, line)) {
            if (line.empty() || line[0] == '#') continue;
            istringstream ss(line);
            double t, tx, ty, tz, qx, qy, qz, qw;
            if (!(ss >> t >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) continue;
            if (t > 1e15) t /= 1e9;
            Quaterniond q(qw, qx, qy, qz);
            Matrix4d T = Matrix4d::Identity();
            T.block<3,3>(0,0) = q.toRotationMatrix();
            T.block<3,1>(0,3) = Vector3d(tx, ty, tz);
            poses[idx] = T.inverse();  // T_cw
            ids.push_back(idx); 
            ts.push_back(t); 
            idx++;
        }
        return {poses, ids, ts};
    }
    
    // Load depth images from ROS bag
    void loadDepthFromBag(const string& bag_file, const vector<int>& kf_indices,
                          const vector<double>& timestamps, const string& depth_topic) {
        ifstream test(bag_file);
        if (!test.good()) return;
        test.close();
        
        rosbag::Bag bag; 
        bag.open(bag_file, rosbag::bagmode::Read);
        rosbag::View view(bag, rosbag::TopicQuery(depth_topic));
        
        // Compute time offset
        double time_offset = 0.0; 
        int offset_samples = 0;
        for (const auto& m : view) {
            auto msg = m.instantiate<sensor_msgs::Image>();
            if (msg) { 
                time_offset += m.getTime().toSec() - msg->header.stamp.toSec(); 
                if (++offset_samples >= 10) break; 
            }
        }
        time_offset /= offset_samples;
        
        vector<pair<double, int>> ts_kf_pairs;
        for (size_t i = 0; i < timestamps.size(); ++i) 
            ts_kf_pairs.emplace_back(timestamps[i] + time_offset, kf_indices[i]);
        sort(ts_kf_pairs.begin(), ts_kf_pairs.end());
        
        set<int> matched_kf; 
        size_t search_idx = 0;
        for (const auto& m : view) {
            double msg_time = m.getTime().toSec();
            while (search_idx < ts_kf_pairs.size() && msg_time > ts_kf_pairs[search_idx].first + 0.05) 
                search_idx++;
            if (search_idx >= ts_kf_pairs.size()) break;
            
            double target_ts = ts_kf_pairs[search_idx].first;
            int kf_idx = ts_kf_pairs[search_idx].second;
            
            if (msg_time >= target_ts - 0.05 && msg_time <= target_ts + 0.05 && 
                matched_kf.find(kf_idx) == matched_kf.end()) {
                auto msg = m.instantiate<sensor_msgs::Image>(); 
                if (!msg) continue;
                
                cv::Mat img_float;
                if (msg->encoding == "16UC1" || msg->encoding == "mono16") {
                    cv::Mat raw(msg->height, msg->width, CV_16UC1, (void*)&msg->data[0]);
                    raw.convertTo(img_float, CV_32F, 0.001);
                } else if (msg->encoding == "32FC1") {
                    img_float = cv::Mat(msg->height, msg->width, CV_32FC1, (void*)&msg->data[0]).clone();
                }
                
                if (!img_float.empty()) { 
                    depth_images[kf_idx] = img_float; 
                    matched_kf.insert(kf_idx); 
                    search_idx++; 
                }
            }
        }
        bag.close();
    }
    
    // Backproject pixel to world coordinates
    Vector3d backprojectToWorld(double u, double v, double depth, const Matrix4d& T_wc) const {
        double x_cam = (u - cx) * depth / fx;
        double y_cam = (v - cy) * depth / fy;
        Vector3d pt_cam(x_cam, y_cam, depth);
        return T_wc.block<3,3>(0,0) * pt_cam + T_wc.block<3,1>(0,3);
    }
    
    // Get adaptive distance threshold based on depth
    double getAdaptiveThreshold(double depth) const {
        return base_distance_threshold + 0.01 * depth;
    }
    
    // Check if ray is occluded by mesh
    bool isRayOccluded(const Vector3d& cam_pos, const Vector3d& target_pt, int target_mesh_idx) const {
        if (!mesh_bvh) return false;
        Vector3d ray_dir = target_pt - cam_pos;
        double target_dist = ray_dir.norm();
        ray_dir.normalize();
        double hit_t;
        return mesh_bvh->raycast(cam_pos, ray_dir, target_dist - occlusion_margin, target_mesh_idx, hit_t);
    }
    
    // Search for best matching depth in neighborhood
    tuple<bool, double, double, double, double> searchBestMatch(
        const cv::Mat& depth_img, double u_proj, double v_proj,
        const Vector3d& mesh_pt, const Matrix4d& T_wc, double depth_proj, double threshold
    ) const {
        double best_u = u_proj, best_v = v_proj, best_depth = 0, best_dist = threshold;
        bool found = false;
        
        for (int dv = -search_radius; dv <= search_radius; ++dv) {
            for (int du = -search_radius; du <= search_radius; ++du) {
                int u = (int)u_proj + du, v = (int)v_proj + dv;
                if (u < 0 || u >= width || v < 0 || v >= height) continue;
                
                float dm = depth_img.at<float>(v, u);
                if (dm <= min_depth || dm >= max_depth) continue;
                if (fabs(dm - depth_proj) > depth_consistency_threshold * 2.0) continue;
                
                Vector3d pt_meas = backprojectToWorld(u, v, dm, T_wc);
                double dist = (pt_meas - mesh_pt).norm();
                
                if (dist < best_dist) { 
                    best_dist = dist; 
                    best_u = u; 
                    best_v = v; 
                    best_depth = dm; 
                    found = true; 
                }
            }
        }
        return {found, best_u, best_v, best_depth, best_dist};
    }
    
    // Verify single observation
    pair<bool, Observation> verifyObservation(
        const Vector3d& mesh_pt, const Vector3d* normal_ptr,
        int kf_idx, const Matrix4d& T_cw, int mesh_idx = -1
    ) {
        Observation obs; 
        obs.distance_3d = 1e10;
        
        stat_total_checked++;
        
        // Project to camera
        Vector4d pt_cam = T_cw * Vector4d(mesh_pt.x(), mesh_pt.y(), mesh_pt.z(), 1.0);
        double depth_proj = pt_cam(2);
        if (depth_proj <= min_depth || depth_proj > max_depth) return {false, obs};
        
        double u = fx * pt_cam(0) / depth_proj + cx;
        double v = fy * pt_cam(1) / depth_proj + cy;
        int margin = search_radius + 1;
        if (!(u >= margin && u < width - margin && v >= margin && v < height - margin)) 
            return {false, obs};
        
        auto it = depth_images.find(kf_idx);
        if (it == depth_images.end()) return {false, obs};
        
        const cv::Mat& depth_img = it->second;
        Matrix4d T_wc = T_cw.inverse();
        Vector3d cam_pos = T_wc.block<3,1>(0,3);
        double adaptive_threshold = getAdaptiveThreshold(depth_proj);
        
        // Bilinear interpolation
        int u0 = (int)u, v0 = (int)v;
        double du = u - u0, dv = v - v0;
        float d00 = depth_img.at<float>(v0, u0), d01 = depth_img.at<float>(v0, u0+1);
        float d10 = depth_img.at<float>(v0+1, u0), d11 = depth_img.at<float>(v0+1, u0+1);
        
        int valid_count = 0;
        if (d00 > min_depth && d00 < max_depth) valid_count++;
        if (d01 > min_depth && d01 < max_depth) valid_count++;
        if (d10 > min_depth && d10 < max_depth) valid_count++;
        if (d11 > min_depth && d11 < max_depth) valid_count++;
        
        double u_best, v_best, depth_meas, distance_3d;
        bool match_found = false;
        
        if (valid_count >= 2) {
            depth_meas = d00*(1-du)*(1-dv) + d01*du*(1-dv) + d10*(1-du)*dv + d11*du*dv;
            if (depth_meas > min_depth && depth_meas < max_depth) {
                Vector3d pt_meas = backprojectToWorld(u, v, depth_meas, T_wc);
                distance_3d = (pt_meas - mesh_pt).norm();
                if (distance_3d <= adaptive_threshold) { 
                    u_best = u; 
                    v_best = v; 
                    match_found = true; 
                }
            }
        }
        
        // Neighborhood search if direct match failed
        if (!match_found && search_radius > 0) {
            auto [found, ub, vb, db, dist] = searchBestMatch(
                depth_img, u, v, mesh_pt, T_wc, depth_proj, adaptive_threshold);
            if (found) { 
                u_best = ub; 
                v_best = vb; 
                depth_meas = db; 
                distance_3d = dist; 
                match_found = true; 
            }
        }
        
        if (!match_found) return {false, obs};
        if (distance_3d > adaptive_threshold) return {false, obs};
        
        // Depth consistency check
        double diff = depth_proj - depth_meas;
        if (fabs(diff) > depth_consistency_threshold) return {false, obs};
        
        // Grazing angle check
        double grazing = 1.0;
        if (normal_ptr && normal_ptr->norm() > 0.5) {
            Vector3d view = (mesh_pt - cam_pos).normalized();
            double cos_ang = fabs(view.dot(*normal_ptr));
            if (cos_ang < cos_min_angle) return {false, obs};
            grazing = cos_ang;
        }
        
        // Occlusion check
        if (mesh_idx >= 0 && mesh_bvh) {
            if (isRayOccluded(cam_pos, mesh_pt, mesh_idx)) return {false, obs};
        }
        
        // Compute weight
        double dist_weight = exp(-distance_3d / (base_distance_threshold * 0.6));
        double depth_weight = exp(-fabs(diff) / 0.08) / (1.0 + depth_proj / 8.0);
        double weight = dist_weight * depth_weight * grazing;
        if (weight < 0.05) return {false, obs};
        
        stat_passed++;
        
        obs.kf_idx = kf_idx; 
        obs.u = u_best; 
        obs.v = v_best;
        obs.depth_measured = depth_meas; 
        obs.depth_projected = depth_proj;
        obs.weight = weight; 
        obs.confidence = 1.0; 
        obs.distance_3d = distance_3d;
        return {true, obs};
    }
    
    // Filter observations by consecutive frames
    vector<Observation> filterObservations(vector<Observation>& obs_list) {
        if ((int)obs_list.size() < min_consecutive_observations) return {};
        sort(obs_list.begin(), obs_list.end(), 
             [](const Observation& a, const Observation& b) { return a.kf_idx < b.kf_idx; });
        
        vector<vector<Observation>> seqs;
        vector<Observation> seq = {obs_list[0]};
        
        for (size_t i = 1; i < obs_list.size(); ++i) {
            if (obs_list[i].kf_idx - obs_list[i-1].kf_idx <= max_frame_gap) 
                seq.push_back(obs_list[i]);
            else { 
                if ((int)seq.size() >= min_consecutive_observations) 
                    seqs.push_back(seq); 
                seq = {obs_list[i]}; 
            }
        }
        if ((int)seq.size() >= min_consecutive_observations) 
            seqs.push_back(seq);
        
        vector<Observation> result;
        for (auto& s : seqs) 
            for (auto& o : s) 
                result.push_back(o);
        
        if ((int)result.size() > max_observations_per_point) {
            sort(result.begin(), result.end(), 
                 [](const Observation& a, const Observation& b) { return a.weight > b.weight; });
            result.resize(max_observations_per_point);
            sort(result.begin(), result.end(), 
                 [](const Observation& a, const Observation& b) { return a.kf_idx < b.kf_idx; });
        }
        return result;
    }
    
    // Main correspondence finding function
    map<int, pair<Vector3d, vector<Observation>>> findCorrespondences(
        const map<int, Matrix4d>& poses,
        const vector<Vector3d>& vertices,
        const vector<Vector3d>& normals
    ) {
        // Step 1: Find visible vertices for each keyframe
        cout << "\n    Step 1: Finding visible vertices per keyframe..." << endl;
        map<int, vector<int>> kf_to_vertices;
        long long total_pairs = 0;
        
        for (const auto& [kf_idx, T_cw] : poses) {
            vector<int> visible;
            for (int mi = 0; mi < (int)vertices.size(); ++mi) {
                Vector4d pc = T_cw * Vector4d(vertices[mi].x(), vertices[mi].y(), vertices[mi].z(), 1.0);
                double z = pc.z();
                if (z > min_depth && z < max_depth) {
                    double u = fx * pc.x() / z + cx;
                    double v = fy * pc.y() / z + cy;
                    if (u >= 0 && u < width && v >= 0 && v < height) {
                        visible.push_back(mi);
                        total_pairs++;
                    }
                }
            }
            kf_to_vertices[kf_idx] = visible;
        }
        cout << "      KFs: " << kf_to_vertices.size() << ", Candidate pairs: " << total_pairs << endl;
        
        // Step 2: Verify observations in parallel
        cout << "    Step 2: Verifying observations..." << endl;
        stat_total_checked = 0;
        stat_passed = 0;
        
        int n_kfs = kf_to_vertices.size();
        vector<pair<int, vector<int>>> kf_list(kf_to_vertices.begin(), kf_to_vertices.end());
        int n_threads = omp_get_max_threads();
        vector<map<int, vector<Observation>>> thread_results(n_threads);
        
        atomic<int> progress{0};
        #pragma omp parallel
        {
            int tid = omp_get_thread_num();
            auto& local = thread_results[tid];
            
            #pragma omp for schedule(dynamic, 10)
            for (int i = 0; i < n_kfs; ++i) {
                int kf_idx = kf_list[i].first;
                const auto& mesh_indices = kf_list[i].second;
                auto T_it = poses.find(kf_idx);
                if (T_it == poses.end()) continue;
                const Matrix4d& T_cw = T_it->second;
                
                for (int mi : mesh_indices) {
                    const Vector3d* np = (mi < (int)normals.size()) ? &normals[mi] : nullptr;
                    auto [ok, obs] = verifyObservation(vertices[mi], np, kf_idx, T_cw, mi);
                    if (ok) local[mi].push_back(obs);
                }
                
                if (++progress % 50 == 0) {
                    #pragma omp critical
                    cout << "      Progress: " << progress << "/" << n_kfs << " KFs\r" << flush;
                }
            }
        }
        cout << endl;
        
        // Merge thread results
        map<int, vector<Observation>> point_observations;
        for (int t = 0; t < n_threads; ++t) {
            for (auto& kv : thread_results[t]) {
                auto& dst = point_observations[kv.first];
                dst.insert(dst.end(), kv.second.begin(), kv.second.end());
            }
        }
        
        cout << "      Checked: " << stat_total_checked << ", Passed: " << stat_passed << endl;
        cout << "      Points with observations: " << point_observations.size() << endl;
        
        // Step 3: Filter and build final result
        cout << "    Step 3: Filtering observations..." << endl;
        map<int, pair<Vector3d, vector<Observation>>> result;
        
        int before_filter = point_observations.size();
        for (auto& kv : point_observations) {
            int mesh_idx = kv.first;
            vector<Observation> filtered = filterObservations(kv.second);
            if ((int)filtered.size() >= min_consecutive_observations) {
                result[mesh_idx] = {vertices[mesh_idx], filtered};
            }
        }
        cout << "      Before filter: " << before_filter << ", After filter: " << result.size() << endl;
        
        return result;
    }
    
    // Save correspondences to file
    void saveCorrespondences(
        const map<int, pair<Vector3d, vector<Observation>>>& data,
        const string& output_file
    ) {
        ofstream f(output_file);
        f << "# Ray Casting Correspondences\n";
        f << "# point_id x y z num_observations is_anchor\n";
        f << "#   kf_idx u v depth_measured depth_projected weight confidence pose_change distance_3d\n";
        f << "# Total points: " << data.size() << "\n#\n";
        
        for (const auto& kv : data) {
            int pid = kv.first;
            const Vector3d& pt = kv.second.first;
            const auto& obs = kv.second.second;
            
            // is_anchor = 0 for all points (let triangulator decide)
            f << pid << " " << fixed << setprecision(6) 
              << pt.x() << " " << pt.y() << " " << pt.z() 
              << " " << obs.size() << " 0\n";
            
            for (const auto& o : obs) {
                f << "  " << o.kf_idx << " " << setprecision(2) 
                  << o.u << " " << o.v << " "
                  << setprecision(4) << o.depth_measured << " " << o.depth_projected << " "
                  << o.weight << " " << o.confidence << " 0.0 " << o.distance_3d << "\n";
            }
        }
        f.close();
    }
};

// ============================================================
// Main
// ============================================================
int main() {
    auto start = steady_clock::now();
    
    cout << string(60, '=') << endl;
    cout << "Ray Casting for Mesh-Depth Correspondences" << endl;
    cout << string(60, '=') << endl;
    
    // Camera parameters
    map<string, double> cam = {
        {"fx", 377.535257164}, {"fy", 377.209841379},
        {"cx", 328.193371286}, {"cy", 240.426878936},
        {"width", 640}, {"height", 480}
    };
    
    // Processing parameters
    map<string, double> params = {
        {"min_depth", 0.1}, 
        {"max_depth", 10.0},
        {"base_distance_threshold", 0.08},
        {"depth_consistency_threshold", 0.1},
        {"max_grazing_angle", 75.0},
        {"min_consecutive_observations", 1},
        {"max_frame_gap", 2},
        {"max_observations_per_point", 15},
        {"occlusion_margin", 0.02},
        {"search_radius", 0}
    };
    
    // Paths
    string base = "/Datasets/Work_Data/ACL_OneLoop20251220_214841/loop_1_1670449626.805310";
    string bag = "/Datasets/Kimera/Kimera_Clipped_bag/12_07_acl_jackal2_clipped.bag";
    string topic = "/acl_jackal2/forward/depth/image_rect_raw";
    string mesh_path = base + "/pre/mesh.ply";
    string traj_path = base + "/pre/standard_trajectory_no_loop.txt";
    string output = "/Datasets/Work_Data/ACL_OneLoop20251220_214841/Output/optimization_data.txt";
    
    (void)system(("mkdir -p " + output.substr(0, output.find_last_of('/'))).c_str());
    
    RayCaster caster(cam, params);
    
    // Load trajectory
    cout << "\n[1] Loading trajectory..." << endl;
    auto t0 = steady_clock::now();
    auto [poses, kf_indices, timestamps] = caster.loadTrajectory(traj_path);
    auto t1 = steady_clock::now();
    cout << "    Poses: " << poses.size() << "  [" 
         << duration_cast<milliseconds>(t1-t0).count()/1000.0 << "s]" << endl;
    
    // Load depth images
    cout << "\n[2] Loading depth images..." << endl;
    t0 = steady_clock::now();
    caster.loadDepthFromBag(bag, kf_indices, timestamps, topic);
    t1 = steady_clock::now();
    cout << "    Depth images: " << caster.depth_images.size() << "  [" 
         << duration_cast<milliseconds>(t1-t0).count()/1000.0 << "s]" << endl;
    
    // Load mesh
    cout << "\n[3] Loading mesh..." << endl;
    vector<Vector3d> vertices, normals; 
    vector<Vector3i> faces;
    t0 = steady_clock::now();
    loadMesh(mesh_path, vertices, normals, faces);
    t1 = steady_clock::now();
    cout << "    Vertices: " << vertices.size() << ", Faces: " << faces.size() 
         << "  [" << duration_cast<milliseconds>(t1-t0).count()/1000.0 << "s]" << endl;
    
    // Build BVH
    cout << "\n[4] Building BVH..." << endl;
    t0 = steady_clock::now();
    BVH bvh; 
    bvh.build(vertices, faces);
    caster.setBVH(&bvh);
    t1 = steady_clock::now();
    cout << "    Nodes: " << bvh.nodes.size() << "  [" 
         << duration_cast<milliseconds>(t1-t0).count()/1000.0 << "s]" << endl;
    
    // Find correspondences
    cout << "\n[5] Finding correspondences..." << endl;
    t0 = steady_clock::now();
    auto correspondences = caster.findCorrespondences(poses, vertices, normals);
    t1 = steady_clock::now();
    cout << "    Final correspondences: " << correspondences.size() << "  [" 
         << duration_cast<milliseconds>(t1-t0).count()/1000.0 << "s]" << endl;
    
    // Save results
    cout << "\n[6] Saving results..." << endl;
    if (!correspondences.empty()) {
        caster.saveCorrespondences(correspondences, output);
        cout << "    Output: " << output << endl;
    }
    
    // Summary
    auto total = duration_cast<milliseconds>(steady_clock::now() - start).count() / 1000.0;
    cout << "\n" << string(60, '=') << endl;
    cout << "Total time: " << total << "s" << endl;
    cout << string(60, '=') << endl;
    
    return 0;
}