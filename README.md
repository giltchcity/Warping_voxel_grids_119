# Warping_voxel_grids_119



# Mesh Warping Pipeline for SLAM Loop Closure Correction

A complete pipeline for correcting 3D mesh maps after SLAM loop closure detection. This system warps pre-loop-closure meshes to align with post-loop-closure geometry using depth image observations and As-Rigid-As-Possible (ARAP) deformation.

---

## Table of Contents

1. [Overview](#overview)
2. [Pipeline Architecture](#pipeline-architecture)
3. [Dependencies](#dependencies)
4. [Directory Structure](#directory-structure)
5. [Execution Order](#execution-order)
6. [Detailed Component Documentation](#detailed-component-documentation)
   - [Step 1: Ray Caster](#step-1-ray-caster-c)
   - [Step 2: Lindstrom Triangulator](#step-2-lindstrom-triangulator-c)
   - [Step 3: Mesh Warping (ARAP)](#step-3-mesh-warping-arap-python)
   - [Step 4: Mesh to TSDF Conversion](#step-4-mesh-to-tsdf-conversion-python)
   - [Step 5: Evaluation](#step-5-evaluation-python)
7. [Performance Metrics](#performance-metrics)
8. [Output Files](#output-files)

---

## Overview

When a SLAM system detects a loop closure, the camera trajectory is corrected, but the 3D mesh map built before the loop closure remains misaligned. This pipeline:

1. **Finds correspondences** between mesh vertices and depth images using ray casting
2. **Triangulates target positions** for mesh vertices based on corrected camera poses
3. **Deforms the mesh** using ARAP to smoothly warp vertices to their target positions
4. **Converts to TSDF** for downstream applications
5. **Evaluates** the result against ground truth

---

## Pipeline Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           INPUT DATA                                         │
├─────────────────────────────────────────────────────────────────────────────┤
│  • Pre-loop mesh (mesh.ply)                                                  │
│  • Pre-loop trajectory (standard_trajectory_no_loop.txt)                     │
│  • Post-loop trajectory (standard_trajectory_with_loop.txt)                  │
│  • Depth images (ROS bag)                                                    │
│  • Ground truth mesh (mesh_post_pointcloud.ply)                              │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  STEP 1: Ray Caster (C++)                                                    │
│  ─────────────────────────────────────────────────────────────────────────── │
│  • Project mesh vertices to depth images                                     │
│  • Verify correspondences with multi-stage filtering                         │
│  • Output: optimization_data.txt (12,888 points with observations)           │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  STEP 2: Lindstrom Triangulator (C++)                                        │
│  ─────────────────────────────────────────────────────────────────────────── │
│  • Triangulate 3D target positions from depth observations                   │
│  • Use POST-loop poses for correct world coordinates                         │
│  • Output: optimized_points_final.txt (source → target displacements)        │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  STEP 3: Mesh Warping - ARAP (Python)                                        │
│  ─────────────────────────────────────────────────────────────────────────── │
│  • Match control points to mesh vertices                                     │
│  • Apply As-Rigid-As-Possible deformation                                    │
│  • Output: mesh_deformed_arap.ply                                            │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  STEP 4: Mesh to TSDF (Python)                                               │
│  ─────────────────────────────────────────────────────────────────────────── │
│  • Poisson reconstruction (D8 + D9)                                          │
│  • Compute signed distance field                                             │
│  • Prune outliers                                                            │
│  • Output: warped_map.tsdf, warped_tsdf_mesh.ply                   │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  STEP 5: Evaluation (Python)                                                 │
│  ─────────────────────────────────────────────────────────────────────────── │
│  • Compare Pre / Post / Warped / TSDF vs Ground Truth                        │
│  • Compute Precision, Recall, F-score at 10cm, 25cm, 50cm thresholds         │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Dependencies

### C++ Dependencies
- **Eigen3** - Linear algebra
- **OpenCV** - Image processing
- **ROS** (rosbag, sensor_msgs) - Depth image loading
- **OpenMP** - Parallel processing
- **happly** - PLY file I/O (header-only)

### Python Dependencies
- **numpy** - Numerical computing
- **open3d** - 3D geometry processing
- **libigl** (igl) - ARAP deformation
- **scipy** - KDTree for spatial queries
- **scikit-image** - Marching cubes
- **protobuf** - TSDF serialization

---



---

## Detailed Component Documentation

---

### Step 1: Ray Caster (C++)

**File:** `/Datasets/Voxmap/Ray_Casting/main.cpp`

**Purpose:** Find correspondences between mesh vertices and depth image measurements.

#### Camera Parameters

```cpp
fx = 377.535257164    // Focal length X (pixels)
fy = 377.209841379    // Focal length Y (pixels)
cx = 328.193371286    // Principal point X (pixels)
cy = 240.426878936    // Principal point Y (pixels)
width = 640           // Image width
height = 480          // Image height
```

#### Processing Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `min_depth` | 0.1 m | Minimum valid depth |
| `max_depth` | 10.0 m | Maximum valid depth |
| `base_distance_threshold` | 0.08 m | Base 3D distance threshold for matching |
| `depth_consistency_threshold` | 0.1 m | Max allowed depth difference |
| `max_grazing_angle` | 75° | Maximum surface grazing angle |
| `min_consecutive_observations` | 1 | Minimum observations per point |
| `max_frame_gap` | 2 | Maximum frame gap in sequences |
| `max_observations_per_point` | 15 | Cap on observations per point |
| `occlusion_margin` | 0.02 m | Margin for occlusion testing |
| `search_radius` | 0 | Neighborhood search radius (pixels) |

#### Algorithm Flow

```
1. LOAD TRAJECTORY (TUM Format)
   ┌─────────────────────────────────────────────────────────────────┐
   │ Input: standard_trajectory_no_loop.txt                          │
   │ Format: timestamp tx ty tz qx qy qz qw                          │
   │                                                                  │
   │ For each line:                                                   │
   │   • Parse timestamp and 7-DOF pose (position + quaternion)      │
   │   • Convert quaternion to rotation matrix                        │
   │   • Build T_wc (world-to-camera transform)                       │
   │   • Store T_cw = T_wc.inverse() (camera-to-world)               │
   │                                                                  │
   │ Output: 789 poses indexed by keyframe ID                         │
   └─────────────────────────────────────────────────────────────────┘

2. LOAD DEPTH IMAGES FROM ROS BAG
   ┌─────────────────────────────────────────────────────────────────┐
   │ Input: 12_07_acl_jackal2_clipped.bag                            │
   │ Topic: /acl_jackal2/forward/depth/image_rect_raw                │
   │                                                                  │
   │ Process:                                                         │
   │   a) Compute time offset between bag time and header stamp      │
   │      (average over first 10 messages)                           │
   │   b) Sort keyframe timestamps                                    │
   │   c) For each depth message:                                     │
   │      - Find matching keyframe within ±50ms                       │
   │      - Convert 16UC1 or 32FC1 to float32 (meters)               │
   │      - Store in depth_images map                                 │
   │                                                                  │
   │ Output: 789 depth images (640×480, float32)                      │
   └─────────────────────────────────────────────────────────────────┘

3. LOAD MESH
   ┌─────────────────────────────────────────────────────────────────┐
   │ Input: pre/mesh.ply                                              │
   │                                                                  │
   │ Using happly library:                                            │
   │   • Load vertex positions (x, y, z) as Vector3d                 │
   │   • Load face indices (triangles)                                │
   │                                                                  │
   │ Compute vertex normals:                                          │
   │   For each face:                                                 │
   │     fn = (v1 - v0) × (v2 - v0)    // Face normal               │
   │     Accumulate fn to each vertex's normal                        │
   │   Normalize all vertex normals                                   │
   │                                                                  │
   │ Output: 72,611 vertices, 123,018 faces                           │
   └─────────────────────────────────────────────────────────────────┘

4. BUILD BVH (Bounding Volume Hierarchy)
   ┌─────────────────────────────────────────────────────────────────┐
   │ Purpose: Accelerate ray-triangle intersection for occlusion     │
   │                                                                  │
   │ Construction (recursive):                                        │
   │   a) Compute AABB for all triangles                             │
   │   b) Sort triangles by centroid along longest axis              │
   │   c) Split at median                                             │
   │   d) Recurse until leaf size ≤ 4 triangles                      │
   │                                                                  │
   │ Ray-AABB intersection: Slab method                               │
   │ Ray-Triangle intersection: Möller-Trumbore algorithm            │
   │                                                                  │
   │ Output: 65,535 BVH nodes                                         │
   └─────────────────────────────────────────────────────────────────┘

5. FIND CORRESPONDENCES
   ┌─────────────────────────────────────────────────────────────────┐
   │ Step 5.1: Find visible vertices per keyframe                    │
   │ ─────────────────────────────────────────────────────────────── │
   │ For each keyframe kf_idx with pose T_cw:                        │
   │   For each mesh vertex mi:                                       │
   │     • Project to camera: P_cam = T_cw × [x, y, z, 1]ᵀ           │
   │     • Check depth: min_depth < z < max_depth                    │
   │     • Project to image: u = fx × x/z + cx, v = fy × y/z + cy   │
   │     • Check bounds: 0 ≤ u < 640, 0 ≤ v < 480                   │
   │     • If valid, add to visible list                              │
   │                                                                  │
   │ Result: 1,994,154 candidate (keyframe, vertex) pairs             │
   └─────────────────────────────────────────────────────────────────┘
   
   ┌─────────────────────────────────────────────────────────────────┐
   │ Step 5.2: Verify observations (parallel with OpenMP)            │
   │ ─────────────────────────────────────────────────────────────── │
   │ For each candidate pair (kf_idx, mesh_idx):                      │
   │                                                                  │
   │   CHECK 1: Projection bounds                                     │
   │     • Re-project vertex to image                                 │
   │     • Verify pixel coordinates with margin                       │
   │                                                                  │
   │   CHECK 2: Depth image availability                              │
   │     • Lookup depth image for this keyframe                       │
   │                                                                  │
   │   CHECK 3: Bilinear depth interpolation                          │
   │     • Sample 4 neighboring pixels: d00, d01, d10, d11           │
   │     • Interpolate: depth_meas = bilinear(d00, d01, d10, d11)    │
   │     • Require ≥ 2 valid depth samples                           │
   │                                                                  │
   │                                                                  │
   │   CHECK 5: Depth consistency                                     │
   │     • diff = |depth_projected - depth_measured|                 │
   │     • Reject if diff > 0.1m                                      │
   │                                                                  │
   │   CHECK 6: Grazing angle                                         │
   │     • view_dir = normalize(mesh_pt - cam_pos)                   │
   │     • cos_angle = |view_dir · vertex_normal|                    │
   │     • Reject if cos_angle < cos(75°) ≈ 0.259                    │
   │                                                                  │
   │   CHECK 7: Occlusion (BVH raycast)                               │
   │     • Cast ray from camera to vertex                             │
   │     • If ray hits another triangle first → occluded             │
   │     • Use occlusion_margin = 2cm                                 │
   │                                                                  │
   │   COMPUTE WEIGHT:                                                │
   │     dist_weight = exp(-distance_3d / 0.048)                      │
   │     depth_weight = exp(-|diff| / 0.08) / (1 + depth/8)          │
   │     weight = dist_weight × depth_weight × cos_angle             │
   │     Reject if weight < 0.05                                      │
   │                                                                  │
   │ Result: 31,930 observations passed                               │
   │         12,888 unique mesh vertices with observations            │
   └─────────────────────────────────────────────────────────────────┘
   
   ┌─────────────────────────────────────────────────────────────────┐
   │ Step 5.3: Filter observations                                    │
   │ ─────────────────────────────────────────────────────────────── │
   │ For each mesh vertex:                                            │
   │   a) Sort observations by keyframe index                         │
   │   b) Find consecutive sequences (gap ≤ 2 frames)                │
   │   c) Keep sequences with ≥ 1 observation                        │
   │   d) If > 15 observations, keep top 15 by weight                │
   │                                                                  │
   │ Result: 12,888 points remain after filtering                     │
   └─────────────────────────────────────────────────────────────────┘

6. SAVE CORRESPONDENCES
   ┌─────────────────────────────────────────────────────────────────┐
   │ Output: optimization_data.txt                                    │
   │                                                                  │
   │ Format:                                                          │
   │   point_id x y z num_observations is_anchor                      │
   │     kf_idx u v depth_meas depth_proj weight conf pose_chg dist  │
   │                                                                  │
   │ Example:                                                         │
   │   0 -5.123456 2.345678 1.234567 3 0                             │
   │     45 320.50 240.25 3.1234 3.1456 0.8765 1.0 0.0 0.0234        │
   │     46 321.20 239.80 3.1345 3.1423 0.8543 1.0 0.0 0.0198        │
   │     47 322.10 239.50 3.1401 3.1398 0.8234 1.0 0.0 0.0156        │
   └─────────────────────────────────────────────────────────────────┘
```

#### Output Statistics

```
Checked: 1,994,154 candidate pairs
Passed:  31,930 observations
Points:  12,888 unique mesh vertices
```

---

### Step 2: Lindstrom Triangulator (C++)

**File:** `/Datasets/CERES_Work/main.cpp`

**Purpose:** Compute target 3D positions for mesh vertices using triangulation from multiple depth observations with **corrected (post-loop) camera poses**.

#### Camera Parameters

```cpp
// Same as Ray Caster
FX = 377.535257164
FY = 377.209841379
CX = 328.193371286
CY = 240.426878936
```

#### Configurable Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `displacement_scale` | 1.0 | Scale factor for displacement magnitude |
| `displacement_offset` | 0 mm | Offset added to displacement |
| `anchor_mode` | 0 | 0=all moving, 1=use input, 2=by threshold |
| `anchor_threshold` | 30 mm | Threshold for mode 2 |

#### Algorithm Flow

```
1. LOAD POSES (Post-Loop Trajectory)
   ┌─────────────────────────────────────────────────────────────────┐
   │ Input: standard_trajectory_with_loop.txt                         │
   │                                                                  │
   │ CRITICAL: Uses POST-loop trajectory, not PRE-loop!              │
   │           This is the corrected trajectory after loop closure.   │
   │                                                                  │
   │ For each line:                                                   │
   │   • Parse TUM format: timestamp tx ty tz qx qy qz qw            │
   │   • Build T_wc from quaternion + translation                     │
   │   • Store both T_wc and T_cw = T_wc.inverse()                   │
   │   • Store camera center = T_wc.block<3,1>(0,3)                  │
   │                                                                  │
   │ Output: 789 corrected poses                                      │
   └─────────────────────────────────────────────────────────────────┘

2. LOAD POINTS WITH OBSERVATIONS
   ┌─────────────────────────────────────────────────────────────────┐
   │ Input: optimization_data.txt (from Step 1)                       │
   │                                                                  │
   │ Parse format:                                                    │
   │   Header line: point_id x y z num_obs is_anchor                  │
   │   Observation lines (indented):                                  │
   │     kf_idx u v depth_meas depth_proj weight conf pose_chg dist  │
   │                                                                  │
   │ For each point:                                                  │
   │   • Store original position (from PRE-loop mesh)                │
   │   • Store all observations                                       │
   │   • Track anchor flag from input                                 │
   │                                                                  │
   │ Output: 12,888 points new position                               │
   └─────────────────────────────────────────────────────────────────┘

3. TRIANGULATE TARGET POSITIONS
   ┌─────────────────────────────────────────────────────────────────┐
   │ For each point with observations:                                │
   │                                                                  │
   │   Step 3.1: Backproject each observation to world               │
   │   ─────────────────────────────────────────────────────────────  │
   │   For each observation (kf_idx, u, v, depth_measured):           │
   │     • Get CORRECTED pose T_wc for keyframe kf_idx               │
   │     • Backproject to camera coordinates:                         │
   │       x_cam = (u - CX) × depth_measured / FX                     │
   │       y_cam = (v - CY) × depth_measured / FY                     │
   │       P_cam = [x_cam, y_cam, depth_measured]ᵀ                   │
   │     • Transform to world:                                        │
   │       P_world = R_wc × P_cam + t_wc                              │
   │                                                                  │
   │   Step 3.2: Compute observation weights                          │
   │   ─────────────────────────────────────────────────────────────  │
   │   w = base_weight × confidence                                   │
   │   w *= 1 / (1 + depth / 3)        // Depth penalty              │
   │   w *= 1 + min(pose_change × 15, 3)  // Pose change bonus       │
   │                                                                  │
   │   Step 3.3: Weighted average (initial estimate)                  │
   │   ─────────────────────────────────────────────────────────────  │
   │   mean = Σ(w_i × P_world_i) / Σ(w_i)                            │
   │                                                                  │
   │   Step 3.4: Outlier rejection (if ≥ 3 observations)             │
   │   ─────────────────────────────────────────────────────────────  │
   │   • Compute distance of each point to mean                       │
   │   • Find median distance                                         │
   │   • Threshold = max(0.02m, median × 2.5)                        │
   │   • Recompute weighted average excluding outliers                │
   │                                                                  │
   │   Step 3.5: Apply scale and offset                               │
   │   ─────────────────────────────────────────────────────────────  │
   │   displacement = triangulated - original                         │
   │   new_disp = ||displacement|| × scale + offset                   │
   │   target = original + normalize(displacement) × new_disp         │
   │                                                                  │
   │ Result: Target position for each control point                   │
   └─────────────────────────────────────────────────────────────────┘



4. EVALUATE TRIANGULATION QUALITY
   ┌─────────────────────────────────────────────────────────────────┐
   │ For each point and observation:                                  │
   │   • Project TARGET position back to camera using T_cw           │
   │   • Compute reprojection error: ||[u,v]_proj - [u,v]_obs||     │
   │   • Compute depth error: |depth_proj - depth_measured|          │
   │                                                                  │
   │ Results:                                                         │
   │   Reprojection: avg=1.84px, median=0.07px                        │
   │   Depth error:  avg=25.1mm, median=10.7mm                        │
   └─────────────────────────────────────────────────────────────────┘

5. SAVE RESULTS
   ┌─────────────────────────────────────────────────────────────────┐
   │ Output: optimized_points_final.txt                               │
   │                                                                  │
   │ Header comments with parameters                                  │
   │ Format per line:                                                 │
   │   id x_orig y_orig z_orig x_target y_target z_target move_mm anchor │
   │                                                                  │
   │ Example:                                                         │
   │   0 -5.123456 2.345678 1.234567 -5.234567 2.456789 1.345678 150.5 0 │
   └─────────────────────────────────────────────────────────────────┘
```

#### Output Statistics

```
Displacement Statistics:
  Mean:   593.8 mm
  Median: 209.7 mm
  Max:    2467.9 mm

Triangulation Quality:
  Reprojection error: avg=1.84px, median=0.07px
  Depth error:        avg=25.1mm, median=10.7mm
```

---

### Step 3: Mesh Warping - ARAP (Python)

**File:** `/Datasets/Voxmap/mesh_warping.py`

**Purpose:** Deform the pre-loop mesh using As-Rigid-As-Possible (ARAP) deformation to align with target positions.

#### Algorithm Flow

```
1. LOAD MESH
   ┌─────────────────────────────────────────────────────────────────┐
   │ Input: pre/mesh.ply                                              │
   │                                                                  │
   │ • Load with Open3D                                               │
   │ • Original: 72,611 vertices, 123,018 faces                       │
   │ • Check watertight: False                                        │
   └─────────────────────────────────────────────────────────────────┘

2. CLEAN MESH (Critical for ARAP!)
   ┌─────────────────────────────────────────────────────────────────┐
   │ Step 2.1: Remove floating triangles                              │
   │   • Cluster connected triangles                                  │
   │   • Keep only largest connected component                        │
   │   • Removed: 8,564 floating triangles                            │
   │                                                                  │
   │ Step 2.2: Fix non-manifold structures (iterate up to 5×)        │
   │   • remove_duplicated_vertices()                                 │
   │   • remove_duplicated_triangles()                                │
   │   • remove_degenerate_triangles()                                │
   │   • remove_non_manifold_edges()                                  │
   │                                                                  │
   │ Step 2.3: Remove unreferenced vertices                           │
   │                                                                  │
   │ Result: 72,611 → 63,343 vertices                                 │
   └─────────────────────────────────────────────────────────────────┘

3. LOAD CONTROL POINTS
   ┌─────────────────────────────────────────────────────────────────┐
   │ Input: optimized_points_final.txt (from Step 2)                  │
   │                                                                  │
   │ Parse each line:                                                 │
   │   id x_orig y_orig z_orig x_target y_target z_target move_mm anchor │
   │                                                                  │
   │ Compute:                                                         │
   │   source = [x_orig, y_orig, z_orig]                              │
   │   target = [x_target, y_target, z_target]                        │
   │   displacement = target - source                                 │
   │                                                                  │
   │ Filter outliers:                                                 │
   │   max_allowed = max(median_disp × 5, 3.0m)                      │
   │   Remove points with ||displacement|| > max_allowed             │
   │                                                                  │
   │ Result: 12,888 control points                                    │
   │   Mean displacement: 594mm                                       │
   │   Max displacement:  2,468mm                                     │
   └─────────────────────────────────────────────────────────────────┘

4. MATCH CONTROL POINTS TO MESH VERTICES
   ┌─────────────────────────────────────────────────────────────────┐
   │ Build KDTree from cleaned mesh vertices                          │
   │                                                                  │
   │ For each control point:                                          │
   │   • Find nearest mesh vertex                                     │
   │   • Record distance and vertex index                             │
   │                                                                  │
   │ Filter by max distance (15cm):                                   │
   │   Valid matches: 12,788 / 12,888                                 │
   │                                                                  │
   │ Deduplicate (multiple controls → same vertex):                   │
   │   • Average displacements for duplicate matches                  │
   │   Result: 12,731 unique control vertices                         │
   │                                                                  │
   │ Match statistics:                                                │
   │   Mean distance: 4.6mm                                           │
   │   Max distance:  1,258.4mm                                       │
   └─────────────────────────────────────────────────────────────────┘

5. ARAP DEFORMATION (libigl)
   ┌─────────────────────────────────────────────────────────────────┐
   │ ARAP: As-Rigid-As-Possible Surface Modeling                      │
   │                                                                  │
   │ Input:                                                           │
   │   V = vertices (63,343 × 3, float64)                            │
   │   F = faces (114,444 × 3, int64)                                │
   │   b = control indices (12,731 × 1, int32)                       │
   │   bc = control targets (12,731 × 3, float64)                    │
   │                                                                  │
   │ Step 5.1: Precomputation                                         │
   │   igl.arap_precomputation(V, F, dim=3, b, arap_data)            │
   │   • Builds cotangent Laplacian                                   │
   │   • Computes local rotation estimation weights                   │
   │                                                                  │
   │ Step 5.2: Iterative solving                                      │
   │   V_new = igl.arap_solve(bc, arap_data, V)                       │
   │   • Local step: Estimate optimal rotation per vertex             │
   │   • Global step: Solve linear system for positions               │
   │   • Iterate until convergence                                    │
   │                                                                  │
   │ Output: V_new (deformed vertex positions)                        │
   │ Compute: displacements = V_new - V                               │
   │                                                                  │
   │ Time: 0.89 seconds                                               │
   └─────────────────────────────────────────────────────────────────┘

6. STATISTICS
   ┌─────────────────────────────────────────────────────────────────┐
   │ Displacement statistics (all mesh vertices):                     │
   │   Mean:   618.4 mm                                               │
   │   Median: 240.9 mm                                               │
   │   Max:    2,467.9 mm                                             │
   │                                                                  │
   │ Control point error (should be ~0 for ARAP):                     │
   │   Mean: 0.00 mm                                                  │
   │   Max:  0.00 mm                                                  │
   └─────────────────────────────────────────────────────────────────┘

7. SAVE DEFORMED MESH
   ┌─────────────────────────────────────────────────────────────────┐
   │ Output: mesh_deformed_arap.ply                                   │
   │                                                                  │
   │ • Create new mesh with deformed vertices                         │
   │ • Keep original triangle connectivity                            │
   │ • Recompute vertex normals                                       │
   │ • Save as PLY                                                    │
   └─────────────────────────────────────────────────────────────────┘

8. F-SCORE EVALUATION (Optional)
   ┌─────────────────────────────────────────────────────────────────┐
   │ Sample 200,000 points from each mesh                             │
   │                                                                  │
   │ Compute F-score at thresholds: 10cm, 25cm, 50cm                  │
   │   Precision = % of predicted points within threshold of GT      │
   │   Recall = % of GT points within threshold of prediction        │
   │   F-score = 2 × P × R / (P + R)                                 │
   │                                                                  │
   │ Results:                                                         │
   │   Mesh          10cm     25cm     50cm                           │
   │   Pre           36.39%   63.36%   80.94%                         │
   │   ARAP          60.31%   84.02%   93.41%                         │
   │   Post          61.06%   83.03%   91.95%                         │
   │                                                                  │
   │ Improvement vs Pre:                                              │
   │   10cm: +23.92%                                                  │
   │   25cm: +20.66%                                                  │
   │   50cm: +12.47%                                                  │
   └─────────────────────────────────────────────────────────────────┘
```

---

### Step 4: Mesh to TSDF Conversion (Python)

**File:** `/Datasets/Voxmap/mesh_to_TSDF.py`

**Purpose:** Convert the warped mesh to a Truncated Signed Distance Field (TSDF) for downstream applications.

#### Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `TRUNCATION` | 0.50 m | TSDF truncation distance |
| `REFINE_THRESHOLD` | 0.08 m | Surface refinement band |
| `PRUNE_THRESHOLD` | 0.25 m | Outlier distance threshold |
| `PRUNE_RADIUS` | 0.30 m | Outlier voxel removal radius |
| `voxel_size` | 0.25 m | TSDF voxel size |
| `vps` | 16 | Voxels per side (per block) |

#### Algorithm Flow

```
1. LOAD DATA
   ┌─────────────────────────────────────────────────────────────────┐
   │ Load TSDF structure (block origins only):                        │
   │   Input: post/map.tsdf                                           │
   │   Parse protobuf format                                          │
   │   Extract: block origins, voxel_size=0.25m, vps=16               │
   │   Result: 298 blocks                                             │
   │                                                                  │
   │ Load warped mesh:                                                │
   │   Input: mesh_deformed_arap.ply                                  │
   │                                                                  │
   │ Load ground truth mesh:                                          │
   │   Input: mesh_post_pointcloud.ply                                │
   └─────────────────────────────────────────────────────────────────┘

2. POISSON RECONSTRUCTION (Dual Resolution)
   ┌─────────────────────────────────────────────────────────────────┐
   │ Purpose: Create clean, watertight surfaces from noisy input      │
   │                                                                  │
   │ Process:                                                         │
   │   a) Sample 500,000 points from mesh                             │
   │   b) Estimate normals                                            │
   │   c) Run Poisson reconstruction                                  │
   │   d) Trim low-density regions (5th percentile)                  │
   │                                                                  │
   │ D8 (Depth 8): Smooth geometry for coverage                       │
   │   Output: 30,588 vertices                                        │
   │   Time: 1.02s                                                    │
   │                                                                  │
   │ D9 (Depth 9): Fine details for precision                         │
   │   Output: 131,675 vertices                                       │
   │   Time: 2.32s                                                    │
   └─────────────────────────────────────────────────────────────────┘

3. COMPUTE BASE TSDF FROM D8 SURFACE
   ┌─────────────────────────────────────────────────────────────────┐
   │ Build RaycastingScene from D8 mesh (Open3D tensor API)           │
   │                                                                  │
   │ For each TSDF block:                                             │
   │   Compute voxel centers:                                         │
   │     local_offset = (idx + 0.5) × voxel_size                      │
   │     center = block_origin + local_offset                         │
   │                                                                  │
   │ Batch query all voxel centers:                                   │
   │   closest_points = scene.compute_closest_points(centers)         │
   │   Returns: closest point on surface, face ID                     │
   │                                                                  │
   │ Compute SDF (projection method):                                 │
   │   vec = voxel_center - closest_point                             │
   │   normal = triangle_normals[face_id]                             │
   │   sdf = dot(vec, normal)    // Signed projection distance       │
   │   sdf = clamp(sdf, -TRUNCATION, +TRUNCATION)                    │
   │                                                                  │
   │ Compute weight (plateau weighting):                              │
   │   PLATEAU = 0.125m (full weight)                                 │
   │   BAND = 0.375m (transition)                                     │
   │   if dist ≤ PLATEAU: weight = 1.0                               │
   │   elif dist ≤ BAND: weight = (BAND - dist) / (BAND - PLATEAU)   │
   │   else: weight = 0.0                                             │
   │                                                                  │
   │ Result: 141,632 active voxels                                    │
   │ Time: 0.27s                                                      │
   └─────────────────────────────────────────────────────────────────┘

4. REFINE SURFACE VOXELS WITH D9
   ┌─────────────────────────────────────────────────────────────────┐
   │ Purpose: Add fine details from D9 where D8 is imprecise          │
   │                                                                  │
   │ Select voxels for refinement:                                    │
   │   Criteria: weight > 0 AND |sdf| < REFINE_THRESHOLD (8cm)       │
   │                                                                  │
   │ For selected voxels:                                             │
   │   • Query closest point on D9 mesh                               │
   │   • Recompute SDF using D9 surface                               │
   │   • Replace D8 SDF with D9 SDF                                   │
   │                                                                  │
   │ Result: 30,846 voxels refined                                    │
   │ Time: 0.08s                                                      │
   └─────────────────────────────────────────────────────────────────┘

5. PRUNE OUTLIER VOXELS
   ┌─────────────────────────────────────────────────────────────────┐
   │ Purpose: Remove false geometry from Poisson in empty regions     │
   │                                                                  │
   │ Step 5.1: Generate temporary mesh from current TSDF              │
   │   • Assemble volume array                                        │
   │   • Run marching cubes                                           │
   │                                                                  │
   │ Step 5.2: Find outlier vertices                                  │
   │   • Compute distance from TSDF mesh to warped reference mesh    │
   │   • Outliers = vertices with distance > PRUNE_THRESHOLD (25cm)  │
   │                                                                  │
   │ Step 5.3: Remove nearby voxels                                   │
   │   • Build KDTree from outlier vertices                           │
   │   • For each active voxel:                                       │
   │     - Query distance to nearest outlier                          │
   │     - If distance < PRUNE_RADIUS (30cm): set weight = 0         │
   │                                                                  │
   │ Result: 25,465 voxels removed                                    │
   │ Time: 0.11s                                                      │
   └─────────────────────────────────────────────────────────────────┘

6. GENERATE OUTPUT MESH
   ┌─────────────────────────────────────────────────────────────────┐
   │ Assemble final TSDF volume:                                      │
   │   • Stack all blocks into 3D array                               │
   │   • Apply weight mask                                            │
   │                                                                  │
   │ Run marching cubes:                                              │
   │   vertices, faces, normals = marching_cubes(volume, level=0)    │
   │                                                                  │
   │ Transform to world coordinates:                                  │
   │   vertices += min_block_origin + 0.5 × voxel_size               │
   │                                                                  │
   │ Result: 56,845 vertices                                          │
   │ Time: 0.07s                                                      │
   └─────────────────────────────────────────────────────────────────┘

7. SAVE OUTPUTS
   ┌─────────────────────────────────────────────────────────────────┐
   │ TSDF: warped_map.tsdf                                       │
   │   Format: Protobuf (Block_pb2, Layer_pb2)                        │
   │   Contains: voxel_size, vps, block origins, SDF + weight data   │
   │                                                                  │
   │ Mesh: warped_tsdf_mesh.ply                                  │
   │   Format: PLY (vertices, triangles, normals)                     │
   └─────────────────────────────────────────────────────────────────┘

8. EVALUATION
   ┌─────────────────────────────────────────────────────────────────┐
   │ Comparison      | 10cm (P/R/F)       | 25cm (P/R/F)       | Error │
   │ ────────────────────────────────────────────────────────────────│
   │ Warped vs GT    | 56.9/64.1/60.3     | 80.2/88.1/84.0     | 17.9cm│
   │ TSDF vs Warped  | 80.1/74.7/77.3     | 98.4/94.9/96.6     | 7.6cm │
   │ TSDF vs GT      | 60.7/63.2/61.9     | 84.4/88.3/86.3     | 16.3cm│
   └─────────────────────────────────────────────────────────────────┘
```

---

### Step 5: Evaluation (Python)

**File:** `/Datasets/Work_Data/ACL_OneLoop20251220_214841/Script/eval_fscore.py`

**Purpose:** Comprehensive comparison of all mesh outputs against ground truth.

#### Algorithm Flow

```
1. LOAD ALL MESHES
   ┌─────────────────────────────────────────────────────────────────┐
   │ GT (Ground Truth): 56,548 vertices                               │
   │ Pre (Before loop): 72,611 vertices                               │
   │ Post (After loop): 70,709 vertices                               │
   │ Warped (ARAP):     63,343 vertices                               │
   │ TSDF (Final):      56,845 vertices                               │
   └─────────────────────────────────────────────────────────────────┘

2. SAMPLE POINT CLOUDS
   ┌─────────────────────────────────────────────────────────────────┐
   │ For each mesh:                                                   │
   │   pcd = mesh.sample_points_uniformly(2,000,000)                  │
   │                                                                  │
   │ This ensures fair comparison regardless of mesh resolution       │
   └─────────────────────────────────────────────────────────────────┘

3. COMPUTE METRICS
   ┌─────────────────────────────────────────────────────────────────┐
   │ For each mesh (Pre, Post, Warped, TSDF) vs GT:                   │
   │                                                                  │
   │   Distance computation:                                          │
   │     dists_pred2gt = pcd_pred.compute_point_cloud_distance(pcd_gt)│
   │     dists_gt2pred = pcd_gt.compute_point_cloud_distance(pcd_pred)│
   │                                                                  │
   │   For each threshold t in [10cm, 25cm, 50cm]:                    │
   │     Precision = mean(dists_pred2gt < t) × 100                   │
   │     Recall = mean(dists_gt2pred < t) × 100                      │
   │     F-score = 2 × P × R / (P + R)                               │
   │                                                                  │
   │   Error metrics:                                                 │
   │     Precision Error = mean(dists_pred2gt) × 100 (cm)            │
   │     Recall Error = mean(dists_gt2pred) × 100 (cm)               │
   └─────────────────────────────────────────────────────────────────┘

4. OUTPUT RESULTS TABLE
   ┌────────────────────────────────────────────────────────────────────────────────┐
   │                    10cm                     25cm                     50cm       │
   │ Mesh     Precision  Recall  F-score  Precision  Recall  F-score  Precision ... │
   │ ─────────────────────────────────────────────────────────────────────────────── │
   │ Pre        40.9     49.5     44.8      60.5     70.8     65.3      77.8    ... │
   │ Post       66.4     78.8     72.1      79.5     90.9     84.8      89.2    ... │
   │ Warped     66.5     77.2     71.4      82.3     89.6     85.8      92.9    ... │
   │ TSDF       71.0     73.7     72.3      86.4     89.5     87.9      93.7    ... │
   └────────────────────────────────────────────────────────────────────────────────┘

5. F-SCORE IMPROVEMENT VS PRE
   ┌─────────────────────────────────────────────────────────────────┐
   │ Mesh     |   10cm   |   25cm   |   50cm   |                      │
   │ ─────────────────────────────────────────────                    │
   │ Post     |  +27.3   |  +19.5   |  +10.8   |                      │
   │ Warped   |  +26.7   |  +20.5   |  +12.1   |                      │
   │ TSDF     |  +27.5   |  +22.6   |  +12.7   |                      │
   └─────────────────────────────────────────────────────────────────┘
```

---

## Performance Metrics

### Pipeline Timing Summary

| Step | Component | Time |
|------|-----------|------|
| 1 | Ray Caster | 0.43s |
| 2 | Lindstrom Triangulator | 0.17s |
| 3 | Mesh Warping (ARAP) | 0.89s |
| 4 | Mesh to TSDF | 3.87s |
| | **Total Pipeline** | **5.36s** |

*(Depth image loading: 13.2s - one-time I/O cost)*

*(Evaluation/F-score computation times not included in pipeline total)*

### Step 4 (Mesh to TSDF) Breakdown

| Sub-step | Time |
|----------|------|
| Poisson D8 | 1.02s |
| Poisson D9 | 2.32s |
| Base TSDF | 0.27s |
| D9 Refinement | 0.08s |
| Pruning | 0.11s |
| Mesh Generation | 0.07s |
| **Total** | **3.87s** |

---

## Output Files

| File | Description | Size |
|------|-------------|------|
| `optimization_data.txt` | Ray casting correspondences | 12,888 points |
| `optimized_points_final.txt` | Triangulated control points | 12,888 points |
| `mesh_deformed_arap.ply` | ARAP-deformed mesh | 63,343 vertices |
| `warped_map.tsdf` | Final TSDF map | 298 blocks |
| `warped_tsdf_mesh.ply` | Final mesh from TSDF | 56,845 vertices |

---

## Quality Metrics (Final Results)

### F-score vs Ground Truth

| Mesh | 10cm | 25cm | 50cm |
|------|------|------|------|
| Pre (Before loop) | 44.8% | 65.3% | 82.0% |
| Post (After loop) | 72.1% | 84.8% | 92.8% |
| Warped (ARAP) | 71.4% | 85.8% | 94.1% |
| **TSDF (Final)** | **72.3%** | **87.9%** | **94.7%** |

### Improvement over Pre-loop Mesh

| Mesh | 10cm | 25cm | 50cm |
|------|------|------|------|
| Post | +27.3% | +19.5% | +10.8% |
| Warped | +26.7% | +20.5% | +12.1% |
| **TSDF** | **+27.5%** | **+22.6%** | **+12.7%** |

---
INPUT: Pre-loop Mesh + Pre/Post Trajectory + Depth Images
OUTPUT: Warped TSDF Map + Mesh


## Author

Jixian T
