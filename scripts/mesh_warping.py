
"""
Mesh Deformation with libigl ARAP
"""

import numpy as np
import open3d as o3d
import os
import time
from scipy.spatial import cKDTree
from collections import defaultdict

try:
    import igl
    HAS_LIBIGL = True
    print("[OK] libigl loaded")
except ImportError:
    HAS_LIBIGL = False
    print("[Warning] libigl not installed")


# ==========================================
# Utility Functions
# ==========================================

def compute_fscore(pcd_pred, pcd_gt, thresholds=[0.10, 0.25, 0.50]):
    """Compute F-score at multiple thresholds"""
    pts_pred = np.asarray(pcd_pred.points)
    pts_gt = np.asarray(pcd_gt.points)
    
    tree_pred = cKDTree(pts_pred)
    tree_gt = cKDTree(pts_gt)
    
    dists_pred_to_gt, _ = tree_gt.query(pts_pred, k=1)
    dists_gt_to_pred, _ = tree_pred.query(pts_gt, k=1)
    
    results = {}
    for thresh in thresholds:
        precision = (dists_pred_to_gt < thresh).sum() / len(dists_pred_to_gt)
        recall = (dists_gt_to_pred < thresh).sum() / len(dists_gt_to_pred)
        fscore = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0
        results[thresh] = {'precision': precision*100, 'recall': recall*100, 'fscore': fscore*100}
    
    return results


def clean_mesh(mesh):
    """
    Clean mesh: keep largest connected component + fix non-manifold structures
    This is critical for ARAP to work!
    """
    print("   [Mesh Cleaning]...")
    v_orig = len(mesh.vertices)
    
    # 1. Keep largest connected component
    triangle_clusters, cluster_n_triangles, _ = mesh.cluster_connected_triangles()
    triangle_clusters = np.asarray(triangle_clusters)
    cluster_n_triangles = np.asarray(cluster_n_triangles)
    
    if len(cluster_n_triangles) > 0:
        largest_cluster_idx = cluster_n_triangles.argmax()
        triangles_to_remove = triangle_clusters != largest_cluster_idx
        n_remove = np.sum(triangles_to_remove)
        mesh.remove_triangles_by_mask(triangles_to_remove)
        print(f"      - Removed {n_remove} floating triangles")
    
    # 2. Iteratively clean non-manifold structures
    for i in range(5):
        n_before = len(mesh.triangles)
        mesh.remove_duplicated_vertices()
        mesh.remove_duplicated_triangles()
        mesh.remove_degenerate_triangles()
        mesh.remove_non_manifold_edges()
        if len(mesh.triangles) == n_before:
            break
    
    mesh.remove_unreferenced_vertices()
    print(f"      - Vertices: {v_orig} -> {len(mesh.vertices)}")
    
    return mesh


def match_control_points(vertices, src_pts, displacements, max_dist=0.15):
    """
    Match control points to nearest mesh vertices
    """
    tree = cKDTree(vertices)
    distances, indices = tree.query(src_pts, k=1)
    
    valid_mask = distances < max_dist
    
    print(f"   Match distance: mean={distances.mean()*1000:.1f}mm, max={distances.max()*1000:.1f}mm")
    print(f"   Valid matches (<{max_dist*100}cm): {valid_mask.sum()}/{len(src_pts)}")
    
    # Deduplicate (multiple control points may match same vertex)
    unique_controls = {}
    for i, (idx, disp) in enumerate(zip(indices, displacements)):
        if not valid_mask[i]:
            continue
        if idx not in unique_controls:
            unique_controls[idx] = [disp]
        else:
            unique_controls[idx].append(disp)
    
    final_indices = []
    final_displacements = []
    for idx, disps in unique_controls.items():
        final_indices.append(idx)
        final_displacements.append(np.mean(disps, axis=0))
    
    print(f"   After dedup: {len(final_indices)} control vertices")
    
    return np.array(final_indices), np.array(final_displacements)


# ==========================================
# ARAP Deformation
# ==========================================

def arap_deformation(vertices, triangles, control_indices, control_displacements):
    """
    libigl ARAP deformation
    
    Args:
        vertices: (N, 3) vertex coordinates
        triangles: (M, 3) face indices
        control_indices: (K,) control point indices in vertices
        control_displacements: (K, 3) control point displacements
    
    Returns:
        displacements: (N, 3) displacements for all vertices
    """
    if not HAS_LIBIGL:
        raise ImportError("libigl not installed!")
    
    # Strict type conversion (prevent C++ crash)
    V = np.ascontiguousarray(vertices, dtype=np.float64)
    F = np.ascontiguousarray(triangles, dtype=np.int64)
    b = np.ascontiguousarray(control_indices.flatten(), dtype=np.int32)
    bc = np.ascontiguousarray((V[control_indices] + control_displacements), dtype=np.float64)
    
    print(f"   ARAP: V={V.shape[0]}, F={F.shape[0]}, controls={b.shape[0]}")
    
    # ARAP precomputation
    print("   -> Precomputing...")
    arap_data = igl.ARAPData()
    igl.arap_precomputation(V, F, 3, b, arap_data)
    
    # ARAP solve
    print("   -> Solving...")
    V_new = igl.arap_solve(bc, arap_data, V)
    
    return V_new - V


# ==========================================
# Main
# ==========================================

def main():
    print("=" * 60)
    print("Mesh Deformation with libigl ARAP")
    print("=" * 60)

    # ========================================
    # Path Configuration
    # ========================================
    root_dir = "/home/jixian/Desktop/orbslam3_docker/Datasets/Work_Data/ACL_OneLoop20251220_214841/"
    loop_subdir = "loop_1_1670449626.805310/"
    
    mesh_file = os.path.join(root_dir, loop_subdir, "pre/mesh.ply")
    points_file = os.path.join(root_dir, "Output/optimized_points_final.txt")
    gt_mesh_file = os.path.join(root_dir, "O3D_GT/mesh_post_pointcloud.ply")
    post_mesh_file = os.path.join(root_dir, loop_subdir, "post/mesh.ply")
    
    output_dir = os.path.join(root_dir, "Output/Warped_Results")
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, "mesh_deformed_arap.ply")

    print(f"\nFiles:")
    print(f"   Input: {mesh_file}")
    print(f"   Output: {output_file}")

    # ========================================
    # Step 1: Load Mesh
    # ========================================
    print("\n" + "=" * 60)
    print("Step 1: Loading mesh...")
    print("=" * 60)
    
    mesh = o3d.io.read_triangle_mesh(mesh_file)
    print(f"   Original vertices: {len(mesh.vertices)}")
    print(f"   Original faces: {len(mesh.triangles)}")
    print(f"   Watertight: {mesh.is_watertight()}")
    
    # Backup original mesh (for visualization and evaluation)
    mesh_pre = o3d.geometry.TriangleMesh(mesh)
    
    # Clean mesh (required for ARAP)
    mesh = clean_mesh(mesh)
    
    vertices = np.asarray(mesh.vertices)
    triangles = np.asarray(mesh.triangles)

    # ========================================
    # Step 2: Load Control Points
    # ========================================
    print("\n" + "=" * 60)
    print("Step 2: Loading control points...")
    print("=" * 60)
    
    all_src, all_dst = [], []
    
    with open(points_file, 'r') as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
            parts = line.strip().split()
            if len(parts) >= 8:
                src = [float(parts[1]), float(parts[2]), float(parts[3])]
                dst = [float(parts[4]), float(parts[5]), float(parts[6])]
                all_src.append(src)
                all_dst.append(dst)
    
    all_src = np.array(all_src)
    all_dst = np.array(all_dst)
    all_disp = all_dst - all_src
    
    print(f"   Total points: {len(all_src)}")
    
    # Filter outlier displacements
    disp_norms = np.linalg.norm(all_disp, axis=1)
    max_allowed = max(np.median(disp_norms) * 5, 3.0)
    valid_mask = disp_norms < max_allowed
    
    n_filtered = (~valid_mask).sum()
    if n_filtered > 0:
        print(f"   [Warning] Filtered {n_filtered} outlier points")
    
    all_src = all_src[valid_mask]
    all_disp = all_disp[valid_mask]
    
    print(f"   Valid points: {len(all_src)}")
    print(f"   Displacement: mean={np.linalg.norm(all_disp, axis=1).mean()*1000:.0f}mm, "
          f"max={np.linalg.norm(all_disp, axis=1).max()*1000:.0f}mm")

    # ========================================
    # Step 3: Match Control Points
    # ========================================
    print("\n" + "=" * 60)
    print("Step 3: Matching control points...")
    print("=" * 60)
    
    control_indices, control_displacements = match_control_points(
        vertices, all_src, all_disp, max_dist=0.15
    )
    
    if len(control_indices) < 100:
        print("   [Warning] Too few control points")

    # ========================================
    # Step 4: ARAP Deformation
    # ========================================
    print("\n" + "=" * 60)
    print("Step 4: ARAP Deformation...")
    print("=" * 60)
    
    t0 = time.time()
    
    displacements = arap_deformation(
        vertices, triangles,
        control_indices, control_displacements
    )
    
    print(f"\n   [OK] ARAP completed!")
    print(f"   Time: {time.time() - t0:.2f}s")
    
    vertices_new = vertices + displacements

    # ========================================
    # Step 5: Statistics
    # ========================================
    print("\n" + "=" * 60)
    print("Step 5: Statistics...")
    print("=" * 60)
    
    mesh_disp = np.linalg.norm(displacements, axis=1) * 1000
    
    print(f"   Displacement statistics:")
    print(f"      Mean: {mesh_disp.mean():.1f} mm")
    print(f"      Median: {np.median(mesh_disp):.1f} mm")
    print(f"      Max: {mesh_disp.max():.1f} mm")
    
    ctrl_error = np.linalg.norm(displacements[control_indices] - control_displacements, axis=1) * 1000
    print(f"\n   Control point error:")
    print(f"      Mean: {ctrl_error.mean():.2f} mm")
    print(f"      Max: {ctrl_error.max():.2f} mm")

    # ========================================
    # Step 6: Save
    # ========================================
    print("\n" + "=" * 60)
    print("Step 6: Saving...")
    print("=" * 60)
    
    mesh_warped = o3d.geometry.TriangleMesh()
    mesh_warped.vertices = o3d.utility.Vector3dVector(vertices_new)
    mesh_warped.triangles = mesh.triangles
    mesh_warped.compute_vertex_normals()
    
    o3d.io.write_triangle_mesh(output_file, mesh_warped)
    print(f"   Saved: {output_file}")

    # ========================================
    # Step 7: F-score
    # ========================================
    print("\n" + "=" * 60)
    print("Step 7: F-score Evaluation")
    print("=" * 60)
    
    NUM_SAMPLES = 200000
    thresholds = [0.10, 0.25, 0.50]
    
    if os.path.exists(gt_mesh_file):
        print(f"\n   Sampling {NUM_SAMPLES} points...")
        
        try:
            gt_mesh = o3d.io.read_triangle_mesh(gt_mesh_file)
            if len(gt_mesh.triangles) > 0:
                pcd_gt = gt_mesh.sample_points_uniformly(NUM_SAMPLES)
            else:
                pcd_gt = o3d.io.read_point_cloud(gt_mesh_file)
        except:
            pcd_gt = o3d.io.read_point_cloud(gt_mesh_file)
        
        pcd_warped = mesh_warped.sample_points_uniformly(NUM_SAMPLES)
        pcd_pre = mesh_pre.sample_points_uniformly(NUM_SAMPLES)
        
        results_warped = compute_fscore(pcd_warped, pcd_gt, thresholds)
        results_pre = compute_fscore(pcd_pre, pcd_gt, thresholds)
        
        results_post = None
        if os.path.exists(post_mesh_file):
            mesh_post = o3d.io.read_triangle_mesh(post_mesh_file)
            pcd_post = mesh_post.sample_points_uniformly(NUM_SAMPLES)
            results_post = compute_fscore(pcd_post, pcd_gt, thresholds)
        
        print("\n" + "=" * 60)
        print("F-score Results")
        print("=" * 60)
        
        print(f"\n   {'Mesh':<25} {'10cm':<12} {'25cm':<12} {'50cm':<12}")
        print("   " + "-" * 60)
        print(f"   {'Pre (Before loop)':<25} {results_pre[0.10]['fscore']:.2f}%       {results_pre[0.25]['fscore']:.2f}%       {results_pre[0.50]['fscore']:.2f}%")
        print(f"   {'ARAP':<25} {results_warped[0.10]['fscore']:.2f}%       {results_warped[0.25]['fscore']:.2f}%       {results_warped[0.50]['fscore']:.2f}%")
        if results_post:
            print(f"   {'Post (After loop)':<25} {results_post[0.10]['fscore']:.2f}%       {results_post[0.25]['fscore']:.2f}%       {results_post[0.50]['fscore']:.2f}%")
        
        print("\n   Improvement vs Pre:")
        for thresh in thresholds:
            diff = results_warped[thresh]['fscore'] - results_pre[thresh]['fscore']
            print(f"      {int(thresh*100)}cm: {'+' if diff>=0 else ''}{diff:.2f}%")
        
        if results_post:
            print("\n   Comparison vs Post:")
            for thresh in thresholds:
                diff = results_warped[thresh]['fscore'] - results_post[thresh]['fscore']
                status = "[OK]" if diff >= 0 else "[X]"
                print(f"      {int(thresh*100)}cm: {status} {'+' if diff>=0 else ''}{diff:.2f}%")
    else:
        print(f"   [Warning] GT not found: {gt_mesh_file}")

    # ========================================
    # Step 8: Visualization
    # ========================================
    try:
        response = input("\nVisualize? [y/n]: ")
        if response.lower() == 'y':
            # Original mesh (gray wireframe)
            mesh_wire = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
            mesh_wire.paint_uniform_color([0.6, 0.6, 0.6])
            
            # Deformed mesh (blue)
            mesh_warped.paint_uniform_color([0.2, 0.6, 1.0])
            
            # Control point source positions (red)
            ctrl_pts = vertices[control_indices]
            pcd_ctrl = o3d.geometry.PointCloud()
            pcd_ctrl.points = o3d.utility.Vector3dVector(ctrl_pts)
            pcd_ctrl.paint_uniform_color([1, 0, 0])
            
            # Control point target positions (green)
            target_pts = ctrl_pts + control_displacements
            pcd_target = o3d.geometry.PointCloud()
            pcd_target.points = o3d.utility.Vector3dVector(target_pts)
            pcd_target.paint_uniform_color([0, 1, 0])
            
            o3d.visualization.draw_geometries(
                [mesh_wire, mesh_warped, pcd_ctrl, pcd_target],
                window_name="ARAP Mesh Deformation"
            )
    except:
        pass

    print("\n" + "=" * 60)
    print("Done!")
    print("=" * 60)


if __name__ == "__main__":
    main()