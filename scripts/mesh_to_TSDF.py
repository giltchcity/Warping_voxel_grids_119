

#!/usr/bin/env python3
"""
V164: Final Release 

Pipeline:
1. Poisson Reconstruction (D8 & D9)
   - D8: Smooth geometry for coverage
   - D9: Fine details for precision

2. Base TSDF from D8 Surface
   - RaycastingScene for fast closest point query
   - Projection SDF: sdf = dot(vec, normal)
   - Plateau weight: smooth transition

3. Local Refinement with D9
   - Only refine voxels near surface (|SDF| < threshold)
   - Replace SDF with D9's more accurate value
   - Preserves D8's coverage while adding D9's precision

4. Outlier Pruning
   - Find mesh vertices far from reference
   - Remove nearby voxels to eliminate noise

Input:  Warped mesh (deformed by SLAM correction)
Output: Clean TSDF representing the corrected map
"""

import numpy as np
import open3d as o3d
import struct
import os
import time
import copy
from skimage import measure
from scipy.spatial import cKDTree

import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import Block_pb2
    import Layer_pb2
    HAS_PROTOBUF = True
except ImportError:
    HAS_PROTOBUF = False

np.random.seed(42)

# ================== IO Functions ==================

def read_varint(f):
    result, shift = 0, 0
    while True:
        b = f.read(1)
        if not b: return None
        b = b[0]
        result |= (b & 0x7F) << shift
        if not (b & 0x80): break
        shift += 7
    return result

def write_varint(value):
    result = []
    while value > 127:
        result.append((value & 0x7F) | 0x80)
        value >>= 7
    result.append(value)
    return bytes(result)

def load_tsdf_structure(filepath):
    """Load TSDF block structure (origins only, no voxel data)"""
    blocks = {}
    voxel_size = 0.25
    vps = 16
    with open(filepath, 'rb') as f:
        f.read(4)
        f.seek(20)
        while True:
            try:
                block_size = read_varint(f)
                if block_size is None: break
                block_data = f.read(block_size)
                if len(block_data) != block_size: break
                block = Block_pb2.BlockProto()
                block.ParseFromString(block_data)
                vps = block.voxels_per_side or 16
                voxel_size = block.voxel_size or 0.25
                origin = np.array([block.origin_x, block.origin_y, block.origin_z])
                block_size_m = voxel_size * vps
                idx_key = tuple(np.round(origin / block_size_m).astype(int))
                blocks[idx_key] = {'origin': origin}
            except:
                break
    return blocks, voxel_size, vps

def save_tsdf(blocks, voxel_size, vps, path):
    """Save TSDF to protobuf format"""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'wb') as f:
        f.write(write_varint(len(blocks)))
        layer = Layer_pb2.LayerProto()
        layer.voxel_size = float(voxel_size)
        layer.voxels_per_side = int(vps)
        layer.type = "tsdf"
        lb = layer.SerializeToString()
        f.write(write_varint(len(lb)))
        f.write(lb)
        for key, block in blocks.items():
            if 'distances' not in block:
                continue
            bp = Block_pb2.BlockProto()
            bp.voxels_per_side = int(vps)
            bp.voxel_size = float(voxel_size)
            bp.origin_x = float(block['origin'][0])
            bp.origin_y = float(block['origin'][1])
            bp.origin_z = float(block['origin'][2])
            bp.has_data = True
            for d, w in zip(block['distances'], block['weights']):
                bp.voxel_data.append(struct.unpack('I', struct.pack('f', float(d)))[0])
                bp.voxel_data.append(struct.unpack('I', struct.pack('f', float(w)))[0])
            bb = bp.SerializeToString()
            f.write(write_varint(len(bb)))
            f.write(bb)

# ================== Core Algorithm ==================

def poisson_reconstruction(mesh, depth, trim_quantile=0.05):
    """
    Poisson surface reconstruction
    
    Input:  Triangle mesh
    Output: Watertight mesh with smooth normals
    
    Why: Poisson creates clean, hole-free surfaces from noisy input
    """
    pcd = mesh.sample_points_uniformly(number_of_points=500000)
    pcd.estimate_normals()
    
    mesh_poisson, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=depth, scale=1.1, linear_fit=False
    )
    
    # Remove low-density regions (artifacts from Poisson)
    densities = np.asarray(densities)
    threshold = np.percentile(densities, trim_quantile * 100)
    mesh_poisson.remove_vertices_by_mask(densities < threshold)
    
    mesh_poisson.compute_vertex_normals()
    mesh_poisson.compute_triangle_normals()
    return mesh_poisson

def compute_base_tsdf(blocks, mesh, truncation, voxel_size, vps):
    """
    Compute TSDF using mesh surface query (RaycastingScene)
    
    Input:  Empty blocks structure, source mesh
    Output: Blocks filled with SDF distances and weights
    
    Why: Surface query is faster and more accurate than point cloud sampling
         - Direct triangle lookup instead of KDTree
         - Batch processing all voxels at once
    """
    # Build acceleration structure
    mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(mesh_t)
    tri_normals = np.asarray(mesh.triangle_normals)
    
    # Precompute local voxel offsets within a block
    idx = np.arange(vps ** 3)
    lx, ly, lz = idx % vps, (idx // vps) % vps, idx // (vps * vps)
    local_offsets = np.stack([lx + 0.5, ly + 0.5, lz + 0.5], axis=-1).astype(np.float32) * voxel_size
    
    # Weight parameters (Plateau weighting)
    PLATEAU = voxel_size * 0.5   # Full weight region: 0.125m
    BAND = voxel_size * 1.5      # Transition band: 0.375m
    
    # Collect all voxel centers
    all_pts = []
    block_info = []
    cursor = 0
    
    for key, block in blocks.items():
        pts = local_offsets + block['origin']
        block['voxel_centers'] = pts
        all_pts.append(pts)
        block_info.append((key, cursor, cursor + len(pts)))
        cursor += len(pts)
    
    all_pts = np.vstack(all_pts).astype(np.float32)
    
    # Batch query: find closest point on mesh for each voxel
    query = o3d.core.Tensor(all_pts, dtype=o3d.core.Dtype.Float32)
    closest = scene.compute_closest_points(query)
    closest_pts = closest['points'].numpy()
    face_ids = closest['primitive_ids'].numpy()
    
    # Compute SDF: projection distance along normal
    vecs = all_pts - closest_pts
    closest_normals = tri_normals[face_ids]
    sdf_all = np.einsum('ij,ij->i', vecs, closest_normals)  # dot product
    dists_all = np.linalg.norm(vecs, axis=1)
    
    # Plateau weight: smooth transition from 1 to 0
    weights_all = np.zeros_like(dists_all)
    weights_all[dists_all <= PLATEAU] = 1.0
    trans_mask = (dists_all > PLATEAU) & (dists_all <= BAND)
    weights_all[trans_mask] = (BAND - dists_all[trans_mask]) / (BAND - PLATEAU)
    
    # Distribute results back to blocks
    for key, start, end in block_info:
        blocks[key]['distances'] = np.clip(sdf_all[start:end], -truncation, truncation)
        blocks[key]['weights'] = weights_all[start:end]
    
    return int(np.sum(weights_all > 0))

def refine_surface_voxels(blocks, mesh_fine, truncation, voxel_size, vps, threshold):
    """
    Refine surface voxels using higher-resolution mesh
    
    Input:  Blocks with base TSDF, fine mesh (D9)
    Output: Blocks with refined SDF near surface
    
    Why: D9 has more geometric detail than D8
         Only refine near-surface voxels to preserve D8's coverage
    """
    mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(mesh_fine)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(mesh_t)
    tri_normals = np.asarray(mesh_fine.triangle_normals)
    
    # Collect voxels needing refinement
    all_centers = []
    all_indices = []
    
    for key, block in blocks.items():
        if 'distances' not in block:
            continue
        
        sdf = block['distances']
        weights = block['weights']
        
        # Select: has weight AND near surface
        mask = (weights > 0) & (np.abs(sdf) < threshold)
        indices = np.where(mask)[0]
        
        if len(indices) == 0:
            continue
        
        all_centers.append(block['voxel_centers'][indices])
        all_indices.append((key, indices))
    
    if len(all_centers) == 0:
        return 0
    
    # Batch query
    all_centers = np.vstack(all_centers).astype(np.float32)
    query = o3d.core.Tensor(all_centers, dtype=o3d.core.Dtype.Float32)
    
    closest = scene.compute_closest_points(query)
    closest_pts = closest['points'].numpy()
    face_ids = closest['primitive_ids'].numpy()
    
    # Compute new SDF
    vecs = all_centers - closest_pts
    new_sdf = np.einsum('ij,ij->i', vecs, tri_normals[face_ids])
    new_sdf = np.clip(new_sdf, -truncation, truncation)
    
    # Update blocks
    cursor = 0
    refined = 0
    for key, indices in all_indices:
        n = len(indices)
        blocks[key]['distances'][indices] = new_sdf[cursor:cursor + n]
        cursor += n
        refined += n
    
    return refined

def prune_outlier_voxels(blocks, mesh_ref, threshold, radius, voxel_size, vps):
    """
    Remove voxels that produce outlier mesh vertices
    
    Input:  Blocks with TSDF, reference mesh
    Output: Blocks with outlier voxels zeroed out
    
    Why: Poisson may create geometry in empty regions
         Pruning removes these false positives
    """
    # Generate temporary mesh from current TSDF
    keys = np.array(list(blocks.keys()))
    min_idx = keys.min(axis=0)
    max_idx = keys.max(axis=0)
    vol_shape = ((max_idx - min_idx + 1) * vps).astype(int)
    
    volume = np.ones(vol_shape, dtype=np.float32) * 0.5
    weight_vol = np.zeros(vol_shape, dtype=np.float32)
    
    for key, block in blocks.items():
        if 'distances' not in block:
            continue
        rel = np.array(key) - min_idx
        s = (rel * vps).astype(int)
        e = s + vps
        volume[s[0]:e[0], s[1]:e[1], s[2]:e[2]] = np.array(block['distances']).reshape((vps,) * 3, order='F')
        weight_vol[s[0]:e[0], s[1]:e[1], s[2]:e[2]] = np.array(block['weights']).reshape((vps,) * 3, order='F')
    
    try:
        verts, _, _, _ = measure.marching_cubes(volume, level=0.0, spacing=(voxel_size,) * 3, mask=weight_vol > 0)
    except:
        return 0
    
    verts += min_idx.astype(float) * vps * voxel_size + voxel_size * 0.5
    
    # Find outlier vertices (far from reference)
    pcd_mesh = o3d.geometry.PointCloud()
    pcd_mesh.points = o3d.utility.Vector3dVector(verts)
    pcd_ref = o3d.geometry.PointCloud()
    pcd_ref.points = mesh_ref.vertices
    dists = np.asarray(pcd_mesh.compute_point_cloud_distance(pcd_ref))
    
    bad_pts = verts[dists > threshold]
    if len(bad_pts) == 0:
        return 0
    
    # Remove voxels near outlier vertices
    tree = cKDTree(bad_pts)
    killed = 0
    
    for block in blocks.values():
        if 'weights' not in block:
            continue
        
        w = block['weights']
        active = np.where(w > 0)[0]
        if len(active) == 0:
            continue
        
        centers = block['voxel_centers'][active]
        d, _ = tree.query(centers, k=1, distance_upper_bound=radius, workers=-1)
        
        kill_indices = active[d <= radius]
        if len(kill_indices) > 0:
            w[kill_indices] = 0.0
            killed += len(kill_indices)
    
    return killed

def tsdf_to_mesh(blocks, voxel_size=0.25, vps=16):
    """Extract mesh from TSDF using marching cubes"""
    keys = np.array(list(blocks.keys()))
    min_idx = keys.min(axis=0)
    max_idx = keys.max(axis=0)
    vol_shape = ((max_idx - min_idx + 1) * vps).astype(int)
    
    volume = np.ones(vol_shape, dtype=np.float32) * 0.5
    weight_vol = np.zeros(vol_shape, dtype=np.float32)
    
    for key, block in blocks.items():
        if 'distances' not in block:
            continue
        rel = np.array(key) - min_idx
        s = (rel * vps).astype(int)
        e = s + vps
        volume[s[0]:e[0], s[1]:e[1], s[2]:e[2]] = np.array(block['distances']).reshape((vps,) * 3, order='F')
        weight_vol[s[0]:e[0], s[1]:e[1], s[2]:e[2]] = np.array(block['weights']).reshape((vps,) * 3, order='F')
    
    verts, faces, normals, _ = measure.marching_cubes(volume, level=0.0, spacing=(voxel_size,) * 3, mask=weight_vol > 0)
    verts += min_idx.astype(float) * vps * voxel_size + voxel_size * 0.5
    
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(verts)
    mesh.triangles = o3d.utility.Vector3iVector(faces)
    mesh.vertex_normals = o3d.utility.Vector3dVector(normals)
    return mesh

# ================== Evaluation ==================

def compute_metrics(mesh_pred, mesh_gt):
    """Compute precision, recall, F-score at multiple thresholds"""
    pcd_pred = mesh_pred.sample_points_uniformly(200000)
    pcd_gt = mesh_gt.sample_points_uniformly(200000)
    
    d_pred_to_gt = np.asarray(pcd_pred.compute_point_cloud_distance(pcd_gt))
    d_gt_to_pred = np.asarray(pcd_gt.compute_point_cloud_distance(pcd_pred))
    
    results = {}
    for t in [0.10, 0.25, 0.50]:
        precision = np.mean(d_pred_to_gt < t) * 100
        recall = np.mean(d_gt_to_pred < t) * 100
        fscore = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0
        results[t] = (precision, recall, fscore)
    
    results['error'] = np.mean(d_pred_to_gt) * 100
    return results

def print_evaluation_table(metrics_dict):
    """Print evaluation results in table format"""
    print("-" * 95)
    print(f"{'Comparison':<15} | {'10cm (P/R/F)':<20} | {'25cm (P/R/F)':<20} | {'50cm (P/R/F)':<20} | {'Error':>8}")
    print("-" * 95)
    
    for name, m in metrics_dict.items():
        row = f"{name:<15} |"
        for t in [0.10, 0.25, 0.50]:
            p, r, f = m[t]
            row += f" {p:>5.1f}/{r:>5.1f}/{f:>5.1f}  |"
        row += f" {m['error']:>7.1f}cm"
        print(row)
    
    print("-" * 95)

# ================== Main ==================

def main():
    print("=" * 70)
    print("Mesh to TSDF Conversion (V164)")
    print("=" * 70)
    
    if not HAS_PROTOBUF:
        print("Error: protobuf required")
        return
    
    # Configuration
    ROOT = "/home/jixian/Desktop/orbslam3_docker/Datasets/Work_Data/ACL_OneLoop20251220_214841/"
    TSDF_PATH = ROOT + "loop_1_1670449626.805310/pre/map.tsdf"
    GT_PATH = ROOT + "O3D_GT/mesh_post_pointcloud.ply"
    WARPED_PATH = ROOT + "Output/Warped_Results/mesh_deformed_arap.ply"
    OUTPUT_DIR = ROOT + "Output/Warped_Results/"
    
    # Parameters
    TRUNCATION = 0.50
    REFINE_THRESHOLD = 0.08  # 7cm
    PRUNE_THRESHOLD = 0.25   # 25cm
    PRUNE_RADIUS = 0.30      # 30cm
    
    # Load data
    print("\n[1] Loading data...")
    mesh_gt = o3d.io.read_triangle_mesh(GT_PATH)
    mesh_warped = o3d.io.read_triangle_mesh(WARPED_PATH)
    mesh_warped.compute_vertex_normals()
    blocks, voxel_size, vps = load_tsdf_structure(TSDF_PATH)
    print(f"    Blocks: {len(blocks)}, Voxel size: {voxel_size}m")
    
    timings = {}
    
    # Step 1: Poisson reconstruction
    print("\n[2] Poisson reconstruction...")
    t0 = time.time()
    mesh_d8 = poisson_reconstruction(mesh_warped, depth=8)
    timings['poisson_d8'] = time.time() - t0
    print(f"    D8: {len(mesh_d8.vertices):,} vertices  [{timings['poisson_d8']:.2f}s]")
    
    t0 = time.time()
    mesh_d9 = poisson_reconstruction(mesh_warped, depth=9)
    timings['poisson_d9'] = time.time() - t0
    print(f"    D9: {len(mesh_d9.vertices):,} vertices  [{timings['poisson_d9']:.2f}s]")
    
    # Step 2: Base TSDF
    print("\n[3] Computing base TSDF from D8 surface...")
    t0 = time.time()
    active = compute_base_tsdf(blocks, mesh_d8, TRUNCATION, voxel_size, vps)
    timings['base_tsdf'] = time.time() - t0
    print(f"    Active voxels: {active:,}  [{timings['base_tsdf']:.2f}s]")
    
    # Step 3: D9 refinement
    print(f"\n[4] Refining surface voxels with D9 (threshold={REFINE_THRESHOLD*100:.0f}cm)...")
    t0 = time.time()
    refined = refine_surface_voxels(blocks, mesh_d9, TRUNCATION, voxel_size, vps, REFINE_THRESHOLD)
    timings['refine'] = time.time() - t0
    print(f"    Refined voxels: {refined:,}  [{timings['refine']:.2f}s]")
    
    # Step 4: Pruning
    print(f"\n[5] Pruning outliers (threshold={PRUNE_THRESHOLD*100:.0f}cm, radius={PRUNE_RADIUS*100:.0f}cm)...")
    t0 = time.time()
    killed = prune_outlier_voxels(blocks, mesh_warped, PRUNE_THRESHOLD, PRUNE_RADIUS, voxel_size, vps)
    timings['prune'] = time.time() - t0
    print(f"    Removed voxels: {killed:,}  [{timings['prune']:.2f}s]")
    
    # Step 5: Generate output
    print("\n[6] Generating output mesh...")
    t0 = time.time()
    mesh_final = tsdf_to_mesh(blocks, voxel_size, vps)
    mesh_final.compute_vertex_normals()
    timings['mesh_gen'] = time.time() - t0
    print(f"    Output: {len(mesh_final.vertices):,} vertices  [{timings['mesh_gen']:.2f}s]")
    
    # Save
    output_tsdf = os.path.join(OUTPUT_DIR, "warped_map.tsdf")
    output_mesh = os.path.join(OUTPUT_DIR, "warped_tsdf_mesh.ply")
    save_tsdf(blocks, voxel_size, vps, output_tsdf)
    o3d.io.write_triangle_mesh(output_mesh, mesh_final)
    
    # Timing summary
    total = sum(timings.values())
    print("\n" + "=" * 70)
    print("Timing Summary")
    print("=" * 70)
    print(f"    Poisson D8:     {timings['poisson_d8']:>6.2f}s")
    print(f"    Poisson D9:     {timings['poisson_d9']:>6.2f}s")
    print(f"    Base TSDF:      {timings['base_tsdf']:>6.2f}s")
    print(f"    D9 Refinement:  {timings['refine']:>6.2f}s")
    print(f"    Pruning:        {timings['prune']:>6.2f}s")
    print(f"    Mesh Gen:       {timings['mesh_gen']:>6.2f}s")
    print(f"    ─────────────────────────")
    print(f"    Total:          {total:>6.2f}s")
    
    # Evaluation
    print("\n" + "=" * 70)
    print("Evaluation Results")
    print("=" * 70)
    
    m_warp_gt = compute_metrics(mesh_warped, mesh_gt)
    m_tsdf_warp = compute_metrics(mesh_final, mesh_warped)
    m_tsdf_gt = compute_metrics(mesh_final, mesh_gt)
    
    print_evaluation_table({
        'Warped vs GT': m_warp_gt,
        'TSDF vs Warped': m_tsdf_warp,
        'TSDF vs GT': m_tsdf_gt
    })
    
    # Output files
    print(f"\nOutput files:")
    print(f"    TSDF: {output_tsdf}")
    print(f"    Mesh: {output_mesh}")
    
    # Visualization
    print("\nOpening visualization...")
    m1 = copy.deepcopy(mesh_warped)
    m1.paint_uniform_color([0.0, 0.8, 0.0])
    m1.translate([-50, 0, 0])
    
    m2 = copy.deepcopy(mesh_final)
    m2.paint_uniform_color([1.0, 0.6, 0.0])
    
    m3 = copy.deepcopy(mesh_gt)
    m3.paint_uniform_color([0.9, 0.9, 0.9])
    m3.translate([50, 0, 0])
    
    o3d.visualization.draw_geometries(
        [m1, m2, m3],
        width=1600, height=900,
        window_name="Warped (Green) | TSDF (Orange) | GT (Gray)"
    )

if __name__ == "__main__":
    main()