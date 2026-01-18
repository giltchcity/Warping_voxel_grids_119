
#!/usr/bin/env python3
"""
Mesh Evaluation Tool
Compare: Pre, Post, Warped, TSDF vs GT
"""

import open3d as o3d
import numpy as np
import os
import copy


def main():
    print("="*70)
    print("📊 Mesh Evaluation: Pre / Post / Warped / TSDF vs GT")
    print("="*70)

    # ================= Path Configuration =================
    root_dir = "/home/jixian/Desktop/orbslam3_docker/Datasets/Work_Data/ACL_OneLoop20251220_214841/"
    loop_subdir = "loop_1_1670449626.805310/"
    
    paths = {
        'Pre': root_dir + loop_subdir + "pre/mesh.ply",
        'Post': root_dir + loop_subdir + "post/mesh.ply",
        'Warped': root_dir + "Output/Warped_Results/mesh_deformed_arap.ply",
        'TSDF': root_dir + "Output/Warped_Results/warped_tsdf_mesh.ply"

    }
    
    path_gt = "/home/jixian/Desktop/orbslam3_docker/Datasets/Work_Data/ACL_OneLoop20251220_214841/O3D_GT/mesh_post_pointcloud.ply"

    # ================= Load Data =================
    print(f"\n📂 Loading meshes...")
    
    mesh_gt = o3d.io.read_triangle_mesh(path_gt)
    mesh_gt.compute_vertex_normals()
    print(f"   GT: {len(mesh_gt.vertices):,} vertices")
    
    meshes = {}
    for name, path in paths.items():
        if os.path.exists(path):
            mesh = o3d.io.read_triangle_mesh(path)
            mesh.compute_vertex_normals()
            meshes[name] = mesh
            print(f"   {name}: {len(mesh.vertices):,} vertices")
        else:
            print(f"   {name}: ⚠️ File not found")

    # ================= Sampling =================
    NUM_SAMPLES = 2000000
    print(f"\n⏳ Sampling {NUM_SAMPLES:,} points...")
    
    pcd_gt = mesh_gt.sample_points_uniformly(number_of_points=NUM_SAMPLES)
    
    pcds = {}
    for name, mesh in meshes.items():
        pcds[name] = mesh.sample_points_uniformly(number_of_points=NUM_SAMPLES)

    # ================= Compute Metrics =================
    thresholds = [0.10, 0.25, 0.50]
    
    results = {}
    for name in meshes.keys():
        pcd_pred = pcds[name]
        dists_pred2gt = np.asarray(pcd_pred.compute_point_cloud_distance(pcd_gt))
        dists_gt2pred = np.asarray(pcd_gt.compute_point_cloud_distance(pcd_pred))
        
        results[name] = {}
        for t in thresholds:
            p = np.mean(dists_pred2gt < t) * 100
            r = np.mean(dists_gt2pred < t) * 100
            f = 2 * p * r / (p + r) if (p + r) > 0 else 0
            results[name][t] = {'Precision': p, 'Recall': r, 'F-score': f}
        
        results[name]['error'] = {
            'Precision': np.mean(dists_pred2gt) * 100,
            'Recall': np.mean(dists_gt2pred) * 100
        }

    # ================= Output Table =================
    print(f"\n{'='*160}")
    print(f"📊 Evaluation Results vs GT")
    print(f"{'='*160}")
    
    # Header Row 1
    print(f"{'':8} |{'10cm':^45}|{'25cm':^45}|{'50cm':^45}| {'Error (cm)':^24}")
    
    # Header Row 2
    header2 = f"{'Mesh':8} |"
    for _ in thresholds:
        header2 += f"{'Precision':^14}|{'Recall':^14}|{'F-score':^14}|"
    header2 += f"{'Precision':^12}|{'Recall':^12}"
    print(header2)
    print("-" * 160)
    
    # Data rows
    for name in ['Pre', 'Post', 'Warped', 'TSDF']:
        if name not in results:
            continue
        
        r = results[name]
        err = r['error']
        
        row = f"{name:8} |"
        for t in thresholds:
            row += f"{r[t]['Precision']:^14.1f}|{r[t]['Recall']:^14.1f}|{r[t]['F-score']:^14.1f}|"
        row += f"{err['Precision']:^12.1f}|{err['Recall']:^12.1f}"
        print(row)
    
    print("-" * 160)
    
    # Improvement rows
    if 'Pre' in results:
        print(f"\n📈 F-score Improvement vs Pre:")
        print(f"{'Mesh':8} |{'10cm':^15}|{'25cm':^15}|{'50cm':^15}|")
        print("-" * 55)
        
        for name in ['Post', 'Warped', 'TSDF']:
            if name not in results:
                continue
            row = f"{name:8} |"
            for t in thresholds:
                diff = results[name][t]['F-score'] - results['Pre'][t]['F-score']
                row += f"{diff:^+15.1f}|"
            print(row)

    # ================= Visualization =================
    print(f"\n{'='*70}")
    print("🎨 Visualization")
    print("="*70)
    print("   1. Show all meshes in separate windows")
    print("   0. Skip")
    
    choice = input("\nSelect: ").strip()
    
    if choice == '1':
        print("\n🖼️ Opening 5 windows...")
        
        # Colors
        red = [1.0, 0.3, 0.3]      # GT
        orange = [1.0, 0.7, 0.3]   # Pre, Post
        green = [0.3, 0.8, 0.3]    # Warped
        blue = [0.3, 0.6, 1.0]     # TSDF
        
        colors = {
            'GT': red,
            'Pre': orange,
            'Post': orange,
            'Warped': green,
            'TSDF': blue
        }
        
        all_meshes = {'GT': mesh_gt, **meshes}
        names_order = ['GT', 'Pre', 'Post', 'Warped', 'TSDF']
        
        for name in names_order:
            if name not in all_meshes:
                continue
            
            m = copy.deepcopy(all_meshes[name])
            m.compute_vertex_normals()
            m.paint_uniform_color(colors[name])
            
            coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0)
            
            color_name = "Red" if colors[name] == red else ("Orange" if colors[name] == orange else ("Green" if colors[name] == green else "Blue"))
            print(f"   Opening: {name} ({color_name})")
            
            o3d.visualization.draw_geometries(
                [m, coord], 
                window_name=f"{name}",
                width=1200, height=800
            )

    print("\n✅ Done!")


if __name__ == "__main__":
    main()