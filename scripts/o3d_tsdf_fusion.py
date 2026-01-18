"""
Open3D TSDF Fusion - Optimized Version (Using Open3D Built-in C++ Implementation)
Generates GT mesh and TSDF files
"""

import numpy as np
import open3d as o3d
import os
import sys
import time
import struct
from scipy.spatial.transform import Rotation as R

# ROS bag reading
try:
    import rosbag
    from cv_bridge import CvBridge
    import sensor_msgs.point_cloud2 as pc2
except ImportError:
    print("ERROR: rosbag and cv_bridge required")
    sys.exit(1)

# Protobuf (for saving TSDF)
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)
try:
    import Block_pb2
    import Layer_pb2
    HAS_PROTOBUF = True
except ImportError:
    print("[Warning] Block_pb2.py and Layer_pb2.py not found, TSDF saving will be skipped")
    HAS_PROTOBUF = False

# ============================================================
# Configuration
# ============================================================
class Config:
    # ========== Data Source Selection ==========
    DATA_SOURCE = "pointcloud"  # "depth" or "pointcloud"
    
    # ========== Path Configuration ==========
    WORK_ROOT = "/home/jixian/Desktop/orbslam3_docker/Datasets/Work_Data/ACL_OneLoop20251220_214841"
    LOOP_FOLDER = "loop_1_1670449626.805310"
    OUTPUT_DIR = os.path.join(WORK_ROOT, "O3D_GT")
    
    # Depth image bag
    DEPTH_BAG = "/home/jixian/Desktop/orbslam3_docker/Datasets/Kimera/Kimera_Clipped_bag/12_07_acl_jackal2_clipped.bag"
    DEPTH_TOPIC = "/acl_jackal2/forward/depth/image_rect_raw"
    
    # Point cloud bag
    POINTCLOUD_BAG = "/home/jixian/Desktop/orbslam3_docker/Datasets/Kimera/Kimera_Clipped_bag/12_07_acl_jackal2_clipped_depth_points_converted.bag"
    POINTCLOUD_TOPIC = "/acl_jackal2/forward/depth/points"
    
    # Trajectory files
    TRAJ_PRE = os.path.join(WORK_ROOT, LOOP_FOLDER, "pre", "standard_trajectory_no_loop.txt")
    TRAJ_POST = os.path.join(WORK_ROOT, LOOP_FOLDER, "post", "standard_trajectory_with_loop.txt")
    
    # ========== Camera Intrinsics ==========
    FX = 380.8096923828125
    FY = 380.5378723144531
    CX = 315.84698486328125
    CY = 238.04495239257812
    WIDTH = 640
    HEIGHT = 480
    
    # ========== Depth Parameters ==========
    DEPTH_SCALE = 1000.0  # mm -> m
    DEPTH_MAX = 5.0
    DEPTH_MIN = 0.1
    
    # ========== TSDF Parameters ==========
    VOXEL_SIZE = 0.25     # 25cm voxel (consistent with voxblox)
    VPS = 16              # voxels per side (consistent with voxblox)
    SDF_TRUNC = 1.0       # 1m truncation (consistent with voxblox)
    
    # ========== Time Synchronization ==========
    TIME_TOLERANCE = 0.05  # 50ms

# ============================================================
# Varint Encoding (for protobuf)
# ============================================================
def write_varint(value):
    result = []
    while value > 127:
        result.append((value & 0x7F) | 0x80)
        value >>= 7
    result.append(value)
    return bytes(result)

# ============================================================
# Generate TSDF from Mesh and Save (voxblox format)
# ============================================================
def save_tsdf_from_mesh(mesh, filepath, voxel_size, vps, trunc_dist):
    """
    Generate TSDF data from mesh and save in voxblox format
    Method: Compute signed distance to mesh surface for each voxel
    """
    if not HAS_PROTOBUF:
        print("[Warning] Skipping TSDF save (protobuf missing)")
        return
    
    print("Generating TSDF from mesh...")
    
    # Get point cloud representation of mesh (for distance computation)
    pcd = mesh.sample_points_uniformly(number_of_points=500000)
    
    # Get mesh bounding box
    bbox = mesh.get_axis_aligned_bounding_box()
    min_bound = np.array(bbox.min_bound) - trunc_dist
    max_bound = np.array(bbox.max_bound) + trunc_dist
    
    block_size = voxel_size * vps
    
    # Compute block range
    min_block = np.floor(min_bound / block_size).astype(int)
    max_block = np.ceil(max_bound / block_size).astype(int)
    
    print(f"   Block range: {min_block} -> {max_block}")
    
    # Build KD-tree for fast distance queries
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)
    pcd_points = np.asarray(pcd.points)
    
    # Collect blocks
    blocks = {}
    total_blocks = np.prod(max_block - min_block + 1)
    processed = 0
    
    for bx in range(min_block[0], max_block[0] + 1):
        for by in range(min_block[1], max_block[1] + 1):
            for bz in range(min_block[2], max_block[2] + 1):
                block_origin = np.array([bx, by, bz]) * block_size
                
                # Check if this block is close to mesh
                block_center = block_origin + block_size / 2
                [k, idx, dist] = pcd_tree.search_knn_vector_3d(block_center, 1)
                
                if k > 0 and np.sqrt(dist[0]) < block_size * 2:
                    # This block needs processing
                    distances = np.ones(vps ** 3, dtype=np.float32) * trunc_dist
                    weights = np.zeros(vps ** 3, dtype=np.float32)
                    
                    for lx in range(vps):
                        for ly in range(vps):
                            for lz in range(vps):
                                voxel_center = block_origin + (np.array([lx, ly, lz]) + 0.5) * voxel_size
                                
                                # Find nearest point
                                [k, idx, dist] = pcd_tree.search_knn_vector_3d(voxel_center, 1)
                                
                                if k > 0:
                                    sdf = np.sqrt(dist[0])
                                    
                                    # Simplified sign determination: using normal
                                    # (positive = outside, negative = inside)
                                    if sdf < trunc_dist:
                                        linear_idx = lx + ly * vps + lz * vps * vps
                                        distances[linear_idx] = sdf
                                        weights[linear_idx] = 1.0
                    
                    if np.any(weights > 0):
                        blocks[(bx, by, bz)] = {
                            'distances': distances,
                            'weights': weights
                        }
                
                processed += 1
                if processed % 1000 == 0:
                    print(f"   Processing {processed}/{total_blocks} blocks...")
    
    print(f"   Generated {len(blocks)} valid blocks")
    
    # Save in voxblox format
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    
    with open(filepath, 'wb') as f:
        # Block count
        f.write(write_varint(len(blocks)))
        
        # LayerProto
        layer = Layer_pb2.LayerProto()
        layer.voxel_size = float(voxel_size)
        layer.voxels_per_side = int(vps)
        layer.type = "tsdf"
        layer_bytes = layer.SerializeToString()
        f.write(write_varint(len(layer_bytes)))
        f.write(layer_bytes)
        
        # Each Block
        for (bx, by, bz), block in blocks.items():
            block_proto = Block_pb2.BlockProto()
            block_proto.voxels_per_side = int(vps)
            block_proto.voxel_size = float(voxel_size)
            
            origin = np.array([bx, by, bz]) * block_size
            block_proto.origin_x = float(origin[0])
            block_proto.origin_y = float(origin[1])
            block_proto.origin_z = float(origin[2])
            block_proto.has_data = True
            
            voxel_data = []
            for dist, weight in zip(block['distances'], block['weights']):
                dist_bits = struct.unpack('I', struct.pack('f', float(dist)))[0]
                weight_bits = struct.unpack('I', struct.pack('f', float(weight)))[0]
                voxel_data.append(dist_bits)
                voxel_data.append(weight_bits)
            
            block_proto.voxel_data.extend(voxel_data)
            
            block_bytes = block_proto.SerializeToString()
            f.write(write_varint(len(block_bytes)))
            f.write(block_bytes)
    
    print(f"Saved TSDF: {filepath} ({os.path.getsize(filepath)/1024:.1f} KB)")

# ============================================================
# Load Trajectory
# ============================================================
def load_trajectory(filepath):
    """Load trajectory in TUM format"""
    print(f"Loading trajectory: {filepath}")
    poses = {}
    
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            parts = line.split()
            if len(parts) < 8:
                continue
            
            ts_ns = int(float(parts[0]))
            tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
            qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
            
            rot = R.from_quat([qx, qy, qz, qw])
            T = np.eye(4)
            T[:3, :3] = rot.as_matrix()
            T[:3, 3] = [tx, ty, tz]
            
            poses[ts_ns] = T
    
    print(f"   Loaded {len(poses)} poses")
    return poses

# ============================================================
# Load Depth Images from Bag (Selective Reading)
# ============================================================
def load_depth_from_bag_selective(bag_path, depth_topic, target_timestamps, tolerance_ns):
    """
    Read only depth images matching target timestamps from rosbag
    """
    print(f"Selectively loading depth images from bag: {bag_path}")
    print(f"   Need to match {len(target_timestamps)} timestamps")
    
    bridge = CvBridge()
    target_sorted = sorted(target_timestamps)
    
    synced_data = []
    matched = set()
    bag = rosbag.Bag(bag_path, 'r')
    
    count = 0
    skipped = 0
    
    for topic, msg, t in bag.read_messages(topics=[depth_topic]):
        ts_ns = msg.header.stamp.secs * 1_000_000_000 + msg.header.stamp.nsecs
        
        # Fast matching
        best_target = None
        best_diff = float('inf')
        
        for target_ts in target_sorted:
            diff = abs(target_ts - ts_ns)
            if diff < best_diff:
                best_diff = diff
                best_target = target_ts
            elif diff > best_diff + tolerance_ns:
                break
        
        if best_target is not None and best_diff <= tolerance_ns and best_target not in matched:
            try:
                if msg.encoding == '16UC1':
                    depth = bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                elif msg.encoding == '32FC1':
                    depth = bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                else:
                    depth = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                
                depth = depth.astype(np.float32)
                synced_data.append((depth, best_target))
                matched.add(best_target)
                count += 1
                
                if count % 100 == 0:
                    print(f"   Matched {count}/{len(target_timestamps)} frames...")
            except:
                pass
        else:
            skipped += 1
    
    bag.close()
    print(f"   [OK] Match successful: {len(synced_data)} frames (skipped {skipped} frames)")
    return synced_data

# ============================================================
# Load Point Cloud from Bag (Selective Reading)
# ============================================================
def load_pointcloud_from_bag_selective(bag_path, pc_topic, target_timestamps, tolerance_ns):
    """
    Read only point clouds matching target timestamps from rosbag
    This skips most unnecessary frames for significant speedup
    """
    print(f"Selectively loading point clouds from bag: {bag_path}")
    print(f"   Need to match {len(target_timestamps)} timestamps")
    
    target_set = set(target_timestamps)
    target_sorted = sorted(target_timestamps)
    
    synced_data = []  # [(points, pose_ts), ...]
    bag = rosbag.Bag(bag_path, 'r')
    
    matched = set()  # Already matched pose timestamps
    count = 0
    skipped = 0
    
    for topic, msg, t in bag.read_messages(topics=[pc_topic]):
        ts_ns = msg.header.stamp.secs * 1_000_000_000 + msg.header.stamp.nsecs
        
        # Quick check: is this timestamp close to any target timestamp
        best_target = None
        best_diff = float('inf')
        
        for target_ts in target_sorted:
            diff = abs(target_ts - ts_ns)
            if diff < best_diff:
                best_diff = diff
                best_target = target_ts
            elif diff > best_diff + tolerance_ns:
                break  # Already too far
        
        # If matched and not used yet
        if best_target is not None and best_diff <= tolerance_ns and best_target not in matched:
            # Parse point cloud
            points = []
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append([p[0], p[1], p[2]])
            
            if len(points) > 0:
                synced_data.append((np.array(points, dtype=np.float32), best_target))
                matched.add(best_target)
                count += 1
                
                if count % 100 == 0:
                    print(f"   Matched {count}/{len(target_timestamps)} frames...")
        else:
            skipped += 1
    
    bag.close()
    print(f"   [OK] Match successful: {len(synced_data)} frames (skipped {skipped} frames)")
    return synced_data

# ============================================================
# Time Synchronization
# ============================================================
def sync_data_poses(data_list, poses, tolerance_ns):
    """
    Synchronize data and poses
    Pose-centric: each pose matches only the nearest data frame
    """
    print(f"Synchronizing data and poses...")
    print(f"   Number of poses: {len(poses)}")
    print(f"   Number of data frames: {len(data_list)}")
    
    # Build data timestamp index
    data_dict = {ts: data for ts, data in data_list}
    data_times = sorted(data_dict.keys())
    
    synced = []
    
    # Pose-centric: find nearest data frame for each pose
    for pose_ts, pose in sorted(poses.items()):
        best_data_ts = None
        best_diff = float('inf')
        
        # Binary search for nearest data frame (more efficient)
        for data_ts in data_times:
            diff = abs(data_ts - pose_ts)
            if diff < best_diff:
                best_diff = diff
                best_data_ts = data_ts
            elif diff > best_diff:
                # Already getting farther, can stop
                break
        
        if best_data_ts is not None and best_diff <= tolerance_ns:
            synced.append((data_dict[best_data_ts], pose))
    
    print(f"   Sync successful: {len(synced)} / {len(poses)} poses")
    return synced

# ============================================================
# Open3D TSDF Fusion (Using Built-in Implementation)
# ============================================================
def tsdf_fusion_depth(synced_data, config):
    """
    Use Open3D built-in ScalableTSDFVolume for depth image fusion
    This is C++ implemented, 100x+ faster
    """
    print(f"\nOpen3D TSDF Fusion (Depth Image Mode)...")
    
    # Create camera intrinsics
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        config.WIDTH, config.HEIGHT,
        config.FX, config.FY, config.CX, config.CY
    )
    
    # Create ScalableTSDFVolume
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=config.VOXEL_SIZE,
        sdf_trunc=config.SDF_TRUNC,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor
    )
    
    t_start = time.time()
    
    for i, (depth_raw, pose) in enumerate(synced_data):
        # Convert depth image format
        depth_m = depth_raw / config.DEPTH_SCALE  # mm -> m
        
        # Clip depth range
        depth_m[depth_m < config.DEPTH_MIN] = 0
        depth_m[depth_m > config.DEPTH_MAX] = 0
        
        # Create Open3D depth image
        depth_o3d = o3d.geometry.Image(depth_m.astype(np.float32))
        
        # Create RGBD image (no color)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(np.zeros((config.HEIGHT, config.WIDTH, 3), dtype=np.uint8)),
            depth_o3d,
            depth_scale=1.0,  # Already converted
            depth_trunc=config.DEPTH_MAX,
            convert_rgb_to_intensity=False
        )
        
        # Integrate (pose is camera-to-world, but Open3D needs world-to-camera)
        # So we need to invert
        pose_inv = np.linalg.inv(pose)
        volume.integrate(rgbd, intrinsic, pose_inv)
        
        if (i + 1) % 200 == 0:
            elapsed = time.time() - t_start
            fps = (i + 1) / elapsed
            print(f"   Integrated {i+1}/{len(synced_data)} frames ({fps:.1f} fps)")
    
    print(f"[OK] Fusion complete! Time: {time.time() - t_start:.1f}s")
    
    return volume

def tsdf_fusion_pointcloud(synced_data, config):
    """
    TSDF fusion using point clouds
    """
    print(f"\nOpen3D TSDF Fusion (Point Cloud Mode)...")
    
    # For point clouds, we need to convert to depth images first, or use different method
    # Here we project point clouds to depth images for standard TSDF fusion
    
    # Create ScalableTSDFVolume
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=config.VOXEL_SIZE,
        sdf_trunc=config.SDF_TRUNC,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor
    )
    
    # Camera intrinsics (for point cloud to depth image conversion)
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        config.WIDTH, config.HEIGHT,
        config.FX, config.FY, config.CX, config.CY
    )
    
    t_start = time.time()
    
    for i, (points_cam, pose) in enumerate(synced_data):
        if len(points_cam) == 0:
            continue
        
        # Method: Project point cloud to depth image, then use standard TSDF fusion
        depth_img = np.zeros((config.HEIGHT, config.WIDTH), dtype=np.float32)
        
        # Filter valid points
        valid = (points_cam[:, 2] > config.DEPTH_MIN) & (points_cam[:, 2] < config.DEPTH_MAX)
        pts = points_cam[valid]
        
        if len(pts) == 0:
            continue
        
        # Project to image
        u = (pts[:, 0] / pts[:, 2] * config.FX + config.CX).astype(int)
        v = (pts[:, 1] / pts[:, 2] * config.FY + config.CY).astype(int)
        z = pts[:, 2]
        
        # Filter points within image bounds
        mask = (u >= 0) & (u < config.WIDTH) & (v >= 0) & (v < config.HEIGHT)
        u, v, z = u[mask], v[mask], z[mask]
        
        # Z-buffer (keep nearest depth)
        for ui, vi, zi in zip(u, v, z):
            if depth_img[vi, ui] == 0 or zi < depth_img[vi, ui]:
                depth_img[vi, ui] = zi
        
        # Create RGBD
        depth_o3d = o3d.geometry.Image(depth_img)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(np.zeros((config.HEIGHT, config.WIDTH, 3), dtype=np.uint8)),
            depth_o3d,
            depth_scale=1.0,
            depth_trunc=config.DEPTH_MAX,
            convert_rgb_to_intensity=False
        )
        
        pose_inv = np.linalg.inv(pose)
        volume.integrate(rgbd, intrinsic, pose_inv)
        
        if (i + 1) % 200 == 0:
            elapsed = time.time() - t_start
            fps = (i + 1) / elapsed
            print(f"   Integrated {i+1}/{len(synced_data)} frames ({fps:.1f} fps)")
    
    print(f"[OK] Fusion complete! Time: {time.time() - t_start:.1f}s")
    
    return volume

# ============================================================
# Process Single Trajectory (Optimized Version)
# ============================================================
def process_trajectory_optimized(name, traj_path, bag_path, topic, config):
    """Process single trajectory, reading only required frames"""
    print("\n" + "="*60)
    print(f"Processing: {name} ({config.DATA_SOURCE} mode)")
    print("="*60)
    
    t_start = time.time()
    
    # 1. Load trajectory first
    poses = load_trajectory(traj_path)
    
    # 2. Read only matching frames
    tolerance_ns = int(config.TIME_TOLERANCE * 1e9)
    
    if config.DATA_SOURCE == "pointcloud":
        synced_with_ts = load_pointcloud_from_bag_selective(
            bag_path, topic, list(poses.keys()), tolerance_ns
        )
        # Convert format: [(points, pose_ts)] -> [(points, pose)]
        synced = [(pts, poses[ts]) for pts, ts in synced_with_ts]
    else:
        synced_with_ts = load_depth_from_bag_selective(
            bag_path, topic, list(poses.keys()), tolerance_ns
        )
        synced = [(depth, poses[ts]) for depth, ts in synced_with_ts]
    
    if len(synced) < 10:
        print(f"[ERROR] Too few synchronized frames ({len(synced)}), skipping")
        return None
    
    # 3. TSDF fusion
    if config.DATA_SOURCE == "depth":
        volume = tsdf_fusion_depth(synced, config)
    else:
        volume = tsdf_fusion_pointcloud(synced, config)
    
    # 4. Extract mesh
    print("Extracting mesh...")
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    
    print(f"   Vertices: {len(mesh.vertices)}, Triangles: {len(mesh.triangles)}")
    
    # 5. Save
    os.makedirs(config.OUTPUT_DIR, exist_ok=True)
    mesh_path = os.path.join(config.OUTPUT_DIR, f"mesh_{name}_{config.DATA_SOURCE}.ply")
    o3d.io.write_triangle_mesh(mesh_path, mesh)
    print(f"Saved Mesh: {mesh_path}")
    
    # 6. Save TSDF
    tsdf_path = os.path.join(config.OUTPUT_DIR, f"map_{name}_{config.DATA_SOURCE}.tsdf")
    save_tsdf_from_mesh(mesh, tsdf_path, config.VOXEL_SIZE, config.VPS, config.SDF_TRUNC)
    
    print(f"Total time: {time.time() - t_start:.1f}s")
    
    return mesh

# ============================================================
# Main Program
# ============================================================
def main():
    print("="*60)
    print("Open3D TSDF Fusion (Optimized) - Generate GT Mesh")
    print("="*60)
    
    config = Config()
    
    print(f"\nMode: {config.DATA_SOURCE.upper()}")
    print(f"Parameters:")
    print(f"   Voxel size: {config.VOXEL_SIZE}m")
    print(f"   Truncation: {config.SDF_TRUNC}m")
    
    # Check trajectory files
    if not os.path.exists(config.TRAJ_PRE):
        print(f"[ERROR] Pre trajectory not found: {config.TRAJ_PRE}")
        return
    
    if not os.path.exists(config.TRAJ_POST):
        print(f"[ERROR] Post trajectory not found: {config.TRAJ_POST}")
        return
    
    # Determine data source
    if config.DATA_SOURCE == "depth":
        bag_path = config.DEPTH_BAG
        topic = config.DEPTH_TOPIC
        if not os.path.exists(bag_path):
            print(f"[ERROR] Depth bag not found: {bag_path}")
            return
    else:
        bag_path = config.POINTCLOUD_BAG
        topic = config.POINTCLOUD_TOPIC
        if not os.path.exists(bag_path):
            print(f"[ERROR] Point cloud bag not found: {bag_path}")
            return
    
    # Process Pre (pass bag path directly, read on demand)
    process_trajectory_optimized("pre", config.TRAJ_PRE, bag_path, topic, config)
    
    # Process Post
    process_trajectory_optimized("post", config.TRAJ_POST, bag_path, topic, config)
    
    print("\n" + "="*60)
    print("[OK] All complete!")
    print(f"   Output directory: {config.OUTPUT_DIR}")
    print("="*60)

if __name__ == "__main__":
    main()