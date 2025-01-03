import pyrealsense2 as rs
import numpy as np
import pickle
import time
from pynput import keyboard
from threading import Event
import open3d as o3d

def fps_point_cloud(frame_data,x_range=(-0.3, 0.3), y_range=(-0.32, 0.26), z_range=(0.58, 1.3), num_points=1024):
    # 提取 x, y, z 坐标
    points = frame_data[:, :3]  # (N, 3)

    # 根据给定范围筛选点
    mask_x = (points[:, 0] >= x_range[0]) & (points[:, 0] <= x_range[1])
    mask_y = (points[:, 1] >= y_range[0]) & (points[:, 1] <= y_range[1])
    mask_z = (points[:, 2] >= z_range[0]) & (points[:, 2] <= z_range[1])

    # 合并三个坐标轴的筛选条件
    mask = mask_x & mask_y & mask_z

    # 根据筛选条件获取点云
    filtered_points = points[mask]  # (filtered_N, 3)
    filtered_rgb = frame_data[mask, 3:]  # (filtered_N, 3)
    # print(len(filtered_points))
    # 如果符合条件的点数小于 num_points，返回所有点
    if len(filtered_points) <= num_points:
        selected_points = filtered_points
        selected_rgb = filtered_rgb
    else:
        # 使用 open3d 创建点云对象 (确保它是 CPU-based 点云)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(filtered_points)
        # 执行 FPS 下采样
        downsampled_pcd = pcd.farthest_point_down_sample(num_points)

        # 获取下采样后的点
        selected_points = np.asarray(downsampled_pcd.points)

        # 获取下采样后的点的索引：这里我们通过原始点云中的匹配索引来获取 RGB 信息
        original_points = np.asarray(pcd.points)
        selected_indices = []
        for selected_point in selected_points:
            # 通过欧氏距离找到最近的点（假设下采样后的点离原始点云中的点非常接近）
            distances = np.linalg.norm(original_points - selected_point, axis=1)
            selected_index = np.argmin(distances)
            selected_indices.append(selected_index)

        # 对应的 RGB 信息
        selected_rgb = filtered_rgb[selected_indices, :]

    # 合并选中的点和 RGB 信息
    downsampled_frame = np.hstack([selected_points, selected_rgb])  # (num_points, 6)
    return downsampled_frame

quit_program = False

def on_press(key):
    global quit_program
    try:
        if key.char == 'q':
            quit_program = True
            return False  # Stop listener
    except AttributeError:
        pass

def collect_images(color_path, depth_path, timestamps_path,point_cloud_file, fps=30):
    """使用 keep() 方法采集 RGB 和深度数据"""
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, fps)
    pipeline.start(config)

    align = rs.align(rs.stream.color)
    color_images = []  # To store all color frames
    depth_images = []  # 用于存储 depth frame 的引用
    timestamps = []
    point_cloud_data = []


    crop_width, crop_height = 480, 480
    crop_x = (640 - crop_width) // 2
    crop_y = (480 - crop_height) // 2

    try:
        print("Collecting images. Press 'q' to exit.")

        with keyboard.Listener(on_press=on_press) as listener:
            frame_ind=0
            while not quit_program:
                try:
                    frames = pipeline.wait_for_frames()
                    aligned_frames = align.process(frames)

                    color_frame = aligned_frames.get_color_frame()
                    depth_frame = aligned_frames.get_depth_frame()
                    timestamp = time.perf_counter()

                    if not color_frame or not depth_frame:
                        continue

                    # 显式保留帧
                    color_frame.keep()
                    depth_frame.keep()

                    color_image = np.asanyarray(color_frame.get_data())
                    depth_image = np.asanyarray(depth_frame.get_data())

                    cropped_color = color_image[crop_y:crop_y + crop_height, crop_x:crop_x + crop_width]
                    cropped_depth = depth_image[crop_y:crop_y + crop_height, crop_x:crop_x + crop_width]
                    non_cropped_color = color_image
                    non_cropped_depth = depth_image


                    # Generate point cloud
                    pc = rs.pointcloud()
                    point_cloud = pc.calculate(depth_frame)
                    pc.map_to(color_frame)
                    

                    # Extract vertices and filter to the cropped region
                    vertices = np.asanyarray(point_cloud.get_vertices()).view(np.float32).reshape(480, 640, 3)
                    cropped_vertices = vertices[crop_y:crop_y + crop_height, crop_x:crop_x + crop_width]
                    depth_images.append(cropped_vertices)
                    cropped_vertices = vertices.reshape(-1, 3)  # (Nc, 3)
                    color_images.append(cropped_color)
                    # Flatten cropped color and normalize
                    cropped_colors = non_cropped_color.reshape(-1, 3)  # Normalize colors to [0, 1]

                    # Combine [x, y, z, r, g, b]
                    point_cloud_frame = np.hstack((cropped_vertices, cropped_colors))

                    point_cloud_data.append(point_cloud_frame) # 裁剪和降采样

                    # 将帧加入列表
                    timestamps.append(timestamp)



                    print(f"Frame {frame_ind}collected at timestamp {timestamp}.")
                    frame_ind+=1
                except RuntimeError as e:
                    print(f"Frame timeout : {e}.")
                    continue

                if not listener.running:
                    break  # 如果监听器停止，退出循环

    finally:
        pipeline.stop()

        # 保存 RGB 和深度帧数据
        # color_images = [np.asanyarray(f.get_data()) for f in color_images]
        # depth_images = [np.asanyarray(f.get_data()) for f in depth_images]

        np.save(color_path, np.array(color_images))
        np.save(depth_path, np.array(depth_images))
        with open(timestamps_path, "wb") as f:
            pickle.dump(timestamps, f)

        with open(point_cloud_file, "wb") as f:
            pickle.dump(point_cloud_data, f)
        print(f"Point cloud data saved to {point_cloud_file}")

        print("Saved RGB and depth images to disk.")




    
if __name__ == "__main__":
    color_img_path="pour_water_data/color_images.npy"
    depth_img_path="pour_water_data/depth_images.npy"
    bag_path="pour_water_data/rgb_depth.bag"
    timestamps_path="pour_water_data/timestamps.pkl"
    point_cloud_path="pour_water_data/point_cloud.pkl"
    """太慢了，两三秒一帧"""
    collect_images(color_img_path,depth_img_path,timestamps_path,point_cloud_path)
