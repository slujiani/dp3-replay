import pyrealsense2 as rs
import numpy as np
import pickle  # 用于保存为.pkl文件
import time
import torch
from convert_data import fps_point_cloud

def collect_current_cropped_point_cloud() -> torch.Tensor:
    print("++++++++++++++++++")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)
    align_to = rs.stream.color   # Align depth to color
    align = rs.align(align_to)

    crop_width, crop_height = 480, 480
    crop_x = (640 - crop_width) // 2
    crop_y = (480 - crop_height) // 2
    
    try:
        print("Collecting cropped data...")
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        # Get color and depth frames
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        
        # Convert frames to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Generate point cloud
        pc = rs.pointcloud()
        pc.map_to(color_frame)
        point_cloud = pc.calculate(depth_frame)

        # Extract vertices and filter to the cropped region
        vertices = np.asanyarray(point_cloud.get_vertices()).view(np.float32).reshape(480, 640, 3)
        cropped_vertices = vertices[crop_y:crop_y + crop_height, crop_x:crop_x + crop_width]

        cropped_vertices = vertices.reshape(-1, 3)  # (Nc, 3)

        # Flatten cropped color and normalize
        cropped_colors = color_image.reshape(-1, 3)  # Normalize colors to [0, 1]

        # Combine [x, y, z, r, g, b]
        point_cloud_frame = np.hstack((cropped_vertices, cropped_colors))
        prossessed_pc_frame=fps_point_cloud(point_cloud_frame)
        point_cloud_tensor = torch.from_numpy(prossessed_pc_frame) #numpy array转换成torch.tensor
        print(f"get the cropped point_cloud_frame")
        
    finally:            
            pipeline.stop()
            return point_cloud_tensor


def collect_cropped_point_cloud(duration=5, fps=10, output_file="data.pkl"):
    # Initialize RealSense pipeline and configuration
    print("++++++++++++++++++----------------------")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # Start the pipeline
    pipeline.start(config)

    # Align depth to color
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Parameters
    frame_interval = 0.09  # Time between frames
    total_frames = duration * fps  # Total number of frames to collect
    point_cloud_data = []  # Store collected data
    color_images = []  # To store all color frames
    depth_images = []  # To store all depth frames
    timestamp_first_frame = None

    # Center crop dimensions
    # 1732782045.7261379
    crop_width, crop_height = 480, 480
    crop_x = (640 - crop_width) // 2
    crop_y = (480 - crop_height) // 2
    try:
        print("Collecting cropped data...")
        frame_count = 0

        while frame_count < total_frames:
            # Wait for frames and align depth to color

            frames = pipeline.wait_for_frames()
            # print("the "+str(frame_count)+" is : "+str(time.time()))
            if frame_count == 0:
                start = time.time()

            # elif frame_count == 149:
            #     stop = time.time()
            aligned_frames = align.process(frames)
            # Get color and depth frames
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            # Get timestamp of the first frame
            if timestamp_first_frame is None:
                timestamp_first_frame = frames.get_timestamp()

            # Convert frames to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Crop center region
            cropped_color = color_image[crop_y:crop_y + crop_height, crop_x:crop_x + crop_width]
            cropped_depth = depth_image[crop_y:crop_y + crop_height, crop_x:crop_x + crop_width]

            non_cropped_color = color_image
            non_cropped_depth = depth_image


            # Generate point cloud
            pc = rs.pointcloud()
            pc.map_to(color_frame)
            point_cloud = pc.calculate(depth_frame)

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
            point_cloud_data.append(point_cloud_frame)


            frame_count += 1
            print(f"Frame {frame_count}/{total_frames} collected.")

            # Sleep to control frame rate
            time.sleep(frame_interval)

    finally:
        # Stop the pipeline
        print("Data collection completed.", time.time())
        pipeline.stop()

        np.save("color_images.npy", np.array(color_images))
        np.save("depth_images.npy", np.array(depth_images))
        # Save data to .pkl file
        with open(output_file, "wb") as f:
            pickle.dump({"point_cloud": point_cloud_data}, f)
        print(f"Data saved to {output_file}")
        # print(time.time())
        print(f"Timestamp of the first frame: {timestamp_first_frame}")
        # print(150 / (stop - start))
        print("start time : " + str(start))


def main():
    # Run the function
    print("start", time.time())
    collect_cropped_point_cloud()

if __name__ == "__main__":
    main()