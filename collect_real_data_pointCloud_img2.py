import pyrealsense2 as rs
import numpy as np
import pickle
import time
from pynput import keyboard
from threading import Thread, Lock
import os
import shutil
from queue import Queue

camera_data=Queue()
batch_size=64
batch_ind=0
data_dir = "pour_water_data0116-21-250.7/"


quit_program = False

def on_press(key):
    global quit_program
    try:
        if key.char == 'q':
            quit_program = True
            return False  # Stop listener
    except AttributeError:
        pass

def collect_images(fps=30):
    """使用 keep() 方法采集 RGB 和深度数据"""
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, fps)
    pipeline.start(config)

    align = rs.align(rs.stream.color)
    


    crop_width, crop_height = 480, 480
    crop_x = (640 - crop_width) // 2
    crop_y = (480 - crop_height) // 2

    try:
        print("Collecting images. Press 'q' to exit.")

        with keyboard.Listener(on_press=on_press) as listener:
            frame_ind=0
            while not quit_program:
                try:
                    # start_time=time.time()
                    frames = pipeline.wait_for_frames()
                    aligned_frames = align.process(frames)

                    color_frame = aligned_frames.get_color_frame()
                    depth_frame = aligned_frames.get_depth_frame()
                    timestamp = time.time()

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

                    cropped_vertices = vertices.reshape(-1, 3)  # (Nc, 3)

                    cropped_colors = non_cropped_color.reshape(-1, 3)  # Normalize colors to [0, 1]

                    # Combine [x, y, z, r, g, b]
                    point_cloud_frame = np.hstack((cropped_vertices, cropped_colors))

                    camera_data.put({
                        'image':cropped_color,
                        'depth':cropped_vertices,
                        'point_cloud':point_cloud_frame,
                        'timestamp':timestamp
                    })

                    print(f"Frame {frame_ind} collected at timestamp {timestamp}.")
                    
                    frame_ind+=1
                    # end_time=time.time()
                    # time_space=end_time-start_time
                    # print(f"yici de shijianwei:{time_space}")
                    time.sleep(0.035)#16frames/1s
                except RuntimeError as e:
                    print(f"Frame timeout : {e}.")
                    continue

                if not listener.running:
                    break  # 如果监听器停止，退出循环

    finally:
        pipeline.stop()


def save_batch(batch_len):
    """分批保存数据"""
    global batch_ind
    camera_path=os.path.join(data_dir,f"camera_{batch_ind}.pkl")
    camera_batch_data=[]
    for _ in range(batch_len):
        camera_batch_data.append(camera_data.get())
    
    with open(camera_path, "wb") as camera_file:
        pickle.dump(camera_batch_data, camera_file)
    print(f"Batch {batch_ind} all saved.")
    batch_ind+=1
    

def save_data_thread():
    while not quit_program:
        if camera_data.qsize()>= batch_size:
            save_batch(batch_size)
    que_len=camera_data.qsize()
    if que_len>0:
        save_batch(que_len)

    
if __name__ == "__main__":
    if os.path.exists(data_dir):
        shutil.rmtree(data_dir)
    if not os.path.exists(data_dir):
            os.makedirs(data_dir)
    save_thread=Thread(target=save_data_thread)
    collect_thread=Thread(target=collect_images)
    save_thread.start()
    collect_thread.start()
    save_thread.join()
    collect_thread.join()
