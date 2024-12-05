import os
import pickle
import tqdm
import numpy as np
import pickle

def preprocess_point_cloud(frame_data, x_range=(-0.25, 0.2), y_range=(-0.4, 0.21), z_range=(0.75, 1.3)):
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
    print(len(filtered_points))
    filtered_rgb = frame_data[mask, 3:]  # (filtered_N, 3)

    # 如果符合条件的点数超过 1024，随机选择 1024 个点
    if len(filtered_points) > 1024:
        selected_indices = np.random.choice(len(filtered_points), 1024, replace=False)
        selected_points = filtered_points[selected_indices]
        selected_rgb = filtered_rgb[selected_indices]
    else:
        selected_points = filtered_points
        selected_rgb = filtered_rgb

    # 将选中的点和 RGB 信息合并
    downsampled_frame = np.hstack([selected_points, selected_rgb])  # (1024, 6)
    return downsampled_frame

def main():
    expert_data_path = '/home/prlab/PycharmProjects/point_cloud/'
    save_data_path = '/home/prlab/PycharmProjects/point_cloud/convert.npy'

    demo_dirs = os.path.join(expert_data_path, 'data.pkl')
    print(demo_dirs)
    with open(demo_dirs, "rb") as f:
        data = pickle.load(f)
    # 访问点云数据
    point_cloud_data = data["point_cloud"]
    print(f"Loaded point cloud data with {len(point_cloud_data)} frames.")
    data_length = len(point_cloud_data)
    point_cloud_arrays = []

    for step_idx in tqdm.tqdm(range(data_length)):
        obs_pointcloud = data['point_cloud'][step_idx]
        obs_pointcloud = preprocess_point_cloud(obs_pointcloud)
        point_cloud_arrays.append(obs_pointcloud)
    np.save(save_data_path, point_cloud_arrays)

if __name__ == "__main__":
    main()
# obs_pointcloud = demo['point_cloud'][step_idx]
#
# obs_pointcloud = preprocess_point_cloud(obs_pointcloud, use_cuda=True)
#
# # storage
# total_count = 0
# img_arrays = []
# point_cloud_arrays = []
# depth_arrays = []
# state_arrays = []
# action_arrays = []
# episode_ends_arrays = []
#
# dir_name = os.path.dirname(demo_dirs)
#
# cprint('Processing {}'.format(demo_dirs), 'green')
# with open(demo_dirs, 'rb') as f:
#     demo = pickle.load(f)
#
# pcd_dirs = os.path.join(dir_name, 'pcd')
# if not os.path.exists(pcd_dirs):
#     os.makedirs(pcd_dirs)
#
# demo_length = len(demo['point_cloud'])
# # dict_keys(['point_cloud', 'rgbd', 'agent_pos', 'action'])
# for step_idx in tqdm.tqdm(range(demo_length)):
#     total_count += 1
#     obs_image = demo['image'][step_idx]
#     obs_depth = demo['depth'][step_idx]
#     obs_image = preproces_image(obs_image)
#     obs_depth = preproces_image(np.expand_dims(obs_depth, axis=-1)).squeeze(-1)
#     obs_pointcloud = demo['point_cloud'][step_idx]
#     robot_state = demo['agent_pos'][step_idx]
#     action = demo['action'][step_idx]
#
#     obs_pointcloud = preprocess_point_cloud(obs_pointcloud, use_cuda=True)
#     img_arrays.append(obs_image)
#     action_arrays.append(action)
#     point_cloud_arrays.append(obs_pointcloud)
#     depth_arrays.append(obs_depth)
#     state_arrays.append(robot_state)
#
# episode_ends_arrays.append(total_count)
#
# # create zarr file
# zarr_root = zarr.group(save_data_path)
# zarr_data = zarr_root.create_group('data')
# zarr_meta = zarr_root.create_group('meta')
#
# img_arrays = np.stack(img_arrays, axis=0)
# if img_arrays.shape[1] == 3:  # make channel last
#     img_arrays = np.transpose(img_arrays, (0, 2, 3, 1))
# point_cloud_arrays = np.stack(point_cloud_arrays, axis=0)
# depth_arrays = np.stack(depth_arrays, axis=0)
# action_arrays = np.stack(action_arrays, axis=0)
# state_arrays = np.stack(state_arrays, axis=0)
# episode_ends_arrays = np.array(episode_ends_arrays)
#
# compressor = zarr.Blosc(cname='zstd', clevel=3, shuffle=1)
# img_chunk_size = (100, img_arrays.shape[1], img_arrays.shape[2], img_arrays.shape[3])
# point_cloud_chunk_size = (100, point_cloud_arrays.shape[1], point_cloud_arrays.shape[2])
# depth_chunk_size = (100, depth_arrays.shape[1], depth_arrays.shape[2])
# if len(action_arrays.shape) == 2:
#     action_chunk_size = (100, action_arrays.shape[1])
# elif len(action_arrays.shape) == 3:
#     action_chunk_size = (100, action_arrays.shape[1], action_arrays.shape[2])
# else:
#     raise NotImplementedError
# zarr_data.create_dataset('img', data=img_arrays, chunks=img_chunk_size, dtype='uint8', overwrite=True,
#                          compressor=compressor)
# zarr_data.create_dataset('point_cloud', data=point_cloud_arrays, chunks=point_cloud_chunk_size, dtype='float64',
#                          overwrite=True, compressor=compressor)
# zarr_data.create_dataset('depth', data=depth_arrays, chunks=depth_chunk_size, dtype='float64', overwrite=True,
#                          compressor=compressor)
# zarr_data.create_dataset('action', data=action_arrays, chunks=action_chunk_size, dtype='float32', overwrite=True,
#                          compressor=compressor)
# zarr_data.create_dataset('state', data=state_arrays, chunks=(100, state_arrays.shape[1]), dtype='float32',
#                          overwrite=True, compressor=compressor)
# zarr_meta.create_dataset('episode_ends', data=episode_ends_arrays, chunks=(100,), dtype='int64', overwrite=True,
#                          compressor=compressor)
#
# # print shape
# cprint(f'img shape: {img_arrays.shape}, range: [{np.min(img_arrays)}, {np.max(img_arrays)}]', 'green')
# cprint(
#     f'point_cloud shape: {point_cloud_arrays.shape}, range: [{np.min(point_cloud_arrays)}, {np.max(point_cloud_arrays)}]',
#     'green')
# cprint(f'depth shape: {depth_arrays.shape}, range: [{np.min(depth_arrays)}, {np.max(depth_arrays)}]', 'green')
# cprint(f'action shape: {action_arrays.shape}, range: [{np.min(action_arrays)}, {np.max(action_arrays)}]', 'green')
# cprint(f'state shape: {state_arrays.shape}, range: [{np.min(state_arrays)}, {np.max(state_arrays)}]', 'green')
# cprint(
#     f'episode_ends shape: {episode_ends_arrays.shape}, range: [{np.min(episode_ends_arrays)}, {np.max(episode_ends_arrays)}]',
#     'green')
# cprint(f'total_count: {total_count}', 'green')
# cprint(f'Saved zarr file to {save_data_path}', 'green')
