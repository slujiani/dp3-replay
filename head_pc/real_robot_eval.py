if __name__ == "__main__":
    import sys
    import os
    import pathlib

    ROOT_DIR = str(pathlib.Path(__file__).parent.parent.parent)
    sys.path.append(ROOT_DIR)
    os.chdir(ROOT_DIR)

import hydra
import zerorpc
import copy
import torch
import dill
import numpy as np
import random
from termcolor import cprint
from omegaconf import OmegaConf
from diffusion_policy_3d.policy.dp3 import DP3

from gen_data import collect_current_cropped_point_cloud


OmegaConf.register_new_resolver("eval", eval, replace=True)

class real_robot_eval:
    def __init__(self,cfg: OmegaConf) -> None:
        self.cfg=cfg
        self.output_dir = '/home/prlab/3D-Diffusion-Policy/3D-Diffusion-Policy/data/outputs/realdex_pour-dp3-1202_seed0'
        seed = cfg.training.seed
        print("===========================")
        # print(cfg)
        torch.manual_seed(seed)
        np.random.seed(seed)
        random.seed(seed)

        self.model: DP3 = hydra.utils.instantiate(cfg.policy)
        self.ema_model: DP3 = None
        if cfg.training.use_ema:
            try:
                self.ema_model = copy.deepcopy(self.model)
            except: # minkowski engine could not be copied. recreate it
                self.ema_model = hydra.utils.instantiate(cfg.policy)


        # configure training state
        self.optimizer = hydra.utils.instantiate(
            cfg.optimizer, params=self.model.parameters())

        # configure training state
        self.global_step = 0
        self.epoch = 0

        self.head_pc=zerorpc.Client(timeout=60, heartbeat=30)
        self.head_pc.connect("tcp://172.16.0.1:4242")
        print("和arm pc 连接成功")
    
    def eval(self):
        #加载检查点
        cfg = copy.deepcopy(self.cfg)
        lastest_ckpt_path = self.get_checkpoint_path(tag="latest")
        # print(lastest_ckpt_path)
        if lastest_ckpt_path.is_file():
            cprint(f"Resuming from checkpoint {lastest_ckpt_path}", 'magenta')
            self.load_checkpoint(path=lastest_ckpt_path)

        self.model.eval()  # 将模型设置为评估模式
        self.model.cuda()
        iterations=5

        #设置观测帧数、预测action帧数
        self.head_pc.set_n_obs_steps(self.model.n_obs_steps)
        self.head_pc.set_n_action_steps(self.model.n_action_steps)

        for _ in range(iterations):
            # 获取观测值
            obs={}
            
            pcs_tensor=collect_current_cropped_point_cloud(self.model.n_obs_steps)  #获取裁剪好的 处理好的点云
            # print("********点云********")
            point_cloud=pcs_tensor.unsqueeze(0)
            obs['point_cloud']=point_cloud 
            # print(point_cloud.shape)
            # print("********点云********")
            # print(self.model.n_obs_steps)

            agent_pos=self.head_pc.get_current_pose()  #获取机械臂的观测值，和点云的帧数相同
            agent_pos_tensor=torch.Tensor(agent_pos)
            # print(f"agent_pos = {agent_pos_tensor}")
            obs['agent_pos']=agent_pos_tensor.unsqueeze(0)
            with torch.no_grad():
                action_dict = self.model.predict_action(obs)  # 获取模型预测的动作
                action_list= action_dict['action'].squeeze().to('cpu').numpy().tolist()
                print("**************action************")
                print(action_dict['action'].shape)
                print(action_dict['action'])
                print(type(action_list))
            self.head_pc.move_relative(action_list)  # 将动作发送到机械臂

    def get_checkpoint_path(self, tag='latest'):
        if tag=='latest':
            return pathlib.Path(self.output_dir).joinpath('checkpoints', f'{tag}.ckpt')
        elif tag=='best': 
            # the checkpoints are saved as format: epoch={}-test_mean_score={}.ckpt
            # find the best checkpoint
            checkpoint_dir = pathlib.Path(self.output_dir).joinpath('checkpoints')
            all_checkpoints = os.listdir(checkpoint_dir)
            best_ckpt = None
            best_score = -1e10
            for ckpt in all_checkpoints:
                if 'latest' in ckpt:
                    continue
                score = float(ckpt.split('test_mean_score=')[1].split('.ckpt')[0])
                if score > best_score:
                    best_ckpt = ckpt
                    best_score = score
            return pathlib.Path(self.output_dir).joinpath('checkpoints', best_ckpt)
        else:
            raise NotImplementedError(f"tag {tag} not implemented")
        
    def load_checkpoint(self, path=None, tag='latest',
            exclude_keys=None, 
            include_keys=None, 
            **kwargs):
        if path is None:
            path = self.get_checkpoint_path(tag=tag)
        else:
            path = pathlib.Path(path)
        payload = torch.load(path.open('rb'), pickle_module=dill, map_location='cpu')
        self.load_payload(payload, 
            exclude_keys=exclude_keys, 
            include_keys=include_keys)
        return payload
    
    def load_payload(self, payload, exclude_keys=None, include_keys=None, **kwargs):
        if exclude_keys is None:
            exclude_keys = tuple()
        if include_keys is None:
            include_keys = payload['pickles'].keys()

        for key, value in payload['state_dicts'].items():
            if key not in exclude_keys:
                self.__dict__[key].load_state_dict(value, **kwargs)
        for key in include_keys:
            if key in payload['pickles']:
                self.__dict__[key] = dill.loads(payload['pickles'][key])

@hydra.main(
    version_base=None,
    config_path=str(pathlib.Path(__file__).parent.joinpath(
        'diffusion_policy_3d', 'config'))
)
def main(cfg):
    workspace = real_robot_eval(cfg)
    workspace.eval()

if __name__ == "__main__":
    main()