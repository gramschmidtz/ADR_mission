import os
import yaml
import torch
import copy
import numpy as np
from tqdm import tqdm
from scipy.io import savemat

from adr_mission.states import KEState
from adr_mission.utils import ke_to_mee
from adr_mission.logger import ObjectTrajectory
from adr_mission.propagation import rk4
from adr_mission.dynamics import SpaceObject

def main():
    # 1. 설정 로드
    with open("configs/constants.yaml", "r", encoding="utf-8") as f:
        constants_cfg = yaml.safe_load(f)

    with open("configs/simulation.yaml", "r", encoding="utf-8") as g:
        simulation_cfg = yaml.safe_load(g)

    with open("configs/vehicle.yaml", "r", encoding="utf-8") as h:
        base_vehicle_cfg = yaml.safe_load(h)

    with open("configs/debris.yaml", "r", encoding="utf-8") as d:
        debris_configs = yaml.safe_load(d)

    # 2. 시뮬레이션 환경 구성
    R_e = constants_cfg['physics']['R_e_m']
    device = "cpu"
    dt = simulation_cfg['simulation_time']['dt']
    total_time = simulation_cfg['simulation_time']['total_time'] * 3600.0
    num_steps = int(total_time // dt)
    batch_size = 1
    
    target_debris_keys = ['debris0001', 'debris0002', 'debris0003']
    
    debris_objects = {}
    debris_states = {}
    debris_loggers = {}
    
    print("초기 상태 설정 중...")
    for key in target_debris_keys:
        cfg = debris_configs[key]
        
        # 질량 설정 업데이트
        current_vehicle_cfg = copy.deepcopy(base_vehicle_cfg)
        current_vehicle_cfg['propulsion']['m_0_kg'] = cfg['mass']
        
        # 초기 궤도 상태 설정
        a_val = R_e + (cfg['alt0_km'] * 1000.0)
        ke_init = KEState(
            torch.tensor([[a_val]], dtype=torch.float32),
            torch.tensor([[cfg['e']]], dtype=torch.float32),
            torch.tensor([[cfg['i']]], dtype=torch.float32),
            torch.tensor([[cfg['RAAN']]], dtype=torch.float32),
            torch.tensor([[cfg['AOP']]], dtype=torch.float32),
            torch.tensor([[cfg['nu']]], dtype=torch.float32),
            torch.tensor([[cfg['mass']]], dtype=torch.float32)
        )
        
        # MEE 변환 및 로거 초기화
        debris_states[key] = ke_to_mee(ke_init)
        debris_objects[key] = SpaceObject(current_vehicle_cfg, constants_cfg)
        debris_loggers[key] = ObjectTrajectory(num_steps, batch_size, 7, device)

    # 제어 입력 (파편은 추력 0)
    u_zero = torch.zeros((batch_size, 3), dtype=torch.float32, device=device)

    # 3. 적분 루프
    t = 0.0
    print(f"시뮬레이션 시작 (대상: {', '.join(target_debris_keys)})")
    for step in tqdm(range(num_steps), desc="Simulating Debris"):
        for key in target_debris_keys:
            debris_loggers[key].update(t, debris_states[key])
            debris_states[key] = rk4(
                debris_objects[key].dynamics_model, 
                debris_states[key], 
                u_zero, 
                t, 
                dt
            )
        t += dt

    # 4. 결과 저장 (원본 MEE 상태 그대로 저장)
    os.makedirs("matlab", exist_ok=True)
    mat_filename = "matlab/multi_debris_trajectory.mat"
    
    # 시간축 데이터 추출
    try:
        time_history = debris_loggers[target_debris_keys[0]].time_storage[:num_steps].cpu().numpy()
    except AttributeError:
        time_history = debris_loggers[target_debris_keys[0]].times[:num_steps].cpu().numpy()
        
    save_dict = {'times': time_history}
    
    for key in target_debris_keys:
        # 형태: [Time, Batch, 7]
        save_dict[f'{key}_states'] = debris_loggers[key].get_full_trajectory().cpu().numpy()

    savemat(mat_filename, save_dict)
    print(f"\n시뮬레이션 완료. 결과가 '{mat_filename}'에 저장되었습니다.")

if __name__ == "__main__":
    main()