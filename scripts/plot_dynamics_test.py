# scripts/plot_dynamics_test.py
import os
import yaml
import torch
import numpy as np
from tqdm import tqdm
from scipy.io import savemat

from adr_mission.states import MEEState
from adr_mission.logger import MissionTrajectory
from adr_mission.propagation import rk4
from adr_mission.dynamics import SpaceObject, Spacecraft

def main():
    # 1. 설정 로드
    with open("configs/constants.yaml", "r", encoding="utf-8") as f:
        constants_cfg = yaml.safe_load(f)

    with open("configs/vehicle.yaml", "r", encoding="utf-8") as g:
        vehicle_cfg = yaml.safe_load(g)
    
    with open("configs/simulation.yaml", "r", encoding="utf-8") as h:
        simulation_cfg = yaml.safe_load(h)

    # 2. 초기 궤도 상태 설정
    R_e = constants_cfg['physics']['R_e_m']
    alt0 = simulation_cfg['initial_states']['alt0_km'] * 1000.0
    a = R_e + alt0
    
    # MEE 초기값
    p_init = torch.tensor([[a]], dtype=torch.float32)
    f_init = torch.tensor([[simulation_cfg['initial_states']['f']]], dtype=torch.float32)
    g_init = torch.tensor([[simulation_cfg['initial_states']['g']]], dtype=torch.float32)
    h_init = torch.tensor([[simulation_cfg['initial_states']['h']]], dtype=torch.float32)
    k_init = torch.tensor([[simulation_cfg['initial_states']['k']]], dtype=torch.float32)
    L_init = torch.tensor([[simulation_cfg['initial_states']['L']]], dtype=torch.float32)
    m_init = torch.tensor([[vehicle_cfg['propulsion']['m_0_kg']]], dtype=torch.float32)

    initial_state = MEEState(p=p_init, f=f_init, g=g_init, h=h_init, k=k_init, L=L_init, mass=m_init)

    # 3. 시뮬레이션 환경 구성
    device = "cpu"
    dt = simulation_cfg['simulation_time']['dt']
    total_time = simulation_cfg['simulation_time']['total_time']*3600.0
    num_steps = int(total_time//dt)
    batch_size = 1
    
    # SpaceDebris
    debris = SpaceObject(vehicle_cfg, constants_cfg)
    debris_logger = MissionTrajectory(num_steps, batch_size, device)
    debris_state = initial_state
    u_zero = torch.zeros((batch_size, 3), dtype=torch.float32, device=device)

    # Spacecraft
    is_chaser_on = simulation_cfg['spacecraft']['onoff'].lower() == "on"
    if is_chaser_on:
        chaser = Spacecraft(vehicle_cfg, constants_cfg)
        chaser_logger = MissionTrajectory(num_steps, batch_size, device)
        chaser_state = initial_state
        
        # YAML에서 입력받은 리스트를 텐서로 변환
        input_list = simulation_cfg['spacecraft']['test_input']
        u_chaser = torch.tensor([input_list], dtype=torch.float32, device=device)
        
        # 제어 입력 기록용 배열 (매트랩 플롯용)
        u_history = np.zeros((num_steps, 3))

    t = 0.0
    print("시뮬레이션 시작...")

    # 4. 적분 루프
    for step in tqdm(range(num_steps), desc="Simulating Orbit"):
        debris_logger.update(t, debris_state)
        debris_state = rk4(chaser.dynamics_model, debris_state, u_zero, t, dt)
        
        if is_chaser_on:
            chaser_logger.update(t, chaser_state)
            chaser_state = rk4(chaser.dynamics_model, chaser_state, u_chaser, t, dt)
            # 현재 스텝의 입력 기록
            u_history[step] = u_chaser.cpu().numpy()[0]
        
        t += dt

    # 5. 매트랩 파일로 저장
    os.makedirs("matlab", exist_ok=True)
    mat_filename = "matlab/trajectory_test.mat"
    
    save_dict = {
        'times': debris_logger.time_storage[:debris_logger.step].cpu().numpy(),
        'debris_states': debris_logger.get_full_trajectory().cpu().numpy()
    }
    if is_chaser_on:
        save_dict['chaser_states'] = chaser_logger.get_full_trajectory().cpu().numpy()
        save_dict['chaser_inputs'] = u_history[:chaser_logger.step]

    savemat(mat_filename, save_dict)
    print(f"시뮬레이션 완료. 결과가 '{mat_filename}'에 저장되었습니다.")

if __name__ == "__main__":
    main()