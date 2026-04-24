# scripts/plot_dynamics_test.py

import os
import yaml
import torch
import math

from adr_mission.states import MEEState
from adr_mission.logger import MissionTrajectory
from adr_mission.propagation import rk4
from adr_mission.dynamics import SpaceObject

def main():
    # 1. 설정 로드 (경로는 실제 디렉토리 구조에 맞게 조정)
    with open("configs/constants.yaml", "r", encoding="utf-8") as f:
        constants_config = yaml.safe_load(f)

    with open("configs/vehicle.yaml", "r", encoding="utf-8") as g:
        vehicle_config = yaml.safe_load(g)
    
    with open("configs/simulation.yaml", "r", encoding="utf-8") as h:
        simulation_config = yaml.safe_load(h)

    constants_cfg = constants_config
    vehicle_cfg = vehicle_config
    simulation_cfg = simulation_config

    # 2. 초기 궤도 상태 설정 (고도 500km 원형 궤도, 적도면)
    R_e = constants_cfg['physics']['R_e_m']
    alt0 = simulation_cfg['initial_states']['alt0_km'] *1000.0
    a = R_e + alt0
    
    # MEE 초기값 (배치 사이즈 1)
    p_init = torch.tensor([[a]], dtype=torch.float32)
    f_init = torch.tensor([[simulation_cfg['initial_states']['f']]], dtype=torch.float32)
    g_init = torch.tensor([[simulation_cfg['initial_states']['g']]], dtype=torch.float32)
    h_init = torch.tensor([[simulation_cfg['initial_states']['h']]], dtype=torch.float32)
    k_init = torch.tensor([[simulation_cfg['initial_states']['k']]], dtype=torch.float32)
    L_init = torch.tensor([[simulation_cfg['initial_states']['L']]], dtype=torch.float32)
    m_init = torch.tensor([[vehicle_cfg['propulsion']['m_0_kg']]], dtype=torch.float32)

    initial_state = MEEState(p=p_init, f=f_init, g=g_init, h=h_init, k=k_init, L=L_init, mass=m_init)

    # 3. 객체 초기화
    device = "cpu"
    chaser = SpaceObject(vehicle_cfg, constants_cfg)
    
    # 시뮬레이션 파라미터 (약 1주기 반 정도 시뮬레이션)
    dt = simulation_cfg['simulation_time']['dt']
    total_time = simulation_cfg['simulation_time']['total_time']*3600.0
    num_steps = int(total_time//dt)
    batch_size = 1

    logger = MissionTrajectory(num_steps, batch_size, device)
    
    current_state = initial_state
    t = 0.0
    
    # 제어 입력 (추력 없음) - [Batch, 3] 형태
    u_zero = torch.zeros((batch_size, 3), dtype=torch.float32, device=device)

    print("시뮬레이션 시작...")
    # 4. 적분 루프
    for step in range(num_steps):
        logger.update(t, current_state)
        current_state = rk4(chaser.dynamics_model, current_state, u_zero, t, dt)
        t += dt

    # 5. 매트랩 파일로 저장
    os.makedirs("matlab", exist_ok=True)
    mat_filename = "matlab/trajectory_test.mat"
    
    # logger 내부 저장 데이터 추출 및 저장 형태 변환 (옵션)
    from scipy.io import savemat
    savemat(mat_filename, {
        'states': logger.get_full_trajectory().cpu().numpy(),
        'times': logger.time_storage[:logger.step].cpu().numpy()
    })
    
    print(f"시뮬레이션 완료. 결과가 '{mat_filename}'에 저장되었습니다.")

if __name__ == "__main__":
    main()