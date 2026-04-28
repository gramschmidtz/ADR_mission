# scripts/calculate_RAAN_drift.py
import os
import yaml
import torch
import numpy as np
from tqdm import tqdm
from scipy.io import savemat

from adr_mission.states import KEState
from adr_mission.utils import ke_to_mee
from adr_mission.logger import ObjectTrajectory
from adr_mission.propagation import rk4
from adr_mission.transfer_model import RAANTransfer

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
    
    # KE 초기값
    a_init = torch.tensor([[a]], dtype=torch.float32)
    e_init = torch.tensor([[simulation_cfg['initial_states']['e']]], dtype=torch.float32)
    i_init = torch.tensor([[simulation_cfg['initial_states']['i']]], dtype=torch.float32)
    RAAN_init = torch.tensor([[simulation_cfg['initial_states']['RAAN']]], dtype=torch.float32)
    AOP_init = torch.tensor([[simulation_cfg['initial_states']['AOP']]], dtype=torch.float32)
    nu_init = torch.tensor([[simulation_cfg['initial_states']['nu']]], dtype=torch.float32)
    m_init = torch.tensor([[vehicle_cfg['propulsion']['m_0_kg']]], dtype=torch.float32)
    
    ke_init = KEState(a_init, e_init, i_init, RAAN_init, AOP_init, nu_init, m_init)

    # 3. 시뮬레이션 환경 구성
    device = "cpu"
    dt = simulation_cfg['simulation_time']['dt']
    total_time = simulation_cfg['simulation_time']['total_time']*3600.0
    num_steps = int(total_time//dt)
    batch_size = 1
    
    # RAAN
    RAAN = RAANTransfer(constants_cfg)
    RAAN_logger = ObjectTrajectory(num_steps, batch_size, 1, device)
    u_zero = torch.zeros((batch_size, 3), dtype=torch.float32, device=device)

    t = 0.0
    print("시뮬레이션 시작...")

    # 4. 적분 루프
    for step in tqdm(range(num_steps), desc="Simulating Orbit"):
        RAAN_logger.update(t, ke_init.RAAN)
        ke_init.RAAN = rk4(RAAN.raan_transfer_model, ke_init.RAAN, u_zero, t, dt)
        
        t += dt

    # 5. 매트랩 파일로 저장
    os.makedirs("matlab", exist_ok=True)
    mat_filename = "matlab/raan_trajectory_test.mat"
    
    save_dict = {
        'times': RAAN_logger.time_storage[:RAAN_logger.step].cpu().numpy(),
        'debris_states': RAAN_logger.get_full_trajectory().cpu().numpy()
    }

    savemat(mat_filename, save_dict)
    print(f"시뮬레이션 완료. 결과가 '{mat_filename}'에 저장되었습니다.")

if __name__ == "__main__":
    main()