# scripts/make_random_debris.py
import yaml
import random
import os

def generate_debris_config(num_debris=5000, output_path='configs/debris.yaml'):
    # 출력 디렉토리가 없으면 생성
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    debris_data = {}

    for i in range(1, num_debris + 1):
        # debris 이름 생성 (debris0001, debris0002, ...)
        name = f"debris{i:04d}"
        
        # 랜덤 및 고정 파라미터 설정
        debris_data[name] = {
            'alt0_km': round(random.uniform(500, 1500), 2),  # 고도 500~1500km
            'e': 0.0,                                        # 이심률 고정
            'i': 87.9,                                       # 궤도 경사각 고정
            'RAAN': round(random.uniform(0, 360), 2),        # 승교점 적경 0~360도
            'AOP': 0.0,                                      # 근지점 인수 고정
            'nu': round(random.uniform(0, 360), 2),          # 진근점 이각 0~360도
            'mass': round(random.uniform(100, 300), 2)       # 질량 100~300kg
        }

    # YAML 파일로 저장
    with open(output_path, 'w', encoding='utf-8') as f:
        # sort_keys=False를 통해 생성 순서대로 저장 (debris0001부터 차례대로)
        yaml.dump(debris_data, f, default_flow_style=False, sort_keys=False)
    
    print(f"Successfully generated {num_debris} debris entries in {output_path}")

if __name__ == "__main__":
    generate_debris_config()