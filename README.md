# ADR_mission README

## 1. 프로젝트 개요

`ADR_mission`은 **저추력 기반 다중 우주 파편 제거(ADR, Active Debris Removal) 미션**을 단계적으로 구현하기 위한 레포지토리다.  
최종적으로는 논문 수준의 전체 파이프라인, 즉

- spacecraft / debris dynamics
- 단일 transfer model
- RAAN-phasing logic
- transfer cost / time 계산
- 데이터셋 생성
- surrogate model
- sequence search
- 미션 수준 검증 및 시각화

까지 포함하는 구조를 목표로 한다.

즉, 이 레포지토리의 **최종 목표는 “논문 전체 구현”** 이다.
다만 구현은 단계적으로 진행하며, 현재 시점의 **subgoal**은 다음과 같다.

> **초기 상태와 입력 프로필이 주어졌을 때, 파이썬에서 dynamics를 적분하고, 그 결과를 MATLAB에서 시각화하여 spacecraft가 지구를 공전하는 모습을 확인한다.**

중요한 점은, 이 subgoal이 전체 구조와 분리된 임시 목적이 아니라는 것이다.  
오히려 이 단계는 이후의 모든 구현을 떠받치는 가장 기초적인 기반이다.

- transfer model도 결국 dynamics propagation 위에 세워진다.
- phasing logic도 결국 orbital state evolution 위에 세워진다.
- dataset generation도 결국 leg cost evaluator가 필요하다.
- surrogate model도 truth dynamics/transfer solver가 있어야 학습 가능하다.
- sequence search도 결국 각 leg의 cost/time estimator를 호출한다.

따라서 현재는 “다이내믹스 구현 + MATLAB 시각화”가 먼저지만,  
레포지토리 구조는 처음부터 **논문 전체 구현을 감당할 수 있는 형태**로 설계한다.

---

## 2. 구현 로드맵: 최종 목표와 subgoal의 관계

이 프로젝트는 아래와 같은 계층적 목표 구조를 가진다.

### 최종 목표
논문 전체 구현:

1. dynamics of the system
2. transfer model
3. RAAN-phasing orbit selection
4. training database generation
5. ANN surrogate
6. sequence search
7. mission-level validation and visualization

### 현재 subgoal
그 최종 목표를 위한 1단계 기반 구축:

1. spacecraft state 정의
2. dynamics equation 구현
3. control input profile 정의
4. numerical propagation 수행
5. propagation result 저장
6. MATLAB 시각화

즉, subgoal은 별도의 축소 프로젝트가 아니라  
**최종 목표를 구성하는 첫 번째 레이어**다.

---

## 3. 설계 철학

## 3.1 구조는 최종 목표를 기준으로 설계하고, 구현은 단계적으로 진행한다
이 프로젝트에서 가장 중요한 원칙은 다음이다.

> **“지금 필요한 것만 구현하되, 구조는 나중에 필요한 것까지 감당할 수 있게 만든다.”**

즉,

- 지금 당장 필요하다는 이유로 dynamics를 script 하나에 몰아 넣지 않는다.
- MATLAB 시각화가 당장 중요하다고 해서 레포지토리를 MATLAB 중심 구조로 재편하지 않는다.
- 아직 ANN/search를 안 만든다고 해서 그 자리를 없애지 않는다.

현재는 subgoal 중심으로 구현하지만,
레포지토리 구조는 계속해서 아래 확장을 자연스럽게 받을 수 있어야 한다.

- Cartesian → MEE 확장
- unforced orbit → low-thrust transfer 확장
- single propagation → transfer leg evaluation 확장
- single case visualization → batch dataset generation 확장
- truth model → surrogate model 확장
- single leg → multiple debris sequence search 확장

---

## 3.2 Python은 계산 엔진, MATLAB은 시각화 계층이다
두 언어의 역할은 명확히 나눈다.

### Python
- dynamics 구현
- 수치 적분
- transfer evaluation
- 데이터셋 생성
- surrogate 학습/추론
- sequence search
- 결과 export

### MATLAB
- 궤도 시각화
- 3D orbit animation
- state/control history plot
- 시각적 검증

즉:

> **Python이 source of truth이고, MATLAB은 visualization layer다.**

이 원칙은 subgoal 단계에서도 중요하고, 최종 목표 단계에서도 그대로 유지된다.

---

## 3.3 truth model과 approximation/search layer를 분리한다
최종적으로는 ANN surrogate와 sequence search가 들어가지만,  
그 전에 항상 **truth model**이 먼저 있어야 한다.

구조적으로는 아래 순서를 따른다.

1. dynamics truth model
2. transfer truth model
3. dataset generation
4. surrogate model
5. search
6. truth-based post validation

이 순서가 유지되면 나중에 어떤 모델을 바꿔도 전체 구조가 흔들리지 않는다.

---

## 3.4 “단일 궤도 시각화”도 미래 확장을 염두에 두고 만든다
현재 subgoal은 spacecraft가 지구를 도는 모습을 MATLAB에서 보는 것이다.  
하지만 이 결과 포맷과 코드 구조는 나중에 아래에도 그대로 쓰일 수 있어야 한다.

- deorbit transfer visualization
- phasing orbit visualization
- multiple-leg mission timeline visualization
- selected sequence animation

따라서 지금 만드는 export schema와 state history 구조는  
나중에 확장 가능한 형태여야 한다.

---

## 4. 권장 레포지토리 구조

```text
ADR_mission/
├─ README.md
├─ pyproject.toml
├─ requirements.txt
│
├─ configs/
│  ├─ environment.yaml
│  ├─ vehicle.yaml
│  ├─ simulation.yaml
│  ├─ transfer.yaml
│  ├─ training.yaml
│  └─ search.yaml
│
├─ src/
│  └─ adr_mission/
│     ├─ __init__.py
│     ├─ constants.py
│     ├─ types.py
│     │
│     ├─ utils/
│     │  ├─ units.py
│     │  ├─ math_utils.py
│     │  ├─ time_utils.py
│     │  ├─ angle_utils.py
│     │  └─ io_utils.py
│     │
│     ├─ orbit/
│     │  ├─ state_definitions.py
│     │  ├─ frames.py
│     │  ├─ conversions.py
│     │  ├─ gravity.py
│     │  ├─ drag.py
│     │  ├─ thrust.py
│     │  ├─ eom_cartesian.py
│     │  ├─ eom_mee.py
│     │  ├─ perturbations.py
│     │  └─ derived_quantities.py
│     │
│     ├─ propagation/
│     │  ├─ integrators.py
│     │  ├─ propagator.py
│     │  ├─ simulation_runner.py
│     │  └─ stop_conditions.py
│     │
│     ├─ scenario/
│     │  ├─ initial_conditions.py
│     │  ├─ orbit_factory.py
│     │  ├─ spacecraft_factory.py
│     │  ├─ control_profiles.py
│     │  └─ scenario_builder.py
│     │
│     ├─ mission/
│     │  ├─ transfer/
│     │  │  ├─ disposal.py
│     │  │  ├─ rendezvous.py
│     │  │  ├─ transfer_model.py
│     │  │  └─ leg_result.py
│     │  │
│     │  ├─ phasing/
│     │  │  ├─ raan_rate.py
│     │  │  ├─ phasing_time.py
│     │  │  ├─ altitude_selector.py
│     │  │  └─ objective.py
│     │  │
│     │  ├─ sequence/
│     │  │  ├─ sequence_state.py
│     │  │  ├─ evaluator.py
│     │  │  ├─ pruning.py
│     │  │  └─ tree_search.py
│     │  │
│     │  └─ validation/
│     │     ├─ truth_recheck.py
│     │     └─ mission_summary.py
│     │
│     ├─ data/
│     │  ├─ debris_generator.py
│     │  ├─ dataset_builder.py
│     │  ├─ preprocess.py
│     │  └─ io.py
│     │
│     ├─ ml/
│     │  ├─ model.py
│     │  ├─ trainer.py
│     │  ├─ inference.py
│     │  ├─ metrics.py
│     │  └─ scaler.py
│     │
│     ├─ export/
│     │  ├─ result_schema.py
│     │  ├─ npz_exporter.py
│     │  ├─ csv_exporter.py
│     │  ├─ mat_exporter.py
│     │  └─ metadata_exporter.py
│     │
│     └─ analysis/
│        ├─ sanity_checks.py
│        ├─ diagnostics.py
│        ├─ quicklook_plots.py
│        └─ compare_truth_vs_surrogate.py
│
├─ scripts/
│  ├─ run_simulation.py
│  ├─ run_transfer_case.py
│  ├─ build_dataset.py
│  ├─ train_surrogate.py
│  ├─ search_sequence.py
│  ├─ export_for_matlab.py
│  └─ run_test_case.py
│
├─ matlab/
│  ├─ README_matlab.md
│  ├─ load_result.m
│  ├─ plot_earth_orbit.m
│  ├─ animate_orbit_3d.m
│  ├─ plot_state_history.m
│  ├─ plot_control_history.m
│  ├─ plot_transfer_timeline.m
│  ├─ plot_sequence_summary.m
│  ├─ helpers/
│  │  ├─ draw_earth.m
│  │  ├─ axis_equal_3d.m
│  │  ├─ load_metadata.m
│  │  └─ wrap_angle_deg.m
│  └─ examples/
│     ├─ demo_single_orbit.m
│     ├─ demo_transfer_case.m
│     └─ demo_sequence_case.m
│
├─ outputs/
│  ├─ runs/
│  │  └─ <timestamp_run_id>/
│  │     ├─ states.mat
│  │     ├─ states.npz
│  │     ├─ states.csv
│  │     ├─ metadata.json
│  │     └─ quicklook.png
│  ├─ datasets/
│  └─ models/
│
└─ tests/
   ├─ test_units.py
   ├─ test_gravity.py
   ├─ test_drag.py
   ├─ test_thrust.py
   ├─ test_eom_cartesian.py
   ├─ test_eom_mee.py
   ├─ test_propagator.py
   ├─ test_transfer_model.py
   ├─ test_phasing.py
   ├─ test_export_schema.py
   ├─ test_dataset_builder.py
   ├─ test_surrogate_io.py
   └─ test_sequence_search.py
```

---

## 5. 이 구조를 이렇게 두는 이유

이 구조는 **최종 목표를 기준으로 만든 구조**다.  
다만 그 안에서 현재 subgoal이 가장 먼저 구현되는 것이다.

### 지금 바로 핵심이 되는 축
- `orbit/`
- `propagation/`
- `scenario/`
- `export/`
- `matlab/`

### 이후 점점 중요해지는 축
- `mission/transfer/`
- `mission/phasing/`
- `data/`
- `ml/`
- `mission/sequence/`

즉, 현재는 앞쪽 폴더들이 먼저 채워지고,  
프로젝트가 진행될수록 뒤쪽 폴더들이 자연스럽게 채워진다.

이렇게 해야
- 지금 필요한 구현도 편하고
- 나중에 구조를 다시 갈아엎지 않아도 된다.

---

## 6. 모듈별 역할

## 6.1 `orbit/`
프로젝트 전체의 물리적 기초를 담당한다.

### 핵심 역할
- spacecraft/debris state 정의
- 궤도 상태 표현
- 좌표계 변환
- 중력/drag/thrust 모델
- EOM 정의
- 파생량 계산

### 현재 subgoal과의 관계
현재 가장 먼저 구현할 부분이 바로 여기다.  
특히 첫 번째 working milestone에서는 아래가 중요하다.

- `state_definitions.py`
- `gravity.py`
- `thrust.py`
- `eom_cartesian.py`
- `derived_quantities.py`

### 향후 확장
나중에는 여기에
- `eom_mee.py`
- more accurate perturbation model
- transfer-friendly state representation
이 자연스럽게 추가된다.

---

## 6.2 `propagation/`
주어진 dynamics를 실제로 적분하는 레이어다.

### 역할
- integrator 관리
- EOM 호출
- stop condition 처리
- state history 생성

### 현재 subgoal과의 관계
현재 목표인 “초기 상태와 입력 프로필이 주어졌을 때 SC가 지구를 도는 모습 계산”은  
사실상 이 레이어와 `orbit/` 레이어가 핵심이다.

### 향후 확장
나중에는 transfer leg나 long-duration propagation에도 같은 구조를 그대로 쓴다.

---

## 6.3 `scenario/`
실험 케이스 구성 레이어

### 역할
- 초기 궤도 생성
- spacecraft 파라미터 생성
- control profile 생성
- simulation case 조립

### 현재 subgoal과의 관계
지금은 single spacecraft propagation case를 만드는 데 쓰인다.

### 향후 확장
나중에는
- departure debris orbit
- arrival debris orbit
- disposal orbit
- phasing orbit
같은 case builder로 확장 가능하다.

---

## 6.4 `mission/transfer/`
단일 transfer leg 평가 레이어

### 역할
- debris departure orbit에서 disposal orbit으로 이동
- disposal orbit에서 phasing orbit으로 이동
- phasing 후 arrival debris orbit으로 이동
- 각 단계의 cost / time 계산

### 현재 subgoal과의 관계
지금 당장은 full transfer solver까지 갈 필요가 없을 수 있다.  
하지만 현재 만드는 propagation 구조가 결국 여기서 재사용된다.

즉 지금 하는 일은, 미래의 `transfer_model.py`를 위한 기반 작업이다.

---

## 6.5 `mission/phasing/`
RAAN phasing 관련 로직

### 역할
- RAAN drift rate 계산
- phasing time 계산
- phasing altitude selection
- weighted objective 계산

### 현재 subgoal과의 관계
현재는 직접 구현 우선순위가 가장 높지는 않다.  
하지만 추후 논문 구현을 위해 별도 레이어로 미리 자리를 분리한다.

---

## 6.6 `data/`
truth model 기반 데이터셋 생성

### 역할
- debris candidate population 생성
- input/output pair 생성
- 학습용 데이터 저장

### 현재 subgoal과의 관계
아직 나중 단계지만, export schema와 result structure는 지금부터 일관되게 유지해야 한다.  
그래야 single propagation 결과와 transfer 결과가 같은 철학으로 저장된다.

---

## 6.7 `ml/`
surrogate model 관련 레이어

### 역할
- model 정의
- 학습
- 추론
- scaling
- 평가

### 현재 subgoal과의 관계
아직 직접 구현 단계는 아니지만, future consumer가 존재한다는 사실을 고려해  
truth model 출력 포맷을 설계해야 한다.

---

## 6.8 `mission/sequence/`
sequence search 레이어

### 역할
- state expansion
- evaluator 호출
- pruning
- best sequence selection

### 현재 subgoal과의 관계
지금 당장 구현 대상은 아니지만, 최종 목표의 중요한 일부이므로 구조 안에 남겨 둔다.

---

## 6.9 `export/`
결과 저장과 MATLAB/외부 툴 연동

### 역할
- Python 결과 저장
- MATLAB 친화적 포맷 export
- metadata 저장

### 현재 subgoal과의 관계
지금 단계에서 아주 중요하다.  
현재는 single orbit propagation 결과를 MATLAB에서 볼 수 있어야 한다.

### 향후 확장
나중에는
- transfer segment export
- sequence summary export
- surrogate vs truth comparison export
까지 동일한 schema 철학으로 확장한다.

---

## 6.10 `matlab/`
시각화 계층

### 역할
- single orbit 3D visualization
- transfer orbit visualization
- state/control time history plot
- sequence summary plot
- animation

### 현재 subgoal과의 관계
지금은 single orbit visualization부터 구현한다.

### 향후 확장
나중에는
- transfer timeline
- multiple-leg sequence visualization
- mission summary plot
까지 확장 가능하다.

즉 MATLAB 폴더도 “지금만 위한 임시 폴더”가 아니라,  
프로젝트 전체의 visualization layer다.

---

## 7. 현재 subgoal을 어떻게 전체 구조 안에서 달성하는가

현재 가장 먼저 구현할 것은 아래 흐름이다.

1. `scenario/`에서 초기 조건 생성
2. `orbit/`에서 dynamics 정의
3. `propagation/`에서 state history 적분
4. `export/`에서 `.mat` 및 metadata 저장
5. `matlab/`에서 시각화

이 흐름은 이후에도 그대로 재사용된다.

### 예시
- 지금: single orbit case
- 다음: low-thrust single transfer case
- 이후: phasing case
- 최종: sequence result visualization

즉, subgoal은 구조 밖의 임시 작업이 아니라  
전체 파이프라인의 첫 번째 예시 실행이다.

---

## 8. 현재 단계에서의 권장 구현 우선순위

## 단계 1: 최소 동작 dynamics + propagation + MATLAB visualization
- Cartesian state 정의
- central gravity
- zero thrust / constant thrust profile
- propagation
- `.mat` export
- MATLAB 3D orbit plot

## 단계 2: 파생량 및 진단
- altitude
- speed
- mass history
- quick sanity check
- Python quicklook + MATLAB state plots

## 단계 3: perturbation 및 richer control
- drag
- J2
- piecewise control profile
- duty cycle

## 단계 4: transfer로 확장
- disposal orbit transfer
- phasing orbit stay
- arrival orbit matching

## 단계 5: 논문 전체 확장
- training dataset
- ANN surrogate
- sequence search
- mission-level comparison

이 순서를 따르면 subgoal이 자연스럽게 최종 목표의 일부로 흡수된다.

---

## 9. 현재 단계에서의 기술적 권장사항

## 9.1 첫 working EOM은 Cartesian으로 구현한다
장기적으로 MEE가 필요할 수 있지만, 첫 propagation 및 MATLAB visualization에는 Cartesian이 가장 직관적이다.

권장 상태:
```text
x = [r_x, r_y, r_z, v_x, v_y, v_z, m]
```

이 표현은
- 디버깅이 쉽고
- 3D 시각화와 직결되며
- MATLAB plotting도 단순하다.

그 뒤에 MEE를 추가하더라도 구조는 유지된다.

---

## 9.2 control profile은 독립 모듈로 둔다
현재 subgoal의 핵심 문장은 “초기 상태와 입력 프로필이 주어졌을 때”이다.  
따라서 입력 프로필은 반드시 독립 계층이어야 한다.

권장 형태:
```python
u = control_profile(t, x, params)
```

이 구조가 있어야
- 무추력
- 일정 추력
- piecewise on/off
- 향후 최적제어 기반 입력
을 모두 자연스럽게 수용할 수 있다.

---

## 9.3 export schema는 초반부터 고정 철학을 가진다
지금은 single orbit case만 저장하더라도, 결과 포맷은 미래 확장을 고려해야 한다.

권장 필드 예:
- `time_s`
- `state`
- `control`
- `position_km`
- `velocity_km_s`
- `mass_kg`
- `altitude_km`
- `labels`
- `metadata`

이 형식이면 나중에 transfer / sequence 결과도 일관되게 담기 쉽다.

---

## 9.4 MATLAB은 계산을 재구현하지 않는다
MATLAB은 시각화 전용 계층이다.

즉 MATLAB에서는
- dynamics 재구현 금지
- propagation 재실행 금지
- truth model 계산 금지

MATLAB은 Python 결과를 읽고, 그리고, 애니메이션하는 역할만 한다.

---

## 10. 코드 컨벤션

## 10.1 레포지토리 이름과 패키지 이름
- repository: `ADR_mission`
- python package: `adr_mission`

---

## 10.2 docstring 언어
**docstring은 한국어도 허용한다.**

다만 아래는 반드시 포함한다.
- 입력 의미
- 출력 의미
- 단위
- shape
- 중요한 가정

예:

```python
def two_body_eom(t: float, x: np.ndarray, params: dict) -> np.ndarray:
    """
    2체 문제 기반 spacecraft dynamics를 계산한다.

    Args:
        t: 현재 시간 [s]
        x: 상태벡터 [rx, ry, rz, vx, vy, vz, m]
        params: 환경 및 추력 파라미터

    Returns:
        상태 미분값 [drx, dry, drz, dvx, dvy, dvz, dm]

    Notes:
        - 위치 단위는 km
        - 속도 단위는 km/s
        - 질량 단위는 kg
    """
```

---

## 10.3 네이밍 규칙
- 파일명: snake_case
- 함수명: snake_case
- 클래스명: PascalCase
- 상수명: UPPER_SNAKE_CASE
- 단위 포함 변수명 적극 사용

예:
- `position_km`
- `velocity_km_s`
- `time_s`
- `mass_kg`
- `raan_deg`

---

## 10.4 단위 규칙
내부 기본 단위는 아래를 권장한다.
- 거리: km
- 시간: s
- 질량: kg
- 각도: rad 내부 처리, 필요 시 입출력만 deg

이 규칙은 지금 subgoal에도 중요하고,  
나중에 transfer / phasing / surrogate 단계에서도 매우 중요하다.

---

## 11. 테스트 전략

## 현재 단계에서 먼저 필요한 테스트
- 무추력 2체 문제에서 원궤도 유지
- 에너지 보존 sanity check
- thrust on 시 질량 감소 확인
- exporter가 `.mat` 파일을 정상 생성하는지 확인
- MATLAB loader가 결과를 정상 읽는지 확인

## 이후 추가될 테스트
- drag / J2 개별 모델 검증
- transfer leg consistency
- phasing time sanity check
- dataset schema consistency
- surrogate input/output compatibility
- sequence search regression test

즉 테스트도 subgoal만 위한 것이 아니라,  
현재 테스트 위에 미래 테스트가 자연스럽게 쌓이는 형태로 간다.

---

## 12. 협업 시 반드시 지킬 원칙

1. 구조는 최종 목표를 기준으로 유지한다.
2. 현재는 subgoal을 먼저 구현하되, 임시 구조를 만들지 않는다.
3. Python은 계산 엔진, MATLAB은 시각화 계층이다.
4. control profile과 dynamics는 분리한다.
5. export schema는 자주 깨지지 않도록 관리한다.
6. truth model이 먼저고, surrogate/search는 그 위에 온다.

---

## 13. 최종 정리

`ADR_mission`의 최종 목표는 **논문 전체 구현**이다.  
현재의 “다이내믹스를 구현해서 MATLAB에서 시각화”하는 작업은  
그 목표와 별개인 축소 프로젝트가 아니라, 바로 그 최종 목표를 위한 첫 번째 subgoal이다.

따라서 이 레포지토리는

- 지금 당장 필요한 single-orbit propagation도 수행할 수 있어야 하고,
- 이후 transfer / phasing / surrogate / sequence search까지 자연스럽게 확장 가능해야 한다.

한 문장으로 요약하면 다음과 같다.

> **구조는 논문 전체 구현을 목표로 설계하고, 현재는 그 구조 안에서 dynamics propagation + MATLAB visualization subgoal을 먼저 달성한다.**
