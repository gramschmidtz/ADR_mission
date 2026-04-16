from adr_mission.orbit.eom_mee import MEEDynamicsConfig, MEEState, mee_kinematics_matrix


def test_h_k_rows_match_paper():
    mee = MEEState(p_km=7000.0, f=0.01, g=0.02, h=0.1, k=0.2, L_rad=0.3, m_kg=400.0)
    cfg = MEEDynamicsConfig()
    A = mee_kinematics_matrix(mee, cfg)
    assert A[3][2] != A[4][2]  # sin/cos rows must be distinct
