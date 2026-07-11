"""全シナリオ共通の SSO 500km 軌道初期条件。"""

from common.types import R_EARTH

SSO_ALTITUDE_M = 500.0e3
SSO_A_M = R_EARTH + SSO_ALTITUDE_M
SSO_E = 0.0
SSO_I_DEG = 97.4
SSO_RAAN_DEG = 196.0
SSO_NU0_DEG = 0.0

ORBITAL_PERIOD_S = 5700.0  # ≈ 2*pi*sqrt(a^3/mu)、シナリオ長の目安計算用
