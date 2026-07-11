"""FSW 単体テスト（assert ベース、pytest 不使用）。

.venv/bin/python tests/test_fsw_basic.py で実行。
sim/ は import しない。合成 SensorData で FSW 単体を駆動する。
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from common.types import (
    FSW_DT,
    GroundCommand,
    Mode,
    SensorData,
)
from fsw.adcs.controller import bdot_moment, pd_control
from fsw.main import FlightSoftware
from fsw.quat_utils import (
    quat_error,
    quat_mult,
    quat_normalize,
    quat_propagate,
    quat_to_rot,
)


def make_sensors(
    t: float,
    gyro=None,
    mag=None,
    sun=None,
    pos=None,
    vel=None,
    wheel_speeds=None,
    battery_voltage=7.4,
    battery_soc=0.8,
    solar_power=2.0,
    battery_temp=20.0,
    gs_visible=False,
) -> SensorData:
    """テスト用の合成 SensorData を組み立てるヘルパー。"""
    return SensorData(
        t=t,
        gyro=np.zeros(3) if gyro is None else np.asarray(gyro, dtype=float),
        mag=np.array([2.0e-5, 0.0, 1.0e-5]) if mag is None else np.asarray(mag, dtype=float),
        sun=sun,
        gps_pos_eci=np.array([6.871e6, 0.0, 0.0]) if pos is None else np.asarray(pos, dtype=float),
        gps_vel_eci=np.array([0.0, 7.6e3, 0.0]) if vel is None else np.asarray(vel, dtype=float),
        wheel_speeds=np.zeros(3) if wheel_speeds is None else np.asarray(wheel_speeds, dtype=float),
        battery_voltage=battery_voltage,
        battery_soc=battery_soc,
        solar_power=solar_power,
        battery_temp=battery_temp,
        gs_visible=gs_visible,
    )


# ---------------------------------------------------------------------
# 1. 起動 SAFE → 60s 相当 step 後 DETUMBLE へ遷移
# ---------------------------------------------------------------------
def test_boot_safe_to_detumble():
    fsw = FlightSoftware()
    assert fsw.get_telemetry().mode == Mode.SAFE or fsw.mode_mgr.mode == Mode.SAFE

    t = 0.0
    dt = FSW_DT
    n_steps = int(65.0 / dt)  # 65s ぶん進める（60s 超過を確認）
    for _ in range(n_steps):
        sensors = make_sensors(t, gyro=np.zeros(3))
        fsw.step(sensors, dt)
        t += dt

    tlm = fsw.get_telemetry()
    assert tlm.mode == Mode.DETUMBLE, f"expected DETUMBLE, got {tlm.mode}"
    print("test_boot_safe_to_detumble: PASS")


# ---------------------------------------------------------------------
# 2. 回転行列・クォータニオン演算の数学テスト
# ---------------------------------------------------------------------
def test_quat_math():
    # 恒等クォータニオンで R = I
    q_id = np.array([0.0, 0.0, 0.0, 1.0])
    r_id = quat_to_rot(q_id)
    assert np.allclose(r_id, np.eye(3), atol=1e-10), r_id

    # 90度 Z 軸回転: q = [0,0,sin(45deg),cos(45deg)]
    angle = np.pi / 2.0
    axis = np.array([0.0, 0.0, 1.0])
    q_z90 = quat_normalize(np.array([
        axis[0] * np.sin(angle / 2),
        axis[1] * np.sin(angle / 2),
        axis[2] * np.sin(angle / 2),
        np.cos(angle / 2),
    ]))
    r_z90 = quat_to_rot(q_z90)
    # ECI→body 回転で x_eci=[1,0,0] は body 系で [0,-1,0] になるはず
    # (body 座標系が ECI に対し +90度 Z 回転しているとき、ECI の x 軸は
    #  body から見ると -Y 方向)
    v_eci = np.array([1.0, 0.0, 0.0])
    v_body = r_z90 @ v_eci
    expected = np.array([0.0, -1.0, 0.0])
    assert np.allclose(v_body, expected, atol=1e-8), v_body

    # 回転行列は直交行列（R^T R = I, det=1）
    assert np.allclose(r_z90.T @ r_z90, np.eye(3), atol=1e-10)
    assert np.isclose(np.linalg.det(r_z90), 1.0, atol=1e-10)

    # クォータニオン伝播: ゼロ角速度で q は不変
    q_prop = quat_propagate(q_id, np.zeros(3), 0.1)
    assert np.allclose(q_prop, q_id, atol=1e-12)

    # 一定角速度伝播が回転を積分する方向に整合しているか（小角度近似）
    omega = np.array([0.0, 0.0, 0.1])  # rad/s
    dt = 0.01
    q_next = quat_propagate(q_id, omega, dt)
    # 微小角なので q_next のベクトル部 z 成分 ≈ omega_z*dt/2
    assert np.isclose(q_next[2], omega[2] * dt / 2.0, atol=1e-4)

    # quat_error: 誤差が小さいとき q_err はほぼ恒等
    q_err = quat_error(q_id, q_id)
    assert np.allclose(q_err, q_id, atol=1e-10)

    print("test_quat_math: PASS")


# ---------------------------------------------------------------------
# 3. B-dot が回転を減衰させる向きのモーメントを出す（符号テスト）
# ---------------------------------------------------------------------
def test_bdot_sign():
    # 磁場が増加している（dB/dt > 0）とき、m は負の符号（逆向き）になるべき
    mag_prev = np.array([1.0e-5, 0.0, 0.0])
    mag_now = np.array([2.0e-5, 0.0, 0.0])
    dt = 0.1
    m = bdot_moment(mag_prev, mag_now, dt, k_bdot=1.0e4)
    # dB/dt > 0 (x成分) → m = -k*dB/dt は x成分が負
    assert m[0] < 0, f"expected negative x moment, got {m}"

    # 逆に減少しているときは正
    m2 = bdot_moment(mag_now, mag_prev, dt, k_bdot=1.0e4)
    assert m2[0] > 0, f"expected positive x moment, got {m2}"

    # B-dot トルク tau = m x B が角運動量を減衰させる向きであることを確認。
    # m は概ね -dB/dt 方向。角速度 omega による見かけの B 変化は
    # dB/dt ≈ -omega x B（body 固定センサでの回転磁場）なので
    # m ≈ k*(omega x B)、トルク tau = m x B = k*(omega x B) x B は
    # 一般に omega の B 直交成分を減衰させる向きに働く。
    # ここでは単純に「回転により生じる dB/dt に対し逆向きのモーメントが出る」
    # という符号関係のみを確認する（上記で検証済み）。
    print("test_bdot_sign: PASS")


# ---------------------------------------------------------------------
# 4. PD 制御が誤差を減らす向きのトルクを出す
# ---------------------------------------------------------------------
def test_pd_control_direction():
    q_target = np.array([0.0, 0.0, 0.0, 1.0])  # 恒等
    # 現在姿勢は Z 軸まわりに正の小角度だけ回転している
    small_angle = np.radians(5.0)
    q_est = quat_normalize(np.array([0.0, 0.0, np.sin(small_angle / 2), np.cos(small_angle / 2)]))
    omega_est = np.zeros(3)

    tau = pd_control(q_target, q_est, omega_est, kp=4e-5, kd=4e-4)

    # q_err = q_target * q_est^-1 のベクトル部は -z 方向（q_est を打ち消す向き）
    # tau_z は q_err_vec と逆符号（tau = -Kp*q_err_vec）で、q_est が +z 方向に
    # 回転しているので誤差を打ち消すには tau_z < 0 であるべき。
    assert tau[2] < 0, f"expected negative z torque to correct +z rotation, got {tau}"

    # 角速度誤差のみのケース: omega_est > 0 なら Kd 項がそれを減衰する(負の)トルクを出す
    q_est2 = np.array([0.0, 0.0, 0.0, 1.0])
    omega_est2 = np.array([0.0, 0.0, 0.05])  # rad/s, 正の z 回転
    tau2 = pd_control(q_target, q_est2, omega_est2, kp=4e-5, kd=4e-4)
    assert tau2[2] < 0, f"expected damping torque opposing +z rate, got {tau2}"

    print("test_pd_control_direction: PASS")


# ---------------------------------------------------------------------
# 5. SoC 0.2 で SAFE へ遷移、tx_active/camera_trigger が False
# ---------------------------------------------------------------------
def test_low_soc_safe():
    fsw = FlightSoftware()
    t = 0.0
    dt = FSW_DT

    # まず SUNPOINT 相当まで進めておく（DETUMBLE 経由でモード遷移させる）
    for _ in range(int(130.0 / dt)):
        sensors = make_sensors(t, gyro=np.zeros(3), battery_soc=0.8)
        fsw.step(sensors, dt)
        t += dt

    tlm_before = fsw.get_telemetry()
    assert tlm_before.mode in (Mode.SUNPOINT, Mode.DETUMBLE), tlm_before.mode

    # 低 SoC を投入
    cmd = None
    for _ in range(20):
        sensors = make_sensors(t, gyro=np.zeros(3), battery_soc=0.2)
        cmd = fsw.step(sensors, dt)
        t += dt

    tlm = fsw.get_telemetry()
    assert tlm.mode == Mode.SAFE, f"expected SAFE at low SoC, got {tlm.mode}"
    assert cmd.tx_active is False
    assert cmd.camera_trigger is False
    print("test_low_soc_safe: PASS")


# ---------------------------------------------------------------------
# 6. GroundCommand("IMAGE_NADIR", {"count": 2}) 受理で NADIR_IMAGING へ
# ---------------------------------------------------------------------
def test_image_nadir_command():
    fsw = FlightSoftware()
    cmd = GroundCommand(type="IMAGE_NADIR", params={"count": 2})
    fsw.handle_command(cmd)

    assert fsw.mode_mgr.mode == Mode.NADIR_IMAGING, fsw.mode_mgr.mode
    assert fsw.payload.images_requested == 2

    # 1 ステップ実行してテレメトリにも反映されることを確認
    sensors = make_sensors(0.0)
    fsw.step(sensors, FSW_DT)
    tlm = fsw.get_telemetry()
    assert tlm.mode == Mode.NADIR_IMAGING, tlm.mode
    assert tlm.last_cmd == "IMAGE_NADIR"
    print("test_image_nadir_command: PASS")


# ---------------------------------------------------------------------
# 追加: フォルト処理（NaN センサ値）で例外を漏らさず SAFE へ
# ---------------------------------------------------------------------
def test_nan_sensor_fault_handling():
    fsw = FlightSoftware()
    sensors = make_sensors(0.0, gyro=np.array([np.nan, 0.0, 0.0]))
    cmd = fsw.step(sensors, FSW_DT)  # 例外を投げないこと
    tlm = fsw.get_telemetry()
    assert tlm.mode == Mode.SAFE
    assert tlm.fault_flags != 0
    assert cmd is not None
    print("test_nan_sensor_fault_handling: PASS")


# ---------------------------------------------------------------------
# 追加: 撮像シーケンス完了で SUNPOINT へ自動復帰
# ---------------------------------------------------------------------
def test_imaging_sequence_completes_and_returns_to_sunpoint():
    fsw = FlightSoftware()
    cmd = GroundCommand(type="IMAGE_NADIR", params={"count": 1})
    fsw.handle_command(cmd)
    assert fsw.mode_mgr.mode == Mode.NADIR_IMAGING

    # 簡易剛体動力学でクローズドループを回す（FSW が出す wheel_torque を
    # 実際に姿勢へ反映させないと pointing_error が収束しないため）。
    from common.types import INERTIA
    from fsw.quat_utils import quat_propagate

    q_true = np.array([0.0, 0.0, 0.0, 1.0])
    # 初期姿勢に多少のオフセットを与える（q_target と大きくずれた状態からの収束を見る）
    omega_true = np.zeros(3)
    inertia_diag = np.diag(INERTIA)

    t = 0.0
    dt = FSW_DT
    # 円軌道で位置・速度を回す（nadir 目標が軌道レートで回転し、FSW の
    # 目標角速度フィードフォワード・レート誤差撮像条件と整合する設定）
    a_orbit = 6.871e6
    v_circ = 7.6e3
    n_orbit = v_circ / a_orbit  # 円軌道角速度 [rad/s]
    triggered = False
    for _ in range(int(3000.0 / dt)):
        c, s = np.cos(n_orbit * t), np.sin(n_orbit * t)
        pos = a_orbit * np.array([c, s, 0.0])
        vel = v_circ * np.array([-s, c, 0.0])
        sensors = make_sensors(t, gyro=omega_true, pos=pos, vel=vel)
        cmd_out = fsw.step(sensors, dt)
        if cmd_out.camera_trigger:
            triggered = True

        # 剛体動力学の簡易積分（外乱なし、RW トルクのみ）
        omega_dot = cmd_out.wheel_torque / inertia_diag
        omega_true = omega_true + omega_dot * dt
        q_true = quat_propagate(q_true, omega_true, dt)

        t += dt
        if fsw.mode_mgr.mode == Mode.SUNPOINT:
            break

    tlm = fsw.get_telemetry()
    assert triggered, "camera should have triggered at least once"
    assert tlm.images_stored >= 1
    assert tlm.mode == Mode.SUNPOINT, f"expected auto-return to SUNPOINT, got {tlm.mode}"
    print("test_imaging_sequence_completes_and_returns_to_sunpoint: PASS")


# ---------------------------------------------------------------------
# 7. GroundCommand("THRUST", {"duration_s": T}) 受理で THRUST へ遷移
# ---------------------------------------------------------------------
def test_thrust_command_transitions_to_thrust_mode():
    fsw = FlightSoftware()
    cmd = GroundCommand(type="THRUST", params={"duration_s": 3600.0})
    fsw.handle_command(cmd)

    assert fsw.mode_mgr.mode == Mode.THRUST, fsw.mode_mgr.mode
    assert fsw.thrust.duration_s == 3600.0

    sensors = make_sensors(0.0)
    fsw.step(sensors, FSW_DT)
    tlm = fsw.get_telemetry()
    assert tlm.mode == Mode.THRUST, tlm.mode
    assert tlm.last_cmd == "THRUST"
    print("test_thrust_command_transitions_to_thrust_mode: PASS")


# ---------------------------------------------------------------------
# 8. 噴射許可条件不成立（食中: solar_power=0）で thruster_on=False
# ---------------------------------------------------------------------
def test_thrust_not_engaged_when_conditions_unmet():
    fsw = FlightSoftware()
    fsw.handle_command(GroundCommand(type="THRUST", params={"duration_s": 3600.0}))
    assert fsw.mode_mgr.mode == Mode.THRUST

    # 指向誤差はどうあれ、日照条件（solar_power>0）が不成立なら噴射しない
    sensors = make_sensors(0.0, solar_power=0.0, battery_soc=0.9)
    cmd = fsw.step(sensors, FSW_DT)
    assert cmd.thruster_on is False, "should not thrust in eclipse (solar_power=0)"

    # SoC 条件不成立（<=0.5）でも噴射しない
    sensors2 = make_sensors(0.1, solar_power=2.0, battery_soc=0.4)
    cmd2 = fsw.step(sensors2, FSW_DT)
    assert cmd2.thruster_on is False, "should not thrust when battery_soc<=0.5"

    assert fsw.thrust.thrust_time == 0.0
    print("test_thrust_not_engaged_when_conditions_unmet: PASS")


# ---------------------------------------------------------------------
# 9. 累積噴射時間到達で SUNPOINT へ自律復帰（クローズドループ）
# ---------------------------------------------------------------------
def test_thrust_completes_and_returns_to_sunpoint():
    from common.types import INERTIA
    from fsw.quat_utils import quat_propagate

    fsw = FlightSoftware()

    duration_s = 2.0  # 短時間で完了させ、単体テストとして高速に回す
    fsw.handle_command(GroundCommand(type="THRUST", params={"duration_s": duration_s}))
    assert fsw.mode_mgr.mode == Mode.THRUST

    a_orbit = 6.871e6
    v_circ = 7.6e3
    n_orbit = v_circ / a_orbit  # 円軌道角速度 [rad/s]

    # 恒等姿勢から開始し、PD 制御で目標（nadir_quat）へ収束させる
    # （test_imaging_sequence_completes_and_returns_to_sunpoint と同じ手法。
    #  q_est も恒等スタートなので、真値姿勢も恒等から始めて整合させる）
    q_true = np.array([0.0, 0.0, 0.0, 1.0])
    omega_true = np.zeros(3)
    inertia_diag = np.diag(INERTIA)

    t = 0.0
    dt = FSW_DT
    reached_thrust_on = False
    for _ in range(int(600.0 / dt)):  # 600s あれば収束（~410s）+完了に十分
        c, s = np.cos(n_orbit * t), np.sin(n_orbit * t)
        pos = a_orbit * np.array([c, s, 0.0])
        vel = v_circ * np.array([-s, c, 0.0])
        sensors = make_sensors(
            t, gyro=omega_true, pos=pos, vel=vel,
            solar_power=2.0, battery_soc=0.9,
        )
        cmd_out = fsw.step(sensors, dt)
        if cmd_out.thruster_on:
            reached_thrust_on = True

        omega_dot = cmd_out.wheel_torque / inertia_diag
        omega_true = omega_true + omega_dot * dt
        q_true = quat_propagate(q_true, omega_true, dt)

        t += dt
        if fsw.mode_mgr.mode == Mode.SUNPOINT:
            break

    tlm = fsw.get_telemetry()
    assert reached_thrust_on, "thruster should have engaged at least once"
    assert tlm.mode == Mode.SUNPOINT, f"expected auto-return to SUNPOINT, got {tlm.mode}"
    assert tlm.thrust_time >= duration_s - 1e-6, tlm.thrust_time
    assert tlm.propellant_used_g > 0.0
    print("test_thrust_completes_and_returns_to_sunpoint: PASS")


# ---------------------------------------------------------------------
# 10. GroundCommand("OBSERVE", {...}) 受理で OBSERVE へ遷移（docs/09 §2）
# ---------------------------------------------------------------------
def test_observe_command_transitions_to_observe_mode():
    fsw = FlightSoftware()
    cmd = GroundCommand(type="OBSERVE", params={
        "ra_deg": 106.0, "dec_deg": -7.4, "duration_s": 1800.0, "exposure_s": 1.0,
    })
    fsw.handle_command(cmd)

    assert fsw.mode_mgr.mode == Mode.OBSERVE, fsw.mode_mgr.mode
    assert fsw.observe.duration_s == 1800.0
    assert fsw.observe.exposure_s == 1.0

    sensors = make_sensors(0.0)
    fsw.step(sensors, FSW_DT)
    tlm = fsw.get_telemetry()
    assert tlm.mode == Mode.OBSERVE, tlm.mode
    assert tlm.last_cmd == "OBSERVE"
    print("test_observe_command_transitions_to_observe_mode: PASS")


# ---------------------------------------------------------------------
# 11. 露光中は mtq_moment=0、shutter_open=True を出力する（docs/09 §2 静穏化）
# ---------------------------------------------------------------------
def test_observe_mtq_zero_and_shutter_open_during_exposure():
    from common.types import INERTIA
    from fsw.quat_utils import quat_propagate

    fsw = FlightSoftware()
    # 目標を +Z 方向（恒等姿勢と一致）にして、指向誤差 0 から即座に撮像条件を
    # 満たせるようにする（収束待ちを省略し、露光開始そのものを検証する）。
    fsw.handle_command(GroundCommand(type="OBSERVE", params={
        "ra_deg": 0.0, "dec_deg": 90.0, "duration_s": 60.0, "exposure_s": 2.0,
    }))
    assert fsw.mode_mgr.mode == Mode.OBSERVE

    q_true = np.array([0.0, 0.0, 0.0, 1.0])
    omega_true = np.zeros(3)
    inertia_diag = np.diag(INERTIA)
    pos = np.array([6.871e6, 0.0, 0.0])
    vel = np.array([0.0, 7.6e3, 0.0])

    t = 0.0
    dt = FSW_DT
    saw_shutter_open = False
    mtq_zero_during_exposure = True
    for _ in range(int(30.0 / dt)):
        sensors = make_sensors(t, gyro=omega_true, pos=pos, vel=vel)
        cmd_out = fsw.step(sensors, dt)
        if cmd_out.shutter_open:
            saw_shutter_open = True
            if np.any(cmd_out.mtq_moment != 0.0):
                mtq_zero_during_exposure = False

        omega_dot = cmd_out.wheel_torque / inertia_diag
        omega_true = omega_true + omega_dot * dt
        q_true = quat_propagate(q_true, omega_true, dt)
        t += dt
        if saw_shutter_open and not cmd_out.shutter_open:
            # 少なくとも 1 フレーム分の露光を観測できたら十分
            break

    assert saw_shutter_open, "shutter should have opened at least once"
    assert mtq_zero_during_exposure, "mtq_moment must be zero while shutter_open=True"
    print("test_observe_mtq_zero_and_shutter_open_during_exposure: PASS")


# ---------------------------------------------------------------------
# 12. duration_s 経過で SUNPOINT へ自律復帰し images_stored が増加する
# ---------------------------------------------------------------------
def test_observe_completes_and_returns_to_sunpoint():
    from common.types import INERTIA
    from fsw.quat_utils import quat_propagate

    fsw = FlightSoftware()
    duration_s = 3.0  # 短時間で完了させ、単体テストとして高速に回す
    fsw.handle_command(GroundCommand(type="OBSERVE", params={
        "ra_deg": 0.0, "dec_deg": 90.0, "duration_s": duration_s, "exposure_s": 0.5,
    }))
    assert fsw.mode_mgr.mode == Mode.OBSERVE

    images_stored_before = fsw.get_telemetry().images_stored

    q_true = np.array([0.0, 0.0, 0.0, 1.0])
    omega_true = np.zeros(3)
    inertia_diag = np.diag(INERTIA)
    pos = np.array([6.871e6, 0.0, 0.0])
    vel = np.array([0.0, 7.6e3, 0.0])

    t = 0.0
    dt = FSW_DT
    for _ in range(int(60.0 / dt)):  # duration_s=3s に対し十分な余裕
        sensors = make_sensors(t, gyro=omega_true, pos=pos, vel=vel)
        cmd_out = fsw.step(sensors, dt)

        omega_dot = cmd_out.wheel_torque / inertia_diag
        omega_true = omega_true + omega_dot * dt
        q_true = quat_propagate(q_true, omega_true, dt)
        t += dt
        if fsw.mode_mgr.mode == Mode.SUNPOINT:
            break

    tlm = fsw.get_telemetry()
    assert tlm.mode == Mode.SUNPOINT, f"expected auto-return to SUNPOINT, got {tlm.mode}"
    assert tlm.images_stored >= images_stored_before + 1, (
        f"expected images_stored to increase, before={images_stored_before}, after={tlm.images_stored}"
    )
    assert tlm.observe_time > 0.0
    print("test_observe_completes_and_returns_to_sunpoint: PASS")


def main() -> None:
    test_boot_safe_to_detumble()
    test_quat_math()
    test_bdot_sign()
    test_pd_control_direction()
    test_low_soc_safe()
    test_image_nadir_command()
    test_nan_sensor_fault_handling()
    test_imaging_sequence_completes_and_returns_to_sunpoint()
    test_thrust_command_transitions_to_thrust_mode()
    test_thrust_not_engaged_when_conditions_unmet()
    test_thrust_completes_and_returns_to_sunpoint()
    test_observe_command_transitions_to_observe_mode()
    test_observe_mtq_zero_and_shutter_open_during_exposure()
    test_observe_completes_and_returns_to_sunpoint()
    print("\nALL TESTS PASSED")


if __name__ == "__main__":
    main()
