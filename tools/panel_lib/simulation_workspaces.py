"""Separate horizontal, vertical and attitude P-PID parameter workspaces."""
from .dashboard.layout import Workspace, TileSpec, TILE_WAVE, TILE_PARAM
try:
    from ..sim_xz.control_catalog import PARAMETER_GROUPS
except ImportError:  # Direct tools/drone_tcp_panel.py entrypoint.
    from sim_xz.control_catalog import PARAMETER_GROUPS


def simulation_workspaces():
    definitions = (
        ('horizontal', '水平平动 · P—PID',
         (('sim_x', '水平位置 / m'), ('sim_vx', '水平速度 / m/s'), ('sim_pitch', '机体俯仰 / rad'))),
        ('vertical', '高度 · P—PID',
         (('sim_z', '高度 / m'), ('sim_vz', '垂直速度 / m/s'), ('sim_thrust', '实际合推力 / N'))),
        ('attitude', '俯仰姿态 · P—PID',
         (('sim_pitch', '俯仰角 / rad'), ('sim_pitch_rate', '俯仰角速度 / rad/s'), ('sim_tilt', '实际倾转 / rad'))),
    )
    workspaces = []
    for group, title, signals in definitions:
        tiles = [TileSpec(TILE_PARAM, i*3, 0, 3, 2, [name], {'title': label})
                 for i, (name, label) in enumerate(PARAMETER_GROUPS[group])]
        tiles += [TileSpec(TILE_WAVE, i*4, 2, 4, 4, [name], {'title': label})
                  for i, (name, label) in enumerate(signals)]
        # ephemeral：这些工作区绑的是 sim_* 通道，而它们映射到真实的 coax.* 参数。
        # 标成不落盘，接真机时就不可能在仪表盘里翻到它们。
        workspaces.append(Workspace(title, tiles, ephemeral=True))
    return workspaces
