"""Complete planar cascade presets; each signal keeps its own physical unit."""
from .dashboard.layout import Workspace, TileSpec, TILE_WAVE, TILE_PARAM
try:
    from ..sim_xz.control_catalog import HORIZONTAL_GROUPS, VERTICAL_GROUP
except ImportError:  # Direct tools/drone_tcp_panel.py entrypoint.
    from sim_xz.control_catalog import HORIZONTAL_GROUPS, VERTICAL_GROUP


def simulation_workspaces():
    horizontal = [TileSpec(TILE_PARAM, i*3, row*2, 3, 2, [name], {'title': title})
                  for row, group in enumerate(HORIZONTAL_GROUPS)
                  for i, (name, title) in enumerate(group)]
    horizontal += [TileSpec(TILE_WAVE, i*4, 4, 4, 3, [name], {'title': title})
                   for i, (name, title) in enumerate((
                       ('sim_pitch', '俯仰角 / rad'), ('sim_pitch_rate', '俯仰角速度 / rad/s'),
                       ('sim_vx', '水平速度 / m/s')))]
    vertical = [TileSpec(TILE_PARAM, i*3, 0, 3, 2, [name], {'title': title})
                for i, (name, title) in enumerate(VERTICAL_GROUP)]
    vertical += [TileSpec(TILE_WAVE, i*4, 2, 4, 4, [name], {'title': title})
                 for i, (name, title) in enumerate((
                     ('sim_z', '高度 / m'), ('sim_vz', '垂直速度 / m/s'),
                     ('sim_thrust', '实际合推力 / N')))]
    return [Workspace('二维仿真', horizontal), Workspace('高度 P—速度 PID', vertical)]
