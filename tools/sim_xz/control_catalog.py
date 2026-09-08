"""Presentation catalog of the real C P-PID-P-PID and vertical P-PID gains.

Ranges describe slider travel, not replacement firmware validation rules.
Existing nine telemetry channels keep their original indices.
"""
GAIN_CHANNELS = (
    ('sim_pos_x_kp', '1/s', 'gain', 0., 10., 'coax.pos_x_kp'),
    ('sim_vel_x_kp', '1/s', 'gain', 0., 10., 'coax.vel_x_kp'),
    ('sim_att_pitch_kp', '1/s', 'gain', 0., 10., 'coax.att_pitch_kp'),
    ('sim_rate_pitch_kp', 'N.m/(rad/s)', 'gain', 0., 1., 'coax.rate_pitch_kp'),
    ('sim_vel_x_ki', '1/s2', 'gain', 0., 10., 'coax.vel_x_ki'),
    ('sim_vel_x_kd', '1', 'gain', 0., 10., 'coax.vel_x_kd'),
    ('sim_rate_pitch_ki', 'N.m/rad', 'gain', 0., 1., 'coax.rate_pitch_ki'),
    ('sim_rate_pitch_kd', 'N.m.s2/rad', 'gain', 0., 1., 'coax.rate_pitch_kd'),
    ('sim_pos_z_kp', '1/s', 'gain', 0., 10., 'coax.pos_z_kp'),
    ('sim_vel_z_kp', '1/s', 'gain', 0., 10., 'coax.vel_z_kp'),
    ('sim_vel_z_ki', '1/s2', 'gain', 0., 10., 'coax.vel_z_ki'),
    ('sim_vel_z_kd', '1', 'gain', 0., 10., 'coax.vel_z_kd'),
)

HORIZONTAL_GROUPS = (
    (('sim_pos_x_kp', 'X位置 P'), ('sim_vel_x_kp', 'X速度 P'),
     ('sim_vel_x_ki', 'X速度 I'), ('sim_vel_x_kd', 'X速度 D')),
    (('sim_att_pitch_kp', '俯仰角 P'), ('sim_rate_pitch_kp', '俯仰角速度 P'),
     ('sim_rate_pitch_ki', '俯仰角速度 I'), ('sim_rate_pitch_kd', '俯仰角速度 D')),
)
VERTICAL_GROUP = (('sim_pos_z_kp', '高度 P'), ('sim_vel_z_kp', '垂直速度 P'),
                  ('sim_vel_z_ki', '垂直速度 I'), ('sim_vel_z_kd', '垂直速度 D'))

PARAMETER_GROUPS = {
    'horizontal': HORIZONTAL_GROUPS[0],
    'vertical': VERTICAL_GROUP,
    'attitude': HORIZONTAL_GROUPS[1],
}

EXPERIMENT_LABELS = {'位置阶跃 · X': 'position_step', '速度阶跃 · X': 'velocity_step',
                     '俯仰阶跃': 'pitch_step', '高度阶跃 · Z': 'height_step'}
