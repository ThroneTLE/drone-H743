# R-SIM-3 水平/高度参数独立性澄清
日期：2026-09-08。

核对发现C字段原本已独立：position.pos_kp/vel_kp/vel_ki/vel_kd的X与Z分别对应数组元素0和2，并未共用一个增益。此次明确界面分组，不复制控制器或修改原控制律。

## 界面
水平平动 · P—PID：coax.pos_x_kp，coax.vel_x_kp/ki/kd。
高度 · P—PID：coax.pos_z_kp，coax.vel_z_kp/ki/kd。
俯仰姿态 · P—PID：coax.att_pitch_kp，coax.rate_pitch_kp/ki/kd。
三组各4项、参数绑定无交集。启动水平/高度/俯仰实验分别自动选对应工作区。I/D初始值都为0并不意味着共享变量。

## 验证
axis-parameter-tests.txt原文：28 passed in 37.59s；串口/probe均0。
逐一修改水平或高度8个增益中的任意一项，断言其余11项C增益完全不变。
设置X/Z两套不同数值并切换四种实验，断言参数均保留。
后续会话缓存清理与参数/启动联跑：16 passed in 24.68s。缓存问题独立fix 1443d4b6，单项复验1 passed in 5.29s。
本轮截图抓取未获得可用最终图像，未作为证据；实际Panel启动与控件/工作区绑定已程序化验证。

## 边界
只改宿主参数组织和仿真会话生命周期；没有修改C控制律或解除物理耦合。水平运动仍能通过推力方向和总推力限制影响高度响应，但不会改写高度增益。
模式：默认层、simulation、tk-ui；会话缓存问题另走横切修bug。R-SIM-3仍待审核。
