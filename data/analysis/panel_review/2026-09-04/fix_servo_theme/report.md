# 舵机调试页深色主题修复

状态：待审核。基线 `6251253c`。作者在本会话追加了截图中的白底浅字、亮白边框问题，
本项独立提交，与 H1/H2/H4 连接基础修复分开。

根因：clam 的 TSpinbox 不继承 TEntry 的配色；现有主题只设置了 Entry，漏掉 Spinbox
的 fieldbackground/箭头/禁用映射。Notebook 的 lightcolor/darkcolor 和部分状态映射
仍使用浅色默认值，Checkbutton 的禁用背景也会回退为浅色。

新模块 `tools/panel_lib/servo_debug_theme.py` 提供
`apply_servo_debug_theme(parent, palette)`，从现有 ui_palette 获取颜色，只为舵机调试页
子树设置专属 ttk 样式。输入框为深灰底、浅灰文字；焦点蓝边；页签/分组边框柔和灰色；
滑块保留蓝色；PWM 禁用控件使用暗底和灰色文字，避免白块。

允许依赖 Tk/ttk 和传入调色板；没有协议 I/O、保存、参数修改或硬件依赖。
页面只增加一次样式挂载，原数值、范围、按钮 command、BUS/PWM 禁用规则不变；
`drone_tcp_panel.py` 本项零改动，机械校准页与所有固件源码零改动。

验证：

- `baseline.txt`：首轮测试误用不存在的 servo_tab 属性，装置错误原文保留。
- `baseline_rerun.txt`：修正控件定位后 `2 failed, 1 passed in 2.28s`，复现默认浅色字段和边框。
- `focused.txt`：样式、机械安全、调试命令、舵机类型与 PWM 即时通路聚焦 `43 passed in 3.37s`。
- `render_preview.py`：真实 DronePanel 的断开连接测试窗口；保存 BUS/PWM 两种实绘截图，
  不操作作者正在使用的上位机。`bus.png` / `pwm.png` 均已视觉复核；物理打开尝试为 0。
  PWM 截图反映测试窗口本地模拟状态，不是目标板类型回读。
- 最终全量 `1033 passed in 95.07s (0:01:35)`，原文见 `full_tests.txt`；
  Debug `ninja: no work to do.`、零警告，见 `build.txt`。
- 不放宽画布性能限值，不改安全门、协议、校准证据；真实串口打开尝试为 0。

工作模式：作者追加的默认样式修复任务；先固定截图/真实控件复现，再最小修复与回归。
不增加界面流程，不接管实机。仅在 PIPELINE 追加本项独立证据行，既有 REQ 状态不变。

原始 pytest 失败回溯保留输出空白；源码/测试 diff --check 通过。
