# R-CURRENT-2 电流计上位机回读

工作树 D:/stm32hal/drone-H743-dshot，基线 ebf973f0。模式：默认 REQ + tk-ui + protocol-telemetry；
阻塞回归的 Tk 变量清理缺陷另进横切修 bug，见 tk-lifetime-report.md，独立 fix 提交。

「传感器 / 电流计」复用 STATUS?、原连接与接收队列，提供立即读取及仅页面可见时2秒自动刷新。
展示标称电流、原始ADC、电压、回包采样有效性、未校准/饱和、样本年龄、接收年龄、样本/错误计数、ADC状态、来源及比例。
时间取 transport 的 received_at；换连接/代次、断连、3秒无电流回包、非法/不完整回包均不显示有效电流。
不按其他报文刷新电流年龄；未收到 CURRENT 提示确认固件支持；拒绝发送不排队重放。
没有新建协议 function ID、遥测通道或固件命令。STATUS? 带宽静态估算及边界见 doc/current-sensor.md。

测试来自实际 app_current.c 的 C 报告输出（ADC seam输入12000），经真实 DronePanel 队列分派到控件。
`readback-tests.txt`：17 passed；`readback-focused-final.txt`：39 passed（含日志/索引/PIPELINE）；
`readback-layout-final.txt`：10 passed，包含所有叶页的三尺寸三缩放实际几何/可达性矩阵。
旧布局清单23页/4个传感器页更新为24页/5个传感器页，3尺寸报告数69→72，原 bad=[] 门不变。

最终 `python -m pytest tests -q` 原文：`readback-full-verified.txt`，1551 passed in 232.70s，无 warnings/skips。
中间失败原文保留：readback-full.txt（6项旧页数断言）、readback-full-final.txt（Tk清理问题）；
单项重跑没有替代最终全量。DSHOT300/PWM Debug 均实际编译链接零警告，日志见 data/analysis/dshot/2026-09-12/v11-build.txt 与 v11-build-pwm.txt。

截图：current-before.png（固定基线 shell 布局）、current-after.png（当前页面），由自有测试窗口直接捕获。
截图中的47.393 A来自离线ADC=12000，不是目标板实测；实际窗口/字体渲染已检查。
初次按屏幕坐标捕获被别的前台窗口遮挡，已改为自有窗口捕获并重新验证，错误图不作为交付证据。
图形缩放是模拟矩阵，不替代真实多显示器DPI检查。

两巨型入口、Core、USB_DEVICE、.ioc均未修改。没有烧录、复位、目标板命令或物理串口调用。
软件交审，状态最多待审核；实际回读、Curr/GND接线、电流零偏/增益校准仍归审核者。
