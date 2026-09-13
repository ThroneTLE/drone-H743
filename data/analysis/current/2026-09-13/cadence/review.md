# 电流只采一次：任务调用链修复

用户实测截图：ADC raw=0、ADC电压=0、samples=1、errors=0、adc_status=正常、age_ms=51282，刚收到回包仍valid=否。
来源：同目录user-stale-sample.png。截图证明收到的是过期样本；本次未独立读取板上固件ID或任务PC，不能仅凭截图断定具体存储等待点。

已确认的软件缺陷：APP_Task_Background_Step先调用Current，再进入可能耗时很久的APP_Background_Step。其真实路径包含日志初始化/全区扫描和存储锁，单次锁等待上限10000 ms。周期ADC不应依赖这条慢工作链。

真实C复现同时编译app_tasks.c、app_message.c、app_background.c、app_current.c、bsp_current.c及drv_current.c；仅替换OS/ADC/存储边界。在存储工作占用约51秒、另一个任务正常获调度的条件下：

- 修前：`duration=51305 samples=1 age=51305 valid=0 errors=0`，见red.txt。
- 修后：`duration=51305 samples=2565 age=25 valid=1 errors=0`，见green-trace.txt。
- 同样原始ADC=0可输出真实数值契约`current_a=0.000 valid=1`。这是测试输入，不是用户空载实测；Curr信号电压也不是电池母线电压。

最小修复：将ADC初始化和Step从backgroundTask移到现有messageTask；该任务原默认行为仅osDelay(100)。现在在独立于存储的周期中做一次有界ADC读取并等待约20 ms。初始化/采样只有一个任务所有者。
未新建任务，未修改CubeMX、任务优先级、控制律、DShot、PWM或电流比例。数据真的过期仍会被250 ms判据拒绝；超时不伪造0A，下一次成功读取可恢复。

之前为什么没挡住：旧运行测试直接循环调用APP_Current_Step；任务接入测试只验证源码存在调用。它们未运行两个任务与慢存储之间的真实调用链，单次ARM回包验证也不能证明后台会再次获调度。
新增测试锁定51秒慢存储期间持续采样、有效零值、过期失效、ADC超时恢复、单一ADC所有者、报告不采样和无其他任务/输出调用。专项30项通过。

模式：默认REQ、横切修bug；按项目runtime-services与分层规范处理R-CURRENT-1/2阻塞。状态仍待审核。
审核者在新固件上需要确认samples持续增加、连续回包的age_ms保持较小、空载结果有效；本次没有烧录、复位、目标板命令或实机测量。
