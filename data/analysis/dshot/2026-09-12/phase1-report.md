# R-DSHOT-1 第一阶段：纯 Driver 与 CubeMX 输入配置

工作树 `D:/stm32hal/drone-H743-dshot`，基线 `aeca543e`，分支 `codex/dshot300`。
本报告只描述生成代码之前的阶段，不是完整 DShot 输出交付。

## 实现

- `drv_dshot.c/h`：PX4 v1.16.0 固定版本单向编码移植；停止/正常油门合法性、等效 PWM 映射、精确时钟计算、双路交错帧和两个低电平尾槽。
- 原始快照及 SHA256：`tests/fixtures/dshot_px4/README.md`；`.gitattributes` 保持下载字节不受 CRLF 转换影响。
- CMake 编译纯 Driver。当前运行输出未接入新模块，仍为旧 PWM；尚未新增 ESC_PROTOCOL 构建切换。
- `.ioc` 已准备 UART8_RX Stream2 -> TIM1_UP、word/normal、TIM1 PSC=0/ARR=399、PE9/PE11 Very High。
- `doc/dshot-cubemx-checklist.md` 为作者核对并点击 Generate Code 的清单；生成门测试如实拒绝旧代码。

## 验证原文

- `driver-tests.txt`：6 passed；两组 uint16 全域输入检查、2001 组 PX4 双通道位流对拍、黄金向量、精确 ARR+1 和拒绝时输出不变。
- `build-debug.txt`：实际编译新 Driver 并链接成功，没有 warning/error；这是生成前的 PWM 基线构建，不是 DShot 硬件构建。
- `pytest-full.txt`：1 failed, 1492 passed, 4 skipped。唯一失败是未检出的固定历史 FlightLog CSV。
- `fixture-provenance.json`：从原工作树复制测试固定要求的同一 CSV，源/目标 SHA256 一致，原始文件未改。
- `fixture-recheck.txt`：上述原失败用例 1 passed。尚不把此单项复验宣称为生成后的完整回归。
- `generation-gate-before.txt`：2 failed, 1 passed；`.ioc` 资源配置正确，生成代码仍需作者生成。
- `index.txt`：固定原索引超过 96 KiB 的失败；独立 fix 提交压缩摘要密度。
- `index-fix-tests-final.txt`：索引/PIPELINE 契约 10 passed。

## 模式和剩余工作

- dshot-esc：作者批准的开源驱动移植。
- 横切修 bug：新增文件使索引超过上限，修复摘要密度；容量上限未提高。
- 尚未进入诊断协议接入阶段。
- 必须先收到并核对真实 CubeMX 生成结果，再接入 BSP、成对提交、硬禁用、诊断/日志及双协议构建验证。
- R-DSHOT-1 保持进行中；没有烧录、复位、物理串口或电机操作。
