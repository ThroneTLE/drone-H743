# 命令行启动上位机导入错误修复
日期：2026-09-08。横切修bug模式，作用于R-SIM-3交付的命令行启动入口。

根因：直接运行tools/drone_tcp_panel.py时，搜索路径包含tools目录而不保证包含仓库根。simulation_launcher回退导入sim_xz.control_catalog，包__init__却立即加载protocol，后者绝对导入tools.panel_lib，触发ModuleNotFoundError: No module named 'tools'。

修复：sim_xz包公共API改为按需导出。纯参数目录不再加载仿真运行模块；原tools.sim_xz公共API保持。没有修改上位机大文件、sys.path或控制固件。

此前测试只按tools包导入，在仓库根路径存在时未暴露脚本路径问题。新增独立Python进程、-I隔离环境、无关工作目录的真实Panel构造回归，并禁用测试中的自动连接/设置保存。

证据：cli-startup-before.txt：2 failed, 1 passed in 1.89s。cli-startup-after.txt：13 passed in 19.34s；串口/probe均0，包含直接启动、目录轻量导入、公共API与模拟器启动/协议回归。未执行实机操作。
