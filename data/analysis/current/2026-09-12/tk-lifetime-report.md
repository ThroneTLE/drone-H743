# 窗口关闭后 Tk 变量跨线程回收修复

基线 ebf973f0，R-CURRENT-2 / R-DSHOT-2 联合回归期间进入横切修 bug + tk-ui 模式。
首轮新功能全量只暴露6项旧界面页数断言；更新为新增电流计后的完整页面矩阵后，
下一轮 `readback-full-final.txt` 出现1项本机仿真24参数回读超时及25项 Tk Variable 销毁警告。

确定性复现：真实 DronePanel 销毁后，在后台线程释放仍持有的 StringVar，
当前 Python/Tkinter 的 Variable.__del__ 调用 Tcl，报 main thread is not in main loop。
`tk-lifetime-red.txt` 固定1 failed。此类销毁等待可能拖住通信线程；未修改任何通信超时。
旧测试只销毁窗口，没有验证延后的 Python 引用释放由哪个线程执行，因此未挡住。

修复只在 PanelStateMixin.destroy 完成窗口销毁后调用独立 tk_lifecycle 模块，
在当前 UI 线程释放该解释器的 Tk 变量资源，并解除其 Tcl 引用。
按解释器身份匹配，不修改另一个仍打开窗口的变量；不关其他应用，不抑制异常警告。
使用当前 Tkinter Variable 析构函数及其 _tk=None 的已实现约定，升级 Python 后由本回归守护。
Contract：关闭窗口后再释放 Python 变量不进入 Tcl；Boundary：窗口销毁；Test seam：真实 Tk + 后台引用释放 + 另一个存活解释器。

验证：`tk-lifetime-green.txt` 19 passed（含本机仿真24参数往返、电流页），
`tk-lifetime-verified.txt` 独立1 passed；最终联合全量 `readback-full-verified.txt`：
1551 passed in 232.70s，无 warnings、零物理串口/探针调用。
该修复独立提交；不改采样、控制器、硬件配置、历史数据或性能门限。
