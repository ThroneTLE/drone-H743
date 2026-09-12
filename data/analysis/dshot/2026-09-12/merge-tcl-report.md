# 整合回归中的 Tcl 生命周期修复

基线为整合候选da851144。模式：横切修 bug + tk-ui，阻塞R-DSHOT/R-CURRENT合并验证。

merge-full.txt 的关闭测试失败对象实际是 Image.__del__，不是已处理的 Variable。
追加图像保持/后台释放复现后，merge-image-red.txt 稳定1 failed；之前测试没有明确保留图像对象。
只清图像后专项73 passed，但 merge-full-verified.txt 在后台接收线程GC期间出现Windows fatal exception 0x80000003，不能计为全量通过。

进一步检查真实DronePanel：destroy后仍有控件、字体及字体绑定方法持有解释器，诊断引用计数515。
这些Python对象可形成root→child→master→root循环，后台GC可能在错误线程析构整个Tcl解释器。
因此仅抑制Variable/Image警告并不足够。

最终修复在窗口所属线程、Tk.destroy完成后：清理该解释器的变量、图像和字体；
关闭控件/Style改持无Tcl引用的关闭代理，仍以TclError拒绝访问，避免解释器留在Python循环中。
不修改其他解释器，不禁用GC，不无限保留已关闭解释器，不延长网络超时或绘图性能门限。
真实窗口探查后，仅诊断脚本自身还持有解释器（getrefcount=2）。

回归：另一个存活窗口的变量、图像及字体继续可用；独立子进程在后台GC释放关闭窗口循环并正常退出。
merge-tcl-focused.txt：31 passed。最终完整 python -m pytest tests -q：
merge-full-final.txt，1558 passed in 404.35s，无warnings/skips，零物理串口/烧录/探针调用。
固件代码未因本修复改变；整合版DSHOT300/PWM Debug构建原文merge-build.txt/merge-build-pwm.txt均零警告。

结论仅覆盖930f7a35与d5ed69d3已提交部分的整合候选。
作者明确确认原目录仍有任务编辑；原目录HEAD/文件未被本任务替换，最终同步尚未执行。
需待并发任务停止编辑并提交后，再合入其最终版本及复核，不把候选准备完成写成原工程合并完成。
