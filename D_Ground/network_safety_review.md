# 网络配置与线程风险点审查（2026-05-02）

## 审查范围
- `D_Ground/ui_view.py`
- `D_Ground/ground_station.py`
- `D_Ground/comm_link.py`

重点检查：`recvfrom`、`bind`、`close`、`join/wait`、`settimeout`、`closeEvent`、`apply_ip/apply_network`。

## 结论（针对“应用 IP/应用网络配置”按钮）

“应用网络配置”按钮对应 `on_apply_network_clicked()`，该函数本身仅做参数解析并发射信号，不直接进行线程阻塞或 socket 操作。

实际网络切换在 `handle_network_changed()` 中执行，流程是：
1. `self.comm.stop()`：关闭旧 socket、标记线程退出。
2. `self.comm.wait()`：等待旧通信线程结束。
3. 新建 `UDPComm(...)` 并 `start()`。

这是“停止旧线程 + 新建新 socket + 重新 bind”的设计，不是在运行中的同一 socket 上 rebind。

## 风险点逐项

1. `thread.join()`
   - 代码里没有显式 `thread.join()`；PyQt 中使用的是 `QThread.wait()`。
   - `wait()` 在 UI 线程里调用，若接收线程异常卡死，UI 可能短暂阻塞。

2. `sock.recvfrom(...)`
   - 使用阻塞式 `recvfrom(4096)`。
   - 未设置 `settimeout()`。
   - 但 `stop()` 会 `close()` socket，通常可打断阻塞 `recvfrom` 并触发退出路径。

3. `sock.close()`
   - `stop()` 中调用 `self.sock.close()`，随后置 `self.sock = None`。
   - `close()` 与 `recvfrom()` 并发时由 `OSError` 分支兜底，逻辑可接受。

4. “重新 bind socket”
   - 存在：每次应用网络配置时会创建新 `UDPComm` 并在 `run()` 内 `bind(("0.0.0.0", local_port))`。
   - 这是预期行为，但若旧线程未及时退出，短时间内可能遇到端口占用竞争（虽已设置 `SO_REUSEADDR`）。

5. `settimeout`
   - 未使用。
   - 可考虑给 socket 设置短超时（如 0.2~1.0s）以减少 `wait()` 潜在阻塞时长。

6. `closeEvent`
   - UI 中未看到 `closeEvent` 重载。
   - 当前依赖主程序退出后手动调用 `controller.close()` 进行停线程。

7. `apply_ip/apply_network`
   - 未发现独立 `apply_ip`。
   - `apply_network` 逻辑路径清晰，危险调用不在按钮回调里，而在控制器槽函数中。

## 建议

- 若希望避免 UI 卡顿：将 `wait()` 改为带超时等待并记录失败日志。
- 给 UDP socket 增加 `settimeout()`，让线程周期性检查 `is_running`。
- （可选）在主窗口实现 `closeEvent`，确保窗口关闭路径一致执行 `controller.close()`。
