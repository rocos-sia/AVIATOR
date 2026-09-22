# USB 飞行摇杆（Linux）

独立 CMake 工程，C++17，无第三方依赖。适用于 Linux 驱动识别并暴露为
`/dev/input/js*` 的 USB 摇杆。采集库与网页服务分离，不依赖 AVIATOR 主工程。

```bash
cmake -S examples/joystick -B examples/joystick/build
cmake --build examples/joystick/build -j
./examples/joystick/build/joystick_web /dev/input/js0 8080
```

浏览器访问 http://127.0.0.1:8080 ，不要直接双击 HTML。Ctrl+C 退出。
设备路径和端口可省略，默认 `/dev/input/js0`、`8080`。网页嵌入可执行文件，
修改 `index.html` 后重新构建即可，运行时不依赖工作目录。

- `include/joystick.hpp`、`src/joystick.cpp`：采集静态库 `libjoystick.a`。
- `src/main.cpp`：本机 HTTP 服务；`GET /api/state` 返回设备名称、连接状态、错误、所有轴与按钮。
- `index.html`：约 20 Hz 刷新；显示原始轴值、归一化值和按钮状态。

库使用示例（通过 `add_subdirectory` 引入本目录后，链接 `joystick` target）：

```cpp
#include "joystick.hpp"
joystick::Device stick("/dev/input/js0");
// 在调用方循环中周期调用，不要跨线程同时访问同一个 Device。
const auto& state = stick.poll();
if (state.connected) {
    // state.name、state.axes[i]、state.buttons[i]
}
```

`poll()` 非阻塞读取排队事件并维护最新快照；返回引用在下次调用时更新。
包含驱动的初始化事件；拔出设备会清空旧数据，后续调用自动尝试重连同一路径。
轴和按钮从 0 编号，按钮 0/1 表示松开/按下。轴通常为 -32767～32767，
具体 X/Y、油门、扭转轴、苦力帽映射依设备驱动而定，不硬编码机型。
网页显示当前状态，不记录两次刷新之间的短按事件。

没有设备时服务仍启动并显示错误。可用 `ls /dev/input/js*` 检查路径；
若 USB 设备已识别但没有 js 节点，检查内核 `joydev` 驱动（`sudo modprobe joydev`）。
权限不足时为当前用户配置设备读取权限。重新插入后若设备编号改变，需用新路径重启。
服务仅监听本机；这是单线程状态查看示例，慢 HTTP 客户端会暂时延迟采集。

## 震动

页面“按住震动”支持鼠标、触摸，以及按钮获得焦点后的空格/回车。
松开、移出按钮、窗口失焦或切换页面时停止。按住期间每约 200ms 续发，
单次效果最长 600ms，避免页面断开后持续震动。默认强弱电机均为 50% 强度；周期力反馈使用 40 Hz、50% 幅度。

库接口 `stick.rumble(true)` 播放/续期，`stick.rumble(false)` 停止，返回是否成功。
`State::rumble_available` 表示可用，`rumble_error` 提供失败原因。
HTTP 控制接口为 `POST /api/rumble/start` 和 `POST /api/rumble/stop`。

通过当前 js 设备自动定位同一 input 设备的 `/dev/input/event*`，需要该节点的
**读写权限**及驱动支持 `FF_RUMBLE`，或周期力反馈 `FF_PERIODIC`（正弦/三角波）。权限不足或不支持时按钮禁用，采集仍可使用。
不可用时每 2 秒重新检测，更改权限后会自动恢复。退出或拔出时释放震动效果。
实现依据：[Linux 内核力反馈接口文档](https://www.kernel.org/doc/html/latest/input/ff.html)。

回归测试（按钮事件测试需要 Node.js；不需要连接手柄）：

```bash
cmake -S examples/joystick -B examples/joystick/build -DJOYSTICK_BUILD_TESTS=ON
cmake --build examples/joystick/build -j
ctest --test-dir examples/joystick/build --output-on-failure
```

“震动不可用”表示检测尚未成功，旁边会显示具体原因；只有检测成功后按钮才启用。
若提示 `Permission denied`，需要为提示的 **event 节点**配置当前用户的读写权限，
js 节点可读取不代表 event 节点可写。若提示驱动没有对应能力，即使硬件有电机，
当前驱动也未必暴露了可用的力反馈接口。
