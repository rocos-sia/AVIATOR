# 设备配置脚本

`setup_joystick_udev.sh` 为 USB 摇杆配置普通用户读取权限，供 flight_gateway 使用。

```bash
# 找到实际摇杆的稳定 event 路径（不是 js 路径）。
ls -l /dev/input/by-id/*event-joystick

# 先预览，将路径替换成实际设备路径。
./scripts/setup_joystick_udev.sh --device /dev/input/by-id/usb-YOUR_JOYSTICK-event-joystick --dry-run

# 正式安装；默认授权运行 sudo 的用户。
sudo ./scripts/setup_joystick_udev.sh --device /dev/input/by-id/usb-YOUR_JOYSTICK-event-joystick

# 直接以 root 登录时，显式指定普通用户。
sudo ./scripts/setup_joystick_udev.sh --device /dev/input/eventN --user YOUR_USER
```

脚本检查 evdev 字符设备和 `ID_INPUT_JOYSTICK=1`，从同一 USB 父设备读取 VID/PID，创建 `aviator` 组、将用户追加到该组，再写入 `/etc/udev/rules.d/99-aviator-joystick-VID-PID.rules`。规则仅匹配该型号的 joystick event 设备，权限为 `0640`，不授予写设备或力反馈权限。连接多个同型号摇杆时规则会同时适用。

脚本重新加载规则并只触发选定设备的 change 事件。**注销后重新登录**使新组生效；如设备权限尚未变化，再拔插摇杆。使用 `id` 和 `ls -l /dev/input/eventN` 检查，然后以普通用户启动网关。

重复执行更新同一规则文件，不重复追加规则；不会覆盖不带脚本标记的管理员文件。配置失败可能已完成部分步骤，修正错误后可重新运行。`--dry-run` 不执行系统修改。依赖 Linux、udevadm 和标准账户管理工具，不自动安装软件包。

撤销时删除对应规则文件并重新加载规则、拔插设备；若用户不再需要任何 AVIATOR 设备权限，再执行 `sudo gpasswd -d USER aviator` 并重新登录。删除规则不会自动移除用户组成员。
