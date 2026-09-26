#!/usr/bin/env bash
# Configure read access for AVIATOR's Linux evdev joystick input.
set -euo pipefail

usage() {
    cat <<'HELP'
用法：
  sudo scripts/setup_joystick_udev.sh --device /dev/input/by-id/...-event-joystick [--user 用户名]
  scripts/setup_joystick_udev.sh --device /dev/input/eventN --dry-run

自动读取 USB VID/PID，仅匹配被 udev 识别为 joystick 的 event 设备。
将用户加入 aviator 组，并安装 MODE=0640 的规则；不修改整个 /dev。
--user 默认使用 sudo 的原始用户；直接以 root 运行时必须指定普通用户。
--dry-run 只显示计划，不修改用户组、规则或设备权限。
HELP
}
fail() { echo "错误：$*" >&2; exit 1; }

device=''
target_user=${SUDO_USER:-${USER:-}}
dry_run=false
while (($#)); do
    case "$1" in
        --device|--user)
            (($# >= 2)) && [[ -n $2 && $2 != --* ]] || fail "$1 缺少参数"
            if [[ $1 == --device ]]; then device=$2; else target_user=$2; fi
            shift 2 ;;
        --dry-run) dry_run=true; shift ;;
        --help|-h) usage; exit 0 ;;
        *) fail "未知参数：$1（使用 --help 查看用法）" ;;
    esac
done
[[ -n $device ]] || fail '请通过 --device 指定摇杆 event 设备路径'
for tool in udevadm readlink getent id; do
    command -v "$tool" >/dev/null || fail "缺少命令：$tool"
done
[[ -n $target_user && $target_user != -* ]] || fail '请通过 --user 指定普通用户'
getent passwd "$target_user" >/dev/null || fail "用户不存在：$target_user"
[[ $(id -u "$target_user") != 0 ]] || fail '目标用户不能是 root，请使用 --user 指定普通用户'

resolved=$(readlink -f -- "$device") || fail "无法解析设备路径：$device"
[[ $resolved =~ ^/dev/input/event[0-9]+$ && -c $resolved ]] || fail "不是有效的 evdev 字符设备：$device"
properties=$(udevadm info --query=property --name="$resolved")
joystick=false
while IFS= read -r property; do
    [[ $property != ID_INPUT_JOYSTICK=1 ]] || joystick=true
done <<< "$properties"
$joystick || fail 'udev 未将此设备识别为 joystick；请检查路径，避免给键盘或鼠标授权'

devpath=$(udevadm info --query=path --name="$resolved")
[[ $devpath == /devices/* && -d /sys$devpath ]] || fail '无法取得设备 sysfs 路径'
parent=/sys$devpath
vendor=''
product=''
while [[ $parent != /sys && $parent != / ]]; do
    if [[ -r $parent/idVendor && -r $parent/idProduct ]]; then
        read -r vendor < "$parent/idVendor"
        read -r product < "$parent/idProduct"
        break
    fi
    parent=${parent%/*}
done
[[ $vendor =~ ^[[:xdigit:]]{4}$ && $product =~ ^[[:xdigit:]]{4}$ ]] || fail '未找到 USB VID/PID；本脚本仅处理 USB joystick'
vendor=${vendor,,}
product=${product,,}
group=aviator
rule_file=/etc/udev/rules.d/99-aviator-joystick-${vendor}-${product}.rules
marker='# Managed by AVIATOR scripts/setup_joystick_udev.sh'
rule="SUBSYSTEM==\"input\", KERNEL==\"event*\", ENV{ID_INPUT_JOYSTICK}==\"1\", ATTRS{idVendor}==\"$vendor\", ATTRS{idProduct}==\"$product\", GROUP=\"$group\", MODE=\"0640\""
printf '设备：%s\nUSB VID:PID：%s:%s\n用户：%s\n规则文件：%s\n%s\n' \
    "$resolved" "$vendor" "$product" "$target_user" "$rule_file" "$rule"
if $dry_run; then
    echo '仅预览：正式执行会创建 aviator 组、添加用户、安装规则并刷新此设备。'
    exit 0
fi
[[ $EUID == 0 ]] || fail '写入系统规则需要 root，请使用 sudo 运行（可先使用 --dry-run）'
for tool in groupadd usermod install mktemp; do
    command -v "$tool" >/dev/null || fail "缺少命令：$tool"
done
# Never overwrite an unrelated administrator rule or follow a destination symlink.
[[ ! -L $rule_file ]] || fail "规则路径是符号链接，拒绝覆盖：$rule_file"
if [[ -e $rule_file ]]; then
    [[ -f $rule_file ]] || fail '规则路径不是普通文件'
    IFS= read -r first_line < "$rule_file" || true
    [[ ${first_line:-} == "$marker" ]] || fail "规则文件不是本脚本管理的文件，请先人工检查：$rule_file"
fi
if ! getent group "$group" >/dev/null; then groupadd "$group"; fi
usermod -aG "$group" "$target_user"
temporary=$(mktemp)
trap 'rm -f -- "$temporary"' EXIT
printf '%s\n%s\n' "$marker" "$rule" > "$temporary"
install -d -m 0755 /etc/udev/rules.d
install -o root -g root -m 0644 "$temporary" "$rule_file"
udevadm control --reload-rules
if ! udevadm trigger --action=change "/sys$devpath"; then
    echo '规则已安装，但刷新当前设备失败；请拔插摇杆。' >&2
    exit 1
fi
printf '\n配置完成。请注销后重新登录，使用户组生效；必要时拔插摇杆。\n'
printf '规则匹配同 VID/PID 的 USB joystick event 设备。验证：id %s；ls -l %s\n' "$target_user" "$resolved"
