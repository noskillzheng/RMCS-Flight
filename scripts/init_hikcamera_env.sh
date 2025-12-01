#!/bin/bash
# HikCamera环境初始化脚本 - 配置USB权限和udev规则
# 用法: sudo ./init_hikcamera_env.sh [--user <username>]

set -euo pipefail

# 默认值
VENDOR_IDS=("2dfc" "2bdf" "0547")  # 常见HikCamera Vendor IDs
HOST_USER="${SUDO_USER:-$USER}"
SKIP_UDEV=0
DRY_RUN=0

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

function print_error() {
    echo -e "${RED}[错误]${NC} $1" >&2
}

function print_success() {
    echo -e "${GREEN}[成功]${NC} $1"
}

function print_warning() {
    echo -e "${YELLOW}[警告]${NC} $1"
}

function print_info() {
    echo "[信息] $1"
}

# 解析参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --user)
            HOST_USER="$2"
            shift 2
            ;;
        --skip-udev)
            SKIP_UDEV=1
            shift
            ;;
        --dry-run)
            DRY_RUN=1
            shift
            ;;
        --help)
            echo "用法: $0 [选项]"
            echo "选项:"
            echo "  --user <username>   指定用户名（默认当前用户）"
            echo "  --skip-udev         跳过udev规则配置"
            echo "  --dry-run          仅显示将要执行的操作"
            exit 0
            ;;
        *)
            print_error "未知选项: $1"
            exit 1
            ;;
    esac
done

function run_command() {
    if [[ $DRY_RUN -eq 1 ]]; then
        echo "[DRY-RUN] $@"
    else
        "$@"
    fi
}

# 检查是否需要root权限
if [[ $SKIP_UDEV -eq 0 ]] && [[ $EUID -ne 0 ]]; then
    print_error "配置udev规则需要root权限。请使用sudo运行此脚本。"
    exit 1
fi

print_info "========================================="
print_info "HikCamera环境初始化脚本"
print_info "用户: $HOST_USER"
print_info "========================================="

# 1. 检测HikCamera设备
print_info "检测HikCamera设备..."
DEVICE_FOUND=0
DEVICE_INFO=""

if command -v lsusb &> /dev/null; then
    for vid in "${VENDOR_IDS[@]}"; do
        if lsusb | grep -i "$vid" &> /dev/null; then
            DEVICE_FOUND=1
            DEVICE_INFO=$(lsusb | grep -i "$vid" | head -1)
            print_success "检测到HikCamera设备: $DEVICE_INFO"

            # 提取Bus和Device号
            BUS=$(echo "$DEVICE_INFO" | awk '{print $2}')
            DEV=$(echo "$DEVICE_INFO" | awk '{print $4}' | tr -d ':')

            # 立即设置权限
            USB_DEVICE="/dev/bus/usb/$BUS/$DEV"
            if [[ -c "$USB_DEVICE" ]]; then
                run_command chmod 666 "$USB_DEVICE"
                print_success "已设置设备权限: $USB_DEVICE -> 666"
            fi
            break
        fi
    done

    if [[ $DEVICE_FOUND -eq 0 ]]; then
        print_warning "未检测到HikCamera设备，但仍会配置udev规则"
    fi
else
    print_warning "lsusb命令不可用，跳过设备检测"
fi

# 2. 配置udev规则（持久化USB权限）
if [[ $SKIP_UDEV -eq 0 ]]; then
    print_info "配置udev规则..."

    UDEV_RULE_FILE="/etc/udev/rules.d/99-hikcamera.rules"

    # 创建udev规则内容
    UDEV_CONTENT=""
    for vid in "${VENDOR_IDS[@]}"; do
        UDEV_CONTENT+="# HikCamera USB3 devices (Vendor ID: $vid)\n"
        UDEV_CONTENT+="SUBSYSTEM==\"usb\", ATTR{idVendor}==\"$vid\", MODE=\"0666\", GROUP=\"plugdev\"\n"
        UDEV_CONTENT+="SUBSYSTEM==\"usb_device\", ATTR{idVendor}==\"$vid\", MODE=\"0666\", GROUP=\"plugdev\"\n\n"
    done

    # 写入udev规则文件
    if [[ $DRY_RUN -eq 1 ]]; then
        echo "[DRY-RUN] 将写入到 $UDEV_RULE_FILE:"
        echo -e "$UDEV_CONTENT"
    else
        echo -e "$UDEV_CONTENT" > "$UDEV_RULE_FILE"
        print_success "已创建udev规则文件: $UDEV_RULE_FILE"
    fi

    # 重新加载udev规则
    run_command udevadm control --reload-rules
    run_command udevadm trigger
    print_success "udev规则已重新加载"
fi

# 3. 将用户添加到必要的组
print_info "配置用户组..."

GROUPS_ADDED=""
for group in video plugdev dialout; do
    if getent group "$group" &> /dev/null; then
        if ! id -nG "$HOST_USER" | grep -qw "$group"; then
            run_command usermod -aG "$group" "$HOST_USER"
            print_success "已将用户 $HOST_USER 添加到 $group 组"
            GROUPS_ADDED="yes"
        else
            print_info "用户 $HOST_USER 已在 $group 组中"
        fi
    else
        print_warning "组 $group 不存在，跳过"
    fi
done

# 4. 设置/dev/bus/usb权限（临时修复）
print_info "设置USB设备权限..."

if [[ -d /dev/bus/usb ]]; then
    run_command chmod -R o+rw /dev/bus/usb 2>/dev/null || true
    print_success "已设置/dev/bus/usb权限为o+rw"
else
    print_warning "/dev/bus/usb目录不存在"
fi

# 5. 配置X11权限（如果有.Xauthority）
print_info "配置X11访问权限..."

XAUTH_FILE="/home/$HOST_USER/.Xauthority"
if [[ -f "$XAUTH_FILE" ]]; then
    # 设置ACL权限，允许容器用户访问
    if command -v setfacl &> /dev/null; then
        for user in root ubuntu vscode; do
            run_command setfacl -m u:$user:r "$XAUTH_FILE" 2>/dev/null || true
        done
        print_success "已设置.Xauthority ACL权限"
    else
        # 退而求其次，设置文件权限
        run_command chmod 644 "$XAUTH_FILE"
        print_info "已设置.Xauthority权限为644"
    fi
else
    print_warning ".Xauthority文件不存在: $XAUTH_FILE"
fi

# 完成
print_info "========================================="
print_success "HikCamera环境初始化完成！"

if [[ -n "$GROUPS_ADDED" ]]; then
    print_warning "用户已被添加到新组，需要重新登录才能生效"
    print_info "或者运行: newgrp video"
fi

if [[ $DEVICE_FOUND -eq 1 ]]; then
    print_success "设备已就绪，可以在Docker容器中使用"
else
    print_warning "未检测到设备，请连接HikCamera后重试"
fi

print_info "========================================="