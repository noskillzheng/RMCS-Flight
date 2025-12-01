#!/bin/bash
# HikCamera 环境检查脚本 - 在容器内运行，检查所有必要条件
# 如果检查失败，给出明确的修复指导

set -euo pipefail

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 检查结果
CHECK_PASSED=1

function print_error() {
    echo -e "${RED}✗${NC} $1" >&2
    CHECK_PASSED=0
}

function print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

function print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

function print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

function print_section() {
    echo ""
    echo -e "${BLUE}═══════════════════════════════════════════${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════${NC}"
}

# 主检查开始
echo ""
echo "╔══════════════════════════════════════════════╗"
echo "║     HikCamera 环境检查脚本 v2.0             ║"
echo "║     针对 Docker 容器内运行                  ║"
echo "╚══════════════════════════════════════════════╝"

print_section "1. USB 设备检测"

# 检查 lsusb 是否可用
if ! command -v lsusb &> /dev/null; then
    print_warning "lsusb 命令不可用，尝试安装..."
    if command -v apt-get &> /dev/null; then
        sudo apt-get update && sudo apt-get install -y usbutils &> /dev/null || true
    fi
fi

# 检测 HikCamera 设备
HIKCAMERA_FOUND=0
if command -v lsusb &> /dev/null; then
    # 常见的 HikCamera Vendor IDs
    VENDOR_IDS=("2dfc" "2bdf" "0547")

    for vid in "${VENDOR_IDS[@]}"; do
        if lsusb | grep -i "$vid" &> /dev/null; then
            HIKCAMERA_FOUND=1
            DEVICE_INFO=$(lsusb | grep -i "$vid" | head -1)
            print_success "检测到 HikCamera 设备: $DEVICE_INFO"

            # 提取设备路径
            BUS=$(echo "$DEVICE_INFO" | awk '{print $2}')
            DEV=$(echo "$DEVICE_INFO" | awk '{print $4}' | tr -d ':')
            USB_DEVICE="/dev/bus/usb/$BUS/$DEV"

            if [[ -c "$USB_DEVICE" ]]; then
                # 检查设备权限
                PERMS=$(stat -c "%a %U:%G" "$USB_DEVICE")
                print_info "设备权限: $USB_DEVICE -> $PERMS"

                # 检查当前用户是否可以访问
                if [[ -r "$USB_DEVICE" ]] && [[ -w "$USB_DEVICE" ]]; then
                    print_success "当前用户可以访问 USB 设备"
                else
                    print_error "当前用户无法访问 USB 设备"
                    print_info "修复方法: 在宿主机运行 'sudo ./scripts/init_hikcamera_env.sh'"
                fi
            fi
            break
        fi
    done

    if [[ $HIKCAMERA_FOUND -eq 0 ]]; then
        print_error "未检测到 HikCamera 设备"
        print_info "可能的原因："
        print_info "  1. 相机未连接或未通电"
        print_info "  2. USB 线缆故障"
        print_info "  3. 相机 Vendor ID 不在已知列表中"
        print_info "请运行 'lsusb' 查看所有 USB 设备"
    fi
else
    print_error "无法运行 lsusb 命令"
fi

print_section "2. X11 授权检查"

# 检查 DISPLAY 环境变量
if [[ -n "${DISPLAY:-}" ]]; then
    print_success "DISPLAY 已设置: $DISPLAY"
else
    print_error "DISPLAY 环境变量未设置"
    print_info "修复方法: 确保 docker-compose.yml 包含 'DISPLAY=\${DISPLAY}'"
fi

# 检查 XAUTHORITY 环境变量和文件
if [[ -n "${XAUTHORITY:-}" ]]; then
    print_info "XAUTHORITY 环境变量: $XAUTHORITY"

    if [[ -f "$XAUTHORITY" ]]; then
        if [[ -r "$XAUTHORITY" ]]; then
            print_success "X11 授权文件可读: $XAUTHORITY"

            # 尝试列出 xauth 条目
            if command -v xauth &> /dev/null; then
                AUTH_COUNT=$(xauth list 2>/dev/null | wc -l)
                if [[ $AUTH_COUNT -gt 0 ]]; then
                    print_success "找到 $AUTH_COUNT 个 X11 授权条目"
                else
                    print_warning "X11 授权文件存在但没有有效条目"
                    print_info "可能需要在宿主机重新生成: 'xauth generate :0 . trusted'"
                fi
            fi
        else
            print_error "X11 授权文件存在但不可读: $XAUTHORITY"
            print_info "修复方法: 在宿主机运行 'chmod 644 $XAUTHORITY'"
        fi
    else
        print_error "X11 授权文件不存在: $XAUTHORITY"
        print_info "修复方法:"
        print_info "  1. 确保宿主机 ~/.Xauthority 存在"
        print_info "  2. 确保 docker-compose.yml 正确挂载该文件"
    fi
else
    print_error "XAUTHORITY 环境变量未设置"
    print_info "修复方法: 确保 docker-compose.yml 包含 'XAUTHORITY=/tmp/.Xauthority'"
fi

# 测试 X11 连接
if [[ -n "${DISPLAY:-}" ]] && command -v xdpyinfo &> /dev/null; then
    if xdpyinfo &> /dev/null; then
        print_success "可以连接到 X11 服务器"
    else
        print_warning "无法连接到 X11 服务器"
        print_info "可能需要在宿主机运行: 'xhost +si:localuser:$(whoami)'"
    fi
fi

print_section "3. 用户和组权限"

# 显示当前用户信息
CURRENT_USER=$(whoami)
CURRENT_UID=$(id -u)
CURRENT_GID=$(id -g)
CURRENT_GROUPS=$(id -nG)

print_info "当前用户: $CURRENT_USER (UID=$CURRENT_UID, GID=$CURRENT_GID)"
print_info "所属组: $CURRENT_GROUPS"

# 检查关键组成员资格
REQUIRED_GROUPS=("video" "plugdev" "dialout")
for group in "${REQUIRED_GROUPS[@]}"; do
    if id -nG | grep -qw "$group"; then
        print_success "用户在 $group 组中"
    else
        if getent group "$group" &> /dev/null; then
            print_warning "用户不在 $group 组中"
            print_info "修复方法: 在 docker-compose.yml 添加 'group_add: [\"$group\"]'"
        fi
    fi
done

print_section "4. 环境变量汇总"

# 列出所有相关环境变量
print_info "关键环境变量:"
echo "  DISPLAY=${DISPLAY:-<未设置>}"
echo "  XAUTHORITY=${XAUTHORITY:-<未设置>}"
echo "  WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-<未设置>}"
echo "  USER=${USER:-<未设置>}"
echo "  HOME=${HOME:-<未设置>}"

print_section "5. 设备节点权限"

# 检查 /dev 目录挂载
if mountpoint -q /dev; then
    print_success "/dev 目录已挂载"
else
    print_warning "/dev 目录未挂载为独立文件系统"
fi

# 检查一些关键设备节点
DEVICES_TO_CHECK=(
    "/dev/bus/usb"
    "/dev/video0"
    "/dev/dri"
)

for device in "${DEVICES_TO_CHECK[@]}"; do
    if [[ -e "$device" ]]; then
        print_info "发现设备节点: $device"
    fi
done

print_section "诊断总结"

if [[ $CHECK_PASSED -eq 1 ]]; then
    echo -e "${GREEN}╔══════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║         所有检查通过！                      ║${NC}"
    echo -e "${GREEN}║         HikCamera 应该可以正常工作          ║${NC}"
    echo -e "${GREEN}╚══════════════════════════════════════════════╝${NC}"
else
    echo -e "${RED}╔══════════════════════════════════════════════╗${NC}"
    echo -e "${RED}║         发现问题！                          ║${NC}"
    echo -e "${RED}╚══════════════════════════════════════════════╝${NC}"
    echo ""
    echo "快速修复步骤:"
    echo "1. 退出容器"
    echo "2. 在宿主机运行:"
    echo "   sudo ./RMCS/scripts/init_hikcamera_env.sh"
    echo "   xhost +si:localuser:root"
    echo "   xhost +si:localuser:$(whoami)"
    echo "3. 重启容器:"
    echo "   docker compose down && docker compose up -d"
    echo "4. 重新进入容器并运行此检查脚本"
    echo ""
    echo "如果问题持续，请检查:"
    echo "- 相机物理连接"
    echo "- USB 线缆质量"
    echo "- 宿主机 dmesg 输出"
    exit 1
fi

exit 0