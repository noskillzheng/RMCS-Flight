#!/bin/bash
# 宿主机HikCamera环境一键配置脚本
# 用途：自动配置X11授权、USB权限，确保Docker容器能访问海康相机
# 用法：sudo ./setup_host_env.sh

set -euo pipefail

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印函数
print_error() {
    echo -e "${RED}✗${NC} $1" >&2
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

print_section() {
    echo ""
    echo -e "${BLUE}═══════════════════════════════════════════${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════${NC}"
}

# 检查root权限
check_root() {
    if [[ $EUID -ne 0 ]]; then
        print_error "此脚本需要root权限运行"
        print_info "请使用: sudo $0"
        exit 1
    fi
}

# 获取实际用户（即使通过sudo运行）
get_real_user() {
    if [[ -n "${SUDO_USER:-}" ]]; then
        echo "$SUDO_USER"
    else
        echo "$USER"
    fi
}

# 主函数
main() {
    echo ""
    echo "╔══════════════════════════════════════════════╗"
    echo "║     HikCamera Docker环境一键配置脚本        ║"
    echo "║     版本: 1.0                               ║"
    echo "╚══════════════════════════════════════════════╝"

    check_root

    REAL_USER=$(get_real_user)
    REAL_HOME=$(eval echo ~$REAL_USER)

    print_info "当前用户: $REAL_USER"
    print_info "用户目录: $REAL_HOME"

    # 1. 配置X11授权
    print_section "步骤 1/3: 配置X11授权"

    # 检查DISPLAY变量
    if [[ -z "${DISPLAY:-}" ]]; then
        print_warning "DISPLAY环境变量未设置，尝试设置为:0"
        export DISPLAY=:0
    fi
    print_info "DISPLAY = $DISPLAY"

    # 检查.Xauthority文件
    XAUTH_FILE="$REAL_HOME/.Xauthority"
    if [[ ! -f "$XAUTH_FILE" ]]; then
        print_warning ".Xauthority文件不存在，创建新文件"
        touch "$XAUTH_FILE"
        chown "$REAL_USER:$REAL_USER" "$XAUTH_FILE"
        chmod 600 "$XAUTH_FILE"

        # 尝试生成授权
        if command -v xauth &> /dev/null; then
            su - "$REAL_USER" -c "xauth generate $DISPLAY . trusted" 2>/dev/null || true
        fi
    else
        print_success ".Xauthority文件存在: $XAUTH_FILE"
    fi

    # 配置xhost访问
    if command -v xhost &> /dev/null; then
        print_info "配置xhost访问权限..."

        # 允许root访问
        su - "$REAL_USER" -c "xhost +si:localuser:root" 2>/dev/null || print_warning "无法为root添加xhost权限"

        # 允许容器常用用户访问
        for user in ubuntu vscode $REAL_USER; do
            su - "$REAL_USER" -c "xhost +si:localuser:$user" 2>/dev/null || true
        done

        print_success "X11访问权限已配置"
    else
        print_warning "xhost命令不可用，跳过X11权限配置"
        print_info "如需X11支持，请安装: apt-get install x11-xserver-utils"
    fi

    # 2. 配置USB设备权限
    print_section "步骤 2/3: 配置USB设备权限"

    # 检查并运行init_hikcamera_env.sh
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    INIT_SCRIPT="$SCRIPT_DIR/init_hikcamera_env.sh"

    if [[ -f "$INIT_SCRIPT" ]]; then
        print_info "运行HikCamera初始化脚本..."
        bash "$INIT_SCRIPT" --user "$REAL_USER" || print_warning "HikCamera初始化脚本执行失败"
    else
        print_warning "未找到init_hikcamera_env.sh，手动配置USB权限"

        # 手动设置USB权限
        if [[ -d /dev/bus/usb ]]; then
            chmod -R o+rw /dev/bus/usb 2>/dev/null || true
            print_info "已设置/dev/bus/usb权限"
        fi

        # 将用户添加到必要的组
        for group in video plugdev dialout; do
            if getent group "$group" &> /dev/null; then
                usermod -aG "$group" "$REAL_USER" 2>/dev/null || true
                print_info "已将$REAL_USER添加到$group组"
            fi
        done
    fi

    # 3. 验证配置
    print_section "步骤 3/3: 验证配置"

    # 检查USB设备
    if command -v lsusb &> /dev/null; then
        HIK_DEVICES=$(lsusb | grep -iE "2dfc|2bdf|0547" || true)
        if [[ -n "$HIK_DEVICES" ]]; then
            print_success "检测到HikCamera设备:"
            echo "$HIK_DEVICES"
        else
            print_warning "未检测到HikCamera设备"
            print_info "请确保相机已连接并通电"
        fi
    fi

    # 检查X11连接
    if [[ -n "${DISPLAY:-}" ]] && command -v xdpyinfo &> /dev/null; then
        if su - "$REAL_USER" -c "xdpyinfo" &> /dev/null; then
            print_success "X11连接测试成功"
        else
            print_warning "X11连接测试失败"
        fi
    fi

    # 完成
    print_section "配置完成"

    echo -e "${GREEN}╔══════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║         宿主机配置完成！                     ║${NC}"
    echo -e "${GREEN}╚══════════════════════════════════════════════╝${NC}"
    echo ""
    print_info "下一步操作:"
    print_info "1. 重新启动Docker容器: docker-compose down && docker-compose up -d"
    print_info "2. 进入容器测试相机: docker exec -it rmcs-develop-rmcs-develop-1 bash"
    print_info "3. 在容器内运行: launch-rmcs"
    echo ""

    # 如果用户被添加到新组，提醒重新登录
    if [[ -n "${GROUPS_ADDED:-}" ]]; then
        print_warning "用户已被添加到新组，可能需要重新登录才能生效"
        print_info "或者运行: newgrp video"
    fi
}

# 执行主函数
main "$@"