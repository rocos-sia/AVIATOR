#!/bin/bash
# 重命名 TRAC_IK 到 PIN_IK

set -e

# Resolve paths from the repository root when invoked from another directory.
cd "$(dirname "${BASH_SOURCE[0]}")/../.."

# This one-time migration applies only to the historical TRAC-IK layout.
if [ ! -d trac_ik_lib ]; then
    echo "当前仓库不是历史 TRAC-IK 目录结构，无需执行迁移。" >&2
    exit 1
fi

echo "开始重命名 TRAC_IK -> PIN_IK..."

# 1. 重命名源文件和目录
echo "1. 重命名文件和目录..."

# 将历史主库源码和头文件移到根目录；CMake 配置需合并到根目录
if [ -d "trac_ik_lib/include" ]; then
    git mv trac_ik_lib/include include
fi
if [ -d "trac_ik_lib/src" ]; then
    git mv trac_ik_lib/src src
fi

# 重命名示例目录
if [ -d "trac_ik_examples" ]; then
    git mv trac_ik_examples examples
fi

# 重命名 CMake 配置文件
if [ -f "cmake/trac_ikConfig.cmake.in" ]; then
    git mv cmake/trac_ikConfig.cmake.in cmake/pin_ikConfig.cmake.in
fi

# 重命名头文件
if [ -f "include/trac_ik/trac_ik.hpp" ]; then
    mkdir -p include/pin_ik
    git mv include/trac_ik/trac_ik.hpp include/pin_ik/pin_ik.hpp
    git mv include/trac_ik/nlopt_ik.hpp include/pin_ik/nlopt_ik.hpp
    git mv include/trac_ik/pinocchio_tl.hpp include/pin_ik/pinocchio_tl.hpp
    git mv include/trac_ik/pinocchio_types.hpp include/pin_ik/pinocchio_types.hpp
    git mv include/trac_ik/urdf.hpp include/pin_ik/urdf.hpp
    rmdir include/trac_ik 2>/dev/null || true
fi

# 重命名源文件
if [ -f "src/trac_ik.cpp" ]; then
    git mv src/trac_ik.cpp src/pin_ik.cpp
fi

echo "文件和目录重命名完成"

# 2. 替换文件内容中的文本
echo "2. 替换文件内容..."

# 查找所有需要修改的文件（排除 .git 和 build）
FILES=$(find . -type f \( -name "*.hpp" -o -name "*.cpp" -o -name "*.txt" -o -name "*.cmake" -o -name "*.in" -o -name "*.md" \) | grep -v ".git" | grep -v build | grep -v ".swp")

for file in $FILES; do
    # 跳过二进制文件
    if file "$file" | grep -q "text"; then
        # 替换命名空间和类名
        sed -i 's/TRAC_IK/PIN_IK/g' "$file"
        sed -i 's/trac_ik/pin_ik/g' "$file"
        sed -i 's/TRAC-IK/PIN-IK/g' "$file"
        sed -i 's/trac-ik/pin-ik/g' "$file"
        # 保留 trac_ik 在 URL 和引用中（如果需要）
    fi
done

echo "文件内容替换完成"

echo "重命名完成！请运行以下命令验证："
echo "  git status"
echo "  git diff --cached"
