#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 默认 ELF 路径（可通过第一个参数覆盖）
# 优先使用 build/ 下的 ELF，若不存在则尝试 build/Debug/
ELF_PATH="${1:-}"
if [[ -z "$ELF_PATH" ]]; then
  if [[ -f "$PROJECT_ROOT/build/Engineer_2026_H7_Template.elf" ]]; then
    ELF_PATH="$PROJECT_ROOT/build/Engineer_2026_H7_Template.elf"
  else
    ELF_PATH="$PROJECT_ROOT/build/Debug/Engineer_2026_H7_Template.elf"
  fi
fi

CFG_PATH="$PROJECT_ROOT/stm32h723.cfg"

if [[ ! -f "$ELF_PATH" ]]; then
  echo "ELF 文件不存在: $ELF_PATH" >&2
  exit 1
fi

if [[ ! -f "$CFG_PATH" ]]; then
  echo "OpenOCD 配置文件不存在: $CFG_PATH" >&2
  exit 1
fi

echo "使用 ELF: $ELF_PATH"
echo "使用 OpenOCD 配置: $CFG_PATH"

/usr/bin/cmake --build "$PROJECT_ROOT/build"

/usr/bin/openocd \
  -s /usr/share/openocd/scripts \
  -f "$CFG_PATH" \
  -c "tcl_port disabled" \
  -c "gdb_port disabled" \
  -c "tcl_port disabled" \
  -c "program \"$ELF_PATH\"" \
  -c reset \
  -c shutdown

