#!/usr/bin/env bash
set -euo pipefail


REPO="/home/kickpi/Project/CanServer"
SUB="can_server"
SRC=(CMakeLists.txt package.xml main.cpp src include launch config)

cd "$REPO"

echo "==> 1. 拉取远程最新"
git pull --no-rebase

echo "==> 2. 同步源码到 $SUB/"
mkdir -p "$SUB"
rsync -a --delete "${SRC[@]}" "$SUB/"

echo "==> 3. 检查 $SUB/ 是否有改动"
if [[ -z "$(git status --porcelain -- "$SUB")" ]]; then
    echo "    $SUB/ 无改动，退出。"
    exit 0
fi

git status --short -- "$SUB"

echo "==> 4. 提交"
git add "$SUB"
git commit -m "sync: ${1:-$(date '+%F %T')}"

echo "==> 5. 推送"
git push

echo "==> 完成"