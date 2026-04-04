#!/bin/bash

# 사용법: ./sync-patches.sh "커밋 메시지"
if [ -z "$1" ]; then
    echo "사용법: ./sync-patches.sh \"커밋 메시지\""
    exit 1
fi

COMMIT_MSG="$1"
MAIN_DIR="/home/sangukbae/autodrive"
SIM_DIR="$MAIN_DIR/AutoDRIVE-Simulator"
SIM_PATCH_DIR="$MAIN_DIR/AutoDRIVE-Simulator-patches"

echo "=== AutoDRIVE-Simulator 수정 파일 복사 ==="
cd "$SIM_DIR"

# staged + unstaged 변경 파일 모두 수집
CHANGED_FILES=$(git diff --staged --name-only; git diff --name-only)

if [ -z "$CHANGED_FILES" ]; then
    echo "변경된 파일 없음"
else
    echo "$CHANGED_FILES" | while read f; do
        if [ -f "$f" ]; then
            mkdir -p "$SIM_PATCH_DIR/$(dirname "$f")"
            cp "$f" "$SIM_PATCH_DIR/$f"
            echo "  복사: $f"
        fi
    done
fi

echo ""
echo "=== 메인 레포 커밋 및 푸시 ==="
cd "$MAIN_DIR"
git add AutoDRIVE-Simulator-patches/ AutoDRIVE-Devkit/
git commit -m "$COMMIT_MSG"
git push

echo ""
echo "완료!"
