#!/bin/bash
# Unity Editor 2022.3.52f1 설치 스크립트
# 컨테이너 실행 후 최초 1회 실행
# 사전 조건: unityhub GUI에서 로그인 및 라이선스 활성화 완료

set -e

UNITY_VERSION="2022.3.52f1"
UNITY_CHANGESET="1120fcb54228"
EDITOR_PATH="/root/Unity/Hub/Editor/${UNITY_VERSION}"

if [ -d "$EDITOR_PATH" ]; then
    echo "Unity Editor ${UNITY_VERSION} 이미 설치되어 있습니다."
    exit 0
fi

echo "=== Unity Editor ${UNITY_VERSION} 설치 시작 ==="
unityhub --no-sandbox --headless install \
    --version "${UNITY_VERSION}" \
    --changeset "${UNITY_CHANGESET}" \
    --module linux-il2cpp \
    --childModules

echo "=== 설치 완료: ${EDITOR_PATH} ==="
