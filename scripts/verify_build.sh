#!/usr/bin/env bash
# verify_build.sh — Build verification for platform-layer refactoring
# リファクタリング中の全ターゲットビルド検証スクリプト
#
# Usage:
#   source ~/esp/esp-idf/export.sh
#   ./scripts/verify_build.sh [--quick]
#
# --quick: vehicle のみビルド（高速チェック用）

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TARGETS=("vehicle" "controller" "workshop")
FAILED=()
PASSED=()

# Parse args
QUICK=false
if [[ "${1:-}" == "--quick" ]]; then
    QUICK=true
    TARGETS=("vehicle")
fi

echo "========================================="
echo " Build Verification"
echo " Targets: ${TARGETS[*]}"
echo "========================================="
echo ""

for target in "${TARGETS[@]}"; do
    target_dir="$REPO_ROOT/firmware/$target"
    if [[ ! -d "$target_dir" ]]; then
        echo "[SKIP] $target — directory not found"
        continue
    fi

    echo "[BUILD] $target ..."
    if (cd "$target_dir" && idf.py build > /tmp/verify_build_${target}.log 2>&1); then
        echo "[  OK] $target"
        PASSED+=("$target")
    else
        echo "[FAIL] $target — see /tmp/verify_build_${target}.log"
        FAILED+=("$target")
    fi
    echo ""
done

# Also check custom firmware template can be found
if [[ "$QUICK" == false ]]; then
    template_dir="$REPO_ROOT/lib/sfcli/templates/custom_firmware"
    if [[ -d "$template_dir" ]]; then
        echo "[  OK] custom_firmware template exists"
    else
        echo "[WARN] custom_firmware template not found at $template_dir"
    fi
fi

echo ""
echo "========================================="
echo " Results"
echo "========================================="
echo " Passed: ${#PASSED[@]} (${PASSED[*]:-none})"
echo " Failed: ${#FAILED[@]} (${FAILED[*]:-none})"
echo "========================================="

if [[ ${#FAILED[@]} -gt 0 ]]; then
    exit 1
fi
