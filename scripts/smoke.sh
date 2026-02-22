#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# Build (always deterministic for a given compiler/toolchain)
cmake -S . -B build -G Ninja
cmake --build build -j

EXE="./build/agv_sim"

mkdir -p out_smoke_static out_smoke_moving out_smoke_replan

# Run 3 canonical scenarios
STATIC_HASH="$($EXE --mode static --seed 123 --steps 1200 --dt 0.02 --out out_smoke_static --hash 1 2>&1 | tail -n 1 | sed 's/\r//')"
MOVING_HASH="$($EXE --mode moving --seed 123 --steps 1200 --dt 0.02 --noise_pos 2.0 --p_detect 1.0 --out out_smoke_moving --hash 1 2>&1 | tail -n 1 | sed 's/\r//')"
REPLAN_HASH="$($EXE --mode replan_demo --seed 123 --dt 0.02 --out out_smoke_replan --hash 1 2>&1 | tail -n 1 | sed 's/\r//')"

echo "[SMOKE] static = ${STATIC_HASH#FNV1A64=}"
echo "[SMOKE] moving = ${MOVING_HASH#FNV1A64=}"
echo "[SMOKE] replan = ${REPLAN_HASH#FNV1A64=}"

EXPECTED_FILE="scripts/expected.txt"

if [[ ! -f "$EXPECTED_FILE" ]]; then
  cat > "$EXPECTED_FILE" <<EOF
static=${STATIC_HASH#FNV1A64=}
moving=${MOVING_HASH#FNV1A64=}
replan=${REPLAN_HASH#FNV1A64=}
EOF
  echo "[SMOKE] expected.txt created (first run). Re-run smoke to verify determinism."
  exit 0
fi

# Parse expected
EXP_STATIC="$(grep '^static=' "$EXPECTED_FILE" | cut -d= -f2 | tr -d '\r')"
EXP_MOVING="$(grep '^moving=' "$EXPECTED_FILE" | cut -d= -f2 | tr -d '\r')"
EXP_REPLAN="$(grep '^replan=' "$EXPECTED_FILE" | cut -d= -f2 | tr -d '\r')"

GOT_STATIC="${STATIC_HASH#FNV1A64=}"
GOT_MOVING="${MOVING_HASH#FNV1A64=}"
GOT_REPLAN="${REPLAN_HASH#FNV1A64=}"

echo "[SMOKE] expected static = $EXP_STATIC"
echo "[SMOKE] expected moving = $EXP_MOVING"
echo "[SMOKE] expected replan = $EXP_REPLAN"

FAIL=0
if [[ "$GOT_STATIC" != "$EXP_STATIC" ]]; then echo "[SMOKE] FAIL static: got=$GOT_STATIC"; FAIL=1; fi
if [[ "$GOT_MOVING" != "$EXP_MOVING" ]]; then echo "[SMOKE] FAIL moving: got=$GOT_MOVING"; FAIL=1; fi
if [[ "$GOT_REPLAN" != "$EXP_REPLAN" ]]; then echo "[SMOKE] FAIL replan: got=$GOT_REPLAN"; FAIL=1; fi

if [[ "$FAIL" -ne 0 ]]; then
  echo "[SMOKE] FAIL. If you intentionally changed behavior/output, delete scripts/expected.txt and re-run smoke to re-baseline."
  exit 1
fi

echo "[SMOKE] PASS"