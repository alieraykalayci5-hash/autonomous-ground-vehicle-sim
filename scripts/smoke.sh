#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

cmake -S . -B build -G Ninja
cmake --build build -j

run_and_get_hash() {
  local outdir="$1"
  shift
  mkdir -p "$outdir"
  ./build/agv_sim --out "$outdir" "$@" 2> "$outdir/stderr.txt" 1>/dev/null
  grep "FNV1A64=" "$outdir/stderr.txt" | sed 's/.*FNV1A64=//'
}

mkdir -p out_smoke

H1=$(run_and_get_hash out_smoke/static --mode static --seed 123 --steps 900 --dt 0.02 --hash 1)
H2=$(run_and_get_hash out_smoke/moving --mode moving --seed 123 --steps 900 --dt 0.02 --noise_pos 2.0 --hash 1)
H3=$(run_and_get_hash out_smoke/replan --mode replan_demo --seed 123 --steps 900 --dt 0.02 --hash 1)

echo "[SMOKE] static  = $H1"
echo "[SMOKE] moving  = $H2"
echo "[SMOKE] replan  = $H3"

if [[ ! -f out_smoke/expected.txt ]]; then
  echo "$H1" > out_smoke/expected.txt
  echo "$H2" >> out_smoke/expected.txt
  echo "$H3" >> out_smoke/expected.txt
  echo "[SMOKE] expected.txt created (first run). Re-run smoke to verify determinism."
  exit 0
fi

E1=$(sed -n '1p' out_smoke/expected.txt)
E2=$(sed -n '2p' out_smoke/expected.txt)
E3=$(sed -n '3p' out_smoke/expected.txt)

echo "[SMOKE] expected static = $E1"
echo "[SMOKE] expected moving = $E2"
echo "[SMOKE] expected replan = $E3"

if [[ "$H1" != "$E1" || "$H2" != "$E2" || "$H3" != "$E3" ]]; then
  echo "[SMOKE] FAIL"
  exit 2
fi

echo "[SMOKE] PASS"
