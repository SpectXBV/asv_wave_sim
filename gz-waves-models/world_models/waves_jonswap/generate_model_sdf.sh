#!/usr/bin/env bash
# Regenerate model.sdf from model.sdf.template + wave_params.env.
#
# Usage:
#   1. Edit wave_params.env (hs, tp, gamma, wave_direction_deg, etc).
#   2. Run this script.
#   3. Re-launch the sim - model.sdf now has matching physics and visual
#      wave parameters.
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir"

set -a
source ./wave_params.env
set +a

envsubst \
  '${WAVE_ALGORITHM} ${WAVE_TILE_SIZE} ${WAVE_CELL_COUNT} ${WAVE_HS} ${WAVE_TP} ${WAVE_GAMMA} ${WAVE_DIRECTION_DEG} ${WAVE_DIRECTION_SPREAD} ${WAVE_STEEPNESS}' \
  < model.sdf.template > model.sdf

echo "Regenerated model.sdf from wave_params.env:"
grep -A9 '<wave>' model.sdf | head -10
