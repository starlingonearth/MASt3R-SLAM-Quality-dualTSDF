#!/usr/bin/env bash
set -euo pipefail

# Options:
#   -n | --no-calib     run without calibration (use eval_no_calib.yaml)
#   -p | --print-only   print commands without executing

no_calib=ture
print_only=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    -n|--no-calib) no_calib=true; shift ;;
    -p|--print-only) print_only=true; shift ;;
    --) shift; break ;;
    *) echo "Unknown option: $1"; shift ;;
  esac
done

dataset_root="datasets/Replica"
sequences=(
  office0 office1 office2 office3 office4
  room0 room1 room2
)

for seq in "${sequences[@]}"; do
  dataset="${dataset_root}/${seq}" 

  if [ "$no_calib" = true ]; then
    cfg="config/eval_no_calib.yaml"
    save_as="replica/no_calib/${seq}"
  else
    cfg="config/eval_replica.yaml"
    save_as="replica/calib/${seq}"
  fi

  cmd="python main.py \
      --dataset ${dataset} \
      --no-viz \
      --save-as ${save_as} \
      --config ${cfg}"

  echo "[RUN] ${cmd}"
  if [ "$print_only" != true ]; then
    eval ${cmd}
  fi
done
