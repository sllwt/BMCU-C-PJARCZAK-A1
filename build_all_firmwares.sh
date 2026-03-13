#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

command -v pio >/dev/null 2>&1 || { echo "ERROR: nie ma 'pio' w PATH"; exit 1; }
command -v python3 >/dev/null 2>&1 || { echo "ERROR: nie ma 'python3' w PATH"; exit 1; }

OUT_DIR="firmwares"
PIO_ENV="fw"

TXT_MODE="which_to_choose_mode.txt"
TXT_AUTOLOAD="which_to_choose_autoload.txt"
TXT_RGB="which_to_choose_filament_rgb.txt"
TXT_SLOTS="which_to_choose_slots.txt"
OUT_GUIDE="which_to_choose.txt"

[[ -f "${TXT_MODE}" ]]     || { echo "ERROR: brak ${TXT_MODE}"; exit 1; }
[[ -f "${TXT_AUTOLOAD}" ]] || { echo "ERROR: brak ${TXT_AUTOLOAD}"; exit 1; }
[[ -f "${TXT_RGB}" ]]      || { echo "ERROR: brak ${TXT_RGB}"; exit 1; }
[[ -f "${TXT_SLOTS}" ]]    || { echo "ERROR: brak ${TXT_SLOTS}"; exit 1; }

MODE_A1_DIR="standard(A1)"
MODE_P1S_DIR="high_force_load(P1S)"

SOLO_RETRACT="0.095f"
RETRACTS=(
  "0.10" "0.20" "0.25" "0.30" "0.35" "0.40"
  "0.45" "0.50" "0.55" "0.60" "0.65"
  "0.70" "0.75" "0.80" "0.85" "0.90"
)

build_and_copy() {
  local out_path="$1"
  local ams_num="$2"
  local retract_len="$3"
  local dm="$4"
  local rgb="$5"
  local p1s="$6"

  echo "=== BUILD: P1S=${p1s} DM=${dm} RGB=${rgb} AMS_NUM=${ams_num} RETRACT=${retract_len} -> ${out_path}"

  BAMBU_BUS_AMS_NUM="${ams_num}" \
  AMS_RETRACT_LEN="${retract_len}" \
  BMCU_DM_TWO_MICROSWITCH="${dm}" \
  BMCU_ONLINE_LED_FILAMENT_RGB="${rgb}" \
  DBMCU_P1S="${p1s}" \
  pio run -e "${PIO_ENV}"

  local src=".pio/build/${PIO_ENV}/firmware.bin"
  [[ -f "${src}" ]] || { echo "ERROR: brak ${src}"; exit 1; }

  mkdir -p "$(dirname "${out_path}")"
  cp -f "${src}" "${out_path}"
}

rm -rf "${OUT_DIR}"
mkdir -p "${OUT_DIR}"

cp -f "${TXT_MODE}" "${OUT_DIR}/${OUT_GUIDE}"

# 修改编译版本
for p1s in 0; do
  mode_dir="${MODE_A1_DIR}"
  mode_base="${OUT_DIR}/${mode_dir}"
  mkdir -p "${mode_base}"
  cp -f "${TXT_AUTOLOAD}" "${mode_base}/${OUT_GUIDE}"

  # 只编译自动加载开启 (dm=1)
  for dm in 1; do
    dm_dir="AUTOLOAD"
    dm_base="${mode_base}/${dm_dir}"
    mkdir -p "${dm_base}"
    cp -f "${TXT_RGB}" "${dm_base}/${OUT_GUIDE}"

    # 只编译RGB开启 (rgb=1)
    for rgb in 1; do
      rgb_dir="FILAMENT_RGB_ON"
      base="${dm_base}/${rgb_dir}"
      mkdir -p "${base}/AMS_A"  # 只创建AMS_A文件夹
      cp -f "${TXT_SLOTS}" "${base}/${OUT_GUIDE}"

      # 不编译SOLO版本，只编译AMS_A
      for slot in A; do
        ams_num=0
        # 编译所有回抽长度
        for r in "${RETRACTS[@]}"; do
          build_and_copy \
            "${base}/AMS_${slot}/ams_${slot,,}_${r}f.bin" \
            "${ams_num}" \
            "${r}f" \
            "${dm}" \
            "${rgb}" \
            "${p1s}"
        done
      done
    done
  done
done

python3 - "${OUT_DIR}" > "${OUT_DIR}/manifest.txt" <<'PY'
import sys, os, zlib, hashlib

root = sys.argv[1]
entries = []

for dirpath, _, filenames in os.walk(root):
    for fn in filenames:
        p = os.path.join(dirpath, fn)
        rel = os.path.relpath(p, root).replace(os.sep, "/")

        crc = 0
        size = 0
        h = hashlib.sha256()

        with open(p, "rb") as f:
            while True:
                b = f.read(1024 * 1024)
                if not b:
                    break
                size += len(b)
                crc = zlib.crc32(b, crc)
                h.update(b)

        entries.append((rel, h.hexdigest(), f"{crc & 0xffffffff:08X}", size))

entries.sort(key=lambda x: x[0])

out = sys.stdout
out.write("# format: SHA256_HEX CRC32_HEX SIZE_BYTES REL_PATH\n")
for rel, sha256_hex, crc32_hex, size in entries:
    out.write(f"{sha256_hex} {crc32_hex} {size} {rel}\n")
PY

echo
echo "DONE. Wyniki w: ${OUT_DIR}/"
echo "Manifest: ${OUT_DIR}/manifest.txt"
