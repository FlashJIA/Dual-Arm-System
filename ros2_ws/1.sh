#!/usr/bin/env bash
set -euo pipefail

# ==========================================
#       用户自定义配置 (已为你修改默认值)
# ==========================================

# 1. 默认世界名称设为 "empty"
WORLD="${WORLD:-empty}"

# 2. 桌子中心 X 坐标设为 0.5
#    桌子长 2.0米，中心在 0.5，范围就是 [-0.5, 1.5]
#    这样正好托住位于 0.0 和 1.0 的两个机械臂
TABLE_CENTER_X="${TABLE_CENTER_X:-0.5}"
TABLE_CENTER_Y="${TABLE_CENTER_Y:-0.0}"

TABLE_TOP_Z="${TABLE_TOP_Z:-0.30}"
TABLE_SIZE_X="${TABLE_SIZE_X:-2.0}"
TABLE_SIZE_Y="${TABLE_SIZE_Y:-1.0}"
TABLE_THICK="${TABLE_THICK:-0.05}"

# 中间方块和棋盘格的中心 X (基于桌子坐标系)
X="${X:-0.45}"

# 格子间距
SPACING="${SPACING:-0.08}"

# 方块/圆柱尺寸
BLOCK_SIZE="${BLOCK_SIZE:-0.04}"
BLOCK_DROP="${BLOCK_DROP:-0.01}"

# 质量/密度
BLOCK_MASS="${BLOCK_MASS:-0.02}"       # kg
BLOCK_DENSITY="${BLOCK_DENSITY:-}"     # kg/m^3

# 物理摩擦参数
BLOCK_MU="${BLOCK_MU:-2.0}"
BLOCK_MU2="${BLOCK_MU2:-2.0}"
BLOCK_TORSION="${BLOCK_TORSION:-0.5}"
BLOCK_SLIP1="${BLOCK_SLIP1:-0.0}"
BLOCK_SLIP2="${BLOCK_SLIP2:-0.0}"
BLOCK_KP="${BLOCK_KP:-1e5}"
BLOCK_KD="${BLOCK_KD:-10.0}"
BLOCK_MIN_DEPTH="${BLOCK_MIN_DEPTH:-1e-5}"
BLOCK_MAX_VEL="${BLOCK_MAX_VEL:-0.10}"

ALLOW_RENAME="${ALLOW_RENAME:-false}"
SWEEP_SUFFIXES="${SWEEP_SUFFIXES:-true}"
MAX_SUFFIX="${MAX_SUFFIX:-9}"
WORLD_SDF="${WORLD_SDF:-/tmp/lab06_empty_usercmd.world.sdf}"
TILE_THICK="${TILE_THICK:-0.005}"

# ==========================================

need() { command -v "$1" >/dev/null 2>&1 || { echo "ERROR: $1 not found"; exit 1; }; }
need gz
need python3

# ---------- ensure world ----------
ensure_world() {
  # 如果目前还没有检测到 /world/empty/create，尝试去寻找正在运行的世界
  if ! gz service -l | grep -q "/world/${WORLD}/create"; then
    echo "[spawn] Checking for running worlds..."
    DETECTED=$(gz service -l | sed -n 's#.*/world/\([^/]*\)/create.*#\1#p' | head -n1 || true)
    
    if [[ -n "$DETECTED" ]]; then
        WORLD="$DETECTED"
        echo "[spawn] Auto-detected world: ${WORLD}"
    else
        echo "[spawn] No world detected, assuming default: ${WORLD}"
    fi
  else
    echo "[spawn] World '${WORLD}' service found."
  fi

  # 如果真的没有服务，可能需要启动一个新的（通常不需要这步，因为你已经用 launch 启动了）
  if ! gz service -l | grep -q "/world/"; then
    cat > "$WORLD_SDF" <<'SDF'
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="empty">
    <gravity>0 0 -9.81</gravity>
    <plugin name="scene_broadcaster" filename="gz-sim-scene-broadcaster-system"/>
    <plugin name="user_commands" filename="gz-sim-user-commands-system"/>
  </world>
</sdf>
SDF
    echo "[spawn] Launching headless world: $WORLD_SDF"
    nohup gz sim -r "$WORLD_SDF" >/tmp/lab06_world.log 2>&1 &
    sleep 1.0
  fi
  
  # 等待 UserCommands 服务就绪
  echo -n "[spawn] Waiting for /world/${WORLD}/set_pose ... "
  for _ in {1..60}; do
    if gz service -l | grep -q "/world/${WORLD}/set_pose"; then
      echo "OK"
      return
    fi
    sleep 0.25
  done
  echo "TIMEOUT"
  echo "ERROR: /world/${WORLD}/set_pose not available. Check that UserCommands is loaded." >&2
  # 不退出，尝试继续运行，有时候只是 grep 慢了
}

# choose CLI flag style
USE_LONG_FLAGS=false
if gz service --help 2>/dev/null | grep -q -- '--reqtype'; then
  USE_LONG_FLAGS=true
fi

# ---------- service helpers ----------
call_create() {
  local sdf_path="$1" name="$2" x="$3" y="$4" z="$5"
  local payload
  payload=$(cat <<PBUF
sdf_filename: "$sdf_path"
name: "$name"
pose { position { x: $x y: $y z: $z } }
allow_renaming: ${ALLOW_RENAME}
PBUF
)
  if $USE_LONG_FLAGS; then
    gz service -s "/world/${WORLD}/create" \
      --reqtype gz.msgs.EntityFactory \
      --reptype gz.msgs.Boolean \
      --timeout 3000 \
      --req "$payload" >/dev/null
  else
    gz service -s "/world/${WORLD}/create" \
      -m gz.msgs.EntityFactory \
      -r gz.msgs.Boolean \
      -p "$payload" >/dev/null
  fi
}

remove_model_once() {
  local name="$1"
  local payload_sym=$'name: "'"$name"$'"\n'"type: MODEL"
  local payload_num=$'name: "'"$name"$'"\n'"type: 2"
  if $USE_LONG_FLAGS; then
    gz service -s "/world/${WORLD}/remove" \
      --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 3000 \
      --req "$payload_sym" >/dev/null 2>&1 \
    || gz service -s "/world/${WORLD}/remove" \
      --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 3000 \
      --req "$payload_num" >/dev/null 2>&1 || true
  else
    gz service -s "/world/${WORLD}/remove" \
      -m gz.msgs.Entity -r gz.msgs.Boolean -p "$payload_sym" >/dev/null 2>&1 \
    || gz service -s "/world/${WORLD}/remove" \
      -m gz.msgs.Entity -r gz.msgs.Boolean -p "$payload_num" >/dev/null 2>&1 || true
  fi
}

remove_model() {
  local base="$1"
  remove_model_once "$base"
  if [[ "$SWEEP_SUFFIXES" == "true" ]]; then
    for i in $(seq 1 "$MAX_SUFFIX"); do
      remove_model_once "${base}_${i}"
    done
  fi
}

# ---------- write SDFs ----------
SDF_DIR=/tmp/lab06_spawn
mkdir -p "$SDF_DIR"

export BLOCK_SIZE BLOCK_MASS BLOCK_DENSITY
BLOCK_MASS_COMPUTED=$(python3 - <<'PY'
import os
a=float(os.environ.get("BLOCK_SIZE","0.04"))
dens=os.environ.get("BLOCK_DENSITY","").strip()
if dens:
    m=float(dens)*(a**3)
else:
    m=float(os.environ.get("BLOCK_MASS","0.02"))
print(f"{m:.8f}")
PY
)
export BLOCK_MASS_COMPUTED

BLOCK_INERTIA_CUBE=$(python3 - <<'PY'
import os
a=float(os.environ.get("BLOCK_SIZE","0.04"))
m=float(os.environ.get("BLOCK_MASS_COMPUTED","0.02"))
I=(m*(a**2))/6.0
print(f"{I:.8e}")
PY
)
export BLOCK_INERTIA_CUBE

CYL_INERTIA_XX=$(python3 - <<'PY'
import os
m=float(os.environ.get("BLOCK_MASS_COMPUTED","0.02"))
r=float(os.environ.get("BLOCK_SIZE","0.04"))/2.0
h=float(os.environ.get("BLOCK_SIZE","0.04"))
Ixx = (1.0/12.0)*m*(3*(r**2) + h**2)
Izz = 0.5 * m * (r**2)
print(f"{Ixx:.8e} {Izz:.8e}")
PY
)
CYL_IXX=$(echo $CYL_INERTIA_XX | awk '{print $1}')
CYL_IZZ=$(echo $CYL_INERTIA_XX | awk '{print $2}')
export CYL_IXX CYL_IZZ

# --- Table SDF ---
TABLE_SDF="${SDF_DIR}/table_${TABLE_SIZE_X}x${TABLE_SIZE_Y}x${TABLE_THICK}.sdf"
cat > "$TABLE_SDF" <<EOF
<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="lab06_table">
    <static>true</static>
    <link name="top">
      <collision name="c">
        <geometry><box><size>${TABLE_SIZE_X} ${TABLE_SIZE_Y} ${TABLE_THICK}</size></box></geometry>
      </collision>
      <visual name="v">
        <geometry><box><size>${TABLE_SIZE_X} ${TABLE_SIZE_Y} ${TABLE_THICK}</size></box></geometry>
      </visual>
    </link>
  </model>
</sdf>
EOF

write_cube_sdf() {
  local path="$1" r="$2" g="$3" b="$4" model_name="${5:-cube_40mm}"
  cat > "$path" <<EOF
<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="${model_name}">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>${BLOCK_MASS_COMPUTED}</mass>
        <inertia>
          <ixx>${BLOCK_INERTIA_CUBE}</ixx>
          <iyy>${BLOCK_INERTIA_CUBE}</iyy>
          <izz>${BLOCK_INERTIA_CUBE}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="c">
        <geometry><box><size>${BLOCK_SIZE} ${BLOCK_SIZE} ${BLOCK_SIZE}</size></box></geometry>
        <surface>
          <friction>
            <ode>
              <mu>${BLOCK_MU}</mu>
              <mu2>${BLOCK_MU2}</mu2>
              <slip1>${BLOCK_SLIP1}</slip1>
              <slip2>${BLOCK_SLIP2}</slip2>
            </ode>
            <torsional>
              <coefficient>${BLOCK_TORSION}</coefficient>
              <use_patch_radius>1</use_patch_radius>
              <patch_radius>0.02</patch_radius>
              <surface_radius>0.02</surface_radius>
            </torsional>
          </friction>
          <contact>
            <ode>
              <kp>${BLOCK_KP}</kp>
              <kd>${BLOCK_KD}</kd>
              <max_vel>${BLOCK_MAX_VEL}</max_vel>
              <min_depth>${BLOCK_MIN_DEPTH}</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>
      <visual name="v">
        <geometry><box><size>${BLOCK_SIZE} ${BLOCK_SIZE} ${BLOCK_SIZE}</size></box></geometry>
        <material>
          <ambient>${r} ${g} ${b} 1</ambient>
          <diffuse>${r} ${g} ${b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
EOF
}

write_tile_sdf() {
  local path="$1" sx="$2" sy="$3" thick="$4" r="$5" g="$6" b="$7"
  cat > "$path" <<EOF
<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="board_tile">
    <static>true</static>
    <link name="link">
      <collision name="c">
        <geometry><box><size>${sx} ${sy} ${thick}</size></box></geometry>
      </collision>
      <visual name="v">
        <geometry><box><size>${sx} ${sy} ${thick}</size></box></geometry>
        <material>
          <ambient>${r} ${g} ${b} 1</ambient>
          <diffuse>${r} ${g} ${b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
EOF
}

write_cyl_sdf() {
  local path="$1" model_name="$2" r="$3" g="$4" b="$5"
  local radius=$(python3 - <<PY
import os
print(f"{float(os.environ.get('BLOCK_SIZE','0.04'))/2.0:.8f}")
PY
)
  local height="${BLOCK_SIZE}"
  cat > "$path" <<EOF
<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="${model_name}">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>${BLOCK_MASS_COMPUTED}</mass>
        <inertia>
          <ixx>${CYL_IXX}</ixx>
          <iyy>${CYL_IXX}</iyy>
          <izz>${CYL_IZZ}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="c">
        <geometry>
          <cylinder>
            <radius>${radius}</radius>
            <length>${height}</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="v">
        <geometry>
          <cylinder>
            <radius>${radius}</radius>
            <length>${height}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>${r} ${g} ${b} 1</ambient>
          <diffuse>${r} ${g} ${b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
EOF
}

# --- prepare red cube SDF file ---
CUBE_RED_SDF="${SDF_DIR}/cube_red.sdf"
write_cube_sdf "$CUBE_RED_SDF" 0.85 0.10 0.10 "cube_red"

# ---------- run ----------
ensure_world

echo "[spawn] Cleaning up previous lab06 entities..."
remove_model lab06_table
for r in -1 0 1; do
  for c in -1 0 1; do
    remove_model "lab06_tile_${r}_${c}"
  done
done
remove_model lab06_block_red
remove_model lab06_block_red_left
remove_model lab06_block_red_right
remove_model lab06_block_red_row2_a
remove_model lab06_block_red_row2_b
for i in 1 2 3 4 5; do
  remove_model "lab06_cyl_${i}"
done

# spawn table (Using the hardcoded 0.5 center X)
TABLE_CENTER_Z=$(python3 - <<PY
top=${TABLE_TOP_Z}; th=${TABLE_THICK}
print(top - th/2.0)
PY
)
call_create "$TABLE_SDF" "lab06_table" "$TABLE_CENTER_X" "$TABLE_CENTER_Y" "$TABLE_CENTER_Z"
echo "[spawn] Spawned table; top @ z=${TABLE_TOP_Z} Center X=${TABLE_CENTER_X}"

SHIFT_X=$(python3 - <<PY
import os
sp=float(os.environ.get("SPACING","0.08"))
print(f"{2*sp:.8f}")
PY
)

BOARD_CENTER_X=$(python3 - <<PY
base=${X}
shift=${SHIFT_X}
print(f"{base + float(shift):.8f}")
PY
)

TILE_SX="${SPACING}"
TILE_SY="${SPACING}"

for r in -1 0 1; do
  for c in -1 0 1; do
    TILE_X=$(python3 - <<PY
base=${X}
off=${c}
sp=${SPACING}
shift=${SHIFT_X}
print(f"{base + off*sp + float(shift):.8f}")
PY
)
    TILE_Y=$(python3 - <<PY
base=0.0
off=${r}
sp=${SPACING}
print(f"{base + off*sp:.8f}")
PY
)
    TILE_Z=$(python3 - <<PY
top=${TABLE_TOP_Z}
th=${TILE_THICK}
print(f"{top + th/2.0:.8f}")
PY
)
    parity=$(( (r + c) & 1 ))
    if [[ "$parity" -eq 0 ]]; then
      TR="0.0"; TG="0.0"; TB="0.0"
    else
      TR="1.0"; TG="1.0"; TB="1.0"
    fi

    TILE_SDF="${SDF_DIR}/tile_${r}_${c}.sdf"
    write_tile_sdf "$TILE_SDF" "${TILE_SX}" "${TILE_SY}" "${TILE_THICK}" "${TR}" "${TG}" "${TB}"
    call_create "$TILE_SDF" "lab06_tile_${r}_${c}" "$TILE_X" "$TILE_Y" "$TILE_Z"
  done
done

BLOCK_Z_TABLE=$(python3 - <<PY
top=${TABLE_TOP_Z}
sz=${BLOCK_SIZE}
drop=${BLOCK_DROP}
print(f"{top + sz/2.0 + drop:.8f}")
PY
)
call_create "$CUBE_RED_SDF"  "lab06_block_red"   "$X"   "0.0"        "$BLOCK_Z_TABLE"

BLOCK_TOP_Z="$BLOCK_Z_TABLE"
BLOCK_POS_PY=$(python3 - <<PY
sp=${SPACING}
print(f"{sp:.8f}")
PY
)
BLOCK_POS_NY=$(python3 - <<PY
sp=${SPACING}
print(f"{-sp:.8f}")
PY
)
call_create "$CUBE_RED_SDF" "lab06_block_red_left"  "$X"  "${BLOCK_POS_PY}" "$BLOCK_TOP_Z"
call_create "$CUBE_RED_SDF" "lab06_block_red_right" "$X"  "${BLOCK_POS_NY}" "$BLOCK_TOP_Z"

ROW2_X=$(python3 - <<PY
base=${X}
sp=${SPACING}
print(f"{base - sp:.8f}")
PY
)
ROW2_Y_A=$(python3 - <<PY
sp=${SPACING}
print(f"{-sp/2.0:.8f}")
PY
)
ROW2_Y_B=$(python3 - <<PY
sp=${SPACING}
print(f"{sp/2.0:.8f}")
PY
)

call_create "$CUBE_RED_SDF" "lab06_block_red_row2_a" "$ROW2_X" "$ROW2_Y_A" "$BLOCK_TOP_Z"
call_create "$CUBE_RED_SDF" "lab06_block_red_row2_b" "$ROW2_X" "$ROW2_Y_B" "$BLOCK_TOP_Z"

CYL_Z_ON_TILES=$(python3 - <<PY
top=${TABLE_TOP_Z}
th=${TILE_THICK}
h=${BLOCK_SIZE}
print(f"{top + th + h/2.0:.8f}")
PY
)

i=1
for pos in \
  "${X} 0.0" \
  "${X} ${BLOCK_POS_PY}" \
  "${X} ${BLOCK_POS_NY}" \
  "${ROW2_X} ${ROW2_Y_A}" \
  "${ROW2_X} ${ROW2_Y_B}"; do

  old_x=$(echo $pos | awk '{print $1}')
  old_y=$(echo $pos | awk '{print $2}')
  new_x=$(python3 - <<PY
board_c=${BOARD_CENTER_X}
x=float(${old_x})
print(f"{2*float(board_c) - x:.8f}")
PY
)
  new_y="${old_y}"

  CYL_SDF="${SDF_DIR}/cyl_blue_${i}.sdf"
  CYL_MODEL_NAME="cyl_blue_${i}"
  write_cyl_sdf "$CYL_SDF" "$CYL_MODEL_NAME" 0.10 0.10 0.85

  call_create "$CYL_SDF" "lab06_cyl_${i}" "$new_x" "$new_y" "$CYL_Z_ON_TILES"
  i=$((i+1))
done

echo "[spawn] Spawned center red cube at x=${X}, y=0.0, z=${BLOCK_TOP_Z} (on table)"
echo "[spawn] Spawned side blocks at (X, ±SPACING): (X,${BLOCK_POS_PY}), (X,${BLOCK_POS_NY})"
echo "[spawn] Spawned second-row two reds at x=${ROW2_X}, y=${ROW2_Y_A},${ROW2_Y_B} (closer to arm in X by SPACING)"
echo "[spawn] Spawned 3x3 checkerboard tiles shifted +${SHIFT_X} m along table-length (X) direction (thickness=${TILE_THICK} m) on table top at z=${TABLE_TOP_Z}"
echo "[spawn] Spawned 5 blue cylinders mirrored across chessboard center axis X=${BOARD_CENTER_X} (on tiles), diameter & height = ${BLOCK_SIZE} m"
echo "[spawn] Table size: ${TABLE_SIZE_X} x ${TABLE_SIZE_Y} x ${TABLE_THICK} (Center X: ${TABLE_CENTER_X})"
echo "[spawn] Done."