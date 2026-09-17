#!/bin/bash
# X5 md-v0p2: switch the UART7 RX pin between PPS input and normal UART.
# Run as root on the board. No DTS edits or flashing are required. The switch
# takes effect immediately and persists across reboots until changed again. Any
# failed step is covered by trap-based recovery.
#
# Usage:
#   x5-pps-pin.sh --check                      # Probe environment and print the plan only
#   x5-pps-pin.sh pps  [--edge rising|falling|both] [--keep-uart] [--force] [--once]
#   x5-pps-pin.sh uart [--force] [--once]      # Restore normal UART
#   x5-pps-pin.sh status                       # mux / module / /dev/pps* / interrupt count / persisted state
#   x5-pps-pin.sh test [--secs 10]             # Verify interrupt count and PPS sequence increase
#   x5-pps-pin.sh install                      # Install only the boot hook; pps/uart install it automatically
#   x5-pps-pin.sh uninstall                    # Remove hook and persisted config, restoring factory boot behavior
#   x5-pps-pin.sh boot                         # Internal boot-hook entry; do not run manually
#
# Persistence model without source-tree edits, rootfs edits, or reflashing:
#   The mode is stored in <script-dir>/mode, and the boot hook is written to
#   /userdata/startup.sh. This vendor extension point is run at the end of boot
#   by identical /etc/init.d/S99auto_startup files on both buildroot and jammy
#   rootfs variants, so it works with SysV and systemd without changing rootfs.
#   This tree uses HR_SYSTEM_VERIFY="dm-verity"; writing /etc risks bricking, so
#   this script intentionally avoids it.
#   /userdata is an independent ext4 partition and is not cleared by firmware
#   reflashing, so configuration is more durable than firmware.
#   `--once` changes only the current run and leaves persisted config untouched.
#   Emergency switch: touch <script-dir>/DISABLE to make boot do nothing.
#
# Dependency: x5pps.ko in the same directory or $X5PPS_KO, with vermagic
# matching board uname -r. For boot auto-apply, both this script and the ko must
# live in a path that survives reboot, preferably /userdata/x5-pps/.
#
# Two important false positives:
#   1) The board already has /dev/pps0, but it is fake. REAL_PPS_ENABLE in
#      kernel/drivers/pps/clients/hobot-pps.c:25 is commented out, so it runs a
#      1-second mod_timer fake event unrelated to any pin, while x5-rdk.dtsi:869
#      marks it status="okay". Use name == x5pps, not hobot-pps.-1.
#   2) gpio value readback is unreliable because of dwapb EXT input gating.
#      Judge mux only by register readback and
#      /sys/kernel/debug/pinctrl/*/pinmux-pins; judge signal connectivity only
#      through /proc/interrupts.
#
# Register table source: kernel/arch/arm64/boot/dts/hobot/pinmux-gpio.dtsi and
# pinmux-func.dtsi. LSIO iomuxc base is 0x34180000, LSIO_PINMUX_3 = +0x84, UART
# function = MUX_ALT0(0), GPIO = MUX_ALT2(2).
# The mux backend is copied from scripts/gpio/header-gpio-test.sh. That path
# passed 17/17 board tests and handles vendor pinmux-functions two-line format
# parsing plus fallback ordering for "<group> <function>". It is copied instead
# of sourced so this file can be scp'ed standalone.

set -u
LOGTAG="[x5-pps]"
STATE=/var/run/x5-pps.state
[ -d /var/run ] && [ -w /var/run ] 2>/dev/null || STATE=/tmp/x5-pps.state
MODNAME=x5pps
UART_DRV=/sys/bus/platform/drivers/dw-apb-uart

# ---------- Persistence: one switch persists across reboot ----------
# Persist to /userdata, not /etc. This tree sets HR_SYSTEM_VERIFY="dm-verity",
# so rootfs is protected by verity and writing it risks bricking. /userdata is an
# independent writable ext4 partition and is not changed by reflashing firmware.
# The boot hook uses the official /userdata/startup.sh extension point. Both
# buildroot and jammy rootfs variants have byte-identical
# /etc/init.d/S99auto_startup files that execute it at the end of boot when it is
# executable, so no init-system branching or rootfs changes are needed.
# It runs after S65mountall (/userdata mounted) and S70loadko (drivers loaded),
# giving the required ordering naturally.
SELFDIR=$(cd "$(dirname "$0")" 2>/dev/null && pwd) || SELFDIR=$(pwd)
PERSIST="$SELFDIR/mode"           # Persisted mode file (MODE=pps|uart)
DISABLE="$SELFDIR/DISABLE"        # Emergency switch: boot hook does nothing when this file exists
BOOTLOG="$SELFDIR/boot.log"       # Boot-hook log
STARTUP=${X5PPS_STARTUP:-/userdata/startup.sh}   # Override for offline self-tests
HOOK_BEGIN="# >>> x5-pps boot hook >>>"
HOOK_END="# <<< x5-pps boot hook <<<"

# global id|header pin|signal|gpio group|mux register|bit|GPIO mode value|owner uart|uart group|platform device|note
PINS=(
 "379|pin10|LSIO_UART7_RX|lsio_gpio0_0|0x34180084|4|2|uart7|uart7grp|34060000.serial|default PPS input pin; UART7 RX path only"
)
DEFAULT_PIN=379
# In PPS mode, UART7 TX is released as an internal GPIO/IO companion line. It is
# not exposed as a PPS selection parameter.
IO_GPIO=380
IO_HDR=pin8
IO_SIG=LSIO_UART7_TX
IO_GRP=lsio_gpio0_1
IO_ADDR=0x34180084
IO_BIT=6
IO_ALT=2

CMD="" PIN="" EDGE=rising SECS=10 FORCE=0 KEEP_UART=0 ONCE=0
CONSOLE_TTY=""   # Filled by console_is_on for diagnostics when it matches
while [ $# -gt 0 ]; do case "$1" in
  pps|uart|status|test|boot|install|uninstall) CMD=$1;;
  --check)      CMD=check;;
  --pin)        PIN=$2; shift;;
  --edge)       EDGE=$2; shift;;
  --secs)       SECS=$2; shift;;
  --keep-uart)  KEEP_UART=1;;
  --once)       ONCE=1;;
  --force)      FORCE=1;;
  -h|--help)    awk 'NR>1{ if(/^#/) print; else exit }' "$0"; exit 0;;
  *) echo "unknown arg: $1" >&2; exit 1;;
esac; shift; done
[ -n "$CMD" ] || { awk 'NR>1{ if(/^#/) print; else exit }' "$0"; exit 1; }

log(){ echo "$LOGTAG $*"; }
die(){ echo "$LOGTAG FATAL: $*" >&2; exit 1; }

# ---------- Pin-table lookup ----------
ROW=""
pin_lookup(){ # global id -> fill ROW / P_* variables; return 1 without stale state when not found
  local want=$1 r
  ROW=""                      # Must clear; misses must not reuse a previous row and restore the wrong register
  for r in "${PINS[@]}"; do
    if [ "${r%%|*}" = "$want" ]; then ROW=$r; break; fi
  done
  [ -n "$ROW" ] || return 1
  IFS='|' read -r P_GPIO P_HDR P_SIG P_GRP P_ADDR P_BIT P_ALT P_UART P_UGRP P_DEV P_NOTE <<<"$ROW"
  return 0
}
pin_list(){ local r; for r in "${PINS[@]}"; do echo "${r%%|*}"; done; }

# ---------- mux backend, copied from header-gpio-test.sh and validated 17/17 PASS on board ----------
MUX_BACKEND="" REG_TOOL="" LSIO_DBG="" LSIO_FUNC=""

func_name(){ # debugfs directory -> first function name
  # Support two pinmux-functions formats:
  #   mainline:     "function 0: lsio_iomuxc, groups = [ ... ]"
  #   HOBOT vendor: "function 0: lsio_iomuxc" with group list on the next line
  sed -n 's/^function[ 0-9]*: *\([^,]*\).*/\1/p' "$1/pinmux-functions" 2>/dev/null | head -1 | tr -d ' \t\r'
}

detect_mux_backend(){
  mountpoint -q /sys/kernel/debug 2>/dev/null || mount -t debugfs none /sys/kernel/debug 2>/dev/null
  LSIO_DBG=$(ls -d /sys/kernel/debug/pinctrl/*lsio_iomuxc* 2>/dev/null | head -1)
  if [ -n "$LSIO_DBG" ] && [ -w "$LSIO_DBG/pinmux-select" ]; then
    LSIO_FUNC=$(func_name "$LSIO_DBG")
    [ -n "$LSIO_FUNC" ] && MUX_BACKEND=pinmux-select
  fi
  if [ -e /dev/mem ]; then
    if   command -v devmem  >/dev/null 2>&1; then REG_TOOL=devmem
    elif command -v hexdump >/dev/null 2>&1; then REG_TOOL=hexdump-dd
    elif command -v python3 >/dev/null 2>&1; then REG_TOOL=python3
    fi
  fi
  [ -z "$MUX_BACKEND" ] && [ -n "$REG_TOOL" ] && MUX_BACKEND=register
  [ -z "$MUX_BACKEND" ] && die "No available mux backend: pinmux-select unavailable (missing debugfs or function-name parse failure), and no direct register tool (devmem/hexdump/python3 + /dev/mem)"
  log "mux backend: $MUX_BACKEND(register tool: ${REG_TOOL:-none})${LSIO_FUNC:+ lsio function=$LSIO_FUNC}"
  # State save and readback verification depend on register reads; without them
  # the script can only switch blindly.
  [ -z "$REG_TOOL" ] && log "Warning: no register read/write tool, cannot save original value or verify readback; blind switch only (install devmem if possible)"
}

reg_read(){ # addr -> decimal value to stdout
  case "$REG_TOOL" in
    devmem)     local h; h=$(devmem "$1" 32) || return 1; echo $((h));;
    hexdump-dd) local h; h=$(hexdump -n4 -s $(($1)) -e '1/4 "%08x"' /dev/mem 2>/dev/null)
                [ -n "$h" ] || return 1; echo $((0x$h));;
    python3)    python3 -c "
import mmap,os
a=$(($1)); pg=a&~0xfff
f=os.open('/dev/mem',os.O_RDONLY|os.O_SYNC)
m=mmap.mmap(f,0x1000,mmap.MAP_SHARED,mmap.PROT_READ,offset=pg)
print(int.from_bytes(m[a-pg:a-pg+4],'little'))";;
    *) return 1;;
  esac
}

reg_write(){ # addr val(decimal)
  case "$REG_TOOL" in
    devmem)     devmem "$1" 32 "$2";;
    hexdump-dd) # Single 4-byte little-endian write; bs=4 guarantees 32-bit access and register addresses are 4-aligned
                local v=$2 fmt
                fmt=$(printf '\\%03o\\%03o\\%03o\\%03o' $((v&255)) $(((v>>8)&255)) $(((v>>16)&255)) $(((v>>24)&255)))
                printf "$fmt" | dd of=/dev/mem bs=4 seek=$(($1/4)) count=1 conv=notrunc 2>/dev/null \
                  || printf "$fmt" | dd of=/dev/mem bs=4 seek=$(($1/4)) count=1 2>/dev/null;;
    python3)    python3 - "$(($1))" "$2" <<'PYEOF'
import mmap,os,sys
a=int(sys.argv[1]); v=int(sys.argv[2]); pg=a&~0xfff
f=os.open("/dev/mem",os.O_RDWR|os.O_SYNC)
m=mmap.mmap(f,0x1000,offset=pg)
m[a-pg:a-pg+4]=v.to_bytes(4,'little'); m.close(); os.close(f)
PYEOF
;;
    *) return 1;;
  esac
}

reg_rmw(){ # addr bit val  (2-bit read-modify-write plus readback verification)
  local addr=$1 bit=$2 val=$3 cur new
  [ -n "$REG_TOOL" ] || return 1
  cur=$(reg_read "$addr") || return 1
  new=$(( (cur & ~(3<<bit)) | (val<<bit) ))
  reg_write "$addr" "$new" || return 1
  cur=$(reg_read "$addr") || return 0
  [ $(( (cur>>bit) & 3 )) -eq "$val" ]
}

mux_field(){ # addr bit -> current 2-bit value, or "?" when unreadable
  local cur
  cur=$(reg_read "$1" 2>/dev/null) || { echo "?"; return 1; }
  echo $(( (cur>>$2) & 3 ))
}

mux_select(){ # group addr bit val -> 0 on success. Try pinmux-select first, then direct register write
  local grp=$1 addr=$2 bit=$3 val=$4
  if [ "$MUX_BACKEND" = pinmux-select ] && [ -n "$LSIO_FUNC" ]; then
    # Kernel pinmux.c parses "<group> <function>"; retry the reverse order as a fallback.
    if echo "$grp $LSIO_FUNC" > "$LSIO_DBG/pinmux-select" 2>/dev/null \
    || echo "$LSIO_FUNC $grp" > "$LSIO_DBG/pinmux-select" 2>/dev/null; then
      # Successful pinmux-select write does not prove mux changed; verify by
      # register readback when a register tool is available.
      if [ -n "$REG_TOOL" ]; then
        [ "$(mux_field "$addr" "$bit")" = "$val" ] && return 0
        log "pinmux-select($grp) write succeeded but readback mismatched; falling back to direct register write"
      else
        return 0
      fi
    else
      log "pinmux-select failed($grp), falling back to direct register write (tool: ${REG_TOOL:-none})"
    fi
  fi
  reg_rmw "$addr" "$bit" "$val"
}

# ---------- Peripheral / module / pps helpers ----------
console_is_on(){ # $1 = platform device name -> 0 when console is on it. Predicate only; never exits.
  local n devpath
  n=$(sed -n 's/.*console=ttyS\([0-9]\).*/\1/p' /proc/cmdline | head -1)
  [ -z "$n" ] && return 1
  devpath=$(readlink -f "/sys/class/tty/ttyS$n/device" 2>/dev/null) || return 1
  CONSOLE_TTY="ttyS$n"
  case "$devpath" in *"$1"*) return 0;; esac
  return 1
}

console_guard(){ # $1 = platform device name; block when console is on it unless --force is used
  console_is_on "$1" || return 0
  [ "$FORCE" = 1 ] && { log "Warning: console is on $1; --force continues anyway (keep SSH ready)"; return 0; }
  die "console=$CONSOLE_TTY is on $1; unbinding will lose the console. Use SSH and add --force"
}

tty_of(){ ls "/sys/bus/platform/devices/$1/tty" 2>/dev/null | head -1; }

mod_loaded(){ [ -d "/sys/module/$MODNAME" ]; }
mod_param(){ cat "/sys/module/$MODNAME/parameters/$1" 2>/dev/null; }

find_ko(){
  local c
  for c in "${X5PPS_KO:-}" "$(dirname "$(readlink -f "$0")")/x5pps.ko" \
           /userdata/x5-pps/x5pps.ko "/lib/modules/$(uname -r)/extra/x5pps.ko"; do
    [ -n "$c" ] && [ -f "$c" ] && { echo "$c"; return 0; }
  done
  return 1
}

ko_vermagic(){
  local v=""
  command -v modinfo >/dev/null 2>&1 && v=$(modinfo -F vermagic "$1" 2>/dev/null | head -1)
  [ -z "$v" ] && v=$(tr -c '[:print:]\n' '\n' < "$1" | sed -n 's/^vermagic=//p' | head -1)
  echo "$v"
}

pps_dir(){ # PPS device created by this module; accept name==x5pps, never hobot-pps.-1
  local id d
  id=$(mod_param pps_id)
  if [ -n "$id" ] && [ "$id" != "-1" ] && [ -d "/sys/class/pps/pps$id" ]; then
    echo "/sys/class/pps/pps$id"; return 0
  fi
  for d in /sys/class/pps/pps*; do
    [ -e "$d/name" ] || continue
    [ "$(cat "$d/name" 2>/dev/null)" = "$MODNAME" ] && { echo "$d"; return 0; }
  done
  return 1
}

irq_count(){ # Sum this module's IRQ counts in /proc/interrupts across CPUs
  # Add only CPU count columns: ncpu is determined from the header NF. $1 is
  # "N:", followed by ncpu count columns. Never add later chip/hwirq/Level/name
  # fields because hwirq can be numeric and would corrupt the sum.
  # Prefer matching by IRQ number; fall back to the name in the final column.
  local irqn=${1:-}
  awk -v n="$MODNAME" -v want="$irqn" '
    NR==1 { ncpu=NF; next }
    {
      hit = (want != "" && $1 == want":") || (want == "" && $NF == n)
      if (!hit) next
      s=0; for (i=2; i<=ncpu+1 && i<=NF; i++) if ($i ~ /^[0-9]+$/) s+=$i
      print s; exit
    }' /proc/interrupts 2>/dev/null
}

assert_seq(){ # -> "sec nsec seq"; empty when unreadable
  local d v
  d=$(pps_dir) || return 1
  v=$(cat "$d/assert" 2>/dev/null) || return 1
  [ -n "$v" ] || return 1
  echo "$v" | sed 's/[.#]/ /g'
}

# ---------- State file ----------
state_save(){
  cat > "$STATE" <<EOF
PIN=$P_GPIO
SIG=$P_SIG
GRP=$P_GRP
UGRP=$P_UGRP
ADDR=$P_ADDR
BIT=$P_BIT
UART=$P_UART
DEV=$P_DEV
DRV=$UART_DRV
EDGE=$EDGE
ORIG_REG=${ORIG_REG:-}
UNBOUND=${UNBOUND:-0}
EOF
}
state_load(){ [ -f "$STATE" ] || return 1; . "$STATE"; return 0; }

unbind_uart(){ # $1=platform device -> 0 unbound, 1 already unbound/missing, 2 unbind failed
  [ -e "/sys/bus/platform/devices/$1" ] || { log "$1 device does not exist; skipping unbind"; return 1; }
  [ -e "/sys/bus/platform/devices/$1/driver" ] || { log "$1 is already not bound to a driver; skipping unbind"; return 1; }
  echo "$1" > "$UART_DRV/unbind" 2>/dev/null || { log "Failed to unbind $1"; return 2; }
  log "Unbound $1"
  return 0
}

rebind_uart(){ # $1=platform device
  [ -e "/sys/bus/platform/devices/$1/driver" ] && { log "$1 already bound; no rebind needed"; return 0; }
  if echo "$1" > "$UART_DRV/bind" 2>/dev/null; then log "Rebound $1"; return 0; fi
  sleep 1
  echo "$1" > "$UART_DRV/bind" 2>/dev/null && { log "Rebound $1"; return 0; }
  log "!! Failed to rebind $1; run manually: echo $1 > $UART_DRV/bind (or reboot)"
  return 1
}

# ---------- Persistence ----------
persist_mode(){ # -> pps|uart; no config means uart, the factory behavior
  # Must source in a subshell. $PERSIST contains PIN=/EDGE=; sourcing directly
  # would overwrite the caller's PIN, which previously broke do_uart.
  ( MODE=uart; [ -f "$PERSIST" ] && . "$PERSIST" 2>/dev/null; echo "${MODE:-uart}" )
}

persist_save(){ # $1 = pps|uart
  if [ "$ONCE" = 1 ]; then
    log "--once: changed only this run; persisted config unchanged (reboot still uses $(persist_mode) mode)"
    return 0
  fi
  mkdir -p "$SELFDIR" 2>/dev/null
  # Every value below must be quoted. STAMP may look like
  # "Wed Sep  9 11:55:36 CST 2026"; without quotes, source interprets it as
  # "STAMP=Wed" plus a Sep command, making "." return non-zero and do_boot
  # misclassify the config as unreadable. Offline self-tests caught this.
  if ! cat > "$PERSIST" <<EOF
# x5-pps persisted mode, written by x5-pps-pin.sh. Do not edit manually.
# At boot: /etc/init.d/S99auto_startup -> $STARTUP -> this script boot.
# Restore normal UART: $SELFDIR/${0##*/} uart     Remove completely: $SELFDIR/${0##*/} uninstall
MODE="$1"
PIN="${P_GPIO:-$DEFAULT_PIN}"
EDGE="$EDGE"
KEEP_UART="$KEEP_UART"
FORCE="$FORCE"
STAMP="$(date 2>/dev/null || echo unknown)"
EOF
  then
    log "!! Cannot write $PERSIST (read-only directory?); this switch will be lost after reboot"
    return 1
  fi
  hook_install || return 1
  if [ "$1" = pps ]; then
    log "Persisted config written: every reboot will enter PPS input (GPIO${P_GPIO:-?} edge=$EDGE) until you run '${0##*/} uart'"
  else
    log "Persisted config written: every reboot will keep normal UART until you run '${0##*/} pps'"
  fi
}

hook_install(){ # Idempotently append the hook line to /userdata/startup.sh without overwriting existing content
  local self="$SELFDIR/${0##*/}"
  local sdir; sdir=$(dirname "$STARTUP")
  [ -d "$sdir" ] || { log "!! Missing $sdir; boot hook cannot be installed (persisted config written but reboot will not auto-apply)"; return 1; }
  if [ -f "$STARTUP" ] && grep -qF "$HOOK_BEGIN" "$STARTUP" 2>/dev/null; then
    chmod +x "$STARTUP" 2>/dev/null
    log "Boot hook already exists in $STARTUP; no duplicate install needed"
    return 0
  fi
  if [ ! -f "$STARTUP" ]; then
    printf '#!/bin/sh\n# Created by x5-pps-pin.sh. Executed by /etc/init.d/S99auto_startup at the end of boot.\n\n' \
      > "$STARTUP" || { log "!! Failed to create $STARTUP"; return 1; }
  fi
  # Append only, without touching existing lines; field systems may already use
  # this file for other components.
  {
    echo "$HOOK_BEGIN"
    echo "[ -x $self ] && $self boot >> $BOOTLOG 2>&1 &"
    echo "$HOOK_END"
  } >> "$STARTUP" || { log "!! Failed to append to $STARTUP"; return 1; }
  chmod +x "$STARTUP" 2>/dev/null
  log "Boot hook installed into $STARTUP (existing content preserved)"
}

hook_uninstall(){
  [ -f "$STARTUP" ] || { log "$STARTUP does not exist; no hook to remove"; return 0; }
  grep -qF "$HOOK_BEGIN" "$STARTUP" 2>/dev/null || { log "$STARTUP has no hook from this script"; return 0; }
  local tmp="$STARTUP.x5pps.tmp"
  awk -v b="$HOOK_BEGIN" -v e="$HOOK_END" '
    index($0,b){skip=1; next}
    index($0,e){skip=0; next}
    !skip' "$STARTUP" > "$tmp" 2>/dev/null || { log "!! Failed to rewrite $STARTUP"; rm -f "$tmp"; return 1; }
  cat "$tmp" > "$STARTUP" || { log "!! Failed to write back $STARTUP"; rm -f "$tmp"; return 1; }
  rm -f "$tmp"
  log "Removed boot hook from $STARTUP (other content preserved)"
}

hook_status(){
  log "--- Persistence (post-reboot behavior) ---"
  [ -e "$DISABLE" ] && echo "  Emergency switch $DISABLE exists => boot hook always does nothing"
  if [ "$(persist_mode)" = pps ]; then
    echo "  Persisted mode: PPS input   $(grep -hE '^(PIN|EDGE|FORCE|STAMP)=' "$PERSIST" 2>/dev/null | tr '\n' ' ')"
  elif [ -f "$PERSIST" ]; then
    echo "  Persisted mode: normal UART (explicitly written)"
  else
    echo "  Persisted mode: normal UART (no $PERSIST, factory behavior)"
  fi
  if [ -f "$STARTUP" ] && grep -qF "$HOOK_BEGIN" "$STARTUP" 2>/dev/null; then
    if [ -x "$STARTUP" ]; then echo "  Boot hook: installed ($STARTUP)"
    else echo "  Boot hook: installed but $STARTUP is not executable => S99auto_startup will not run it!  chmod +x $STARTUP"; fi
  else
    echo "  Boot hook: not installed => reboot will not auto-switch"
  fi
  [ -s "$BOOTLOG" ] && echo "  Last boot log: $BOOTLOG  (tail -20 $BOOTLOG)"
  return 0
}

# ================= Subcommands =================

do_check(){
  detect_mux_backend
  echo
  log "Available pins (MUX_ALT0=UART function, MUX_ALT2=GPIO/PPS):"
  printf '  %-5s %-7s %-15s %-14s %-11s %-4s %-6s %-16s %s\n' \
    global header signal gpio_group mux_reg bit owner platform_device current_mux
  local r cur mark
  for r in "${PINS[@]}"; do
    IFS='|' read -r g hdr sig grp addr bit alt ua ugrp dev note <<<"$r"
    cur=$(mux_field "$addr" "$bit")
    case "$cur" in
      0) mark="ALT0=UART";;
      2) mark="ALT2=GPIO";;
      '?') mark="unreadable";;
      *) mark="ALT$cur=?";;
    esac
    printf '  %-5s %-7s %-15s %-14s %-11s %-4s %-6s %-16s %s\n' \
      "$g" "$hdr" "$sig" "$grp" "$addr" "$bit" "$ua" "$dev" "$mark"
    [ -n "$note" ] && printf '        %s\n' "$note"
  done
  echo
  log "Default pin: $DEFAULT_PIN"
  local ko
  if ko=$(find_ko); then
    log "Found module: $ko  vermagic=[$(ko_vermagic "$ko")]"
    log "Board kernel: [$(uname -r)]"
    [ "$(ko_vermagic "$ko" | awk '{print $1}')" = "$(uname -r)" ] \
      && log "vermagic matches OK" || log "!! vermagic mismatch; insmod will be rejected"
  else
    log "!! x5pps.ko not found (checked \$X5PPS_KO / script directory / /userdata/x5-pps/ / /lib/modules/\$(uname -r)/extra/)"
  fi
  echo
  log "Current /sys/class/pps/*:"
  local d nm
  for d in /sys/class/pps/pps*; do
    [ -e "$d/name" ] || continue
    nm=$(cat "$d/name" 2>/dev/null)
    if [ "$nm" = "$MODNAME" ]; then
      printf '  %s  name=%-14s path=%-10s  <- real source created by this script\n' "${d##*/}" "$nm" "$(cat "$d/path" 2>/dev/null)"
    else
      printf '  %s  name=%-14s path=%-10s  <- fake source (hobot-pps is a 1-second mod_timer fake, unrelated to pins)\n' \
        "${d##*/}" "$nm" "$(cat "$d/path" 2>/dev/null)"
    fi
  done
  [ -e /sys/class/pps/pps0 ] || log "  (none)"
  echo
  log "--check finished without modifications"
}

do_pps(){
  local ko vm
  ko=$(find_ko) || die "x5pps.ko not found; put it beside this script or export X5PPS_KO=/path/x5pps.ko"
  vm=$(ko_vermagic "$ko" | awk '{print $1}')
  if [ "$vm" != "$(uname -r)" ]; then
    [ "$FORCE" = 1 ] || die "vermagic mismatch: ko=[$vm] board=[$(uname -r)]; rebuild for this kernel (make KDIR=...) or try --force"
    log "Warning: vermagic mismatch($vm vs $(uname -r)); trying anyway because --force is set"
  fi

  if mod_loaded; then
    local cur_gpio; cur_gpio=$(mod_param gpio)
    if [ "$cur_gpio" = "$P_GPIO" ]; then
      log "Already in PPS mode(gpio=$cur_gpio); no need to switch again"; do_status; return 0
    fi
    die "Module is loaded on gpio=$cur_gpio. Run '$0 uart' first, then switch to $P_GPIO"
  fi

  if [ "$KEEP_UART" = 1 ]; then
    # --keep-uart does not unbind, so the console's device binding is kept and
    # is not hard-blocked. The pad is still borrowed, specifically UART7 RX pin
    # 379, so console input may be affected without extra errors.
    console_is_on "$P_DEV" && \
      log "Warning: console($CONSOLE_TTY) is on $P_UART. --keep-uart does not unbind, but UART7 RX and TX IO mux both change; console may lose input/output without extra errors"
  else
    console_guard "$P_DEV"
  fi

  ORIG_REG=""
  if [ -n "$REG_TOOL" ]; then
    local v; v=$(reg_read "$P_ADDR") && ORIG_REG=$(printf '0x%08x' "$v")
    log "Saved original $P_ADDR value = ${ORIG_REG:-read failed}"
  fi

  UNBOUND=0
  # Errors after this point must roll back.
  rollback(){
    log "Error occurred, rolling back..."
    rmmod "$MODNAME" 2>/dev/null
    if [ -n "$ORIG_REG" ] && [ -n "$REG_TOOL" ]; then
      reg_write "$P_ADDR" "$((ORIG_REG))" 2>/dev/null
    else
      mux_select "$P_UGRP" "$P_ADDR" "$P_BIT" 0 2>/dev/null || true
      mux_select "$IO_GRP" "$IO_ADDR" "$IO_BIT" 0 2>/dev/null || true
    fi
    [ "$UNBOUND" = 1 ] && rebind_uart "$P_DEV"
    rm -f "$STATE"
  }
  trap 'rollback' EXIT INT TERM

  if [ "$KEEP_UART" = 1 ]; then
    log "--keep-uart: not unbinding $P_UART, only changing mux (UART PM suspend/resume may silently reclaim the pad)"
  else
    unbind_uart "$P_DEV"; case $? in
      0) UNBOUND=1;;
      2) die "Failed to unbind $P_DEV; not changing mux (pad remains UART, field state unchanged)";;
      *) :;;   # Already unbound; continue
    esac
  fi
  state_save

  mux_select "$P_GRP" "$P_ADDR" "$P_BIT" "$P_ALT" \
    || die "Failed to switch mux to ALT$P_ALT; pad remains in UART function"
  if [ -n "$REG_TOOL" ]; then
    local now; now=$(mux_field "$P_ADDR" "$P_BIT")
    [ "$now" = "$P_ALT" ] || die "mux readback = ALT$now, not ALT$P_ALT; switch did not take effect"
    log "mux switched: $P_ADDR bit$P_BIT = ALT$P_ALT (GPIO), readback verified"
  else
    log "mux wrote ALT$P_ALT (no register tool, readback not verified)"
  fi

  mux_select "$IO_GRP" "$IO_ADDR" "$IO_BIT" "$IO_ALT" \
    || die "Failed to switch UART7 TX mux to GPIO/IO"
  if [ -n "$REG_TOOL" ]; then
    local io_now; io_now=$(mux_field "$IO_ADDR" "$IO_BIT")
    [ "$io_now" = "$IO_ALT" ] || die "UART7 TX mux readback = ALT$io_now, not ALT$IO_ALT"
    log "UART7 TX released as GPIO/IO: GPIO$IO_GPIO ($IO_HDR $IO_SIG), readback verified"
  else
    log "Attempted to switch UART7 TX to GPIO/IO (no register tool, readback not verified)"
  fi

  insmod "$ko" gpio="$P_GPIO" pps_id=2 edge="$EDGE" \
    || die "insmod failed; check the end of dmesg (common cause: gpio occupied by another driver -> cat /sys/kernel/debug/gpio)"

  trap - EXIT INT TERM
  state_save
  log "Switched to PPS input: GPIO$P_GPIO ($P_HDR $P_SIG) edge=$EDGE"
  persist_save pps
  do_status
  echo
  log "Required verification: $0 test --secs 10   -- no interrupt-count increase means the signal is not connected"
  log "Restore:               $0 uart             (also persists UART mode so reboot will not enter PPS)"
}

do_uart(){
  # Pin identification priority: 1) gpio stored by the module, the most
  # reliable source; 2) state file; 3) command-line --pin.
  # state_load is ". $STATE" and may overwrite command-line $PIN, so save the
  # command-line value first and use the state file only if the first two levels
  # cannot identify the pin.
  local cli_pin="${PIN:-}" g=""

  if mod_loaded; then
    g=$(mod_param gpio)
    log "Unloading $MODNAME (gpio=${g:-?}, assert=$(mod_param assert_count) clear=$(mod_param clear_count))"
    rmmod "$MODNAME" || die "rmmod failed (is a process holding /dev/ppsN? fuser -v /dev/pps*)"
    [ -n "$g" ] && pin_lookup "$g" || log "Module gpio parameter is abnormal(${g:-empty}); falling back to state file/--pin"
  fi
  if [ -z "$ROW" ] && state_load; then
    # Also load fallback details such as ORIG_REG/UNBOUND from the state file.
    [ -n "${PIN:-}" ] && pin_lookup "$PIN" || true
  fi
  [ -z "$ROW" ] && [ -n "$cli_pin" ] && { pin_lookup "$cli_pin" || die "--pin $cli_pin is not in the table"; }
  [ -n "$ROW" ] || die "Cannot identify which pin to restore: module not loaded and no state file($STATE). Specify --pin 379"

  # 2) Rebind uart. During probe, pinctrl writes back the "default" state and
  # mux automatically returns to ALT0.
  if [ -e "/sys/bus/platform/devices/$P_DEV" ]; then rebind_uart "$P_DEV"; fi

  # 3) Verify readback; write ALT0 directly if it did not return.
  if [ -n "$REG_TOOL" ]; then
    local now; now=$(mux_field "$P_ADDR" "$P_BIT")
    if [ "$now" != "0" ]; then
      log "After rebind, mux is still ALT$now; writing ALT0 directly..."
      if [ -n "${ORIG_REG:-}" ]; then
        reg_write "$P_ADDR" "$((ORIG_REG))" && log "Restored full register from state-file original value $ORIG_REG"
      fi
      now=$(mux_field "$P_ADDR" "$P_BIT")
      [ "$now" = "0" ] || mux_select "$P_UGRP" "$P_ADDR" "$P_BIT" 0 || true
      now=$(mux_field "$P_ADDR" "$P_BIT")
    fi
    [ "$now" = "0" ] && log "mux readback = ALT0(UART function), verified" \
                     || log "!! mux readback = ALT$now, not restored; reboot will fully restore it"
  else
    mux_select "$P_UGRP" "$P_ADDR" "$P_BIT" 0 || true
    mux_select "$IO_GRP" "$IO_ADDR" "$IO_BIT" 0 || true
    log "Attempted to write UART7 RX/TX back to ALT0 (no register tool, readback not verified)"
  fi

  if [ -n "$REG_TOOL" ]; then
    local io_now; io_now=$(mux_field "$IO_ADDR" "$IO_BIT")
    [ "$io_now" = "0" ] && log "UART7 TX mux readback = ALT0(UART function), verified" \
                        || log "!! UART7 TX mux readback = ALT$io_now, not restored; reboot will fully restore it"
  fi

  local t; t=$(tty_of "$P_DEV")
  [ -n "$t" ] && log "UART is back: /dev/$t ($P_UART $P_DEV)" \
              || log "!! No tty under $P_DEV; UART did not come up. Reboot will fully restore it"
  rm -f "$STATE"
  log "Restored normal UART function: GPIO$P_GPIO ($P_HDR $P_SIG)"
  persist_save uart
}

do_status(){
  local d nm id
  echo
  log "--- Module ---"
  if mod_loaded; then
    printf '  %s loaded: gpio=%s edge=%s irq=%s pps_id=%s assert=%s clear=%s\n' \
      "$MODNAME" "$(mod_param gpio)" "$(mod_param edge)" "$(mod_param irq)" \
      "$(mod_param pps_id)" "$(mod_param assert_count)" "$(mod_param clear_count)"
  else
    echo "  $MODNAME not loaded (currently not PPS mode)"
  fi

  log "--- Pin mux ---"
  local r cur mark
  for r in "${PINS[@]}"; do
    IFS='|' read -r g hdr sig grp addr bit alt ua ugrp dev note <<<"$r"
    cur=$(mux_field "$addr" "$bit")
    case "$cur" in 0) mark="ALT0 UART function";; 2) mark="ALT2 GPIO/PPS";; \?) mark="unreadable";; *) mark="ALT$cur ?";; esac
    printf '  GPIO%-4s %-7s %-15s %-16s %s%s\n' "$g" "$hdr" "$sig" "$mark" \
      "$(tty_of "$dev" | sed 's|^|tty=/dev/|')" \
      "$([ -e "/sys/bus/platform/devices/$dev/driver" ] && echo "" || echo "  [unbound]")"
  done

  if [ -n "$REG_TOOL" ]; then
    printf "  UART7 TX IO mux: GPIO%s %s = ALT%s (%s)\n" "$IO_GPIO" "$IO_SIG" "$(mux_field "$IO_ADDR" "$IO_BIT")" "$IO_HDR"
  fi

  log "--- /sys/class/pps ---"
  for d in /sys/class/pps/pps*; do
    [ -e "$d/name" ] || continue
    nm=$(cat "$d/name" 2>/dev/null)
    if [ "$nm" = "$MODNAME" ]; then
      printf '  %-6s name=%-14s path=%-10s mode=%s  <- real source (this script)\n' \
        "${d##*/}" "$nm" "$(cat "$d/path" 2>/dev/null)" "$(cat "$d/mode" 2>/dev/null)"
      printf '         assert=%s\n' "$(cat "$d/assert" 2>/dev/null)"
    else
      printf '  %-6s name=%-14s <- fake source: hobot-pps is a 1-second mod_timer fake unrelated to pins; do not use it for acceptance\n' \
        "${d##*/}" "$nm"
    fi
  done

  log "--- /proc/interrupts ---"
  grep -w "$MODNAME" /proc/interrupts 2>/dev/null || echo "  (no interrupt line for $MODNAME)"

  hook_status
}

do_test(){
  mod_loaded || die "Module not loaded; run '$0 pps' first"
  local d; d=$(pps_dir) || die "Cannot find pps device with name==$MODNAME (do not use the fake hobot-pps.-1)"
  local irqn; irqn=$(mod_param irq)
  # Fill P_* from the gpio recorded by the module; otherwise the FAIL branch
  # would reference $P_HDR under set -u and hit unbound variable immediately.
  [ -z "$ROW" ] && pin_lookup "$(mod_param gpio)" 2>/dev/null || true

  local i1 a1 s1 n1 q1 i2 a2 s2 n2 q2
  i1=$(irq_count "$irqn"); a1=$(mod_param assert_count); a1=${a1:-0}
  read -r s1 n1 q1 <<<"$(assert_seq)"
  log "Sampling ${SECS}s ... (gpio=$(mod_param gpio) irq=$irqn edge=$(mod_param edge))"
  log "  start: irq_count=${i1:-?} assert_count=$a1 seq=${q1:-?}"
  sleep "$SECS"
  i2=$(irq_count "$irqn"); a2=$(mod_param assert_count); a2=${a2:-0}
  read -r s2 n2 q2 <<<"$(assert_seq)"
  log "  end: irq_count=${i2:-?} assert_count=$a2 seq=${q2:-?}"

  local d_irq="?" d_a d_q="?"
  [ -n "${i1:-}" ] && [ -n "${i2:-}" ] && d_irq=$((i2-i1))
  d_a=$((a2-a1))
  [ -n "${q1:-}" ] && [ -n "${q2:-}" ] && d_q=$((q2-q1))
  echo
  log "Within ${SECS}s: interrupts +$d_irq, assert events +$d_a, PPS seq +$d_q"

  # Average period: use integer nanoseconds because awk double precision cannot
  # safely hold epoch+ns. 10# forces decimal for nsec with leading zeros.
  if [ -n "${s1:-}" ] && [ -n "${s2:-}" ] && [ "${d_q:-0}" != "?" ] && [ "${d_q:-0}" -gt 0 ]; then
    local dns per
    dns=$(( (10#$s2 - 10#$s1) * 1000000000 + (10#$n2 - 10#$n1) ))
    per=$(( dns / d_q ))
    printf '%s average period = %d.%06d s (expected 1.000000)\n' "$LOGTAG" $((per/1000000000)) $(( (per%1000000000)/1000 ))
  fi

  echo
  if [ "$d_a" -eq 0 ]; then
    log "Result: FAIL -- no events. Check: 1) is the signal wire actually connected to ${P_HDR:-this pin}? 2) is ground shared?"
    log "        3) is mux ALT2($0 status)? 4) is edge direction correct (try --edge falling)?"
    log "        5) is the PPS source 3.3V? The pad is 3.3V; 1.8V output needs level shifting."
    return 1
  fi
  local lo=$((SECS-2)) hi=$((SECS+2))
  [ "$lo" -lt 1 ] && lo=1
  if [ "$d_a" -ge "$lo" ] && [ "$d_a" -le "$hi" ]; then
    log "Result: PASS -- received $d_a events in ${SECS}s, about 1 Hz; real PPS input is connected"
    log "Countercheck: unplug the signal and run again; it must become 0 to rule out the fake hobot-pps source"
    return 0
  fi
  log "Result: signal exists but rate is not 1 Hz (received $d_a events in ${SECS}s). Jitter, both-edge counting, or wrong source frequency?"
  log "        edge=both doubles the count; confirm the signal source is really 1PPS"
  return 1
}

do_boot(){ # Boot-hook entry. Always return 0 so boot is never blocked.
  echo "===== $(date 2>/dev/null || echo '(no RTC)') x5-pps boot hook ====="
  [ -e "$DISABLE" ] && { log "Emergency switch $DISABLE exists; boot makes no changes"; return 0; }
  [ -f "$PERSIST" ] || { log "No persisted config; keeping factory normal UART"; return 0; }
  # Intentionally source into globals here: PIN/EDGE/KEEP_UART/FORCE are the
  # parameters needed for this run.
  . "$PERSIST" 2>/dev/null || { log "Failed to read persisted config; keeping factory normal UART"; return 0; }
  [ "${MODE:-uart}" = pps ] || { log "Persisted mode=${MODE:-uart}; keeping normal UART and doing nothing"; return 0; }

  EDGE=${EDGE:-rising}; KEEP_UART=${KEEP_UART:-0}; FORCE=${FORCE:-0}
  ONCE=1   # Boot path must not rewrite persisted config or alter user settings.
  pin_lookup "${PIN:-$DEFAULT_PIN}" || { log "!! PIN=${PIN:-empty} in persisted config is not in the table; skipping switch"; return 0; }

  # During boot there may be no operator nearby. If console is lost, recovery may
  # require physical serial access, so only proceed when --force was persisted.
  if console_is_on "$P_DEV" && [ "$FORCE" != 1 ]; then
    log "!! console($CONSOLE_TTY) is on $P_UART; boot switching may lose console, skipped unless persisted config used --force"
    return 0
  fi
  log "Switching PPS by persisted config: GPIO$P_GPIO ($P_HDR $P_SIG) edge=$EDGE"
  ( do_pps ) || log "!! Boot switch failed (see above); normal UART kept and boot flow unaffected"
  return 0
}

do_install(){
  [ "$(persist_mode)" = pps ] \
    || log "Note: current persisted mode is uart (boot will not switch). To auto-enter PPS after reboot, run '${0##*/} pps'"
  hook_install
  hook_status
}

do_uninstall(){
  hook_uninstall
  [ -f "$PERSIST" ] && { rm -f "$PERSIST" && log "Deleted persisted config $PERSIST"; }
  log "Boot will no longer switch anything. Current runtime state is unchanged; run '${0##*/} uart --once' to restore normal UART immediately"
}

# ================= Main flow =================
[ "$(id -u)" = 0 ] || die "root is required"

# check/install/uninstall are probe/recovery paths and must work even when the
# environment is poor, such as missing debugfs or devmem. Keep them before
# detect_mux_backend, which dies if no backend is found.
case "$CMD" in
  check)     do_check;      exit 0;;
  install)   do_install;    exit 0;;
  uninstall) do_uninstall;  exit 0;;
esac

detect_mux_backend

case "$CMD" in
  pps)
    pin_lookup "${PIN:-$DEFAULT_PIN}" || die "Unknown pin '${PIN:-$DEFAULT_PIN}'; available: $(pin_list | tr '\n' ' ')"
    case "$EDGE" in rising|falling|both) ;; *) die "--edge must be rising, falling, or both";; esac
    log "Target: GPIO$P_GPIO ($P_HDR $P_SIG) owner=$P_UART($P_DEV) edge=$EDGE"
    [ -n "$P_NOTE" ] && log "Note: $P_NOTE"
    do_pps;;
  uart)
    [ -n "$PIN" ] && { pin_lookup "$PIN" || die "Unknown pin '$PIN'"; }
    do_uart;;
  status)
    [ -n "$PIN" ] && pin_lookup "$PIN"
    do_status;;
  test)
    if mod_loaded; then pin_lookup "$(mod_param gpio)" 2>/dev/null || true; fi
    do_test;;
  boot)      do_boot;;
  *) die "Unknown command $CMD";;
esac
