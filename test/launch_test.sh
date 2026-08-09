#!/bin/bash

# Run a test binary, optionally under gdb/valgrind, and dump all-thread stacks on hang.
#
# TOOL=none|direct (default)  - run the binary directly (sanitizer-friendly)
# TOOL=gdb                    - run under gdb (noisy; use for local debugging)
# TOOL=valgrind               - run under valgrind
#
# Hang / timeout stacks:
#   HANG_TIMEOUT=<seconds>  - if set, dump stacks and kill when exceeded (set this
#                             slightly below ctest --timeout so we dump before
#                             ctest SIGKILLs the process group).
#   On SIGTERM/SIGINT (ctest soft kill), dump stacks of the test and its children.
#
# Master diagnostics:
#   Prefer miniros::test::requireMasterOrExit() after miniros::init
#   (see test/require_master.h). That exits with code 90 when the master is down.
#   This script also fail-fasts when $bindir/miniroscore.pid names a dead process
#   (before the test, and periodically while waiting), so a dead shared master
#   does not burn HANG_TIMEOUT on every subsequent binary.
#   On exit 90 / hang / signal, dumps miniroscore stacks and copies rosout logs
#   from MINIROS_MASTER_LOG_DIR / ROS_LOG_DIR / $bindir/master-logs.
#
# Exit codes:
#   90  - MASTER_UNAVAILABLE (from test helper, precheck, or manage-master start)
#   124 - hang / timeout / signalled watchdog kill
#
# Examples:
#   ./launch_test.sh ./some_test
#   HANG_TIMEOUT=90 ./launch_test.sh ./some_test
#   TOOL=gdb ./launch_test.sh ./some_test

set -u

TOOL="${TOOL:-none}"
EXIT_MASTER_UNAVAILABLE=90
EXIT_HANG=124

test_bin_dir() {
  local bin=$1
  local dir
  dir=$(dirname -- "$bin")
  (cd "$dir" && pwd)
}

resolve_log_dir() {
  local bindir=$1
  if [[ -n "${MINIROS_MASTER_LOG_DIR:-}" ]]; then
    echo "$MINIROS_MASTER_LOG_DIR"
  elif [[ -n "${ROS_LOG_DIR:-}" ]]; then
    echo "$ROS_LOG_DIR"
  else
    echo "$bindir/master-logs"
  fi
}

# Echo master pid from $bindir/miniroscore.pid, or empty if missing/blank.
read_master_pid() {
  local bindir=$1
  local pidfile="$bindir/miniroscore.pid"
  if [[ ! -f "$pidfile" ]]; then
    return 0
  fi
  tr -d '[:space:]' <"$pidfile" || true
}

# 0 = running, 1 = no pid expected, 2 = pidfile present but process dead.
master_status() {
  local bindir=$1
  local master_pid
  master_pid=$(read_master_pid "$bindir")
  if [[ -z "${master_pid:-}" ]]; then
    return 1
  fi
  if kill -0 "$master_pid" 2>/dev/null; then
    return 0
  fi
  return 2
}

fail_master_unavailable() {
  local bindir=$1
  local reason=$2
  local master_pid
  master_pid=$(read_master_pid "$bindir")
  echo "launch_test.sh: MASTER_UNAVAILABLE ($reason): miniroscore pid='${master_pid:-}' not running" >&2
  dump_master_diagnostics "$bindir" "$reason" >/dev/null || true
  exit "$EXIT_MASTER_UNAVAILABLE"
}

# If a shared master was started (non-empty pidfile) but is dead, abort with 90.
require_shared_master_alive() {
  local bindir=$1
  local reason=$2
  master_status "$bindir"
  local st=$?
  if (( st == 2 )); then
    fail_master_unavailable "$bindir" "$reason"
  fi
}

dump_stacks() {
  local pid=$1
  if ! kill -0 "$pid" 2>/dev/null; then
    return
  fi

  local args
  args=$(ps -o args= -p "$pid" 2>/dev/null || true)
  echo "=== hang/timeout dump: pid=$pid args=$args ===" >&2

  if command -v gdb >/dev/null 2>&1; then
    gdb -batch -p "$pid" \
      -ex "set pagination off" \
      -ex "set debuginfod enabled off" \
      -ex "thread apply all bt" \
      -ex detach 2>&1 || true
  elif command -v eu-stack >/dev/null 2>&1; then
    eu-stack -p "$pid" 2>&1 || true
  else
    echo "launch_test.sh: neither gdb nor eu-stack available for stack dump" >&2
  fi
}

dump_process_tree() {
  local root=$1
  local p
  dump_stacks "$root"
  for p in $(pgrep -P "$root" 2>/dev/null || true); do
    dump_stacks "$p"
  done
}

dump_master_diagnostics() {
  local bindir=$1
  local reason=$2
  local logdir
  logdir=$(resolve_log_dir "$bindir")
  mkdir -p "$logdir"

  local stamp
  stamp=$(date +%Y%m%d-%H%M%S)
  local outdir="$logdir/master-fail-$stamp"
  mkdir -p "$outdir"

  echo "=== MASTER DIAGNOSTICS ($reason) -> $outdir ===" >&2

  local pidfile="$bindir/miniroscore.pid"
  local master_pid=""
  if [[ -f "$pidfile" ]]; then
    master_pid=$(tr -d '[:space:]' <"$pidfile" || true)
    echo "pidfile=$pidfile pid=$master_pid" | tee "$outdir/pid.txt" >&2
  else
    echo "pidfile missing: $pidfile" | tee "$outdir/pid.txt" >&2
  fi

  if [[ -n "$master_pid" ]] && kill -0 "$master_pid" 2>/dev/null; then
    dump_stacks "$master_pid" | tee "$outdir/master-stacks.txt" >&2
    local p
    for p in $(pgrep -P "$master_pid" 2>/dev/null || true); do
      dump_stacks "$p" | tee -a "$outdir/master-stacks.txt" >&2
    done
  else
    echo "master process not running (pid='$master_pid')" | tee -a "$outdir/master-stacks.txt" >&2
  fi

  for f in "$logdir/rosout.log" "$bindir/rosout.log" "$logdir/miniroscore.console.log"; do
    if [[ -f "$f" ]]; then
      cp -f "$f" "$outdir/" 2>/dev/null || true
      echo "copied $f" >&2
    fi
  done

  local b
  for b in "$logdir"/rosout.log.[0-9]*; do
    [[ -f "$b" ]] || continue
    cp -f "$b" "$outdir/" 2>/dev/null || true
  done

  ls -la "$outdir" >&2 || true
  echo "$outdir"
}

maybe_dump_on_master_unavailable() {
  local bindir=$1
  local st=$2
  if (( st == EXIT_MASTER_UNAVAILABLE )); then
    echo "launch_test.sh: test exited $EXIT_MASTER_UNAVAILABLE (MASTER_UNAVAILABLE)" >&2
    dump_master_diagnostics "$bindir" "test-exit-90" >/dev/null || true
  fi
}

run_with_hang_watchdog() {
  local bindir=$1
  shift

  # Shared-master suite: do not start another 280s hang if miniroscore is already gone.
  require_shared_master_alive "$bindir" "precheck-dead"

  "$@" &
  local child=$!
  local timed_out=0
  local master_died=0

  on_signal() {
    echo "launch_test.sh: caught signal, dumping stacks before exit" >&2
    dump_process_tree "$child"
    dump_master_diagnostics "$bindir" "signal" >/dev/null || true
    kill -TERM "$child" 2>/dev/null || true
    sleep 1
    kill -KILL "$child" 2>/dev/null || true
    wait "$child" 2>/dev/null || true
    exit "$EXIT_HANG"
  }
  trap on_signal TERM INT

  if [[ -n "${HANG_TIMEOUT:-}" ]]; then
    local elapsed=0
    while kill -0 "$child" 2>/dev/null; do
      if (( elapsed >= HANG_TIMEOUT )); then
        timed_out=1
        break
      fi
      # Every 5s: if the shared master died mid-test, fail fast (exit 90).
      if (( elapsed > 0 && elapsed % 5 == 0 )); then
        master_status "$bindir"
        if (( $? == 2 )); then
          master_died=1
          break
        fi
      fi
      sleep 1
      elapsed=$((elapsed + 1))
    done

    if (( master_died )); then
      echo "launch_test.sh: shared master died while test pid=$child was running" >&2
      dump_process_tree "$child"
      kill -TERM "$child" 2>/dev/null || true
      sleep 2
      kill -KILL "$child" 2>/dev/null || true
      wait "$child" 2>/dev/null || true
      trap - TERM INT
      fail_master_unavailable "$bindir" "master-died-mid-test"
    fi

    if (( timed_out )); then
      echo "launch_test.sh: HANG_TIMEOUT=${HANG_TIMEOUT}s exceeded for pid $child" >&2
      dump_process_tree "$child"
      dump_master_diagnostics "$bindir" "hang-timeout" >/dev/null || true
      kill -TERM "$child" 2>/dev/null || true
      sleep 2
      kill -KILL "$child" 2>/dev/null || true
      wait "$child" 2>/dev/null || true
      trap - TERM INT
      exit "$EXIT_HANG"
    fi
  fi

  wait "$child"
  local st=$?
  trap - TERM INT
  maybe_dump_on_master_unavailable "$bindir" "$st"
  return "$st"
}

if [[ $# -lt 1 ]]; then
  echo "launch_test.sh: missing test binary" >&2
  exit 1
fi

BINDIR=$(test_bin_dir "$1")

case "$TOOL" in
  gdb)
    gdb -return-child-result -ex "set debuginfod enabled off" -ex run -ex "thread apply all bt" -ex "quit" --args "$@"
    st=$?
    maybe_dump_on_master_unavailable "$BINDIR" "$st"
    exit "$st"
    ;;
  none|direct)
    run_with_hang_watchdog "$BINDIR" "$@"
    exit $?
    ;;
  valgrind)
    run_with_hang_watchdog "$BINDIR" valgrind --track-origins=yes "$@"
    exit $?
    ;;
  *)
    echo "launch_test.sh: unknown TOOL='$TOOL' (use gdb, none, or valgrind)" >&2
    exit 1
    ;;
esac
