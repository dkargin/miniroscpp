#!/bin/bash

# Run a test binary under a debugger or memory checker.
# Set TOOL=gdb (default) or TOOL=valgrind, e.g.:
#   ./launch_test.sh ./some_test
#   TOOL=valgrind ./launch_test.sh ./some_test

TOOL="${TOOL:-gdb}"

case "$TOOL" in
  gdb)
    # Disables debuginfod and prints backtrace if an unexpected error happens.
    gdb -return-child-result -ex "set debuginfod enabled off" -ex run -ex "thread apply all bt" -ex "quit" --args "$@"
    ;;
  valgrind)
    valgrind --track-origins=yes "$@"
    ;;
  *)
    echo "launch_test.sh: unknown TOOL='$TOOL' (use gdb or valgrind)" >&2
    exit 1
    ;;
esac
