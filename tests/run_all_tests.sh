#!/usr/bin/env bash
# run_all_tests.sh — Build and run all test harnesses, report results

set -e
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$REPO_ROOT/build"
TESTS_DIR="$REPO_ROOT/tests"
INCLUDES="-I$REPO_ROOT -I$REPO_ROOT/sdk -I$REPO_ROOT/dsp -I$REPO_ROOT/wdf"
CXX_FLAGS="-std=c++20 -O2 -Wall -Wdouble-promotion -Werror"

mkdir -p "$BUILD_DIR"

PASS=0
FAIL=0
SKIP=0
ERRORS=()

run_test() {
    local name="$1"
    local src="$TESTS_DIR/${name}_test.cpp"
    local bin="$BUILD_DIR/test_${name}"

    if [ ! -f "$src" ]; then
        echo "SKIP: $name (no test file found)"
        SKIP=$((SKIP + 1))
        return
    fi

    printf "%-30s" "$name..."

    if g++ $CXX_FLAGS $INCLUDES "$src" -o "$bin" -lm 2>/tmp/test_build_err; then
        if "$bin" > /tmp/test_run_out 2>&1; then
            echo "PASS"
            PASS=$((PASS + 1))
        else
            echo "FAIL (runtime)"
            cat /tmp/test_run_out
            FAIL=$((FAIL + 1))
            ERRORS+=("$name: runtime failure")
        fi
    else
        echo "FAIL (build)"
        cat /tmp/test_build_err
        FAIL=$((FAIL + 1))
        ERRORS+=("$name: build failure")
    fi
}

echo "======================================"
echo " Polyend Endless — Test Suite"
echo "======================================"

run_test "dsp"
run_test "wdf"
run_test "pnp_bjt"
run_test "jfet"
run_test "wah"
run_test "jrc4558"
run_test "ota"
run_test "photoresistor"
run_test "spring_reverb"
run_test "lm308_rat"
run_test "ReverbPrimitives"
run_test "big_muff"
run_test "soft_focus"
run_test "multitap"
run_test "pitch"

echo ""
echo "======================================"
echo " Results: $PASS passed, $FAIL failed, $SKIP skipped"
echo "======================================"

if [ ${#ERRORS[@]} -gt 0 ]; then
    echo ""
    echo "Failures:"
    for e in "${ERRORS[@]}"; do echo "  - $e"; done
    exit 1
fi

exit 0
