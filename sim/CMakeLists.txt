set(CTEST_PARALLEL_LEVEL 4)

# RTL compiled into every testbench. Keep this the full synthesizable set so a
# testbench can instantiate anything (including acm9767_top) without per-test
# source lists; the tests are small enough that compile time is not an issue.
set(RTL_SOURCES
    "${SRC_FOLDER}/timescale.v"
    "${SRC_FOLDER}/dac_pll.v"
    "${SRC_FOLDER}/reset_sync.sv"
    "${SRC_FOLDER}/cdc_commit_sync.sv"
    "${SRC_FOLDER}/uart.sv"
    "${SRC_FOLDER}/modbus_rtu_slave.sv"
    "${SRC_FOLDER}/dac_regs.sv"
    "${SRC_FOLDER}/async_fifo.sv"
    "${SRC_FOLDER}/hilbert_fir.sv"
    "${SRC_FOLDER}/cic_interp.sv"
    "${SRC_FOLDER}/envelope_limiter.sv"
    "${SRC_FOLDER}/ssb_baseband.sv"
    "${SRC_FOLDER}/dds_channel.sv"
    "${SRC_FOLDER}/dac_output_stage.sv"
    "${SRC_FOLDER}/acm9767_top.v"
)

# Tests are grouped by scope: `unit/<dut>/<scenario>` for per-module tests and
# `integration/<topic>/<scenario>` for cross-module tests. The path under sim/
# is the source file (plus .sv); the first path element becomes a ctest label,
# so `ctest -L unit` and `ctest -L dds_channel` both work.
set(UNIT_TESTS
    uart/frame
    cdc_commit_sync/handshake
    dac_regs/regmap
    dds_channel/waveforms
    dds_channel/sine_table
    dac_output_stage/clock_forward
    async_fifo/basic
    hilbert_fir/response
    cic_interp/tone
    ssb_baseband/sideband
    ssb_baseband/limiter
    ssb_baseband/dcblock
)
set(INTEGRATION_TESTS
    modbus/rtu_slave
    top/modbus_to_dac
    top/ssb_stream
)

set(SIM_COMMON_FOLDER "${SIM_SRC_FOLDER}/common")
set(BASE_TEST_DIR "${CMAKE_CURRENT_BINARY_DIR}/tests")
set(CMAKE_CTEST_ARGUMENTS "--extra-verbose")

if (WIN32)
    set(RUN_SIM_SCRIPT_NAME "run_simulator.bat")
else()
    set(RUN_SIM_SCRIPT_NAME "run_simulator.sh")
endif()

set(TEST_SCRIPT_TEMPLATE "${SIM_COMMON_FOLDER}/${RUN_SIM_SCRIPT_NAME}.in")
configure_file("${SIM_COMMON_FOLDER}/test_config.sv.in" "${CMAKE_CURRENT_BINARY_DIR}/test_config.sv.tmp")
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_config.sv"
     INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_config.sv.tmp")

# Build one ctest case per (scope, scenario) entry. Each test gets:
#   - a build target    <scope>_<flat_name>_BUILD   (compile only)
#   - a simulate target <scope>_<flat_name>_SIM     (compile + run)
#   - a ctest case      <scope>_<flat_name>_run     (compile + run + check exit)
# `flat_name` is the slash-separated path with `/` replaced by `_`.
function(register_test SCOPE TEST_PATH)
    string(REPLACE "/" "_" FLAT_NAME "${TEST_PATH}")
    set(TEST_NAME "${SCOPE}_${FLAT_NAME}")
    set(TEST_SOURCE "${SIM_SRC_FOLDER}/${SCOPE}/${TEST_PATH}.sv")
    set(WORKING_TEST_DIR "${BASE_TEST_DIR}/${SCOPE}/${TEST_PATH}")
    set(TEST_OUTPUT_FILE "${WORKING_TEST_DIR}/${TEST_NAME}.bin")

    string(REGEX REPLACE "/.*$" "" DUT_LABEL "${TEST_PATH}")
    set(TEST_LABELS "${SCOPE};${DUT_LABEL}")

    set(TEST_SOURCES
        "${TEST_SOURCE}"
        "${SIM_COMMON_FOLDER}/svlogger.sv"
        "${CMAKE_CURRENT_BINARY_DIR}/test_config.sv"
        ${ARGN}
    )

    add_custom_command(OUTPUT ${TEST_OUTPUT_FILE}
        COMMAND "${VERILOG_BUILDER}"
                -g2012
                -I "${SRC_FOLDER}"
                -I "${SIM_SRC_FOLDER}"
                -I "${SIM_COMMON_FOLDER}"
                -s main
                -s GSR
                -o ${TEST_OUTPUT_FILE}
                ${RTL_SOURCES}
                "${SIMLIB_SRC}"
                ${TEST_SOURCES}
        DEPENDS ${RTL_SOURCES} ${TEST_SOURCES}
                "${SRC_FOLDER}/acm9767_defs.vh"
                "${SRC_FOLDER}/platform_config.vh"
                "${SRC_FOLDER}/sine_lut_init.vh"
                "${SRC_FOLDER}/hilbert_coefs.vh"
    )

    configure_file(${TEST_SCRIPT_TEMPLATE} "${WORKING_TEST_DIR}/${RUN_SIM_SCRIPT_NAME}" @ONLY)

    add_custom_target("${TEST_NAME}_BUILD" ALL DEPENDS ${TEST_OUTPUT_FILE}
        SOURCES ${RTL_SOURCES} ${TEST_SOURCES} "${SIMLIB_SRC}"
    )

    add_custom_target("${TEST_NAME}_SIM"
        COMMAND "${VERILOG_SIM}" -n "${TEST_OUTPUT_FILE}"
        COMMENT "Run simulation for test \"${TEST_NAME}\"..."
        DEPENDS "${TEST_OUTPUT_FILE}"
        WORKING_DIRECTORY "${WORKING_TEST_DIR}"
    )

    if (WIN32)
        add_test(NAME "${TEST_NAME}_run" COMMAND "${WORKING_TEST_DIR}/${RUN_SIM_SCRIPT_NAME}")
    else()
        add_test(NAME "${TEST_NAME}_run" COMMAND sh "${WORKING_TEST_DIR}/${RUN_SIM_SCRIPT_NAME}")
    endif()

    set_tests_properties("${TEST_NAME}_run" PROPERTIES
        DEPENDS ${TEST_OUTPUT_FILE}
        WORKING_DIRECTORY "${WORKING_TEST_DIR}"
        LABELS "${TEST_LABELS}"
    )
endfunction()

# Compile-only variant for file-driven simulations that a host script runs with
# plusargs (no ctest case of its own): <scope>_<flat_name>_BUILD and the binary
# path in <SCOPE>_<FLAT_NAME>_BIN (parent scope).
function(register_sim_tool SCOPE TEST_PATH)
    string(REPLACE "/" "_" FLAT_NAME "${TEST_PATH}")
    set(TEST_NAME "${SCOPE}_${FLAT_NAME}")
    set(TEST_SOURCE "${SIM_SRC_FOLDER}/${SCOPE}/${TEST_PATH}.sv")
    set(WORKING_TEST_DIR "${BASE_TEST_DIR}/${SCOPE}/${TEST_PATH}")
    set(TEST_OUTPUT_FILE "${WORKING_TEST_DIR}/${TEST_NAME}.bin")
    file(MAKE_DIRECTORY "${WORKING_TEST_DIR}")
    add_custom_command(OUTPUT ${TEST_OUTPUT_FILE}
        COMMAND "${VERILOG_BUILDER}" -g2012
                -I "${SRC_FOLDER}" -I "${SIM_SRC_FOLDER}" -I "${SIM_COMMON_FOLDER}"
                -s main -s GSR -o ${TEST_OUTPUT_FILE}
                ${RTL_SOURCES} "${SIMLIB_SRC}"
                "${TEST_SOURCE}" "${SIM_COMMON_FOLDER}/svlogger.sv" "${CMAKE_CURRENT_BINARY_DIR}/test_config.sv"
        DEPENDS ${RTL_SOURCES} "${TEST_SOURCE}" "${SIM_COMMON_FOLDER}/svlogger.sv"
                "${CMAKE_CURRENT_BINARY_DIR}/test_config.sv"
                "${SRC_FOLDER}/acm9767_defs.vh" "${SRC_FOLDER}/platform_config.vh"
                "${SRC_FOLDER}/sine_lut_init.vh" "${SRC_FOLDER}/hilbert_coefs.vh"
    )
    add_custom_target("${TEST_NAME}_BUILD" ALL DEPENDS ${TEST_OUTPUT_FILE})
    string(TOUPPER "${TEST_NAME}_BIN" _var)
    set(${_var} "${TEST_OUTPUT_FILE}" PARENT_SCOPE)
    set(${_var}_DIR "${WORKING_TEST_DIR}" PARENT_SCOPE)
endfunction()

foreach (TEST_PATH IN LISTS UNIT_TESTS)
    register_test(unit "${TEST_PATH}")
endforeach()

foreach (TEST_PATH IN LISTS INTEGRATION_TESTS)
    register_test(integration "${TEST_PATH}")
endforeach()

# ---------------------------------------------------------------------------
# Modulator -> host analytic demodulator loopback (scripts/ssb_loopback.py):
# the file-driven simulation of the real ssb_baseband + dds_channel chain is
# scored in Python (numpy). `ctest -L loopback`. Skipped when python3/numpy
# are missing.
# ---------------------------------------------------------------------------
register_sim_tool(unit "ssb_baseband/loopback")
find_program(PYTHON3_SIM python3)
if (PYTHON3_SIM)
    execute_process(COMMAND ${PYTHON3_SIM} -c "import numpy" RESULT_VARIABLE _no_numpy OUTPUT_QUIET ERROR_QUIET)
    if (_no_numpy EQUAL 0)
        foreach (_mode usb lsb am fm)
            add_test(NAME loopback_ssb_${_mode}
                COMMAND ${PYTHON3_SIM} "${CMAKE_SOURCE_DIR}/scripts/ssb_loopback.py"
                        --vvp "${VERILOG_SIM}" --sim "${UNIT_SSB_BASEBAND_LOOPBACK_BIN}"
                        --mode ${_mode} --workdir "${UNIT_SSB_BASEBAND_LOOPBACK_BIN_DIR}/${_mode}"
            )
            set_tests_properties(loopback_ssb_${_mode} PROPERTIES LABELS "loopback;ssb_baseband")
        endforeach()
    else()
        message(STATUS "numpy not found - loopback tests disabled")
    endif()
endif()
