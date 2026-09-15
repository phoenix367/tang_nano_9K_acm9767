cmake_minimum_required(VERSION 3.16)

project(acm9767_demo LANGUAGES NONE)

option(IVerilog_PATH "Path to Icarus Verilog bin folder" NONE)
option(Gowin_PATH "Path to Gowin IDE folder (the one containing bin/ and simlib/)" NONE)
option(DUMP_SIM_VARIABLES "Enable dump variables during tests simulation" OFF)

set(SimLogLevel "Info" CACHE STRING "Logging level during tests simulation")
set(SimLogLevelValues "Fatal;Error;Info;Debug;None" CACHE STRING
    "List of possible values for the SimLogLevel cache variable")
set_property(CACHE SimLogLevel PROPERTY STRINGS ${SimLogLevelValues})

if (${SimLogLevel} STREQUAL "Fatal")
    set(TESTS_LOG_LEVEL "SVL_VERBOSE_FATAL")
elseif (${SimLogLevel} STREQUAL "Error")
    set(TESTS_LOG_LEVEL "SVL_VERBOSE_ERROR")
elseif (${SimLogLevel} STREQUAL "Info")
    set(TESTS_LOG_LEVEL "SVL_VERBOSE_INFO")
elseif (${SimLogLevel} STREQUAL "Debug")
    set(TESTS_LOG_LEVEL "SVL_VERBOSE_DEBUG")
elseif (${SimLogLevel} STREQUAL "None")
    set(TESTS_LOG_LEVEL "SVL_VERBOSE_NONE")
else()
    message(FATAL_ERROR "Invalid log level: ${SimLogLevel}")
endif()
message(STATUS "Simulation logging level: ${SimLogLevel}")

# ---------------------------------------------------------------------------
# Toolchain discovery
# ---------------------------------------------------------------------------
find_program(VERILOG_BUILDER iverilog HINTS ${IVerilog_PATH})
if (NOT VERILOG_BUILDER)
    message(FATAL_ERROR "Icarus verilog builder not found")
endif()
message(STATUS "Found Icarus verilog at ${VERILOG_BUILDER}")

find_program(VERILOG_SIM vvp HINTS ${IVerilog_PATH})
if (NOT VERILOG_SIM)
    message(FATAL_ERROR "Icarus verilog simulator not found")
endif()

# The Gowin simlib supplies the rPLL / ODDR / GSR behavioural models the tests
# elaborate, so a Gowin install is required even for simulation-only work.
find_file(SIMLIB_SRC NAMES "prim_tsim.v" HINTS "${Gowin_PATH}/simlib/gw1n")
if (NOT SIMLIB_SRC)
    message(FATAL_ERROR "Gowin simlib (simlib/gw1n/prim_tsim.v) not found under Gowin_PATH=${Gowin_PATH}")
endif()
message(STATUS "Found Gowin simlib at ${SIMLIB_SRC}")

find_program(GOWIN_SHELL gw_sh HINTS "${Gowin_PATH}/bin")
if (NOT GOWIN_SHELL)
    message(FATAL_ERROR "Gowin gw_sh not found under ${Gowin_PATH}/bin")
endif()

# programmer_cli ships alongside the IDE under <root>/Programmer/bin.
get_filename_component(GOWIN_ROOT "${Gowin_PATH}" DIRECTORY)
find_program(GOWIN_PROGRAMMER programmer_cli HINTS "${GOWIN_ROOT}/Programmer/bin")
if (GOWIN_PROGRAMMER)
    message(STATUS "Found Gowin programmer at ${GOWIN_PROGRAMMER}")
else()
    message(STATUS "Gowin programmer_cli not found - hw_program targets disabled")
endif()

set(SRC_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(SIM_SRC_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}/sim")

# ---------------------------------------------------------------------------
# Platform configuration: platform.json is the single source of truth for the
# clocks, PLL ratio, DAC geometry, Modbus identity and UART framing. CMake turns
# it into src/platform_config.vh (`define macros) and src/acm9767.sdc. The JSON
# is parsed with a regex that matches each key at any nesting depth, so the
# section names are cosmetic to CMake and keys must stay globally unique.
# Editing platform.json re-runs cmake.
# ---------------------------------------------------------------------------
set(PLATFORM_JSON "${CMAKE_CURRENT_SOURCE_DIR}/platform.json")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${PLATFORM_JSON}")
file(READ "${PLATFORM_JSON}" _PLATFORM_JSON_CONTENT)

macro(platform_json_int key outvar)
    if (_PLATFORM_JSON_CONTENT MATCHES "\"${key}\"[ \t\r\n]*:[ \t\r\n]*([0-9]+)")
        set(${outvar} "${CMAKE_MATCH_1}")
    else()
        message(FATAL_ERROR "platform.json: key '${key}' is missing or not an integer")
    endif()
endmacro()

macro(platform_json_hex key outvar)
    if (_PLATFORM_JSON_CONTENT MATCHES "\"${key}\"[ \t\r\n]*:[ \t\r\n]*\"(0[xX][0-9A-Fa-f]+)\"")
        math(EXPR ${outvar} "${CMAKE_MATCH_1}")
    else()
        message(FATAL_ERROR "platform.json: key '${key}' is missing or not a hex string (\"0x...\")")
    endif()
endmacro()

macro(platform_json_enum key outvar)
    if (_PLATFORM_JSON_CONTENT MATCHES "\"${key}\"[ \t\r\n]*:[ \t\r\n]*\"([A-Za-z]+)\"")
        set(_enum_name "${CMAKE_MATCH_1}")
        set(${outvar} "")
        foreach(_pair ${ARGN})
            string(REPLACE "=" ";" _kv "${_pair}")
            list(GET _kv 0 _n)
            list(GET _kv 1 _v)
            if (_enum_name STREQUAL _n)
                set(${outvar} "${_v}")
            endif()
        endforeach()
        if (${outvar} STREQUAL "")
            message(FATAL_ERROR "platform.json: key '${key}' has unsupported value '${_enum_name}'")
        endif()
    else()
        message(FATAL_ERROR "platform.json: key '${key}' is missing or not a string")
    endif()
endmacro()

# clock / pll
platform_json_int("sys_clk_hz"      PLATFORM_SYS_CLK_HZ)
platform_json_int("idiv"            PLATFORM_PLL_IDIV)
platform_json_int("fbdiv"           PLATFORM_PLL_FBDIV)
platform_json_int("odiv"            PLATFORM_PLL_ODIV)
# dac
platform_json_int("bits"            PLATFORM_DAC_BITS)
platform_json_int("channels"        PLATFORM_DAC_CHANNELS)
platform_json_int("sine_lut_depth"  PLATFORM_SINE_LUT_DEPTH)
# modbus
platform_json_int("device_id"       PLATFORM_MODBUS_DEVICE_ID)
platform_json_hex("addr_limit"      PLATFORM_MODBUS_ADDR_LIMIT)
platform_json_int("max_read_qty"    PLATFORM_MODBUS_MAX_READ_QTY)
platform_json_int("max_frame"       PLATFORM_MODBUS_MAX_FRAME)
# ssb
platform_json_int("audio_rate_hz"   PLATFORM_SSB_AUDIO_RATE_HZ)
platform_json_int("hilbert_taps"    PLATFORM_SSB_HILBERT_TAPS)
platform_json_int("fifo_depth"      PLATFORM_SSB_FIFO_DEPTH)
platform_json_int("dc_block_shift"    PLATFORM_SSB_DC_SHIFT)
platform_json_int("limiter_lookahead" PLATFORM_SSB_LIM_LOOKAHEAD)
platform_json_int("limiter_threshold" PLATFORM_SSB_LIM_THRESHOLD)
platform_json_int("limiter_release"   PLATFORM_SSB_LIM_RELEASE)
# uart
platform_json_int("baud"            PLATFORM_UART_BAUD)
platform_json_int("data_bits"       PLATFORM_UART_DATA_BITS)
platform_json_int("stop_bits"       PLATFORM_UART_STOP_BITS)
platform_json_enum("parity"         PLATFORM_UART_PARITY  none=0 odd=1 even=2)

# Derived PLL values. Gowin rPLL: CLKOUT = CLKIN * FBDIV / IDIV, VCO = CLKOUT * ODIV.
# The primitive takes IDIV_SEL = idiv-1 and FBDIV_SEL = fbdiv-1 (0..63), ODIV_SEL
# is the divider itself and must be one of 2/4/8/16/32/48/64/80/96/112/128.
if (PLATFORM_PLL_IDIV LESS 1 OR PLATFORM_PLL_IDIV GREATER 64 OR
    PLATFORM_PLL_FBDIV LESS 1 OR PLATFORM_PLL_FBDIV GREATER 64)
    message(FATAL_ERROR "platform.json: pll.idiv / pll.fbdiv must be 1..64")
endif()
set(_ODIV_OK 2 4 8 16 32 48 64 80 96 112 128)
if (NOT PLATFORM_PLL_ODIV IN_LIST _ODIV_OK)
    message(FATAL_ERROR "platform.json: pll.odiv must be one of ${_ODIV_OK}")
endif()
math(EXPR PLATFORM_PLL_IDIV_SEL  "${PLATFORM_PLL_IDIV} - 1")
math(EXPR PLATFORM_PLL_FBDIV_SEL "${PLATFORM_PLL_FBDIV} - 1")
math(EXPR PLATFORM_DAC_CLK_HZ    "${PLATFORM_SYS_CLK_HZ} * ${PLATFORM_PLL_FBDIV} / ${PLATFORM_PLL_IDIV}")
math(EXPR _PLL_VCO_HZ            "${PLATFORM_DAC_CLK_HZ} * ${PLATFORM_PLL_ODIV}")
math(EXPR _PLL_PFD_HZ            "${PLATFORM_SYS_CLK_HZ} / ${PLATFORM_PLL_IDIV}")
# GW1NR-9C rPLL operating window (DS117): VCO 400..1200 MHz, PFD 3..400 MHz.
if (_PLL_VCO_HZ LESS 400000000 OR _PLL_VCO_HZ GREATER 1200000000)
    message(WARNING "platform.json: PLL VCO = ${_PLL_VCO_HZ} Hz is outside the 400..1200 MHz window; adjust pll.odiv")
endif()
if (_PLL_PFD_HZ LESS 3000000 OR _PLL_PFD_HZ GREATER 400000000)
    message(WARNING "platform.json: PLL PFD input = ${_PLL_PFD_HZ} Hz is outside 3..400 MHz; adjust pll.idiv")
endif()
if (PLATFORM_DAC_CLK_HZ GREATER 125000000)
    message(WARNING "platform.json: dac_clk = ${PLATFORM_DAC_CLK_HZ} Hz exceeds the AD9767's 125 MSPS maximum")
endif()
# Channel count: 1 wires only DA1 (clk/wrt/data), 2 adds the DA2 ports + DDS.
if (PLATFORM_DAC_CHANNELS EQUAL 1)
    set(PLATFORM_DAC_CH2_DEFINE "`undef  PLATFORM_DAC_CH2")
elseif (PLATFORM_DAC_CHANNELS EQUAL 2)
    set(PLATFORM_DAC_CH2_DEFINE "`define PLATFORM_DAC_CH2")
else()
    message(FATAL_ERROR "platform.json: dac.channels must be 1 or 2")
endif()
# ---- SSB baseband chain constants ----
# Interpolation ratio baseband -> dac_clk must be an integer (the tick counter
# and the CIC comb delay are exact); the CIC is fixed at 3 stages, DC gain R^2.
math(EXPR PLATFORM_SSB_INTERP "${PLATFORM_DAC_CLK_HZ} / ${PLATFORM_SSB_AUDIO_RATE_HZ}")
math(EXPR _ssb_check "${PLATFORM_SSB_INTERP} * ${PLATFORM_SSB_AUDIO_RATE_HZ}")
if (NOT _ssb_check EQUAL PLATFORM_DAC_CLK_HZ)
    message(FATAL_ERROR "platform.json: dac_clk (${PLATFORM_DAC_CLK_HZ}) is not an integer multiple of ssb.audio_rate_hz (${PLATFORM_SSB_AUDIO_RATE_HZ})")
endif()
math(EXPR _cic_gain "${PLATFORM_SSB_INTERP} * ${PLATFORM_SSB_INTERP}")
# S = floor(log2(R^2)); the CIC output is taken at bits [S+15:S] and the
# residual gain 2^S / R^2 (0.5..1] is compensated by K (Q15) in the FIR block.
set(PLATFORM_SSB_CIC_SHIFT 0)
set(_g "${_cic_gain}")
while (_g GREATER 1)
    math(EXPR _g "${_g} / 2")
    math(EXPR PLATFORM_SSB_CIC_SHIFT "${PLATFORM_SSB_CIC_SHIFT} + 1")
endwhile()
math(EXPR PLATFORM_SSB_GAIN_K "((1 << ${PLATFORM_SSB_CIC_SHIFT}) * 32768 + ${_cic_gain} / 2) / ${_cic_gain}")
if (PLATFORM_SSB_GAIN_K GREATER 32767)
    set(PLATFORM_SSB_GAIN_K 32767)
endif()
# FIFO depth must be a power of two (gray-code pointers).
set(PLATFORM_SSB_FIFO_LOG2 0)
set(_d "${PLATFORM_SSB_FIFO_DEPTH}")
while (_d GREATER 1)
    math(EXPR _d "${_d} / 2")
    math(EXPR PLATFORM_SSB_FIFO_LOG2 "${PLATFORM_SSB_FIFO_LOG2} + 1")
endwhile()
math(EXPR _d "1 << ${PLATFORM_SSB_FIFO_LOG2}")
if (NOT _d EQUAL PLATFORM_SSB_FIFO_DEPTH)
    message(FATAL_ERROR "platform.json: ssb.fifo_depth must be a power of two")
endif()
math(EXPR _odd "${PLATFORM_SSB_HILBERT_TAPS} % 2")
if (NOT _odd EQUAL 1 OR PLATFORM_SSB_HILBERT_TAPS GREATER 255)
    message(FATAL_ERROR "platform.json: ssb.hilbert_taps must be odd and <= 255")
endif()
message(STATUS "SSB: ${PLATFORM_SSB_AUDIO_RATE_HZ} S/s audio, interp x${PLATFORM_SSB_INTERP}, CIC shift ${PLATFORM_SSB_CIC_SHIFT}, K=${PLATFORM_SSB_GAIN_K}, "
               "${PLATFORM_SSB_HILBERT_TAPS}-tap Hilbert, FIFO ${PLATFORM_SSB_FIFO_DEPTH}")

# rPLL's FCLKIN parameter is the input frequency in MHz as a string.
math(EXPR _FCLKIN_MHZ_INT "${PLATFORM_SYS_CLK_HZ} / 1000000")
math(EXPR _FCLKIN_MHZ_FRAC "(${PLATFORM_SYS_CLK_HZ} % 1000000) / 1000")
if (_FCLKIN_MHZ_FRAC EQUAL 0)
    set(PLATFORM_PLL_FCLKIN_MHZ "${_FCLKIN_MHZ_INT}")
else()
    set(PLATFORM_PLL_FCLKIN_MHZ "${_FCLKIN_MHZ_INT}.${_FCLKIN_MHZ_FRAC}")
endif()
# SDC clock period in ns (3 decimals).
math(EXPR _SYS_PERIOD_PS "1000000000000 / ${PLATFORM_SYS_CLK_HZ}")
math(EXPR _SYS_PERIOD_INT "${_SYS_PERIOD_PS} / 1000")
math(EXPR _SYS_PERIOD_FRAC "${_SYS_PERIOD_PS} % 1000")
string(LENGTH "${_SYS_PERIOD_FRAC}" _fl)
while (_fl LESS 3)
    set(_SYS_PERIOD_FRAC "0${_SYS_PERIOD_FRAC}")
    string(LENGTH "${_SYS_PERIOD_FRAC}" _fl)
endwhile()
set(PLATFORM_SYS_CLK_PERIOD_NS "${_SYS_PERIOD_INT}.${_SYS_PERIOD_FRAC}")
math(EXPR _SYS_HALF_PS "${_SYS_PERIOD_PS} / 2")
math(EXPR _SYS_HALF_INT "${_SYS_HALF_PS} / 1000")
math(EXPR _SYS_HALF_FRAC "${_SYS_HALF_PS} % 1000")
string(LENGTH "${_SYS_HALF_FRAC}" _fl)
while (_fl LESS 3)
    set(_SYS_HALF_FRAC "0${_SYS_HALF_FRAC}")
    string(LENGTH "${_SYS_HALF_FRAC}" _fl)
endwhile()
set(PLATFORM_SYS_CLK_HALF_NS "${_SYS_HALF_INT}.${_SYS_HALF_FRAC}")

message(STATUS "Platform: sys_clk ${PLATFORM_SYS_CLK_HZ} Hz, dac_clk ${PLATFORM_DAC_CLK_HZ} Hz "
               "(rPLL x${PLATFORM_PLL_FBDIV}/${PLATFORM_PLL_IDIV}, VCO ${_PLL_VCO_HZ} Hz)")
message(STATUS "Platform: Modbus id ${PLATFORM_MODBUS_DEVICE_ID} (addr_limit=${PLATFORM_MODBUS_ADDR_LIMIT}, "
               "max_read_qty=${PLATFORM_MODBUS_MAX_READ_QTY}), UART ${PLATFORM_UART_BAUD} "
               "${PLATFORM_UART_DATA_BITS}-parity${PLATFORM_UART_PARITY}-${PLATFORM_UART_STOP_BITS}")

configure_file("${SRC_FOLDER}/platform_config.vh.in" "${SRC_FOLDER}/platform_config.vh" @ONLY)
configure_file("${SRC_FOLDER}/acm9767.sdc.in"        "${SRC_FOLDER}/acm9767.sdc"        @ONLY)

# ---------------------------------------------------------------------------
# Simulation tests
# ---------------------------------------------------------------------------
enable_testing()
add_subdirectory("sim")

# ---------------------------------------------------------------------------
# Hardware build targets: drive Gowin's gw_sh in console mode through
# scripts/gw_run.sh (which sets the LD_LIBRARY_PATH / LD_PRELOAD the Linux
# launcher needs). Gowin_PATH is the IDE folder; the wrapper wants its parent.
# ---------------------------------------------------------------------------
set(GPRJ_FILE "${CMAKE_CURRENT_SOURCE_DIR}/acm9767_demo.gprj")
set(HW_WRAPPER "${CMAKE_CURRENT_SOURCE_DIR}/scripts/gw_run.sh")
foreach(_stage synth impl all)
    if (_stage STREQUAL "synth")
        set(_gw_stage syn)
        set(_comment "Gowin synthesis")
    elseif (_stage STREQUAL "impl")
        set(_gw_stage pnr)
        set(_comment "Gowin place-and-route + bitstream")
    else()
        set(_gw_stage all)
        set(_comment "Gowin synthesis + place-and-route + bitstream")
    endif()
    add_custom_target(hw_${_stage}
        COMMAND ${CMAKE_COMMAND} -E env
                GOWIN_PATH=${GOWIN_ROOT}
                GPRJ=${GPRJ_FILE}
                STAGE=${_gw_stage}
                ${HW_WRAPPER}
        COMMENT "${_comment}"
        USES_TERMINAL
    )
endforeach()

# Programming targets. programmer_cli operation indices (see --help):
#   2 = SRAM Program (volatile), 6 = embFlash Erase, Program, Verify (persistent).
if (GOWIN_PROGRAMMER)
    set(BITSTREAM_FILE "${CMAKE_CURRENT_SOURCE_DIR}/impl/pnr/acm9767_demo.fs")
    add_custom_target(hw_program
        COMMAND ${GOWIN_PROGRAMMER} --device GW1NR-9C --run 2 --fsFile ${BITSTREAM_FILE}
        COMMENT "Loading bitstream into SRAM (volatile)"
        USES_TERMINAL
    )
    add_custom_target(hw_program_flash
        COMMAND ${GOWIN_PROGRAMMER} --device GW1NR-9C --run 6 --fsFile ${BITSTREAM_FILE}
        COMMENT "Programming embedded flash (persistent)"
        USES_TERMINAL
    )
endif()

# Regenerate the DDS sine ROM / Hilbert coefficients from platform.json.
find_program(PYTHON3 python3)
if (PYTHON3)
    add_custom_target(sine_lut
        COMMAND ${PYTHON3} "${CMAKE_CURRENT_SOURCE_DIR}/scripts/sine_lut_generator.py"
        COMMENT "Regenerating src/sine_lut_init.vh"
    )
    add_custom_target(hilbert_coefs
        COMMAND ${PYTHON3} "${CMAKE_CURRENT_SOURCE_DIR}/scripts/hilbert_generator.py"
        COMMENT "Regenerating src/hilbert_coefs.vh"
    )
endif()
