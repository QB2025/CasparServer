if(NOT DEFINED SOURCE_DIR)
    message(FATAL_ERROR "SOURCE_DIR was not provided")
endif()

if(NOT DEFINED PATCH_FILE)
    message(FATAL_ERROR "PATCH_FILE was not provided")
endif()

if(NOT DEFINED GIT_EXECUTABLE OR NOT EXISTS "${GIT_EXECUTABLE}")
    message(FATAL_ERROR "GIT_EXECUTABLE is invalid: ${GIT_EXECUTABLE}")
endif()

# First check whether the patch can be applied normally.
execute_process(
    COMMAND "${GIT_EXECUTABLE}" apply --check "${PATCH_FILE}"
    WORKING_DIRECTORY "${SOURCE_DIR}"
    RESULT_VARIABLE APPLY_CHECK_RESULT
    OUTPUT_QUIET
    ERROR_QUIET
)

if(APPLY_CHECK_RESULT EQUAL 0)
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" apply "${PATCH_FILE}"
        WORKING_DIRECTORY "${SOURCE_DIR}"
        RESULT_VARIABLE APPLY_RESULT
    )

    if(NOT APPLY_RESULT EQUAL 0)
        message(FATAL_ERROR "Failed to apply libajantv2 patch")
    endif()

    message(STATUS "Applied libajantv2 AutoCirculate logging patch")
    return()
endif()

# If normal application is impossible, check whether the patch is already
# present by asking Git whether the patch can be reverse-applied.
execute_process(
    COMMAND "${GIT_EXECUTABLE}" apply --reverse --check "${PATCH_FILE}"
    WORKING_DIRECTORY "${SOURCE_DIR}"
    RESULT_VARIABLE REVERSE_CHECK_RESULT
    OUTPUT_QUIET
    ERROR_QUIET
)

if(REVERSE_CHECK_RESULT EQUAL 0)
    message(STATUS "libajantv2 AutoCirculate logging patch already applied")
    return()
endif()

# Neither state is valid. This normally means the pinned upstream source
# changed unexpectedly or the patch no longer describes this checkout.
message(FATAL_ERROR
    "libajantv2 AutoCirculate logging patch can neither be applied nor "
    "recognized as already applied"
)
