# Boost 1.74 uses b2. Build its engine in the build tree, keeping vendored source untouched.
file(MAKE_DIRECTORY "${BINARY_DIR}")
if(NOT EXISTS "${BINARY_DIR}/engine/b2")
    file(COPY "${SOURCE_DIR}/tools/build/src/engine/" DESTINATION "${BINARY_DIR}/engine")
    execute_process(COMMAND "${CMAKE_COMMAND}" -E env "CXX=${CXX}" sh build.sh gcc
        WORKING_DIRECTORY "${BINARY_DIR}/engine" RESULT_VARIABLE result)
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "Could not build Boost b2")
    endif()
endif()
file(WRITE "${BINARY_DIR}/user-config.jam" "using gcc : aviator : \"${CXX}\" ;\n")
execute_process(COMMAND "${BINARY_DIR}/engine/b2"
    "--user-config=${BINARY_DIR}/user-config.jam" --ignore-site-config
    "--build-dir=${BINARY_DIR}/objects" "--prefix=${PREFIX}" --layout=system
    --with-filesystem --with-system --with-thread --with-date_time --with-serialization
    toolset=gcc-aviator variant=release link=shared threading=multi runtime-link=shared
    cxxflags=-fPIC "-j${JOBS}" install
    WORKING_DIRECTORY "${SOURCE_DIR}" RESULT_VARIABLE result)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "Could not build/install Boost")
endif()
