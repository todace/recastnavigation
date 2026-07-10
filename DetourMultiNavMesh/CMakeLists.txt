add_library(DetourMultiNavMesh)
add_library(RecastNavigation::DetourMultiNavMesh ALIAS DetourMultiNavMesh)

set_target_properties(DetourMultiNavMesh PROPERTIES
    DEBUG_POSTFIX -d
    SOVERSION ${SOVERSION}
    VERSION ${LIB_VERSION}
    COMPILE_PDB_OUTPUT_DIRECTORY .
    COMPILE_PDB_NAME "DetourMultiNavMesh-d"
    CXX_STANDARD 98
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

target_include_directories(DetourMultiNavMesh PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/recastnavigation>"
)

target_sources(DetourMultiNavMesh PRIVATE
    Source/DetourJumpLinkBuilder.cpp
    Source/DetourMultiNavMesh.cpp
    Source/DetourMultiNavMeshQuery.cpp
    Source/DetourSurfaceNav.cpp
)

target_link_libraries(DetourMultiNavMesh Detour)

install(TARGETS DetourMultiNavMesh
    EXPORT recastnavigation-targets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT library
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} ${CMAKE_INSTALL_INCLUDEDIR}/recastnavigation
)

install(DIRECTORY Include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/recastnavigation
    FILES_MATCHING PATTERN "*.h"
)

if(MSVC)
    install(FILES "$<TARGET_FILE_DIR:DetourMultiNavMesh>/DetourMultiNavMesh-d.pdb"
        CONFIGURATIONS "Debug"
        DESTINATION "lib"
        OPTIONAL)
endif()
