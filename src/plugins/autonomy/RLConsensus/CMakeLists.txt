#--------------------------------------------------------
# Library Creation
#--------------------------------------------------------
SET (LIBRARY_NAME RLConsensus_plugin)
SET (LIB_MAJOR 0)
SET (LIB_MINOR 0)
SET (LIB_RELEASE 1)

file(GLOB SRCS *.cpp)

ADD_LIBRARY(${LIBRARY_NAME} SHARED
  ${SRCS}
  )

TARGET_LINK_LIBRARIES(${LIBRARY_NAME}
  scrimmage-core
  ScrimmageOpenAIAutonomy_plugin
  RLSimple_plugin
  )

SET (_soversion ${LIB_MAJOR}.${LIB_MINOR}.${LIB_RELEASE})

set_target_properties(${LIBRARY_NAME} PROPERTIES
  SOVERSION ${LIB_MAJOR}
  VERSION ${_soversion}
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_PLUGIN_LIBS_DIR}
  )

install(TARGETS ${LIBRARY_NAME}
  # IMPORTANT: Add the library to the "export-set"
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/${PROJECT_NAME}/plugin_libs
)

set(PROJECT_PLUGINS ${PROJECT_PLUGINS} ${LIBRARY_NAME} PARENT_SCOPE)
