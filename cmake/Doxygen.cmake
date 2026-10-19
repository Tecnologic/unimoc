function(enable_doxygen)
  option(ENABLE_DOXYGEN "Enable doxygen doc builds of source" OFF)
  if(ENABLE_DOXYGEN)
    set(DOXYGEN_CALLER_GRAPH YES)
    set(DOXYGEN_CALL_GRAPH YES)
    set(DOXYGEN_EXTRACT_ALL YES)
    set(DOXYGEN_EXCLUDE_PATTERNS
        "*/cymon/*"
        "*/gitversion/*"
        "*/_deps/*"
        "*/build/*")
    find_package(Doxygen REQUIRED dot)
    doxygen_add_docs(doxygen-docs
      ${PROJECT_SOURCE_DIR}/lib/control
      ${PROJECT_SOURCE_DIR}/lib/observer
      ${PROJECT_SOURCE_DIR}/lib/system
      ${PROJECT_SOURCE_DIR}/lib/units
      ${PROJECT_SOURCE_DIR}/src
      ${PROJECT_SOURCE_DIR}/hardware)

  endif()
endfunction()
