find_package(Doxygen REQUIRED)
find_package(Sphinx REQUIRED)

set(DOXYGEN_SRC_DIR ${PROJECT_SOURCE_DIR}/src)
set(DOXYGEN_SCRIPTS_DIR ${PROJECT_SOURCE_DIR}/scripts)
set(DOXYGEN_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/docs/doxygen)
set(DOXYGEN_XML_DIR ${DOXYGEN_OUTPUT_DIR}/xml)
set(DOXYGEN_INDEX_FILE ${DOXYGEN_OUTPUT_DIR}/xml/index.html)
set(DOXYFILE_IN ${PROJECT_SOURCE_DIR}/docs/Doxyfile.in)
set(DOXYFILE_OUT ${PROJECT_SOURCE_DIR}/docs/Doxyfile)

#Replace variables inside @@ with the current values
configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

#Doxygen won't create this for us
file(MAKE_DIRECTORY ${DOXYGEN_OUTPUT_DIR})

add_custom_command(OUTPUT ${DOXYGEN_INDEX_FILE}
                   DEPENDS ${DOXYGEN_SRC_DIR} ${DOXYGEN_SCRIPTS_DIR}
                   COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
                   MAIN_DEPENDENCY ${DOXYFILE_OUT} ${DOXYFILE_IN}
                   COMMENT "Generating doxygen docs"
                   VERBATIM)

add_custom_target(Doxygen ALL DEPENDS ${DOXYGEN_INDEX_FILE})

set(SPHINX_SOURCE ${PROJECT_SOURCE_DIR}/docs/source)
set(SPHINX_BUILD ${PROJECT_SOURCE_DIR}/docs/sphinx)
set(SPHINX_INDEX_FILE ${SPHINX_BUILD}/index.html)

# Only regenerate Sphinx when:
# - Doxygen has rerun
# - Our doc files have been updated
# - The Sphinx config has been updated
add_custom_command(OUTPUT ${SPHINX_INDEX_FILE}
                   COMMAND 
                     ${SPHINX_EXECUTABLE} -b html
                     # Tell Breathe where to find the Doxygen output
                     -Dbreathe_projects.geolocation=${DOXYGEN_XML_DIR}
                   ${SPHINX_SOURCE} ${SPHINX_BUILD}
                   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                   DEPENDS
                   # Other docs files you want to track should go here (or in some variable)
                   ${CMAKE_CURRENT_SOURCE_DIR}/source/index.rst
                   ${DOXYGEN_INDEX_FILE}
                   MAIN_DEPENDENCY ${SPHINX_SOURCE}/conf.py
                   COMMENT "Generating documentation with Sphinx")

# Nice named target so we can run the job easily
add_custom_target(Sphinx ALL DEPENDS ${SPHINX_INDEX_FILE})

# Add an install target to install the docs
include(GNUInstallDirs)
install(DIRECTORY ${SPHINX_BUILD}
DESTINATION ${CMAKE_INSTALL_DOCDIR})