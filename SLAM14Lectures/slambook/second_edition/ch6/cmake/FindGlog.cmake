MACRO(GLOG_REPORT_NOT_FOUND REASON_MSG)
  UNSET(GLOG_FOUND)
  UNSET(GLOG_INCLUDE_DIRS)
  UNSET(GLOG_LIBRARIES)
  # Make results of search visible in the CMake GUI if glog has not
  # been found so that user does not have to toggle to advanced view.
  MARK_AS_ADVANCED(CLEAR GLOG_INCLUDE_DIR
                         GLOG_LIBRARY)
  # Note <package>_FIND_[REQUIRED/QUIETLY] variables defined by FindPackage()
  # use the camelcase library name, not uppercase.
  IF (Glog_FIND_QUIETLY)
    MESSAGE(STATUS "Failed to find glog - " ${REASON_MSG} ${ARGN})
  ELSEIF (Glog_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "Failed to find glog - " ${REASON_MSG} ${ARGN})
  ELSE()
    # Neither QUIETLY nor REQUIRED, use SEND_ERROR which emits an error
    # that prevents generation, but continues configuration.
    MESSAGE(SEND_ERROR "Failed to find glog - " ${REASON_MSG} ${ARGN})
  ENDIF ()
ENDMACRO(GLOG_REPORT_NOT_FOUND)
# TODO: Add standard Windows search locations for glog.
LIST(APPEND GLOG_CHECK_INCLUDE_DIRS
  /usr/include
  /usr/local/include
  /usr/local/homebrew/include # Mac OS X
  /opt/local/var/macports/software # Mac OS X.
  /opt/local/include)
LIST(APPEND GLOG_CHECK_LIBRARY_DIRS
  /usr/lib
  /usr/local/lib
  /usr/local/homebrew/lib # Mac OS X.
  /opt/local/lib)
# Search supplied hint directories first if supplied.
FIND_PATH(GLOG_INCLUDE_DIR
  NAMES glog/logging.h
  PATHS ${GLOG_INCLUDE_DIR_HINTS}
  ${GLOG_CHECK_INCLUDE_DIRS})
IF (NOT GLOG_INCLUDE_DIR OR
    NOT EXISTS ${GLOG_INCLUDE_DIR})
  GLOG_REPORT_NOT_FOUND(
    "Could not find glog include directory, set GLOG_INCLUDE_DIR "
    "to directory containing glog/logging.h")
ENDIF (NOT GLOG_INCLUDE_DIR OR
       NOT EXISTS ${GLOG_INCLUDE_DIR})
FIND_LIBRARY(GLOG_LIBRARY NAMES glog
  PATHS ${GLOG_LIBRARY_DIR_HINTS}
  ${GLOG_CHECK_LIBRARY_DIRS})
IF (NOT GLOG_LIBRARY OR
    NOT EXISTS ${GLOG_LIBRARY})
  GLOG_REPORT_NOT_FOUND(
    "Could not find glog library, set GLOG_LIBRARY "
    "to full path to libglog.")
ENDIF (NOT GLOG_LIBRARY OR
       NOT EXISTS ${GLOG_LIBRARY})
# Mark internally as found, then verify. GLOG_REPORT_NOT_FOUND() unsets
# if called.
SET(GLOG_FOUND TRUE)
# Glog does not seem to provide any record of the version in its
# source tree, thus cannot extract version.
# Catch case when caller has set GLOG_INCLUDE_DIR in the cache / GUI and
# thus FIND_[PATH/LIBRARY] are not called, but specified locations are
# invalid, otherwise we would report the library as found.
IF (GLOG_INCLUDE_DIR AND
    NOT EXISTS ${GLOG_INCLUDE_DIR}/glog/logging.h)
  GLOG_REPORT_NOT_FOUND(
    "Caller defined GLOG_INCLUDE_DIR:"
    " ${GLOG_INCLUDE_DIR} does not contain glog/logging.h header.")
ENDIF (GLOG_INCLUDE_DIR AND
       NOT EXISTS ${GLOG_INCLUDE_DIR}/glog/logging.h)
# TODO: This regex for glog library is pretty primitive, could it be better?
IF (GLOG_LIBRARY AND
    NOT ${GLOG_LIBRARY} MATCHES ".*glog[^/]*")
  GLOG_REPORT_NOT_FOUND(
    "Caller defined GLOG_LIBRARY: "
    "${GLOG_LIBRARY} does not match glog.")
ENDIF (GLOG_LIBRARY AND
       NOT ${GLOG_LIBRARY} MATCHES ".*glog[^/]*")
# Set standard CMake FindPackage variables if found.
IF (GLOG_FOUND)
  SET(GLOG_INCLUDE_DIRS ${GLOG_INCLUDE_DIR})
  SET(GLOG_LIBRARIES ${GLOG_LIBRARY})
ENDIF (GLOG_FOUND)
# Handle REQUIRED / QUIET optional arguments.
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Glog DEFAULT_MSG
  GLOG_INCLUDE_DIRS GLOG_LIBRARIES)
# Only mark internal variables as advanced if we found glog, otherwise
# leave them visible in the standard GUI for the user to set manually.
IF (GLOG_FOUND)
  MARK_AS_ADVANCED(FORCE GLOG_INCLUDE_DIR
                         GLOG_LIBRARY)
ENDIF (GLOG_FOUND)