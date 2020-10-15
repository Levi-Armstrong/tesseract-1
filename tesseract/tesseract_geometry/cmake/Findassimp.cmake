if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(ASSIMP_ARCHITECTURE "64")
elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
  set(ASSIMP_ARCHITECTURE "32")
endif(CMAKE_SIZEOF_VOID_P EQUAL 8)

if(WIN32)
  set(ASSIMP_ROOT_DIR CACHE PATH "ASSIMP root directory")

  find_package(PkgConfig REQUIRED)
  # assimp is required, so REQUIRE the second attempt
  pkg_check_modules(PC_ASSIMP REQUIRED assimp)
  message(AUTHOR_WARNING "PC ASIMP PREFIX: ${PC_ASSIMP_PREFIX}")
  message(AUTHOR_WARNING "PC ASIMP LIBRARY DIR: ${PC_ASSIMP_LIBRARY_DIR}")
  message(AUTHOR_WARNING "PC ASIMP LIBDIR: ${PC_ASSIMP_LIBDIR}")
  message(AUTHOR_WARNING "PC ASIMP INCLUDE DIR: ${PC_ASSIMP_INCLUDEDIR}")
  message(AUTHOR_WARNING "PC ASIMP LIBRARIES: ${PC_ASSIMP_LIBRARIES}")

  if(MSVC_TOOLSET_VERSION)
    set(ASSIMP_MSVC_VERSION "vc${MSVC_TOOLSET_VERSION}")
  else(MSVC_TOOLSET_VERSION)
    if(MSVC12)
      set(ASSIMP_MSVC_VERSION "vc120")
    elseif(MSVC14)
      set(ASSIMP_MSVC_VERSION "vc140")
    endif(MSVC12)
  endif(MSVC_TOOLSET_VERSION)

  # Find path of each library
  find_path(ASSIMP_INCLUDE_DIR NAMES assimp/anim.h HINTS ${PC_ASSIMP_INCLUDEDIR})
  find_path(ASSIMP_LIBRARY_DIR NAMES assimp-${ASSIMP_MSVC_VERSION}-mt.lib HINTS ${PC_ASSIMP_LIBDIR} "${PC_ASSIMP_PREFIX}/Lib" REQUIRED)
  find_library(ASSIMP_LIBRARIES NAMES assimp-${ASSIMP_MSVC_VERSION}-mt.lib PATHS ${ASSIMP_LIBRARY_DIR} REQUIRED)

  message(AUTHOR_WARNING "ASIMP INCLUDE DIR (FIND PATH): ${ASSIMP_INCLUDE_DIR}")
  message(AUTHOR_WARNING "ASIMP LIBRARY DIR (FIND LIBRARY): ${ASSIMP_LIBRARY_DIR}")
  message(AUTHOR_WARNING "ASIMP LIBRARIES: ${ASSIMP_LIBRARIES}")

else(WIN32)

  find_path(
    assimp_INCLUDE_DIRS
    NAMES assimp/postprocess.h assimp/scene.h assimp/version.h assimp/config.h assimp/cimport.h
    PATHS /usr/local/include
    PATHS /usr/include/
  )

  find_library(
    assimp_LIBRARIES
    NAMES assimp
    PATHS /usr/local/lib/
    PATHS /usr/lib64/
    PATHS /usr/lib/
  )

  if (assimp_INCLUDE_DIRS AND assimp_LIBRARIES)
    SET(assimp_FOUND TRUE)
  ENDIF (assimp_INCLUDE_DIRS AND assimp_LIBRARIES)

  if (assimp_FOUND)
    if (NOT assimp_FIND_QUIETLY)
          message(STATUS "Found asset importer library: ${assimp_LIBRARIES}")
    endif (NOT assimp_FIND_QUIETLY)
  else (assimp_FOUND)
    if (assimp_FIND_REQUIRED)
          message(FATAL_ERROR "Could not find asset importer library")
    endif (assimp_FIND_REQUIRED)
  endif (assimp_FOUND)

endif(WIN32)
