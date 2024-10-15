# Copyright (C) 2016 JRL AIST-CNRS.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

#
# This file contains a collection of macros specific to Visual Studio and the
# MSVC compilers.
#

# GET_MSVC_VERSION
# ----------------
#
# This macro return a string corresponding to the version of the MSVC compiler
# used.
# The value is returned in the variable MSVC_VERSION. It is an empty string if
#  - the compiler used is not MSVC,
#  - it is older than MSVC 6.0
#  - or newer than MSVC 14.0
#
MACRO(GET_MSVC_VERSION)
  if(MSVC60)
    set(MSVC_VERSION "6.0")
  elseif(MSVC70)
    set(MSVC_VERSION "7.0")
  elseif(MSVC71)
    set(MSVC_VERSION "7.1")
  elseif(MSVC80)
    set(MSVC_VERSION "8.0")
  elseif(MSVC90)
    set(MSVC_VERSION "9.0")
  elseif(MSVC10)
    set(MSVC_VERSION "10.0")
  elseif(MSVC11)
    set(MSVC_VERSION "11.0")
  elseif(MSVC12)
    set(MSVC_VERSION "12.0")
  elseif(MSVC14)
    set(MSVC_VERSION "14.0")
  else()
    if(MSVC)
	  set(MSVC_VERSION "14.0")
	  message("MSVC version not found. Set MSVC_VERSION to 14.0 by default. Please update the GET_MSVC_VERSION macro." AUTHOR_WARNING)
	else()
      set(MSVC_VERSION "")
	  message("MSVC version not found. You are not using a MSVC generator." AUTHOR_WARNING)
	endif()
  endif()
ENDMACRO(GET_MSVC_VERSION)


# REQUEST_MINIMUM_MSVC_VERSION(VER)
# ---------------------------------
#
# Return a fatal error if the current MSVC version is strictly lower than VER
# or when no MSVC compiler or an unknown version of it is used.
#
# A reason for the version to be unknown is that it is newer than the versions
# recognize by the above macro GET_MSVC_VERSION. In this case, please update 
# the macro and its documentation.
#
MACRO(REQUIRE_MINIMUM_MSVC_VERSION VERSION)
  GET_MSVC_VERSION()
  if (${MSVC_VERSION})
    if (NOT ${MSVC_VERSION} VERSION_GREATER ${VERSION})
	  message("Minimum MSVC version required is " ${VERSION} " but version " ${MSVC_VERSION} " was found." FATAL_ERROR)
    endif()
  else()
    message("You are requiring a minimum version for MSVC but you do not use a MSVC generator." FATAL_ERROR)
  endif(${MSVC_VERSION})
ENDMACRO(REQUIRE_MINIMUM_MSVC_VERSION)

# GENERATE_MSVC_DOT_USER_FILE(NAME [ADDITIONAL_PATH])
# ----------------------------
#
# Generate the file NAME.vcxproj.user alongside the project file NAME.vcxproj
# that sets up the path for a debugging session by adding to %PATH% the 
# location of the libraries generated by the solution, i.e. it sets the field
# Configuration Properties > Debuging > Environment to 
# PATH=$(SolutionDir)\src\$(Configuration);ADDITIONAL_PATH;%PATH%
# This is intended to be used for the test projects, so that they can be 
# launched from within the Visual Studio IDE by pressing F5.
# ADDITIONAL_PATH is an optional semicolon-separated list of paths.
#
MACRO(GENERATE_MSVC_DOT_USER_FILE NAME)
  if(MSVC)
    REQUIRE_MINIMUM_MSVC_VERSION("10.0")
    
    if(${ARGC} GREATER 1)
      set(MSVC_DOT_USER_ADDITIONAL_PATH_DOT_USER ";${ARGV1}")
    endif()
    
    GET_MSVC_VERSION()
    set(DOT_USER_TEMPLATE_PATH ${CMAKE_SOURCE_DIR}/cmake/msvc.vcxproj.user.in)
    set(DOT_USER_FILE_PATH ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.vcxproj.user)
    configure_file(${DOT_USER_TEMPLATE_PATH} ${DOT_USER_FILE_PATH})
    
    unset(MSVC_DOT_USER_ADDITIONAL_PATH_DOT_USER)
  endif(MSVC)
ENDMACRO(GENERATE_MSVC_DOT_USER_FILE)