#
# $Id$
#
# (C) Copyright 2011 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

find_path (Stealthlink_INCLUDE_DIRS
           NAMES "AsCL/AsCL_Client.h"
           DOC "Include directory, i.e. parent directory of directory \"AsCL\"")

# Linux specific section, look for one of these
if (UNIX)
   find_library (Stealthlink_GTK_LIBRARY AsCL_GTK
                 PATHS "${Stealthlink_INCLUDE_DIRS}/i686-linux-gnu3"
                 DOC "On Linux, directory that contain AsCL_GTK")
   find_library (Stealthlink_XT_LIBRARY AsCL_Xt
                 PATHS "${Stealthlink_INCLUDE_DIRS}/i686-linux-gnu3"
                 DOC "On Linux, directory that contain AsCL_GTK")
endif (UNIX)

if (WIN32)
   find_library (Stealthlink_WIN32_LIBRARY_Release AsCL_Win32
                 PATHS "${Stealthlink_INCLUDE_DIRS}/libWin32"
                 DOC "On Windows, directory that contain AsCL_Win32")
   find_library (Stealthlink_WIN32_LIBRARY_Debug AsCL_Win32D
                 PATHS "${Stealthlink_INCLUDE_DIRS}/libWin32"
                 DOC "On Windows, directory that contain AsCL_Win32D")
endif (WIN32)
# make sure we have everything we need
set (Stealthlink_FOUND FALSE)

# set libraries as either XT or GTK one
if (Stealthlink_XT_LIBRARY)
    set (Stealthlink_LIBRARIES ${Stealthlink_XT_LIBRARY})
endif (Stealthlink_XT_LIBRARY)
if (Stealthlink_GTK_LIBRARY)
    set (Stealthlink_LIBRARIES ${Stealthlink_GTK_LIBRARY})
endif (Stealthlink_GTK_LIBRARY)

if (Stealthlink_WIN32_LIBRARY_Release AND Stealthlink_WIN32_LIBRARY_Debug)
   set (Stealthlink_LIBRARIES
        debug ${Stealthlink_WIN32_LIBRARY_Debug}
        optimized ${Stealthlink_WIN32_LIBRARY_Release}
        )
else (Stealthlink_WIN32_LIBRARY_Release AND Stealthlink_WIN32_LIBRARY_Debug)
   if (Stealthlink_WIN32_LIBRARY_Release)
       set (Stealthlink_LIBRARIES ${Stealthlink_WIN32_LIBRARY_Release})
   endif (Stealthlink_WIN32_LIBRARY_Release)
endif (Stealthlink_WIN32_LIBRARY_Release AND Stealthlink_WIN32_LIBRARY_Debug)

# set to true if one library found along with include directory
if (Stealthlink_INCLUDE_DIRS AND Stealthlink_LIBRARIES)
   set (Stealthlink_FOUND TRUE)
   if (WIN32)
       set (Stealthlink_LIBRARIES ${Stealthlink_LIBRARIES} ws2_32.lib)
   endif (WIN32)
   mark_as_advanced (Stealthlink_INCLUDE_DIRS
                     Stealthlink_GTK_LIBRARY
                     Stealthlink_XT_LIBRARY
                     Stealthlink_LIBRARIES)
endif (Stealthlink_INCLUDE_DIRS AND Stealthlink_LIBRARIES)
