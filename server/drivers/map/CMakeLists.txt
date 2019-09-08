PLAYERDRIVER_OPTION (mapfile build_mapfile ON)
PLAYERDRIVER_REQUIRE_PKG (mapfile build_mapfile gdk-pixbuf-2.0 
    mapfile_includeDirs mapfile_libDirs mapfile_linkLibs mapfile_linkFlags mapfile_cFlags )
PLAYERDRIVER_REQUIRE_PKG (mapfile build_mapfile yaml-cpp 
 mapfile_includeDirs mapfile_libDirs mapfile_linkLibs mapfile_linkFlags mapfile_cFlags)
PLAYERDRIVER_ADD_DRIVER (mapfile build_mapfile
    INCLUDEDIRS ${mapfile_includeDirs} LIBDIRS ${mapfile_libDirs}  LINKLIBS ${mapfile_linkLibs}  
    LINKFLAGS ${mapfile_linkFlags}  CFLAGS ${mapfile_cFlags}
    SOURCES mapfile.cc)
