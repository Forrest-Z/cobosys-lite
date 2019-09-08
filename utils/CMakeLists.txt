OPTION (BUILD_UTILS "Build the various utilities" ON)
MESSAGE (STATUS "===== Player utilities =====")
IF (BUILD_UTILS)
   
    ADD_SUBDIRECTORY (playernav)
    ADD_SUBDIRECTORY (playerprint)
    ADD_SUBDIRECTORY (playerprop)
    ADD_SUBDIRECTORY (playerv)
 
ELSE (BUILD_UTILS)
    MESSAGE (STATUS "Disabled by user. Use ccmake to enable.")
ENDIF (BUILD_UTILS)
MESSAGE (STATUS "============================")
