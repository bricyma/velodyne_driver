add_library(velodyne_input input.cc)
add_library(my_velodyne_rawdata rawdata.cc)

target_link_libraries(velodyne_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

target_link_libraries(my_velodyne_rawdata 
                      ${catkin_LIBRARIES}
                      ${libpcap_LIBRARIES}
) 

if(catkin_EXPORTED_TARGETS)
  add_dependencies(my_velodyne_rawdata ${catkin_EXPORTED_TARGETS})
  add_dependencies(velodyne_input ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS velodyne_input
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS my_velodyne_rawdata
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
