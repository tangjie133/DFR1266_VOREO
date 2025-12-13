if(${IDF_VERSION_MAJOR} STREQUAL 4 AND ${IDF_VERSION_MINOR} STREQUAL 1 AND ${IDF_VERSION_PATCH} STREQUAL 3)
	set(req)	
else()
    set(req esp_timer)
endif()

idf_component_register(
    INCLUDE_DIRS .
    SRCS lib8tion.c
    REQUIRES ${req}
)

# include common cmake file for components
set(ESP_IDF_LIB_CMAKE ${CMAKE_CURRENT_LIST_DIR}/common/cmake/esp-idf-lib.cmake)
if(EXISTS ${ESP_IDF_LIB_CMAKE})
    include(${ESP_IDF_LIB_CMAKE})
else()
    message(WARNING "${ESP_IDF_LIB_CMAKE} not found")
endif()
