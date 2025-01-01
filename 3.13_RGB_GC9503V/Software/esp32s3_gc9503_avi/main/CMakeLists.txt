file(GLOB_RECURSE SOURCES_C *.c)
file(GLOB_RECURSE SOURCES_CPP *.cpp)

idf_component_register(
    SRCS ${SOURCES_C} ${SOURCES_CPP} 
    INCLUDE_DIRS "." )
    
spiffs_create_partition_image(storage ../spiffs FLASH_IN_PROJECT)