find_package (Python REQUIRED COMPONENTS Interpreter)

function(gatt_to_header out_var)
  set(result)
  foreach(in_f ${ARGN})
    file(RELATIVE_PATH out_f ${CMAKE_CURRENT_SOURCE_DIR} ${in_f})
    set(out_f "${CMAKE_CURRENT_BINARY_DIR}/${out_f}")
    string(REGEX REPLACE "\\.[^.]*$" ".h" out_f ${out_f})
    add_custom_command(OUTPUT ${out_f}
        COMMAND ${Python_EXECUTABLE}
        ARGS ${BTStackBase_SOURCE_DIR}/tool/compile_gatt.py ${in_f} ${out_f}
        DEPENDS ${in_f}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Creating gatt header file ${out_f}"
        VERBATIM
      )
    list(APPEND result ${out_f})
  endforeach()
  set(${out_var} "${result}" PARENT_SCOPE)
endfunction()

file(GLOB GATT_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.gatt")

gatt_to_header( gatt_headers ${GATT_FILES} )

add_executable( gatt_counter gatt_counter.c gatt_counter.h )
target_link_libraries( gatt_counter PRIVATE btstack )
target_include_directories( gatt_counter PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

