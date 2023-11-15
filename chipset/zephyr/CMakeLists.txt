add_library( chipset_zephyr STATIC btstack_chipset_zephyr.c )
target_include_directories( chipset_zephyr PUBLIC . )
target_link_libraries( chipset_zephyr PRIVATE btstack )
