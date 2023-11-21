add_library( chipset_realtek btstack_chipset_realtek.c )
target_include_directories( chipset_realtek PUBLIC . )
target_link_libraries( chipset_realtek PRIVATE btstack_core )
