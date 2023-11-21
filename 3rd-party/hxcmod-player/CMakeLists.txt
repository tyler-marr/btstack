add_library( hxcmod-player hxcmod.c mods/nao-deceased_by_disease.c hxcmod.h )
target_include_directories( hxcmod-player PUBLIC . )
target_include_directories( hxcmod-player PUBLIC mods )
