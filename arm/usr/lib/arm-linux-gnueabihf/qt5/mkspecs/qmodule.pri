!host_build|!cross_compile {
    QMAKE_CFLAGS=-g -O2 -fstack-protector-strong -Wformat -Werror=format-security -Wdate-time -D_FORTIFY_SOURCE=2 -D_LARGEFILE_SOURCE -D_FILE_OFFSET_BITS=64
    QMAKE_CXXFLAGS=-g -O2 -fstack-protector-strong -Wformat -Werror=format-security -Wdate-time -D_FORTIFY_SOURCE=2 -D_LARGEFILE_SOURCE -D_FILE_OFFSET_BITS=64
    QMAKE_LFLAGS=-Wl,-Bsymbolic-functions -Wl,-z,relro -Wl,--as-needed
}
QT_CPU_FEATURES.arm = 
QT.global_private.enabled_features = alloca_h alloca dbus dbus-linked gui libudev network posix_fallocate reduce_exports release_tools sql system-zlib testlib widgets xml
QT.global_private.disabled_features = sse2 alloca_malloc_h android-style-assets avx2 private_tests gc_binaries reduce_relocations stack-protector-strong
PKG_CONFIG_EXECUTABLE = /usr/bin/arm-linux-gnueabihf-pkg-config
QMAKE_LIBS_DBUS = /usr/lib/arm-linux-gnueabihf/libdbus-1.so
QMAKE_INCDIR_DBUS = /usr/include/dbus-1.0 /usr/lib/arm-linux-gnueabihf/dbus-1.0/include
QMAKE_LIBS_LIBUDEV = /lib/arm-linux-gnueabihf/libudev.so
QT_COORD_TYPE = double
QMAKE_LIBS_ZLIB = /usr/lib/arm-linux-gnueabihf/libz.so
QT_BUILD_PARTS += libs examples tools
CONFIG += compile_examples enable_new_dtags largefile precompile_header nostrip
QT_HOST_CFLAGS_DBUS += -I/usr/include/dbus-1.0 -I/usr/lib/arm-linux-gnueabihf/dbus-1.0/include
