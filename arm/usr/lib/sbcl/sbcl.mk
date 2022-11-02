CC=cc
LD=ld
CFLAGS=-g -O2 -fdebug-prefix-map=/build/sbcl-stzDu2/sbcl-2.0.1=. -fstack-protector-strong -Wformat -Werror=format-security -g -Wall -Wundef -Wsign-compare -Wpointer-arith -O3 -marm -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -fno-pie
ASFLAGS=-g -O2 -fdebug-prefix-map=/build/sbcl-stzDu2/sbcl-2.0.1=. -fstack-protector-strong -Wformat -Werror=format-security -g -Wall -Wundef -Wsign-compare -Wpointer-arith -O3 -marm -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -fno-pie
LINKFLAGS=-g -no-pie
LDFLAGS=-Wl,-Bsymbolic-functions -Wl,-z,relro -no-pie
__LDFLAGS__= -no-pie
LIBS=-ldl -lz -lm
