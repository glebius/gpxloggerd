CFLAGS+= -I${PREFIX}/include -L${PREFIX}/lib

PROG=	gpxloggerd
MAN=	gpxloggerd.8
SRCS=	gpxloggerd.c
WARNS?=	5
LDADD=	-lgps -lm

.include <bsd.prog.mk>
