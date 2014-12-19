# $Id: Makefile,v 1.3 2010/10/08 13:24:48 glebius Exp $
PROG=	gpxloggerd
MAN=	gpxloggerd.8
SRCS=	gpxloggerd.c

WARNS?=	2

LDADD=	-lgps

# XXX
CFLAGS+= -I${PREFIX}/include -L${PREFIX}/lib

.include <bsd.prog.mk>
.include <bsd.port.mk>
