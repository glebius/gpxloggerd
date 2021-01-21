/*-
 * Copyright (c) 2010,2011 Gleb Smirnoff <glebius@glebi.us>
 * Copyright (c) 2010,2011 by the GPSD project
 * Copyright (c) Amaury Jacquot
 * Copyright (c) Chris Kuethe
 * Compilation copyright is held by the GPSD project.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither name of the GPSD project nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/types.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <grp.h>
#include <libgen.h>
#include <math.h>
#include <pwd.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#include <gps.h>

#include "timespecop.h"

static bool intrack = false;

static char	*o_template;
static FILE	*logfile;
static char	*progname;
static char 	*device;
static const char *server = "localhost";
static const char *port = DEFAULT_GPSD_PORT;
static char	*pidfile;
static struct timespec	timeout =  { 300 /* seconds */, 0 };
static struct timespec	interval = { 1 /* seconds */, 0 };
static double	minmove = 0;	/* meters */
static double	maxseg = 200;	/* meters */
static double	minbearing = 0;	/* degrees */
static bool	verbose = false;

static void
print_gpx_header(void)
{
	fprintf(logfile, "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n");
	fprintf(logfile, "<gpx version=\"1.1\" creator=\"gpxloggerd\"\n");
	fprintf(logfile,
	"        xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n");
	fprintf(logfile, "        xmlns=\"http://www.topografix.com/GPX/1.1\"\n");
	fprintf(logfile,
	"        xsi:schemaLocation=\"http://www.topografix.com/GPS/1/1\n");
	fprintf(logfile, "        http://www.topografix.com/GPX/1/1/gpx.xsd\">\n");
	fflush(logfile);
}

static void
print_gpx_trk_end(void)
{
	fprintf(logfile, "  </trkseg>\n");
	fprintf(logfile, " </trk>\n");
	intrack = false;
	fflush(logfile);
}

static void
print_gpx_footer(void)
{
	if (intrack)
		print_gpx_trk_end();
	fprintf(logfile, "</gpx>\n");
	fclose(logfile);
}

static void
print_gpx_trk_start(void)
{
	fprintf(logfile, " <trk>\n");
	fprintf(logfile, "  <trkseg>\n");
	fflush(logfile);
}

static void
print_fix(struct gps_data_t *gpsdata)
{
	struct gps_fix_t *fix = &gpsdata->fix;
	char tbuf[128];

	fprintf(logfile, "   <trkpt lat=\"%f\" lon=\"%f\">\n",
	    fix->latitude, fix->longitude);
	if (!isnan(fix->altitude))
		fprintf(logfile, "    <ele>%.f</ele>\n", fix->altitude);
	fprintf(logfile, "    <time>%s</time>\n",
	    timespec_to_iso8601(fix->time, tbuf, sizeof(tbuf)));
	if (verbose) {
		switch (fix->mode) {
		case MODE_3D:
			fprintf(logfile, "    <fix>3d</fix>\n");
			break;
		case MODE_2D:
			fprintf(logfile, "    <fix>2d</fix>\n");
			break;
		case MODE_NO_FIX:
			fprintf(logfile, "    <fix>none</fix>\n");
			break;
		default:
			syslog(LOG_WARNING, "%s: unexpected fix %d", __func__,
			    fix->mode);
			break;
		}
		if (gpsdata->satellites_used > 0)
	    		fprintf(logfile, "    <sat>%d</sat>\n",
			    gpsdata->satellites_used);
		if (!isnan(gpsdata->dop.hdop))
			fprintf(logfile, "    <hdop>%.1f</hdop>\n",
			    gpsdata->dop.hdop);
		if (!isnan(gpsdata->dop.vdop))
			fprintf(logfile, "    <vdop>%.1f</vdop>\n",
			    gpsdata->dop.vdop);
		if (!isnan(gpsdata->dop.pdop))
			fprintf(logfile, "    <pdop>%.1f</pdop>\n",
			    gpsdata->dop.pdop);
	}
	fprintf(logfile, "   </trkpt>\n");
	fflush(logfile);
}

static void
opennewfile(char *template)
{
	char	fname[PATH_MAX];
	time_t	t;
	size_t	s;

	t = time(NULL);
	s = strftime(fname, sizeof(fname), template, localtime(&t));
	if (s == 0) {
		syslog(LOG_ERR, "Bad template \"%s\"", template);
		exit(1);
	}
	logfile = fopen(fname, "w");
	if (logfile == NULL) {
		syslog(LOG_ERR, "Failed to open %s: %m", fname);
		exit(1);
	}
	syslog(LOG_DEBUG, "Opened %s for writing", fname);
	print_gpx_header();
}

static void
process(struct gps_data_t *gpsdata)
{
	/* ofix - last logged fix, pfix - previous obtained fix. */
	static struct gps_fix_t ofix, pfix;
	static bool first = true;
	static double obearing;

	struct gps_fix_t *fix = &gpsdata->fix;
	double move, bearing;
	struct timespec ts;

	/* No fix. */
	if (fix->mode < MODE_2D)
		return;

	/* -I filter */
	timespecsub(&fix->time, &ofix.time, &ts);
	if (timespeccmp(&ts, &interval, <))
		return;

	if (minmove || minbearing)
		move = earth_distance(ofix.latitude, ofix.longitude,
		    fix->latitude, fix->longitude);

	/* -m filter */
	if (minmove && !first && move < minmove)
		goto nolog;

	/* -a filter */
	if (minbearing && !first) {
		if (pfix.longitude == fix->longitude &&
		    pfix.latitude == fix->latitude)
			bearing = obearing;
		else
			(void) earth_distance_and_bearings(pfix.latitude,
			    pfix.longitude, fix->latitude, fix->longitude,
			    &bearing, NULL);

		if ( move < maxseg && fabs(bearing - obearing) < minbearing)
			goto nolog;
	}

	/*
	 * Make new track if the jump in time is above timeout.
	 */
	if (timespeccmp(&ts, &timeout, >) && !first) {
		print_gpx_trk_end();
		intrack = false;
	}

	if (!intrack) {
		print_gpx_trk_start();
		intrack = true;
		if (first)
			first = false;
	}

	print_fix(gpsdata);
	ofix = *fix;
	if (minbearing)
		obearing = bearing;
nolog:
	if (minbearing)
		pfix = *fix;
}

static int signal_fd[2];

static void
enqueue_signal(int sig)
{
	if (write(signal_fd[1], &sig, sizeof sig) != sizeof sig)
		syslog(LOG_ERR, "Can't process signal");
}

static void
process_signal(void)
{
	int sig;

	while (read(signal_fd[0], &sig, sizeof sig) == sizeof(sig)) {
		syslog(LOG_DEBUG, "caught signal: %d", sig);
		switch (sig) {
		case SIGHUP:
			if (o_template == NULL)
				return;
			print_gpx_footer();
			opennewfile(o_template);
			print_gpx_header();
			break;
		case SIGINT:
		case SIGTERM:
		case SIGQUIT:
			syslog(LOG_NOTICE, "going down on signal %d", sig);
			print_gpx_footer();
			exit(0);
			break;
		default:
			syslog(LOG_NOTICE, "unexpected signal %d received",
			    sig);
			break;
		}
	}
}

static void
unlink_pidfile(void)
{
	(void )unlink(pidfile);
}

static void
usage(void)
{
	fprintf(stderr, "%s%s%s\n%s\n%s\n%s\n%s\n",
"Usage: ", progname, " [-V] [-d] [-h] [-v] [-D level] [-p pidfile]",
"       [-u user[:group]] [-f template]",
"       [-I interval] [-i timeout]",
"       [-m meters] [-a degrees] [-M meters]",
"       [server[:port[:device]]]");
	fprintf(stderr, "\nDefaults to '%s -i %u %s:%s'\n",
	    progname, (unsigned int )timeout.tv_sec, server, port);
	exit(1);
}

int
main(int argc, char **argv)
{
	struct gps_data_t gpsdata;
	struct sigaction sa;
	struct timeval tv;
	fd_set fds0;
	struct passwd *pwd;
	struct group *grp;
	char *c, *user, *group;
	int ch, fdmax;
	bool o_daemon = false;

	progname = argv[0];
	openlog(basename(progname), LOG_PID | LOG_PERROR, LOG_DAEMON);

	user = group = NULL;
	while ((ch = getopt(argc, argv, "D:df:hI:i:M:m:a:p:Vvu:")) != -1) {
	switch (ch) {
	case 'D':	/* Set debug level. */
	   {
		int debug;

		debug = atoi(optarg);
		gps_enable_debug(debug, stdout);
		break;
	    }
	case 'd':
		o_daemon = true;
		break;
	case 'f':	/* Output file name. */
	    {
		o_template = optarg;
		break;
	    }
	case 'I':	/* Set polling interval. */
		interval.tv_sec = (time_t) atoi(optarg);
		if (interval.tv_sec < 1)
			interval.tv_sec = 1;
		if (interval.tv_sec > 60)
			syslog(LOG_WARNING, "Are you sane?"
			    "Logging interval is more than a minute!");
		break;
	case 'i':	/* Set track timeout. */
		timeout.tv_sec = (time_t) atoi(optarg);
		if (timeout.tv_sec < 1)
			timeout.tv_sec = 1;
		if (timeout.tv_sec >= 3600)
			syslog(LOG_WARNING,
			    "track timeout is an hour or more!");
		break;
	case 'M':
		maxseg = strtod(optarg, NULL);
		break;
	case 'm':
		minmove = strtod(optarg, NULL);
		break;
	case 'a':
		minbearing = strtod(optarg, NULL) * DEG_2_RAD;
		break;
	case 'p':
		pidfile = optarg;
		break;
	case 'v':
		verbose = true;
		break;
	case 'V':
		fprintf(stderr, "gpxloggerd 0.2.2\n");
		exit(0);
	case 'u':
		user = optarg;
		break;
	default:
		usage();
		/* NOTREACHED */
	} /* switch */
	} /* while */

	/* Parse main argument. */
	if (optind < argc) {

		server = argv[optind];
		if ((c = strchr(server, ':')) != NULL) {
			*c = '\0';
			port = ++c;
			if ((c = strchr(port, ':')) != NULL) {
				*c = '\0';
				device = ++c;
			}
		}
	}

	if (o_daemon)
		daemon(0,0);

	/* Write pidfile. */
	if (pidfile != NULL) {
		char pid[6];	/* PID_MAX is 99999 */
		int fd;

		(void )unlink(pidfile);
		if ((fd = open(pidfile, O_WRONLY|O_CREAT|O_TRUNC|O_EXCL,
		    0644)) > 0) {
			snprintf(pid, sizeof(pid), "%lu\n",
			    (long unsigned )getpid());
			if (write(fd, pid, strlen(pid)) < 1)
				syslog(LOG_WARNING, "Failed to write pidfile: %m");
			close(fd);
			atexit(unlink_pidfile);
		} else
			syslog(LOG_WARNING, "Failed to open pidfile: %m");
	}

	/* setuid()/setgid() */
	if (user != NULL) {
		grp = NULL; /* stupid old gcc */
		if ((c = strchr(user, ':')) != NULL) {
			*c = '\0';
			group = ++c;
		}
		if ((pwd = getpwnam(user)) == NULL) {
			syslog(LOG_ERR, "No such user %s", user);
			exit(1);
		}
		if (group != NULL && (grp = getgrnam(group)) == NULL) {
			syslog(LOG_ERR, "No such group %s", group);
			exit(1);
		}
		if (pidfile != NULL && (chown(pidfile, pwd->pw_uid,
		    group != NULL ? grp->gr_gid : getgid()) != 0))
			syslog(LOG_WARNING, "Can't chown pidfile: %m");

		if (group != NULL && setgid(grp->gr_gid) < 0) {
			syslog(LOG_ERR, "Can't set gid %d: %m", grp->gr_gid);
			exit(1);
		}
		if (setuid(pwd->pw_uid) < 0) {
			syslog(LOG_ERR, "Can't set uid %d: %m", pwd->pw_uid);
			exit(1);
		}
	}

	/* Open log file. */
	if (o_template != NULL)
		opennewfile(o_template);
	else {
		if (o_daemon) {
			syslog(LOG_ERR, "Daemon mode and no valid filename specified - exiting.");
			exit(1);
		}
		logfile = stdout;
		print_gpx_header();
	}

	/* Catch and queue all interesting signals. */
	if (pipe(signal_fd) < 0) {
		syslog(LOG_ERR, "pipe() failed: %m");
		exit(1);
	}
	if (fcntl(signal_fd[0], F_SETFL, O_NONBLOCK) < 0) {
		syslog(LOG_ERR, "pipe() failed: %m");
		exit(1);
	}
	sa.sa_flags = SA_RESTART;
	sigfillset(&sa.sa_mask);
	sa.sa_handler = enqueue_signal;
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGQUIT, &sa, NULL);
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGHUP, &sa, NULL);

reopen:
	if (gps_open(server, port, &gpsdata) != 0) {
		syslog(LOG_ERR, "failed to connect to %s:%s: %d, %s",
		    server, port, errno, gps_errstr(errno));
		sleep(3);
		process_signal();
		goto reopen;
	} else
		syslog(LOG_NOTICE, "connected to gpsd at %s:%s",
		    server, port);

	/* Initializes gpsdata structure. */
	gpsdata.status = STATUS_NO_FIX;
	gpsdata.satellites_used = 0;
	gpsdata.dop.hdop = NAN;
	gpsdata.dop.vdop = NAN;
	gpsdata.dop.pdop = NAN;
	gps_stream(&gpsdata, WATCH_ENABLE | (device != NULL ? WATCH_DEVICE : 0),
	    device);

	FD_ZERO(&fds0);
	fdmax = 0;
	FD_SET(gpsdata.gps_fd, &fds0);
	fdmax = gpsdata.gps_fd > fdmax ? gpsdata.gps_fd : fdmax;
	FD_SET(signal_fd[0], &fds0);
	fdmax = signal_fd[0] > fdmax ? signal_fd[0] : fdmax;
	fdmax++;

	tv.tv_usec = 250000;
	tv.tv_sec = 0;
	for (;;) {
		fd_set fds;
		int n;

		memcpy(&fds, &fds0, sizeof(fds));
		n = select(fdmax, &fds, NULL, NULL, &tv);
		if (n == -1) {
			if (errno == EINTR)
				continue;
			syslog(LOG_ERR, "select(2): %m, exiting");
			break;
		}
		if (FD_ISSET(signal_fd[0], &fds))
			process_signal();
		if (FD_ISSET(gpsdata.gps_fd, &fds)) {
			n = gps_read(&gpsdata, NULL, 0);
			if (n < 0) {
				syslog(LOG_ERR, "gps_read(): %m, reopening");
				gps_close(&gpsdata);
				if (o_template != NULL) {
					print_gpx_footer();
					opennewfile(o_template);
				}
				goto reopen;
			}
			process(&gpsdata);
		}
	}
	gps_close(&gpsdata);

	return (0);
}
