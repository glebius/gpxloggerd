NAME
     gpxloggerd – GPX logging daemon

SYNOPSIS
     gpxloggerd [-dhVv] [-D level] [-f template] [-u user [:group]]
                [-p pidfile] [-m meters] [-M meters] [-a degrees]
                [-I interval] [-i timeout] [server [:port [:device]]]

DESCRIPTION
     The gpxloggerd is a tiny daemon that connects to GPSD(8) and logs
     received fix information in the GPX format.  By default the gpxloggerd
     connects to GPSD(8) at the localhost:2947 and logs to stdout GPX data for
     all GPS devices served be the daemon.

     The options are as follows:

     -h      Print usage and exit.

     -V      Print version number and exit.

     -v      Be as verbose as possible when writing GPX log.  For now this
             means logging of ⟨fix⟩, ⟨sat⟩, ⟨hdop⟩, ⟨vdop⟩ and ⟨pdop⟩ fields,
             which bloats size of produced GPX significantly.  By default this
             option is turned off, and only coordinates, time and elevation
             are logged.

     -d      Detach from terminal and become a daemon.

     -D level
             Set the underlying libgps(3) library to the given debug level.

     -f template
             Write data to the given file instead of stdout.  The template
             isn't a bare filename, but can be a format for the strftime(3).

     -u user [:group]
             Lower the daemon priveleges to the specified user and group.

     -p pidfile
             Write the daemon pid to the specified file.

     -m meters
             Suppress logging of track segment in case if it lays within the
             specified distance away from the previous logged track segment.
             This option is useful to avoid polluted GPX traces at parking
             lots and other places, where vehicle stands still for long time.
             The option is turned off by default.

     -a degrees
             Suppress logging of track segment unless the course (azimuth,
             bearing) of the vehicle has changed to a value equal or greater
             than the specified angle in degrees.  This option is useful to
             make traces on straight roads more sparse.  The option is turned
             off by default.

     -M meters
             Set maximum track segment length.  This option is meaningful only
             in conjunction with the -a option.  The default value is 200
             meters.

     -i timeout
             Start new track if the time jump between current fix and previous
             logged one is above the timeout. By default timeout value is 300
             seconds.

     -I interval
             Specify logging interval between trackpoints in seconds.  The
             default one is one second.

     When the gpxloggerd daemon receives SIGHUP it finalizes GPX in the
     current output file, closes it and then starts a new one using the
     configured template as a file name.
