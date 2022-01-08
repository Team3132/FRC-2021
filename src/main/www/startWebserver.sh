#!/bin/bash
# Run this in the top level directory of the USB flash drive.

cat <<EOF

Access logged data with one of these links in your web browser:

  http://localhost:8000/
  http://localhost:8000/Latest_chart.html
  http://localhost:8000/Latest_log.txt
  http://localhost:8000/Latest_graph.html
  http://localhost:8000/date/ - files by date
  http://localhost:8000/data/

Starting webserver. Use Control+C to exit.

EOF

# cd to the same directory as this file so that the
# webserver serves this directory, not the directory
# that the user ran this script from.
cd $(dirname $0)

python -m SimpleHTTPServer
