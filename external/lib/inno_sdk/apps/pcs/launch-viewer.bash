#! /usr/bin/env bash
set -e

DEFAULT_PORT=10000
if [[ $# -lt 1 || $# -gt 3 ]]; then
    echo "Usage: " $0 " <IP-ADDRESS> [PORT] [/path/to/webGL/viewer]"
    exit
fi

IP=$1
echo "IP is: " ${IP}

if [[ $# -gt 1 ]]; then
   PORT=$2
else
    PORT=${DEFAULT_PORT}
fi
echo "PORT is: " ${PORT}

if [[ $# -eq 3 ]]; then
   # ?url=localhost&port=8001/stream
   URL="file://$3/index.html"
else
   URL="http://viewer.innovusion.com/stable/"
fi
echo "URL is: " ${URL}
URL_ARGS="?url=${IP}&port=${PORT}/stream&autoplay=0.0"

browser=$(which google-chrome || which firefox || which xdg-open || which gnome-open)
FURL=\"$URL$URL_ARGS\"
if [[ $browser == *"chrome"* ]]; then
    browser="$browser --allow-file-access-from-files --new-window $FURL"
elif [[ $browser == *"firefox"* ]]; then
    browser="$browser -new-window $FURL"
else
    browser="$browser $FURL"
fi
echo "BROWSER is: " ${browser}


# sample url
# http://viewer.innovusion.com/stable/?url=172.16.88.142&port=10000%2Fstream%2F%3Fname1%3Dvalue1%26name2%3Dvalue2
CMD="nohup $browser &> /dev/null &"

echo "CMD is: " ${CMD}
bash -c "${CMD}"
