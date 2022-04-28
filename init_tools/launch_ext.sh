#!/bin/sh

ps -ef | grep launch_daemon | grep -v grep | awk '{print $2}' | xargs kill -9
ps -ef | grep ros | grep -v grep | awk '{print $2}' | xargs kill -9

exit 0
