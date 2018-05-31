#!/bin/sh
NAME=$1
ID=`ps -ef | grep "$NAME" | grep -v "grep" | awk '{print $2}'`
for id in $ID
do
kill -2 $id
done
