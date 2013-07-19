#!/bin/sh

set -e

ver="$1"

sed -i s/"\(VERSION\[\] = \"\)\(.*\)\(\";\)$"/"\1$ver\3"/ plhwtools.c
git add plhwtools.c

exit 0
