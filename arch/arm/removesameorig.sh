#!/bin/sh
FILES=`find . -name '*.orig'`
for f in $FILES; do
  r=`echo $f | sed s/\.orig//`
  cmp -s $r $f && { echo "$f is indentical to original, removing"; rm -f $f; }
done
