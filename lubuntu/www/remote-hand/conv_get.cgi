#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , 2013.2.24 update 2018.10.14
# utilities for QUERY_STRING
QUERY_CMD=/www/remote-hand/tmp/.QUERY_STRING.cmd
#echo "$QUERY_STRING" >/www/remote-hand/tmp/QUERY_STRING
echo "$QUERY_STRING" | awk '
  BEGIN{FS="&"}
  {
    for ( ct =1 ; ct <= NF ; ct++ ) {
      split($(ct),ary,"=")
      l=length(ary[1])
      i=match(ary[1],/_[0-9]+$/)
      j=k=ary[1]
      gsub(/_[0-9]+$/,"",j)
      m=substr(k,i+1,l)
#      print j"["m"]="ary[2]
      printf("%s[%d]=\"%s\"\n",j,m,ary[2])
    }
  }' | nkf -w --url-input >$QUERY_CMD
chmod +x $QUERY_CMD
. $QUERY_CMD
#rm $QUERY_CMD
