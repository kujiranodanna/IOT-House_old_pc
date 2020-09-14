#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi , update 2020.3.27
# Utilities for voice request delete
VOICE_REQ=/www/remote-hand/tmp/.voice_req.tmp
VOICE_REQ_CMD=/www/remote-hand/tmp/voice_req.pepocmd
CONV=./conv_get.cgi
. $CONV
cat >>$VOICE_REQ_CMD<END
#!/bin/bash
rm $VOICE_REQ
END
