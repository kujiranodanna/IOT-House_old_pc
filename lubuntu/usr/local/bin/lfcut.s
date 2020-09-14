#!/bin/bash
# The MIT License
# Copyright (c) 2020-2027 Isamu.Yamauchi
# This scripts startup.s ^M cutter.
# Copyright 2000.11.8 IZAMU.KARERA
# Usage    startup.s before_txt after_txt
awk '{gsub(/\015$/,"")}{print $0}' $1 > $2
