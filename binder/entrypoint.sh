#!/bin/bash

source ${PYCRAM_WS}/devel/setup.bash
roscore &
cp ${PYCRAM_WS}/src/pycram/binder/webapps.json ${PYCRAM_WS}/src/rvizweb/webapps/app.json 

xvfb-run exec "$@"
