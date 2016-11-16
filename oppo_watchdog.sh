#!/bin/bash -x

FILE_PATH=$(echo $(cd $(dirname "$0") && pwd -P)/$(basename "$0"))
BASEDIR=$(dirname "${FILE_PATH}")
echo "BASEDIR: ${BASEDIR}"

get_pid_by_name() {
    [ -z ${1} ] && echo "ERR Param in get_pid_by_name" && exit 2
    echo $(ps -ef | grep ${1} | grep -v grep | awk '{print $2}')
}

TARGET_PROC_NAME=oppo_tcp_serial_redirect.py
PYTHON_EXE=/root/Env/sck2serial/bin/python
browser_pid=$(get_pid_by_name ${TARGET_PROC_NAME})
if [ -z "${browser_pid}" ]; then
    sudo echo `date` >> /var/watchdog.log
    ${PYTHON_EXE} ${BASEDIR}/oppo_tcp_serial_redirect.py --develop -P 9000 /dev/ttyUSB0 9600 >> /var/log/oppo/oppo_s2s.log 2>&1 &
fi
