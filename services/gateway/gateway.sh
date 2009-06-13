#!/bin/sh
# init.d style script to start and stop a gateway
# Syntax is $0 start | stop <cas_subnet_name>

CAS_SUBNET_NAME=$2

if [ -z "${EPICS_SITE_CONFIG}" ]; then
	EPICS_SITE_CONFIG=/reg/g/pcds/package/epics/RELEASE_SITE
fi

if [ ! -e ${EPICS_SITE_CONFIG} ]; then
	echo "ERROR: cannot find ${EPICS_SITE_CONFIG}."
	exit 1
fi

if [ ! -r $(dirname ${EPICS_SITE_CONFIG})/tools/current/bin/iocenv.sh ]; then
	echo "ERROR: cannot source $(dirname ${EPICS_SITE_CONFIG})/tools/current/bin/iocenv.sh."
	exit 5
fi

# Source the environment for Linux based IOCs
source $(dirname ${EPICS_SITE_CONFIG})/tools/current/bin/iocenv.sh

# Look up the Gateway installation path
load_epics_config_var GATEWAY

# Source the environment
# This will load gateway.env, then gateway.<area>.env
# etc until gateway.<subnet>.<area>.env
LIST=""
NAME=${CAS_SUBNET_NAME}
while [ -n "${NAME}" ]; do
	LIST="${NAME} ${LIST}"
	NAME=$(echo ${NAME} | sed -e 's|[^\.]\+[\.]*||')
done

if [ -e ${GATEWAY}/etc/default.gwenv ]; then
	echo "Loading ${GATEWAY}/etc/default.gwenv"
	source ${GATEWAY}/etc/default.gwenv
fi
for net in ${LIST}; do
	if [ -e ${GATEWAY}/etc/${net}.gwenv ]; then
		echo "Loading ${GATEWAY}/etc/${net}.gwenv"
		source ${GATEWAY}/etc/${net}.gwenv
	fi
done

# Basic check of the configuration
if [ -z "${CAS_IPADDR_LIST}" ]; then
	echo "ERROR: no CA Server IP for ${CAS_SUBNET_NAME}, must exit."
	exit 5
fi
# Calculate all IPs of the computer on which the gateway is running
# We use this variable to tell the gateway to ignore any client
# relayed by another gateway running on this computer
GATEWAY_IPS=
for ip in $(/sbin/ifconfig | grep 'inet addr:' | sed -e 's|^.*inet addr:\([^[:space:]]*\).*$|\1|'); do
	GATEWAY_IPS="${GATEWAY_IPS} ${ip}"
done
GATEWAY_IPS=${GATEWAY_IPS:1}

# This gateway should listen to all clients except the ones on the subnet
# it is running for. To figure that out we use the broadcast addresses 
# (Note that won't work on hosts with Bcast set to 255.255.255.255)
if [ -z "${CAC_IPADDR}" ]; then
	CMD="/sbin/ifconfig | grep -E 'inet addr:.*Bcast:' "
	for ip in ${CAS_IPADDR_LIST}; do
		CMD="${CMD} | grep -v \"inet addr:${ip}\""
	done
	CMD="${CMD} | sed -e 's|^.*Bcast:\([^[:space:]]*\).*$|\1|'"
	CAC_IPADDR=
	for bcast in $(eval ${CMD}); do
		CAC_IPADDR="${CAC_IPADDR} ${bcast}"
	done
	CAC_IPADDR=${CAC_IPADDR:1}
fi

case $1 in
	"start")
		# Setup run-time paths
		if [ ! -d ${OUT_DIR} ]; then
			mkdir -p ${OUT_DIR}
			if [ $? -ne 0 ]; then
				echo "ERROR: could not create log directory ${OUT_DIR}."
				exit 10
			fi
		fi
		if [ ! -d ${HOME_DIR} ]; then
			mkdir -p ${HOME_DIR}
			if [ $? -ne 0 ]; then
				echo "ERROR: could not create run-time directory ${HOME_DIR}."
				exit 11
			fi
		fi
		if [ -e ${CMD_FILE} ]; then
			rm ${CMD_FILE}
			if [ $? -ne 0 ]; then
				echo "ERROR: could not remove old command file ${CMD_FILE}."
				exit 12
			fi
		fi
		# Build the command line
		PARAMS="-pvlist ${CFG_DIR}/${CAS_SUBNET_NAME}.pvlist"
		PARAMS="${PARAMS} -access ${CFG_DIR}/${CAS_SUBNET_NAME}.access"
		PARAMS="${PARAMS} -command ${CMD_FILE}"
		PARAMS="${PARAMS} -home ${HOME_DIR}"
		export EPICS_CAS_INTF_ADDR_LIST="${CAS_IPADDR_LIST}"
		export EPICS_CA_ADDR_LIST="${CAC_IPADDR}"
		export EPICS_CA_AUTO_ADDR_LIST=NO
		if [ -n "${CAS_PORT}" ]; then
			PARAMS="${PARAMS} -sport ${CAS_PORT}"
		fi
		if [ -n "${CAC_PORT}" ]; then
			PARAMS="${PARAMS} -cport ${CAC_PORT}"
		fi
		PARAMS="${PARAMS} -prefix ${PV_PREFIX}"
		if [ -n "${CONNECT_TIMEOUT}" ]; then
			PARAMS="${PARAMS} -connect_timeout ${CONNECT_TIMEOUT}"
		fi
		if [ -n "${INACTIVE_TIMEOUT}" ]; then
			PARAMS="${PARAMS} -inactive_timeout ${INACTIVE_TIMEOUT}"
		fi
		if [ -n "${DEAD_TIMEOUT}" ]; then
			PARAMS="${PARAMS} -dead_timeout ${DEAD_TIMEOUT}"
		fi
		if [ -n "${DISCONNECT_TIMEOUT}" ]; then
			PARAMS="${PARAMS} -disconnect_timeout ${DISCONNECT_TIMEOUT}"
		fi
		if [ -n "${RECONNECT_INHIBIT}" ]; then
			PARAMS="${PARAMS} -reconnect_inhibit"
		fi
		if [ -n "${DISABLE_CACHE}" ]; then
			PARAMS="${PARAMS} -no_cache"
		fi
		if [ -n "${GATEWAY_IPS}" ]; then
			export EPICS_CAS_IGNORE_ADDR_LIST="${GATEWAY_IPS}"
		fi
		PARAMS="${PARAMS} -log ${LOG_FILE}"
		PARAMS="${PARAMS} -server"
		${EPICS_EXTENSIONS}/bin/${EPICS_HOST_ARCH}/gateway ${PARAMS}
		if [ $? -ne 0 ]; then
			echo "ERROR: could not start gateway."
			exit 15
		fi
		;;
	"stop")
		if [ -e ${HOME_DIR}/gateway.killer ]; then
			source ${HOME_DIR}/gateway.killer
			sleep 1
			rm ${HOME_DIR}/gateway.killer
		else
			echo "No gateway running for ${CAS_SUBNET_NAME} on $(hostname)."
			exit 50
		fi
		if [ -e ${CMD_FILE} ]; then
			rm ${CMD_FILE}
		fi
		;;
	"status")
		if [ -e ${HOME_DIR}/gateway.killer ]; then
			PID=$(grep -E '^[[:space:]]*kill[[:space:]]+' ${HOME_DIR}/gateway.killer | sed -e 's|[[:space:]]*kill[[:space:]]\+\([0-9]\+\).*|\1|')
			if [ $(ps -p ${PID} -o state= | wc -l) -ne 0 ]; then
				echo "gateway is running with pid ${PID}"
			else
				echo "gateway not running"
				exit 3
			fi
		else
			echo "gateway not running"
			exit 3
		fi
		;;
	"log")
		if [ -e ${LOG_FILE} ]; then
			if [ ! -e ${HOME_DIR}/gateway.killer ]; then
				echo "WARNING: this looks like an old log file. Press Enter to continue."
				read
			fi
			cat ${LOG_FILE}
		else
			echo "gateway not running"
			exit 3
		fi
		;;
	*)
		echo "Invalid command $1."
		exit 99
esac
exit 0
