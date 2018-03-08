#! /bin/sh
###
### 生成交换分区文件
### 作者：leon
###

NUM=$2
ACTION=$1
SWAP_FILE_NAME="/swapfile"

if [ -f ${SWAP_FILE_NAME} ]; then
	rm ${SWAP_FILE_NAME}
fi

dd if=/dev/zero of=${SWAP_FILE_NAME} bs=64M count=$1
mkswap ${SWAP_FILE_NAME}


if [ "on" == "$1" ]; then
	swapon ${SWAP_FILE_NAME}
elif [ "off" == "$1" ]; then
	swapoff ${SWAP_FILE_NAME}
elif [ "rm" == "$1" ]; then
	swapoff ${SWAP_FILE_NAME}
	rm      ${SWAP_FILE_NAME}
else
	if [[ $1 =~ ^[0-9]+$ ]] then
      	temp = 64 * $1
		echo "create ${temp}M \"${SWAP_FILE_NAME}\""
		dd if=/dev/zero of=${SWAP_FILE_NAME} bs=64M count=$1
		mkswap ${SWAP_FILE_NAME}
	elif [[ $var =~ ^[A-Za-z]+$ ]] then
		echo "[ERROR]invoid parameter!"
	else
		echo "mixed number and string or others "
    fi  
fi


