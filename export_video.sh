LOGFILE=$1
FILENAME="${LOGFILE##*/}"
NAME="${FILENAME%.*}"
python -m osgar.logger $LOGFILE --raw --stream oak.color > upload/$NAME.h26x
ffmpeg -i upload/$NAME.h26x -c copy upload/$NAME.mp4
rm upload/$NAME.h26x

