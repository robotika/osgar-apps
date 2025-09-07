echo -----------------------
pactl list sinks short
pactl list sources short
echo -----------------------
pactl get-default-sink
pactl get-default-source
echo -----------------------

ffplay -nodisp sounds/can_you_hear_me.mp3 -autoexit

