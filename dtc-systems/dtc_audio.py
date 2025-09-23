"""
   DTC audio player
"""
from subprocess import call
from osgar.node import Node


class DTCAudio(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('playing')

    def on_play_sound(self, data):
        self.publish('playing', [data, True])
        print(self.time, f'Playing ... {data}')
        filename = f'sounds/{data}.mp3'
        self._play(filename)
        print(self.time, '... finished.')
        self.publish('playing', [data, False])

    def _play(self, filename):
        call(f'ffplay -nodisp {filename} -autoexit -loglevel error'.split())

#----------------------------------------------

def self_test():
    from unittest.mock import MagicMock
    bus = MagicMock()
    audio_player = DTCAudio(bus=bus, config={})
    audio_player.on_play_sound('can_you_hear_me')


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='DTC Audio - test')
    args = parser.parse_args()
    self_test()

# vim: expandtab sw=4 ts=4
