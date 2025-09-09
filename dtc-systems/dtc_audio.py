"""
   DTC audio player
"""
from subprocess import call
from osgar.node import Node


class DTCAudio(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('playing')

    def on_trigger(self, data):
        self.publish('playing', True)
        print(self.time, 'Playing ...')
        call('ffplay -nodisp sounds/can_you_hear_me.mp3 -autoexit -loglevel error'.split())
        print(self.time, '... finished.')
        self.publish('playing', False)


def self_test():
    from unittest.mock import MagicMock
    bus = MagicMock()
    audio_player = DTCAudio(bus=bus, config={})
    audio_player.on_trigger(None)


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='DTC Audio - test')
    args = parser.parse_args()
    self_test()

# vim: expandtab sw=4 ts=4
