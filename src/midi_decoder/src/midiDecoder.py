#!/bin/env python3
import rospy
from std_msgs.msg import Int32
import sys, mido, time
from mido import MidiFile

class MidiDecoder(object):
    def __init__(self):
        rospy.init_node('midi_decoder', anonymous='True')
        self.midi_pub = rospy.Publisher('/midi/stream',Int32,queue_size=10)
        self.midi_msg = Int32()
        self.midi_msg.data = -1
        
        if len(sys.argv) == 1:
            print('Pass a midi file')
            return
        else:
            midi_file = sys.argv[1]
            with open(midi_file, 'rb') as mfile:
                leader = mfile.read(4)
                if leader != b'MThd':
                    raise ValueError('Not a MIDI file!')
        
        mid_obj = MidiFile(midi_file, clip=True)
        for msg in mid_obj:
            if msg.is_meta:
                print(msg)
            else:
                print(msg)
    
    def send_message(self):
        self.midi_pub.publish(self.midi_msg)

if __name__=='__main__':
    try:
        midi_dec = MidiDecoder()
        # while not rospy.is_shutdown():
        #     midi_dec.send_message()
        #     time.sleep(1)
        scale = [60,62,64,65,67,69,71,72,-1,72,71,69,67,65,64,62,60,-1]
        for note in scale:
            midi_dec.midi_msg.data = note
            midi_dec.send_message()
            time.sleep(.1)
    finally:
        pass