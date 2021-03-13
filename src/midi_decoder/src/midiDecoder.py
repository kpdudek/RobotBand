import sys, mido
from mido import MidiFile

def main():
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
    print(f'Midi Object:\n{mid_obj}')

    idx = 0
    for msg in mid_obj:
        if msg.is_meta:
            print(msg)
            # print(str(msg))
            pass
        else:
            print(msg)
            pass
        
        idx += 1
    # print(idx)


if __name__=='__main__':
    try:
        main()
    finally:
        pass