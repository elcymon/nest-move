import sys
import pyaudio
import numpy as np
import time
import wave
from threading import Thread

class Audio_gen:
    def __init__(self,filename):
        self.p = pyaudio.PyAudio()
        self.filename = filename
    # Initialize audio stream parameters
        self.f = wave.open(filename,'rb')
        self.samples = self.f.readframes(self.f.getnframes())
        self.format = self.p.get_format_from_width(self.f.getsampwidth())
        self.channels = self.f.getnchannels()
        self.rate = self.f.getframerate()
        self.output = True
        self.speak = True
        self.stream = self.p.open(format=self.format,
                    channels=self.channels,
                    rate=self.rate,
                    output=self.output)
        
    def send_to_speaker(self):
        while self.speak:    
            # play. May repeat with different volume values (if done interactively)
            self.stream.write(self.samples)
        
        #close stream
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()

    def close_speaker(self):
        #stop speaker
        self.speak = False
        

if __name__ == '__main__':
    exp_dur = float(sys.argv[1])
    print exp_dur
    audio_gen = Audio_gen('White-noise-sound-20sec-mono-44100Hz.wav')
    thread = Thread(target = audio_gen.send_to_speaker)
    thread.start()
    audio_gen.close_speaker()
