import sys
import pyaudio
import numpy as np
import time
import wave
from threading import Thread

class Audio_gen:
    def __init__(self,filename = None,frequency = None):
        self.p = pyaudio.PyAudio()
        self.output = True
        self.speak = True
        if frequency is None:
            self.filename = filename
        # Initialize audio stream parameters
            self.f = wave.open(filename,'rb')
            self.samples = self.f.readframes(self.f.getnframes())
            self.format = self.p.get_format_from_width(self.f.getsampwidth())
            self.channels = self.f.getnchannels()
            self.rate = self.f.getframerate()
        else:
            self.rate = 44100
            self.format = pyaudio.paFloat32
            self.channels = 1
            duration = 20
            self.samples = (np.sin(2 * np.pi * np.arange(self.rate * duration) * frequency / self.rate)).astype(np.float32)
            
            
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
    audio_gen = Audio_gen(filename = None,#'../../audio_gen/White-noise-sound-20sec-mono-44100Hz.wav',
                            frequency= 500)
    thread = Thread(target = audio_gen.send_to_speaker)
    thread.start()
    audio_gen.close_speaker()
