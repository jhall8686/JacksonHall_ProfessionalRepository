#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import random
from pydub import AudioSegment
from pydub.generators import Sine
import pyaudio
import wave
import io

volume_decrease = 30

# Helper function to create a layered tone
def create_layered_tone(base_freq, duration, volume=0, layers=3):
    tone = AudioSegment.silent(duration=duration)
    for _ in range(layers):
        freq = base_freq + random.randint(-50, 50)
        layer = Sine(freq).to_audio_segment(duration=duration) + volume
        tone = tone.overlay(layer)
    return tone

# Function to decrease the volume of the entire melody
def decrease_volume(melody, decrease_by_db):
    return melody - decrease_by_db

# Define functions to create melodies for each emotion
def create_happy_melody():
    possible_notes = [880, 988, 1046, 1174, 1318]
    melody = AudioSegment.silent(duration=0)
    num_notes = random.randint(4, 6)
    for _ in range(num_notes):
        note = random.choice(possible_notes)
        duration = random.randint(150, 300)
        volume = random.randint(5, 15)
        layers = random.randint(3, 5)
        tone = create_layered_tone(note, duration, volume=volume, layers=layers)
        tone = tone.fade_in(20).fade_out(20)
        melody += tone
    melody = decrease_volume(melody, volume_decrease)
    return melody

def create_sad_melody():
    possible_notes = [220, 196, 174, 147, 131]
    melody = AudioSegment.silent(duration=0)
    num_notes = random.randint(4, 6)
    for _ in range(num_notes):
        note = random.choice(possible_notes)
        duration = random.randint(300, 500)
        volume = random.randint(-15, -5)
        layers = random.randint(3, 5)
        tone = create_layered_tone(note, duration, volume=volume, layers=layers)
        tone = tone.fade_in(50).fade_out(50)
        melody += tone
    melody = decrease_volume(melody, 0)
    return melody

def create_excited_melody():
    possible_notes = [1046, 1174, 1318, 1396, 1568]
    melody = AudioSegment.silent(duration=0)
    num_notes = random.randint(6, 8)
    for _ in range(num_notes):
        note = random.choice(possible_notes)
        duration = random.randint(100, 200)
        volume = random.randint(10, 20)
        layers = random.randint(4, 6)
        tone = create_layered_tone(note, duration, volume=volume, layers=layers)
        tone = tone.fade_in(10).fade_out(10)
        melody += tone
    melody = decrease_volume(melody, volume_decrease)
    return melody

def create_angry_melody():
    possible_notes = [220, 261, 293, 349, 392]
    melody = AudioSegment.silent(duration=0)
    num_notes = random.randint(4, 6)
    for _ in range(num_notes):
        note = random.choice(possible_notes)
        duration = random.randint(150, 300)
        volume = random.randint(-20, -10)
        layers = random.randint(3, 5)
        tone = create_layered_tone(note, duration, volume=volume, layers=layers)
        tone = tone.fade_in(10).fade_out(10)
        melody += tone
    melody = decrease_volume(melody, volume_decrease - 5)
    return melody

def create_scared_melody():
    possible_notes = [1046, 880, 1174, 988]
    melody = AudioSegment.silent(duration=0)
    num_notes = random.randint(5, 7)
    for _ in range(num_notes):
        note = random.choice(possible_notes)
        duration = random.randint(200, 400)
        volume = random.randint(0, 10)
        layers = random.randint(4, 6)
        tone = create_layered_tone(note, duration, volume=volume, layers=layers)
        tone = tone.fade_in(30).fade_out(30)
        melody += tone
    melody = decrease_volume(melody, volume_decrease)
    return melody

def create_curious_melody():
    possible_notes = [659, 698, 784, 880]
    melody = AudioSegment.silent(duration=0)
    num_notes = random.randint(5, 7)
    for _ in range(num_notes):
        note = random.choice(possible_notes)
        duration = random.randint(250, 400)
        volume = random.randint(5, 15)
        layers = random.randint(3, 5)
        tone = create_layered_tone(note, duration, volume=volume, layers=layers)
        tone = tone.fade_in(20).fade_out(20)
        melody += tone
    melody = decrease_volume(melody, volume_decrease)
    return melody

# Function to play a sound using PyAudio
def play_sound(audio_segment):
    raw_data = audio_segment.raw_data
    sample_width = audio_segment.sample_width
    frame_rate = audio_segment.frame_rate
    channels = audio_segment.channels

    p = pyaudio.PyAudio()

    stream = p.open(format=p.get_format_from_width(sample_width),
                    channels=channels,
                    rate=frame_rate,
                    output=True)

    stream.write(raw_data)

    stream.stop_stream()
    stream.close()

    p.terminate()

# ROS callback function
def emotion_callback(msg):
    emotion = msg.data.lower()

    if emotion == "happy":
        play_sound(create_happy_melody())
    elif emotion == "sad":
        play_sound(create_sad_melody())
    elif emotion == "excited":
        play_sound(create_excited_melody())
    elif emotion == "angry":
        play_sound(create_angry_melody())
    elif emotion == "scared":
        play_sound(create_scared_melody())
    elif emotion == "curious":
        play_sound(create_curious_melody())
    else:
        rospy.logwarn(f"Emotion '{emotion}' not recognized")

# ROS node initialization
def emotion_sound_player():
    rospy.init_node('emotion_sound_player', anonymous=True)
    rospy.Subscriber('/audio/emotion', String, emotion_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        emotion_sound_player()
    except rospy.ROSInterruptException:
        pass
