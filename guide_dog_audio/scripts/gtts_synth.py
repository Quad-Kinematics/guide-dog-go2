"""Writes text as speech to a WAV file with Google TTS (gTTS), for tts_node.

Usage: gtts_synth.py <lang> <out.wav> <text>

Needs network access, and sends the text to Google (gTTS uses Google
Translate's speech endpoint, which is not an official API). Output is a
22.05 kHz mono 16-bit WAV with a plain 44-byte header, the format verified on
the Go2. Runs under a Python that has gTTS (see tts_node's gtts_python).
"""

import os
import subprocess
import sys
import tempfile
import wave

from gtts import gTTS

RATE = 22050
TAIL_SILENCE_SEC = 0.2  # keeps the last word from being clipped


def main():
    lang, out_path, text = sys.argv[1], sys.argv[2], sys.argv[3]
    with tempfile.TemporaryDirectory() as tmp:
        mp3 = os.path.join(tmp, 'speech.mp3')
        gTTS(text=text, lang=lang).save(mp3)
        pcm = subprocess.run(
            ['ffmpeg', '-loglevel', 'error', '-i', mp3,
             '-ar', str(RATE), '-ac', '1', '-f', 's16le', '-'],
            check=True, capture_output=True).stdout
    with wave.open(out_path, 'wb') as out:
        out.setnchannels(1)
        out.setsampwidth(2)
        out.setframerate(RATE)
        out.writeframes(pcm + bytes(2 * int(RATE * TAIL_SILENCE_SEC)))


if __name__ == '__main__':
    main()
