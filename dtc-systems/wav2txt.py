import librosa
import speech_recognition as sr


def is_coherent_speech(wav_path, threshold=0.5):
    # Load the audio file
    audio, sample_rate = librosa.load(wav_path, sr=None)
    
    # Check if the duration is reasonable for speech (e.g., at least 1 second)
    duration = librosa.get_duration(y=audio, sr=sample_rate)
    if duration < 1.0:
        return False, "<Audio is too short to be considered speech>"

    # Use speech recognition to transcribe the audio
    recognizer = sr.Recognizer()
    with sr.AudioFile(wav_path) as source:
        audio_data = recognizer.record(source)
    
    try:
        transcription = recognizer.recognize_whisper(audio_data, model="base.en")
        print(transcription)
        words = transcription.split()
        num_words = len(words)
        
        # Check if the number of words is above a certain threshold
        if True:  # (ignore for now) num_words / duration > threshold:
            return True, transcription
        else:
            return False, "<Insufficient word count for coherent speech>"
    
    except sr.UnknownValueError:
        return False, "<Speech recognition could not understand audio>"
    except sr.RequestError:
        return False, "<Could not request results from speech recognition service>"


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('wavfile', help='Input wav file')
    args = parser.parse_args()

    print(is_coherent_speech(args.wavfile))
