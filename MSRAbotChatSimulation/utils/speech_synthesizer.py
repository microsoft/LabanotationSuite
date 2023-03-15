import azure.cognitiveservices.speech as speechsdk
                    
class speech_synthesizer_file:
    def __init__(self,credential, speech_synthesis_voice_name="en-US-TonyNeural"):
        self.speech_key = credential["AZURE_SPEECH_KEY"]
        self.service_region = credential["AZURE_SPEECH_REGION"] 
        self.speech_config = speechsdk.SpeechConfig(subscription=self.speech_key, region=self.service_region)
        self.speech_config.speech_synthesis_voice_name=speech_synthesis_voice_name
        self.speech_config.set_speech_synthesis_output_format(speechsdk.SpeechSynthesisOutputFormat.Audio16Khz32KBitRateMonoMp3)
        self.filename = 'static/audio/tmpaudio.mp3'
        self.audio_config = speechsdk.audio.AudioOutputConfig(filename=self.filename)
        self.speech_synthesizer = speechsdk.SpeechSynthesizer(speech_config=self.speech_config, audio_config=self.audio_config)
        
    def synthesize_speech(self, text):
        speech_synthesis_result = self.speech_synthesizer.speak_text_async(text).get()
        if speech_synthesis_result.reason == speechsdk.ResultReason.SynthesizingAudioCompleted:
            pass
        if speech_synthesis_result.reason == speechsdk.ResultReason.SynthesizingAudioCompleted:
            pass
        elif speech_synthesis_result.reason == speechsdk.ResultReason.Canceled:
            cancellation_details = speech_synthesis_result.cancellation_details
            print("Speech synthesis canceled: {}".format(cancellation_details.reason))
            if cancellation_details.reason == speechsdk.CancellationReason.Error:
                print("Error details: {}".format(cancellation_details.error_details))
            