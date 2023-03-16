from selenium import webdriver
import azure.cognitiveservices.speech as speechsdk
import json


with open('../secrets.json') as f:
    credentials = json.load(f)

def create_web_handler():
    driver = webdriver.Edge(".\\edgedriver_win64\\msedgedriver.exe")
    driver.get('http://localhost:9100')
    driver.set_window_position(291, 0)
    return driver


def speech_recognize_continuous_async_from_microphone(driver):
    """performs continuous speech recognition asynchronously with input from microphone"""
    speech_config = speechsdk.SpeechConfig(subscription=credentials["speech_recognizer"]
                                           ["AZURE_SPEECH_KEY"], region=credentials["speech_recognizer"]["AZURE_SPEECH_REGION"])
    # The default language is "en-us".
    speech_recognizer = speechsdk.SpeechRecognizer(speech_config=speech_config)

    done = False
    flag_robot_is_talking = False

    def recognizing_cb(evt: speechsdk.SpeechRecognitionEventArgs):
        # check if the robot is talking
        mic_enabled = driver.find_element("id", "mic_enabled").get_attribute('value')
        if mic_enabled == "no":  # robot is talking, so ignore the event
            nonlocal flag_robot_is_talking
            flag_robot_is_talking = True

    def recognized_cb(evt: speechsdk.SpeechRecognitionEventArgs):
        nonlocal flag_robot_is_talking
        mic_enabled = driver.find_element("id", "mic_enabled").get_attribute('value')
        if len(evt.result.text) > 0:
            if (not flag_robot_is_talking) and mic_enabled == "yes":
                print('RECOGNIZED: {}'.format(evt.result.text))
                form = driver.find_element("id", "messageText")
                form.send_keys(evt.result.text)
                button = driver.find_element("id", "sendform")
                button.submit()
            else:
                print('Speech was not passed to the robot because the robot was talking or the mic was muted')
        flag_robot_is_talking = False  # reset the flag

    def stop_cb(evt: speechsdk.SessionEventArgs):
        """callback that signals to stop continuous recognition"""
        print('CLOSING on {}'.format(evt))
        nonlocal done
        done = True

    # Connect callbacks to the events fired by the speech recognizer
    speech_recognizer.recognizing.connect(recognizing_cb)
    speech_recognizer.recognized.connect(recognized_cb)
    speech_recognizer.session_stopped.connect(stop_cb)
    speech_recognizer.canceled.connect(stop_cb)

    result_future = speech_recognizer.start_continuous_recognition_async()

    result_future.get()
    print('Continuous Recognition is now running, say something.')

    while not done:
        print('type "stop" then enter when done')
        stop = input()
        if (stop.lower() == "stop"):
            print('Stopping async recognition.')
            speech_recognizer.stop_continuous_recognition_async()
            break

    print("recognition stopped, main thread can exit now.")


driver = create_web_handler()
speech_recognize_continuous_async_from_microphone(driver)
