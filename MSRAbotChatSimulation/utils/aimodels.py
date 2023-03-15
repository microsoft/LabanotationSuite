import openai
import json

with open('./secrets.json') as f:
    credentials = json.load(f)
class Davinci3:
    def __init__(self):
        openai.api_key = credentials["chatengine"]["AZURE_OPENAI_KEY"]
        openai.api_base =  credentials["chatengine"]["AZURE_OPENAI_ENDPOINT"]
        openai.api_type = 'azure'
        openai.api_version = '2022-12-01'
        self.prompt = 'You are an excellent chat bot, named MSRAbot. Please respond to the current message accurately, taking into account your knowledge and our previous conversations.'

    def generate(self, message, history = 'None'):
        deployment_name='davinci3'
        prompt_message = self.prompt + '\n Previous conversations:' + history + '\n Current message:'+ message
        response = openai.Completion.create(engine=deployment_name, 
                                            prompt=prompt_message, 
                                            temperature=0.1,
                                            max_tokens=50,
                                            top_p=1.0,
                                            frequency_penalty=0.5,
                                            presence_penalty=0.5) 
        text = response['choices'][0]['text'].replace('\n', '').replace(' .', '.').strip()
        return text
