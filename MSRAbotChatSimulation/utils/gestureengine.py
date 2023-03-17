from azure.core.credentials import AzureKeyCredential
from azure.ai.language.conversations import ConversationAnalysisClient
import numpy as np
import re
import unicodedata
from transformers import pipeline
import json
import os

class luis: # You need to train your model and deploy it to Azure Language Understanding
    def __init__(self):
        with open('./secrets.json') as f:
            credential = json.load(f)
            credential = credential["intent_recognizer"]
        self.clu_endpoint = credential["AZURE_CONVERSATIONS_ENDPOINT"]
        self.clu_key = credential["AZURE_CONVERSATIONS_KEY"]
        self.project_name = credential["AZURE_CONVERSATIONS_WORKFLOW_PROJECT_NAME"]
        self.deployment_name = credential["AZURE_CONVERSATIONS_WORKFLOW_DEPLOYMENT_NAME"]
    def analyze_input(self, query):
        self.client = ConversationAnalysisClient(self.clu_endpoint, AzureKeyCredential(self.clu_key))
        with self.client:
            result = self.client.analyze_conversation(
                task={
                    "kind": "Conversation",
                    "analysisInput": {
                        "conversationItem": {
                            "participantId": "1",
                            "id": "1",
                            "modality": "text",
                            "language": "en",
                            "text": query
                        },
                        "isLoggingEnabled": False
                    },
                    "parameters": {
                        "projectName": self.project_name,
                        "deploymentName": self.deployment_name,
                        "verbose": True
                    }
                }
            )

        return result['result']['prediction']['topIntent']
    
class bert:
    def __init__(self):
        self.nlp = pipeline('feature-extraction', model="distilroberta-base", tokenizer="distilroberta-base")
        self.database = self.load_database()
    def cosdis(self, feature_1, feature_2):
        return np.dot(feature_1, feature_2)/(np.linalg.norm(feature_1)*np.linalg.norm(feature_2))
    def sentence2vec(self, sentence):
        vec = np.array(self.nlp(self.normalizeString(sentence))[0][0])
        return vec
    def unicodeToAscii(self, s):
        return ''.join(
            c for c in unicodedata.normalize('NFD', s)
            if unicodedata.category(c) != 'Mn'
        )
    def load_database(self):
        if os.path.exists('data/sentence-concept_database.pkl'):
            import pickle
            with open('data/sentence-concept_database.pkl', 'rb') as f:
                return pickle.load(f)
        else:
            with open('data/sentence-concept_database.csv') as f:
                lines = f.readlines()
            output = []
            for line in lines:
                text=line.split(',,')[0].replace('"','').lower()
                vec = self.sentence2vec(text)
                assert len(vec) == 768
                output.append({
                    "concept": line.split(',,')[1].strip().lower(),
                    "sentence": vec,
                })
            # save to pkl
            import pickle
            with open('data/sentence-concept_database.pkl', 'wb') as f:
                pickle.dump(output, f)
        return output
    def normalizeString(self, sent):
        s = self.unicodeToAscii(sent.lower().strip())
        s = re.sub(r"([.!?])", r" \1", s)
        s = re.sub(r"[^a-zA-Z.!?]+", r" ", s)
        s = re.sub(r"\.", r"", s)
        return s
    def analyze_input(self, input):
        vec_input = self.sentence2vec(input)
        dist = -1
        candidate = None
        for i in self.database:
            tmp_dist = self.cosdis(vec_input, i['sentence'])
            if tmp_dist > dist:
                dist = tmp_dist
                candidate = i['concept']
        return candidate


class random:
    def __init__(self):
        labans = os.listdir('data/LabanotationLibrary')
        labans = [i.split('.')[0] for i in labans]
        self.database = labans

    def analyze_input(self, input):
        # pick one random concept from self.database
        candidate = self.database[np.random.randint(0, len(self.database))]
        return candidate
