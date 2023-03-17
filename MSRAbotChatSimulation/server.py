import json
from utils import speech_synthesizer
from typing import List
import fastapi
from fastapi import WebSocket, Request, WebSocketDisconnect, Depends
from fastapi.responses import HTMLResponse, FileResponse
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles
from fastapi.security import HTTPBasic, HTTPBasicCredentials
from starlette.exceptions import HTTPException
from starlette.status import HTTP_401_UNAUTHORIZED
import utils.aimodels as aimodels
import utils.gestureengine as gestureengine
import numpy as np
import os
import copy
from mutagen.mp3 import MP3 as mp3

#------------------------------
# Set your credentials here
credentials_username = "username" 
credentials_password = "password"
# Choose your preferred AImodel
chatmodel = aimodels.ChatGPT # aimodels.Dacinvi3
# Choose your preferred gesture selection mode
gestureengine = gestureengine.bert() #gestureengine.luis() # gestureengine.bert() # gestureengine.random()
#------------------------------


with open('./secrets.json') as f:
    credentials = json.load(f)
laban_dir = 'data/LabanotationLibrary'
jsonfiles = os.listdir(laban_dir)


def load_laban(jsondir, jsonfile, time_speech=None):
    with open(os.path.join(jsondir, jsonfile)) as f:
        data = json.load(f)
        data = data[jsonfile.split('.')[0]]
    data_write = copy.copy(data)
    current_time = 0
    for item in data:
        try:
            duration = int(data[item]['start time'][0]) - current_time
        except BaseException:
            duration = 1000
            print(item)
        if duration < 0:
            import sys
            sys.stderr.write("Error!")

        current_time = current_time + duration
    ratio = time_speech / float(current_time)
    if time_speech is not None:
        if time_speech > current_time:
            for item in data:
                data_write[item]['start time'][0] = str(
                    int(float(data[item]['start time'][0]) * ratio))
    data_save = {}
    data_save[jsonfile.split('.')[0]] = data_write
    # save json file
    with open(os.path.join('static/laban/tmplaban.json'), 'w') as f:
        json.dump(data_save, f, indent=4)


class ConnectionManager:
    def __init__(self):
        self.connections: List[WebSocket] = []
        self.models = {}

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.connections.append(websocket)
        self.models[websocket] = chatmodel()

    def disconnect(self, websocket: WebSocket):
        self.connections.remove(websocket)
        del self.models[websocket]

    async def send_personal_message(self, message: str, websocket: WebSocket):
        await websocket.send_text(message)

    async def broadcast(self, message: str):
        for connection in self.connections:
            await connection.send_text(message)


app = fastapi.FastAPI()
security = HTTPBasic()


async def authenticate(credentials: HTTPBasicCredentials = Depends(security)):
    if credentials.username != credentials_username or credentials.password != credentials_password:
        raise HTTPException(
            status_code=HTTP_401_UNAUTHORIZED,
            detail="Invalid username or password")
    return True

manager = ConnectionManager()
app.mount("/static", StaticFiles(directory="static"), name="static")
templates = Jinja2Templates(directory="templates")

favicon_path = 'favicon.ico'
@app.get('/favicon.ico', include_in_schema=False)
async def favicon():
    return FileResponse(favicon_path)


#@app.get("/",
#    response_class=HTMLResponse,
#    dependencies=[Depends(authenticate)])
@app.get("/",
    response_class=HTMLResponse)
async def index(request: Request):
    return templates.TemplateResponse('index.html', {"request": request})


async def interface(user_input, aimodel):
    # if user_input does not ends with ., ?, or !, add period
    if user_input[-1] not in ['.', '?', '!']:
        user_input = user_input + '.'
    aimodel_message = aimodel.generate(user_input)
    return aimodel_message


async def gestureselector(agent_input):
    # any algorithm is OK, as long as it returns intent from user input.
    # intent should be the prefix of the json file (e.g., away, deictic, etc.)
    intent = gestureengine.analyze_input(agent_input)
    print('intent is:' + intent)
    jsoncandidate = []
    for jsonfile in jsonfiles:
        if jsonfile.startswith(intent):
            jsoncandidate.append(jsonfile)

    if len(jsoncandidate) == 0:
        jsoncandidate = jsonfiles
    np.random.shuffle(jsoncandidate)
    # create speech synthesizer every time as it continuously generates audio
    speech_synthesizer_azure_file = speech_synthesizer.speech_synthesizer_file(
        credentials["speech_synthesizer"], speech_synthesis_voice_name="en-US-TonyNeural")
    speech_synthesizer_azure_file.synthesize_speech(agent_input)
    del speech_synthesizer_azure_file
    mp3_length = mp3('static/audio/tmpaudio.mp3').info.length
    length_ms = int(mp3_length * 1000)
    print('gesture duration:' + str(length_ms))
    load_laban(laban_dir, jsoncandidate[0], time_speech=length_ms)


@app.websocket("/ws/{session_id}")
async def websocket_endpoint(websocket: WebSocket, session_id: str):
    await manager.connect(websocket)
    try:
        while True:
            print('waiting input...' + session_id)
            data = await websocket.receive_text()
            await manager.send_personal_message(f"User: {data}", websocket)
            agent_return = await interface(data, manager.models[websocket])
            if agent_return is not None:
                await gestureselector(agent_return)
                await manager.send_personal_message(f"MSRAbot: {agent_return}", websocket)
            else:
                pass
    except WebSocketDisconnect:
        manager.disconnect(websocket)
