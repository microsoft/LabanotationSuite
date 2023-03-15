## ![logo](/docs/img/MARR_logo.png) [Microsoft Applied Robotics Research Library](https://microsoft.github.io/AppliedRoboticsResearchLibrary/)
### Open Source Samples for Service Robotics
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)  
# [Labanotation Suite](/README.md)

# **MSRAbot Chatting Software**
![MSRAbot](images/MSRAbot.png)

## Description
MSRAbot Chatting simulation software is based on [MSRAbot simulation software](../MSRAbotSimulation/) and provides MSRAbot as a conversational interface to the chat engine. Gestures are selected depending on the MSRAbot's speech context, providing a co-speech gesture that is adjusted based on the length of the utterance.

## Tested System Software
- Microsoft Windows 10, 64-bit
- Microsoft Edge Web Browser *(https://www.microsoft.com/en-us/edge/)*
- Python 3.7.6


## Before you start
* Download the latest browser drivers.
https://developer.microsoft.com/en-us/microsoft-edge/tools/webdriver/
Locate the driver in the [client_handler](./client_handler) folder.
Directory structure should look like this:
```bash
LfO_interface
├───client_handler
│   │───webclient.py
│   │───edgedriver_win64
│   │   │───msedgedriver.exe
```
You can use any browser, but you need to modify the [webclient.py](./client_handler/webclient.py) to use the browser of your choice. You may need to enable long path if you are using Windows 10 or Windows 11.
[Reference](https://www.thewindowsclub.com/how-to-enable-or-disable-win32-long-paths-in-windows-11-10)

## Installation

1. Copy the entire **MSRAbotChatSimulation** folder into a convenient folder on your local computer.
2. Install the required python packages by running the following command in a terminal session with access to your python installation:
```bash
> pip install -r requirements.txt
```
**Note:** If you are already running or have access to an HTTP server, you can copy this folder into that server's file tree.

## Starting Up 

1. Open a command prompt or terminal session with access to your python installation
2. Navigate to the MSRAbotChatSimulation folder containing the file index.html
3. Run the following command 
**Note:** We assume that you are running python version 3 or higher, run this command:
```
> python -m uvicorn server:app --host localhost --port 9100
```
4. Open another terminal, navigate to **client_handler** folder and run this command:
```
> python webclient.py
```

**Note:** webclient.py handles speech recognition when you use microphone as the interface. If you are using text interface, you do not need to run webclient.py and just open the URL http://localhost:9100
## Viewing Gestures
Animated gestures are viewed in the browser by selecting and loading JSON files. View angle, panning, and zoom are available by clicking and holding mouse buttons and scroll wheels within the scene. Animation speed can be adjusted with a slider bar in the upper-right control panel.

### staticElbow
The fixed elbow joint of the MSRAbot physical robot can be modeled or ignored by checking the staticElbow box. This enforces a movement constraint to the model that matches a physical MSRAbot device. A humanoid stick figure model does not have this constraint.

**Note:** If the following showHelpers checkbox is also set, the unconstrained humanoid skeletal model may follow different paths than the MSRAbot model.

### showHelpers

The skeletal components of a humanoid stick figure model can be viewed by checking the showHelpers box and adjusting the opacity of the MSRAbot model.