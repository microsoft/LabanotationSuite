# ![MARR_logo.png](/docs/MARR_logo.png)[Microsoft Applied Robotics Research Library: Labanotation Suite](/README.md)

# **Gesture Authoring Tools**
## [KinectReader](KinectReader/README.md)
A compiled Windows application that connects to a Kinect sensor device and provides a user interface for capturing and storing gestures performed by human subjects. It's primary output data is human stick-figure joint positions in a .csv format, but can also capture corresponding RGB video and audio at the same time.

## [KinectCaptureEditor](KinectCaptureEditor/README.md)
A compiled Windows application that loads human joint position .csv files produced by the KinectReader or other tools, as well as optional corresponding video and audio files. It provides a timeline-based method to trim audio and video joint movement sequences into representative human gestures.

## [LabanEditor](LabanEditor/README.md)
A Python script application that loads a Kinect joint .csv file representing a human gesture, provides algorithmic options for automatically extracting keyframes from the gesture that correspond Labanotation data, and provides a graphical user interface for selection and modification of the extracted keyframes. Additionally, it saves the resulting gesture data in a .json file format suitable for controlling robots running a gesture interpretation driver, as well as .png graphic file renderings of the charts and diagrams used in the interface.

