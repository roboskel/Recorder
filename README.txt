5) recorder

Description : Multi aspect recording nodes using the Kinect. The project is comprised of 4 different nodes that can be run individually or
in any combination the user needs. These are :

    record_skel : Records the joint positions (X,Y,Z) returned by the openni_tracker and writes them in a file named
    skel.txt in the following format : Timestam....
    The recordings are about 1/10 second apart.

    record_hok : Records the readings of a UTM-30LX Hokuyo laser scanner and writes the in a file named hok.txt in the following format .... The recordings are about 1/10 apart.

    listener_s.py : Records audio from an the selected audio recording device at intervals that can be combined to a singe
    stream. The files are writen in the ./rec/audio directory which must be empty before the script is started and abide to
    the following format .wav

    rec3 : Records rgb images and depth data from the Kinect at intervals of about 1/10 second using the opencv libraries.
    The files are saved as rgb.png for the rgb images and dpt.bin for the depth data

    To run : Every node can be run individualy using the corresponding .launch file. For the record_skel and re3 nodes oppeni_launch
    must be launched successfully before hand as if it fails to launch (as it quite often does) the whole node will be rendered useless.

    record_skel:
        roslaunch openni_launch openni.launch
        roslaunch recorder skel.launch User presses START to start recording and the X button to stop it

    record_hok:
        roslaunch openni_launch openni.launch
        roslaunch recorder hok.launch
        User presses START to start recording and the X button to stop it

    listener.py
        roslaunch audio.launch User presses START to start recording and the X button to stop it

    rec3
        roslaunch openni_launch openni.launch
        roslaunch recorder rec.launch User presses START to start recording and the X button to stop it

    ALL the nodes
        roslaunch openni_launch openni.launch
        roslaunch recorder full.launch User presses START to start recording and the X button to stop it
