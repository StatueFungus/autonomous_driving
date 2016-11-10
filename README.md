# Installation

Zum Installieren werden folgende Abhängigkeiten benötigt: **OpenCV (>= 2.4)**, **mavros**, **mavros_msgs**, **imutils**

### OpenCV
Für eine Installationsanleitung von OpenCV siehe [Wiki Eintrag](https://gitlab.com/SGimbel/MPSE-WS1617_B/wikis/opencv%20installation).

### Mavros
```
sudo apt-get install ros-indigo-mavros ros-indigo-mavros-msgs
```

### imutils 
imutils ist eine python Bibliothek, welche einfach mit pip installiert werden kann.

```shell
sudo pip install imutils
```

Sollte pip nicht auf dem System installiert sein, kann es einfach mit dem Befehl `sudo apt-get install python-pip` installiert werden. 


## Projekt bauen

Als nächstes muss ein catkin_workspace an beliebiger Stelle im System angelegt werden: 

```shell
mkdir <workspace_folder>
cd <workspace_folder>
mkdir src 
cd src

catkin_init_workspace
```

Anschließend wird das Repository in den Workspace geklont und der Workspace gebaut:

```
git clone git@gitlab.com:SGimbel/MPSE-WS1617_B.git autonomous_driving

cd ../
catkin_make
```

TODO: source Befehl ausführen und in user profil aufnehmen


