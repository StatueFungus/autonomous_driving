# Installation


## Vorbereitungen

Es wird eine ROS Installation mit der Distribution Indigo benötigt.
Eine detaillierte Installationsanleitung für ROS findet sich unter wiki.ros.org
Zum Installieren werden folgende Abhängigkeiten benötigt: **OpenCV (>= 2.4)**, **mavros**, **mavros_msgs**, **imutils**

### OpenCV
Für eine Installationsanleitung von OpenCV siehe [Wiki Eintrag](https://gitlab.com/SGimbel/MPSE-WS1617_B/wikis/opencv%20installation).

### Mavros
```shell
sudo apt-get install ros-indigo-mavros ros-indigo-mavros-msgs
```

### imutils 
imutils ist eine python Bibliothek, welche einfach mit pip installiert werden kann.

```shell
sudo pip install imutils
```

Sollte pip nicht auf dem System installiert sein, kann es einfach mit dem Befehl `sudo apt-get install python-pip` installiert werden. 


```shell
source /opt/ros/indigo/setup.bash
```

## Projekt bauen

Als nächstes muss ein catkin_workspace an beliebiger Stelle im System angelegt werden: 

```shell
mkdir <workspace_folder>
cd <workspace_folder>
mkdir src 

catkin_init_workspace
```

Der Workspace muss jetzt in den Umgebungsvariablen bekannt gemacht werden

```shell
source devel/setup.bash
```

Anschließend wird das Repository in den Workspace geklont und der Workspace gebaut:

```shell
cd src
git clone git@gitlab.com:SGimbel/MPSE-WS1617_B.git autonomous_driving

cd ../
catkin_make
```


