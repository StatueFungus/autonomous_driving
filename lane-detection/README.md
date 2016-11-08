# Lane Detection Library
Enthält eine in Python geschriebene Bibliothek zur Erkennung von Straßenmarkierungen auf einem Bild.

## Dokumentation 
> TODO

## Installationsanleitung
Damit die Bibliothek genutzt werden kann, muss [OpenCV](http://opencv.org/) (Version >= 2.4) und [Python](https://www.python.org/) (Version >= 2.7) auf dem System installiert sein.
Für eine Installationsanleitung für OpenCV siehe Wiki Eintrag https://gitlab.com/SGimbel/MPSE-WS1617_B/wikis/opencv%20installation.
Zusätzlich muss _imutils_ für Python installiert werden:
```
sudo pip install imutils
```

Zur Installation der *detectionlib* Bibliothek muss lediglich die Datei __setup.py__ wie folgt auf der Kommandozeile ausgeführt werden:

* Paketdatei erstellen
```
python setup.py sdist
```
* Paket in die lokale Python-Umgebung installieren
```
sudo python setup.py install
```

Nach erfolgreicher Installation lässt sich die Bibliothek einfach in ein Python Skript importieren.
```python
import detectionlib
# Import der einzelnen Klassen
from detectionlib import Visualizer
from detectionlib import LineFilter
from detectionlib import LaneDetector
from detectionlib import ImagePreparator
```
## Beispiel zur Nutzung
Das Skript __test.py__ zeigt exemplarisch, wie die Bibliothek benutzt werden kann. Dazu wird das unter dem Ordner __data__ zu findende Video eingelesen
und die zu sehenden Straßenmarkierungen im Video farblich markiert.
Das Testskript kann einfach mit folgendem Kommandozeilenbefehl gestartet werden:
```
python test.py
```