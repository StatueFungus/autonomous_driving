# Modul Lane Detection
Enthält eine in Python geschriebene Bibliothek zur Erkennung von Straßenmarkierungen auf einem Bild.

## Dokumentation 
> TODO

## Installationsanleitung
Damit die Bibliothek genutzt werden kann, muss [OpenCV](http://opencv.org/) und [Python](https://www.python.org/) (Version >= 2.7) auf dem System installiert sein.
Eine detaillierte Installationsanleitung für OpenCV und Python für Ubuntu wird auf folgender Seite beschrieben:
http://www.pyimagesearch.com/2015/06/22/install-opencv-3-0-and-python-2-7-on-ubuntu/

Zur Installation der Bibliothek muss lediglich die Datei __setup.py__ wie folgt auf der Kommandozeile ausgeführt werden:

* Paketdatei erstellen
```
python setup.py sdist
```
* Paket in die lokale Python-Umgebung installieren
```
python setup.py install
```

## Beispiele zur Nutzung
Die Bibliothek kann einfach genutz werden, durch den Import in ein Python Skript.
```python
import detectionlib
# oder direkt die einzelnen Klassen
from detectionlib import Visualizer
from detectionlib import LineFilter
from detectionlib import LaneDetector
from detectionlib import ImagePreparator
```
