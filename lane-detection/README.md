# Modul Lane Detection
Enthält eine in Python geschriebene Bibliothek zur Erkennung von Straßenmarkierungen auf einem Bild.

## Dokumentation 
> TODO

## Installationsanleitung
Zur Installation der Bibliothek muss die Datei _setup.py_ wie folgt auf der Kommandozeile ausgeführt werden:

1. Paketdatei erstellen

```{r, engine='bash', count_lines}
python setup.py sdist
```
2. Paket in die lokale Python-Umgebung installieren

```{r, engine='bash', count_lines}
python setup.py install
```

Damit die Bibliothek genutzt werden kann, muss zusätzlich OpenCV auf dem System installiert sein.
Eine detaillierte Installationsanleitung für OpenCV für Ubuntu findet sich auf folgender Seite:
http://www.pyimagesearch.com/2015/06/22/install-opencv-3-0-and-python-2-7-on-ubuntu/

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
