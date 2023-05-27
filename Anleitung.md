# Dokumentation: BlenderBrain Mischstation

Eine kurze Anleitung zur Bedienung der Mischanlage, sowie eine Checklist für alle Gasblender, die mit dem CF-Mischen vertraut sind.

## Aufbau und Funktion

Die BlenderBrain Mischstation ist eine continuous flow (CF) Mischanlage, mit der Sauerstoffmischungen bis 36%, sowie beliebige Trimixmischungen gemischt werden können. 

### Anschluss und Inbetriebnahme
Um die Mischstation in Betrieb zu nehmen müssen an die mit O2 und He markierten Ventile 6mm Pneumatikschläuche mit den entsprechenden Gasen und die Kompressoransaugung an das untere Ende des Mischrohrs angeschlossen werden. Weiters wird eine 5V Stromversorgung für den Controller benötigt, aktuell ist hierfür ein microUSB Anschluss verlötet, an den ein entsprechendes Ladekabel angeschlossen werden kann.

### Messprinzip
Die Messung basiert auf zwei in den Mischrohren angebrachten galvanischen Sauerstoffsensoren, die jeweils am Ende der Mischstrecke für ein Gas platziert sind. 
Zuerst wird Helium in das Ansaugrohr injiziert, vermischt sich im ersten Mischrohr mit der angesaugten Luft und diese Gasmischung wird dann vom ersten Sauerstoffsensor gemessen. 
Der Sensor detektiert den verminderten Sauerstoffpartialdruck und kann dadurch auf die enthaltene Heliummenge rückrechnen. 
Als Zweites wird der Sauerstoff eingebracht und vermischt sich im zweiten Mischrohr mit dem vorherigen Gasgemisch. Im zweiten Sauerstoffsensor wird der finale Sauerstoffgehalt der Mischung bestimmt.

### Gasinjektion
Im aktuellen Ausbau erfolgt die Gasinjektion von Helium und Sauerstoff manuell durch den Gasblender über Drosselrückschlagventile. 
Ein Ausbau auf eine automatische Mischanlage mit Magnetventilen ist aktuell in Planung.

***Der Gasblender muss dafür sorge tragen, dass NIEMALS im Betrieb der Sauerstoffgehalt der Mischung 40% übersteigt! Es sind keine Sicherheitsmechanismen verbaut, die eine eventuelle Explosion verhindern können.***

### Anzeige und Einstellung
Das Menü ist linear aufgebaut, die Anzeige erfolgt auf acht Siebensegmentanzeigen.

Es gibt zwei Betriebsmodi: *Einstellen* und *Mischvorgang*

#### Einstellen
Im Einstellungsmenü steht auf der linken Seite immer die Bezeichnung der aktuellen Einstellung, auf der rechten Seite steht der Wert.
Die möglichen Größen, die vom Bediener eingestellt werden können sind (in dieser Reihenfolge)

- 'P  0': der aktuelle Druck in der Flasche
- 'He 0': der aktuelle Heliumgehalt in der Flasche
- 'O2 0': der aktuelle Sauerstoffgehalt in der Flasche
- 'P  1': der gewünschte Druck in der Flasche
- 'He 1': der gewünschte Heliumgehalt in der Flasche
- 'O2 1': der gewünschte Sauerstoffgehalt in der Flasche

Die Werte können hierbei immer mit dem Encoder eingestellt werden, die Bestätigung erfolgt durch Druck auf den Encoder (*nicht* auf den Knopf daneben, siehe unten).
Im Anschluss erscheint 'Go' auf dem Display, wenn man hier Bestätigt wird der Mischvorgang gestartet.
Falls sich doch ein Fehler eingeschlichen hat, kann das Menü durch Druck auf den rechten Knopf zurückgesetzt werden, die bereits erfolgten Einstellungen bleiben dabei gespeichert und die Sensoren werden neu kalibriert.

#### Mischvorgang
Während des Mischvorgangs ändert sich die Darstellung auf acht Zahlen, diese sind in vier Blöcke gruppiert:

|O2 Soll|He Soll|O2 Ist|He Ist|

Die Aufgabe des Gasblenders ist es jetzt O2Soll und O2Ist, sowie HeSoll und HeIst so gut wie möglich zur Übereinstimmung zu bringen.
Das erfolgt durch dosiertes öffnen und schließen der Drosselrückschlagventile. 

**Mit dem Sauerstoffventil sehr vorsichtig umgehen, da sonst schnell eine explosive Mischung im Kompressor entstehen kann**

Ein Beispiel: Die eingestellte Mischung ist 21/35, es wird aber Luft angesaugt. Dann wird am Anfang 

|21|35|21|00|

angezeigt. Diese Anzeige wird sich aber schnell auf

|21|99|21|00| 

ändern, da der Controller aktiv versucht dem Bediener klarzumachen, dass mehr Helium nachgeregelt werden muss.
Der Bediener erkennt das und öffnet das Heliumventil etwas, sodass keine reine Luft mehr angesaugt wird. Die Anzeige
ändert sich (zum Beispiel) auf

|40|50|16|20|.

Jetzt muss der Bediener sowohl Helium als auch Sauerstoff nachregeln (der Controller sollte nie mehr als 40% Sauerstoff verlangen - falls doch bitte **NICHT** daran halten, sondern immer unter 40% bleiben und den Fehler an Michael Sandbichler melden)
Kontrolliertes Regeln der Ventile führt dann irgendwann zu einer Stabilisierung der Anzeige, die aber nicht unbedingt der eingestellten Mischung entsprechen muss.
Die geforderte Mischung hängt von der Mischung in der Flasche, der gewünschten Mischung und dem Verlauf des Mischvorgangs ab.
Es könnte sich zum Beispiel auf

|24|33|24|33|

einpendeln.

**Achtung: Wenn jetzt der Reset-Knopf betätigt wird, dann werden die Sensoren auf 21|00 kalibriert - was aber nicht der aktuellen Mischung im Mischrohr entspricht. 
Daher vor Reset immer das Mischrohr mit Luft spülen oder (wenn schon resettet wurde) nach der Luftspülung noch ein weiteres mal resetten.**

# Checkliste

- Kompressor ausschalten
- Speicherflaschen schließen
- Druck aus Kompressor und Rohren ablassen
- Kompressoransaugung an Mischanlage anschließen
- Mischung und Druck der Flasche, in die gemischt werden soll messen
- Flasche an die Füllarmatur anschließen
- Sauerstoff- und Heliumspeicherflasche öffnen und Anschlüsse prüfen
- BlenderBrain einschalten
- Bei 'P  0', 'O2 0' und 'He 0' den aktuellen Flaschendruck, Sauerstoff- und Heliumgehalt einstellen
- Bei 'P  1', 'O2 1' und 'He 1' den gewünschten Flaschendruck, Sauerstoff- und Heliumgehalt einstellen
- Ungefähr gleichzeitig 'Go' am BlenderBrain aktivieren und den Kompressor einschalten
- Helium- und Sauerstoffgehalt mit den Ventilen einregeln. 

***Sauerstoff darf NIE über 40% sein - EXPLOSIONSGEFAHR!***

Wenn die Mischung fertig ist:
- Ventile schließen
- Kompressor ausschalten
- Flasche von Füllarmatur abschließen
- Sauerstoff- und Heliumspeicherflasche schließen
- Kompressoransaugung wieder ans normale Ansaugrohr anschließen
- Druck aus Kompressor und Rohren ablassen ***Wichtig - die nächsten messen vermutlich nicht nach!***
- Speicherflaschen öffnen
