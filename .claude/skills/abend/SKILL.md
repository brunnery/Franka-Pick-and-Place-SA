---
name: abend
description: Abend-Sequenz – Code aufräumen, Tests, Doku nachführen, Stand der Dinge in docs/STATUS.md schreiben, committen und pushen. Aufrufen mit /abend am Ende eines Arbeitstags.
---

# Abend-Sequenz

Ziel: Der Tag ist sauber abgeschlossen, alles liegt auf GitHub, und `docs/STATUS.md` sagt am nächsten Morgen in 2 Minuten, wo wir stehen und womit es weitergeht. Schritte in dieser Reihenfolge, jeden kurz im Chat melden.

## 1. Bestandsaufnahme
- `git status`, `git diff --stat`, `git log --oneline -5`.
- Run-Records des Tages in `data/runs/` (Datum heute) auswerten: Anzahl Läufe pro Schritt, Ergebnis (`result`), Fehlerursachen, auffällige Werte (Landung, Stud-Grösse, Kontakt). Kurze Tabelle für STATUS.md.
- Entscheidungslog des Tages in `docs/PLAN.md` lesen.

## 2. Aufräumen (nur Code dieses Repos, nie `software/src/messages_fr3/`)
- `pyflakes3 software/src/sa_tasks software/src/sa_vision software/launch` → unbenutzte Imports/Variablen entfernen.
- Unbenutzte Funktionen, Parameter und YAML-Einträge suchen (Name per `grep -rn` im Code prüfen); was nirgends mehr gelesen wird, entfernen. Nodes deklarieren Parameter ohne Default: YAML-Eintrag nur löschen, wenn ihn kein Code mehr liest.
- Kommentare/Docstrings, die nicht mehr stimmen, korrigieren. Keine magischen Zahlen im Code (CLAUDE.md).
- Mit `TEMPORARY`/`TEST` markierte Werte nicht stillschweigend ändern, sondern in STATUS.md unter „Vorläufige Werte“ auflisten.
- Nichts löschen, was der Nutzer angelegt hat (Daten, Bilder, `data/images/values/`), ohne zu fragen.

## 3. Prüfen
- Unit-Tests: `cd software/src/<pkg> && PYTHONNOUSERSITE=1 python3 -m pytest test -q` für `sa_tasks` und `sa_vision`. Fehler beheben oder im Bericht klar nennen.
- Build: `cd software && PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select sa_interfaces sa_tasks sa_vision` (cartesian_impedance_control nur, wenn geändert; braucht `source ~/franka_ros2_ws/install/setup.bash`).

## 4. Doku
- `docs/PLAN.md`: Entscheidungslog vollständig für heute (Deutsch).
- `docs/THEORY.md`: jede heute benutzte/geänderte Formel oder Methode drin, Status used/planned/dropped (Englisch).

## 5. Stand der Dinge → `docs/STATUS.md` (Deutsch, überschreiben, nicht anhängen)
Aufbau:
1. **Stand vom <Datum>** – ein Absatz: was läuft, was nicht.
2. **Heute erledigt** – Stichpunkte mit Ergebnis (Zahlen aus den Runs).
3. **Messergebnisse** – kleine Tabelle der Läufe des Tages.
4. **Vorläufige Werte** – alles, was zurückgestellt werden muss (Datei:Zeile, jetziger Wert, Zielwert, warum).
5. **Offene Probleme** – mit Vermutung zur Ursache.
6. **Morgen als Erstes** – nummerierte Aufgabenliste, konkret, mit den Befehlen/Aliasen (`sa_full_all`, `sa_place_approach`, `sa_vision_bolt`, `sa_arduino_test`, …).
7. **Nützliche Befehle** – nur die, die morgen gebraucht werden.

## 6. Commit und Push
- Logisch getrennte Commits (Code + zugehörige THEORY zusammen; Run-Daten/Bilder separat; Doku/STATUS separat). Englische Commit-Messages.
- Nicht committen: `software/src/messages_fr3/` (keine Lizenz), `data/images/values/` (gehört dem Nutzer), Build-Ordner.
- `git push origin main`. Schlägt der Push fehl, Grund melden, nicht forcieren.

## 7. Gedächtnis
- Die Memory-Datei zum Sitzungsstand (`pick-session-state.md`) auf den neuen Stand bringen, kurz, mit Verweis auf `docs/STATUS.md`.

## 8. Abschluss im Chat
Kurz: was aufgeräumt wurde, Testergebnis, Commits (Hashes), Push ok, und die ersten 3 Punkte von „Morgen als Erstes“.
