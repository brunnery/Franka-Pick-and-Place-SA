# Semesterarbeit – Projektkontext & Gameplan

> Lebendes Dokument. Dient als Kontext für Claude (und mich). Bei jeder Designentscheidung hier nachführen.
> Status: **Vorbereitung ohne Workstation** (Stand 07.10.2026)

---

## 1. Ziel in einem Satz

Ein FR3 mit eye-in-hand USB-Kamera und neuem Iris-Gripper soll eine Mutter autonom finden, greifen, über einer Schraube platzieren und aufschrauben – mit direkter Zielpose aus der Kamera statt iterativer Pixel-Fehlerminimierung.

---

## 2. Ausgangslage aus der Bachelorarbeit (BA)

**BA:** *Design and Control of a Low-Cost 3D-Printed End Effector for Autonomous Nut Grasping, Placing and Fastening* (Aug 2026, ETH, PDZ).

### Was existiert / funktioniert hat
- ROS2-Stack: Vision-Node → State-Machine *pick* / *place* → Arduino-Bridge (serial) + Cartesian Impedance Controller (CIC) → FR3.
- Vision: Graustufen-Threshold (80), Gauss 5×5, Morphologie, Konturfilter (Fläche 500–50 000 px, 5–8 Ecken, Kompaktheit 0.50–0.92), EMA auf Centroid (α = 0.25). Mutter und Schraubenkopf **matt schwarz lackiert**.
- Zentrierung: P-Regler auf normierten Pixelfehler (Gain 0.003 bzw. 0.0015 m/Einheit, Threshold 0.08 / 0.03, Loop 5 / 10 Hz).
- Kontakterkennung: Fz < −4 N (Tiefpass 0.1/0.9) → Steifigkeit umschalten (3000/1000 → 250 N/m).
- Arduino: 4 DC-Motoren (2× Backen, Schraubendreher, Adapter), **alles zeitgesteuert** (z. B. Schrauben 5000 ms fix).

### Ergebnisse (7 von 8 Runs ausgewertet)
| Phase | Steady-state Fehler [mm] |
|---|---|
| Grasp | 0.16 ± 0.14 |
| Vision centering | 1.57 ± 0.57 |
| Seating | 1.78 ± 0.18 |
| Release | 0.86 ± 0.14 |
| Screwing | 1.35 ± 0.19 |

Zykluszeit Ø 62 s (erfolgreiche Runs), 1 Run Abbruch weil Zentrierung in Timeout oszillierte.
⚠️ Gemessen wurde Tracking (Ist vs. Soll-Pose), **nicht** Genauigkeit relativ zur Schraube.

### Lessons learned → direkte Anforderungen an die SA
| BA-Schwachstelle | Konsequenz für SA |
|---|---|
| Vision-Zentrierung streut am stärksten, iterativ, oszilliert teils | **Ansatz 2:** Zielpose direkt aus Intrinsics + Höhe berechnen, ein Move |
| Feste Offsets (Kamera→Schraubendreher 0.057 m), Blindfahrt | Kalibrierte Transformation Kamera→Tool (Hand-Eye) statt Handmessung |
| Motoren zeitgesteuert, verlorener Serial-Befehl blockiert Ablauf | Serial-Protokoll mit ACK/DONE + Timeouts, wo möglich Feedback |
| Steifigkeit nie getunt | Phasenabhängige Steifigkeit als bewusster Parameter |
| Future Work der BA: rotierender Gripper, Tiefenkamera, direkte Zielpose | Rotierender Gripper + direkte Zielpose = Kern der SA |
| Genauigkeit relativ zum Objekt nicht gemessen | Ground-Truth-Messung von Anfang an einplanen (siehe §9) |

---

## 3. Hardware-Setup

```
Workstation ──USB-C (serial)──► Arduino ──► Motortreiber ──► M1, M2 (Iris), M3 (Rotation)
     │                              ▲
     │                              └── eigene Stromversorgung (Steckdose)
     ├──USB-A──► USB-Kamera (eye-in-hand, am Gripper)
     └──Ethernet──► FR3 (franka_ros2, CIC)
```

### Neuer Gripper
- **M1 + M2:** treiben je ein Zahnrad, das einen **Iris-Mechanismus** (wie Kamerablende) öffnet/schliesst → variables Sechseck, mehrere Muttergrössen. Motoren laufen gegensinnig.
- **Halten:** Iris bleibt aktiv geschlossen, Mutter dauerhaft eingeklemmt (auch während Rotation).
- **M3:** dreht den ganzen Gripper → schraubt die Mutter auf.

---

## 4. Ablauf (Soll)

### Phase A – Pick (Codebase/Node 1)
1. Auf Reisehöhe über Arbeitsbereich fahren, Bild aufnehmen.
2. Mutter detektieren → Centroid (u, v) + Orientierung (Sechseck-Winkel).
3. **Zielpose direkt berechnen** (§5.1), ein Move auf Anfahrhöhe über die Mutter.
4. Optional 1 Verifikationsbild → ggf. eine Korrektur (kein Regelkreis).
5. Absenken bis Gripper-Unterseite **flush** mit Tisch (Kontakt über Fz-Schwelle, niedrige z-Steifigkeit).
6. Iris schliessen, Halten aktiv lassen. Greifen bestätigen.

### Phase B – Place & Screw (Codebase/Node 2)
1. Anheben auf Reisehöhe.
2. Schraube detektieren, Zielpose berechnen, sodass die **Mutter** (= Tool-Frame, nicht Kamera) über der Schraube steht.
3. Absenken in −z bei gleichzeitigem **Wiggle** in x/y (geringe xy-Steifigkeit) bis Mutter aufsitzt.
4. Optional: kurz rückwärts drehen (M3) um Gewindeanfang zu finden, dann vorwärts.
5. M3 schraubt, Abbruch über Zeit / Drehmoment / z-Fortschritt (§6).
6. Iris öffnen, wegfahren.

---

## 5. Methodik

### 5.1 Vision – Ansatz 2: Direkte Zielpose (Priorität)
Lochkameramodell, Tiefe Z aus bekannter Höhe:

```
X_c = (u − c_x) · Z / f_x
Y_c = (v − c_y) · Z / f_y
Z   = Abstand Kamera → Tischebene (aus FK des FR3 + Hand-Eye + Tischhöhe)
p_base = T_base_ee · T_ee_cam · [X_c, Y_c, Z, 1]ᵀ
```
- Bild vorher **entzerren** (Distortion-Koeffizienten aus Kalibrierung).
- Objekthöhe berücksichtigen (Mutteroberkante bzw. Schraubenspitze ≠ Tischebene).
- Kamera möglichst senkrecht zum Tisch → Ebenen-Schnitt bleibt einfach; sonst allgemeiner Strahl-Ebene-Schnitt (gleiche Formel, nur Rotation mitnehmen).
- Benötigt: **Intrinsics-Kalibrierung** (Schachbrett, ROS2 `camera_calibration`) und **Hand-Eye-Kalibrierung** `T_ee_cam` (z. B. `easy_handeye2` oder OpenCV `calibrateHandEye`).
- Detektion aus BA wiederverwenden (Threshold-Pipeline), aber parametrisierbar über YAML.

### 5.2 Fallback – Ansatz 1: Fehlerminimierung (BA)
Pixelfehler Bildmitte↔Centroid, P-Regler. Nur falls Ansatz 2 nicht genau genug. Mögliche Hybridlösung: Ansatz 2 für Grobpositionierung + 1–2 Iterationen Ansatz 1 zur Feinkorrektur.

### 5.3 Regelung (CIC)
- Bestehenden Cartesian Impedance Controller weiterverwenden.
- Steifigkeit **pro Phase** setzen (Free motion / Kontakt / Wiggle / Screwing), als Parameter in YAML statt hart codiert.
- Wiggle: kleine Kreis-/Spiralbahn der Soll-Pose in xy (Amplitude ~0.5–1 mm, wenige Hz) + konstanter Druck in −z; Erfolg = z-Sprung / Fz-Abfall.

---

## 6. Arduino / Serial-Protokoll (Vorschlag)

Zeilenbasiert ASCII, jeder Befehl wird quittiert:
```
PC → Arduino:  <ID> <CMD> [args]\n
Arduino → PC:  <ID> ACK\n          (sofort)
               <ID> DONE [data]\n  (Aktion fertig)
               <ID> ERR <code>\n
```
| Befehl | Bedeutung |
|---|---|
| `IRIS_OPEN` | Iris öffnen bis Endlage/Zeit |
| `IRIS_CLOSE <pwm>` | schliessen, danach in Haltemodus |
| `IRIS_HOLD <pwm>` | Haltekraft setzen |
| `ROT <dir> <pwm> <ms>` | M3 drehen |
| `STOP` | alle Motoren aus (Not-Halt) |
| `STATUS` | Zustand + Sensorwerte |
| Heartbeat | Arduino stoppt Motoren, wenn > X s kein Befehl |

ROS2-Seite: Bridge-Node als **Action Server** (z. B. `GripperCommand`), damit State-Machines auf DONE/Timeout warten können statt blind zu schlafen.

---

## 7. Software-Architektur (Vorschlag)

> Hinweis: Das Repo hat bereits `arduino/`, `cad/`, `software/`, `data/`, `visualization/`, `docs/`. Der Baum unten zeigt die Zielstruktur für `software/` – Details hängen von F4 (ROS2-Workspace ja/nein) ab.

Statt zwei kopierten Codebases: **ein Repo, gemeinsame Module, zwei getrennt startbare State-Machines** (pick / place). Damit bleibt separates Testen möglich, ohne doppelten Code.

```
semesterarbeit/
├── docs/PLAN.md             ← dieses Dokument
├── CLAUDE.md                ← Kurzkontext/Konventionen für Claude
├── firmware/                ← Arduino-Sketch
├── src/
│   ├── sa_interfaces/       ← msgs/actions (GripperCommand, DetectedObject)
│   ├── sa_vision/           ← Detektion + Pixel→3D (reine Python-Libs + dünner ROS-Node)
│   ├── sa_gripper/          ← Serial-Bridge (Action Server)
│   ├── sa_motion/           ← Pose-Helfer, Wiggle, Steifigkeits-Presets, CIC-Interface
│   └── sa_tasks/            ← State-Machines pick.py, place.py, Launchfiles, config/*.yaml
├── calibration/             ← Intrinsics, Hand-Eye (Ergebnisse versioniert)
├── tools/                   ← Logging, Auswertung, Plots
└── tests/                   ← Unit-Tests ohne Hardware (Mocks)
```
Prinzipien: Mathe/Logik ohne ROS-Abhängigkeit (testbar am Laptop), alle Zahlen in YAML, Hardware über Interfaces mockbar.

---

## 8. Was wir **vor** Workstation-Zugang vorbereiten

| # | Paket | Ohne Workstation testbar? |
|---|---|---|
| 1 | Repo-Struktur, Interfaces, `CLAUDE.md` | ✅ |
| 2 | Arduino-Firmware inkl. Protokoll + Heartbeat | ✅ am Board allein |
| 3 | Pixel→3D-Modul + Unit-Tests (synthetische Daten) | ✅ |
| 4 | Detektion mit beliebiger Webcam / Fotos der Mutter | ✅ am Laptop |
| 5 | Intrinsics-Kalibrierung der eigentlichen Kamera | ✅ am Laptop |
| 6 | State-Machine-Gerüst mit Mock-Robot + Mock-Gripper | ✅ |
| 7 | Logging- und Auswerteskripte (BA-Metriken + neue) | ✅ mit BA-Logs |
| 8 | Hand-Eye-Kalibrierungsablauf (Skript fertig, Durchführung später) | ⚠️ teilweise |
| 9 | CIC-Integration, Steifigkeiten, Realtests | ❌ |

---

## 9. Evaluation (früh festlegen)
- Gleiche Metriken wie BA (Tracking-Fehler je Phase, Zykluszeit, Erfolgsrate) → direkter Vergleich.
- **Neu:** Positionsfehler relativ zum Objekt (z. B. Mutter/Schraube an bekannter Position via Lehre, Abweichung messen).
- Erfolgsrate über N ≥ 10 Runs, verschiedene Startpositionen, ggf. verschiedene Muttergrössen.

---

## 10. Offene Fragen – bitte direkt hier beantworten

> Einfach hinter `Antwort:` schreiben (auch direkt im GitHub-Webeditor). Stichworte reichen. Unbekannt → `?` lassen.

### Software / Umgebung
**F1.** ROS2-Distro auf der Workstation (Humble / Jazzy)? Ubuntu-Version? franka_ros2-Version / libfranka?
Antwort: Ubuntu, weiss aber nicht welche version. 

**F2.** CIC: derselbe Controller wie in der BA? Eigener Code oder aus franka_ros2 / Lab-Repo? Wie werden Soll-Pose und Steifigkeit gesetzt (Topic-Namen, Msg-Typ)?
Antwort: ja der gleiche, aber er soll variabel pro phase benutzt werden können. wahrsheinnlich einfach 1 mit hoher 1 mit niedrigerer stiffness, mehr brauchts vermutlich nicht.

**F3.** BA-Code: Wann kommt er ins BA-Repo? Welche Teile willst du übernehmen (Vision, State-Machine, Bridge, Logging)?
Antwort: ich will eig nichts übernhemen. maximal das prinzip der vision pipeline, state machines sicher nicht, die bridge könnte man vlt auch, logging nicht. man kann also eig von einem clean slate starten.

**F4.** Das SA-Repo hat aktuell `software/` als reines Python-Paket (Windows-venv im README). Soll die SA-Software ein **ROS2-Workspace** werden (Pakete unter `software/src/`) oder bleibt es Python + rclpy ohne colcon?
Antwort: ja das soll dann danach alles über ROS2 laufen. das war bisher einfach ein platzhalter.

### Gripper / Elektronik
**F5.** Motortreiber: Adafruit Motor Shield v2 (wie in `sketch_sep24a`)? Welcher Arduino (Uno/Mega/…)? Welcher Motor hängt an welchem Port (M1–M4)?
Antwort: Arduino R4 Minima mit Motorshield von Adafruit v2.3. motoren ist noch nicht ganz klar aber vermutlich sind die beiden für das Iris Shutter an port M2 und M4, das kann dann aber noch angepasst werden. der dritte motor ist dann vermutlich an Port M1

**F6.** Iris: Wie wird „geschlossen / Mutter gegriffen“ erkannt? (Drucksensor wie BA an A0, Stromsensor, Endschalter, Encoder, nur Zeit?)
Antwort: Über Zeit, die Motoren geben kein Feedback. das wird dann einfach iteriert bis es funktioniert. sollte aber keine grosse Sache sein. 

**F7.** Dauerhaftes Halten: Getriebe selbsthemmend (Schnecke)? Oder muss der Motor mit Halte-PWM bestromt bleiben? Wie heiss werden die Motoren?
Antwort: Die Motoren sollen einfach den Befehl bekommen weiter zu drehen. dann Drücken sie ja quasi einfach alles zusammen, das reicht dann auch. Heiss werden die nicht, habe das gleiche bei der BA auch gemacht und hatte nie probleme.

**F8.** M3 dreht den ganzen Gripper inkl. M1/M2: Kabelführung? (Schleifring / max. Umdrehungen + zurückdrehen / anders)
Antwort: der Dritte Motor, vermutlich an port 1 dreht den ganzen Gripper. Kabelführung inkl. Schleifring ist berücksichtigt. der muss einfach drehen.

**F9.** M3: Encoder vorhanden? Wie soll „fertig geschraubt“ erkannt werden (Zeit, Strom/Stall, z-Weg des Roboters, Fz/Mz vom FR3)?
Antwort: Ebenfalls über Zeit, therotisch geht auch pber einen Torque Sensor am Arm, aber eher unwahrscheinlich

**F10.** Übersetzung Motor → Iris und Motor → Rotation bekannt? (für Umdrehungen ↔ Zeit)
Antwort: das weiss ich leider nicht, spielt aber auch nicht so eine Rolle. das wird dann getestet bis es stimmt.

### Kamera / Vision
**F11.** Kameramodell, Auflösung, FPS? Fokus fix oder Autofokus (Autofokus zerstört Kalibrierung)?
Antwort: HutoPi 720p HD USB camera with OV9726 module, wir haben dafür auch bei der BA keine spezifikationen gefunden und mussten sleber eine kalibrierung machen. ich kann mal schauen ob ich diese Werte wieder finde. 

**F12.** Montage: Senkrecht nach unten? Ungefährer Versatz Kamera → Gripper-Mitte (x, y, z in mm)?
Antwort: Stand 07.10.26, Delta_x = 0.1mm = 0mm, Delta_y = -44.505mm, Delta_z = 52.1mm

**F13.** Bleibt die schwarze Lackierung + Threshold-Detektion? Untergrund (Farbe/Material)? Beleuchtung kontrolliert?
Antwort: eigentlich will ich dass es ohne schwarze lakierung geht. sprich einfach die Mutter auf weissem untergrund.

**F14.** Muss die Orientierung der Mutter (Sechseck-Winkel) bestimmt werden, oder zentriert die Iris die Mutter selbst beim Schliessen?
Antwort: muss nicht bestimmt werden. wenn wir in zukunft viele Fehler deswegen haben kann mans nochmal anpassen.

### Teile / Aufgabe
**F15.** Muttergrössen (M5, M6, …)? Mutterhöhe? Schraubentyp und -länge, wie fixiert (eingeklebt, Platte)?
Antwort: M6, schraube ist eine "stiftschraube" mit einer quadratischen Basis, 10x10x20 LxBxH, gewinde 19mm hoch, fixiert, sprich ist immer schon schraubbereit.

**F16.** Liegen Mutter und Schraube an beliebigen Positionen im Arbeitsbereich oder in einem bekannten Bereich? Mehrere Muttern nacheinander?
Antwort: das Ziel ist, dass beliebige Positionen in einem vorbestimmten Bereich gewählt werden können. da ich nur eine Kamera habe muss die mutte rund schraube jeweils in einer gehardcodeden Area liegen in welcher das visual servoing dann greift.

**F17.** Ist „Mutter flush auf Tisch“ sicher? Gripper-Unterseite vs. Mutterhöhe – greift die Iris dann auf voller Mutterhöhe?
Antwort: wenn gripper unterseite auf dem tisch liegt, werden die greifzähne des Iris mechanismus 3.5mm höher sein als der Tisch. also sprich die Mutter wird flush mit der Gripper unterseie gemacht, nicht mit den gripper Zähnen selbst. spielt aber kein Rolle.

### Organisatorisch
**F18.** Abgabedatum, Zwischenpräsentation, Meilensteine? Ab wann hast du Zugriff auf die Workstation?
Antwort: Theoretisch ist abgabe am 31.10.26. zwischenabgaben gibt es nicht, keine Meilensteine. Am ende soll die erfolgsquote einfach höher sein.

**F19.** Vorgaben Betreuer (Sprache des Berichts, Pflicht-Evaluation, Vergleich mit BA erwünscht)?
Antwort: alles in Englisch, Vergleich mit BA soll sicher auch drin sein. 

**F20.** Sonst noch etwas, das ich wissen sollte / was dich an meinem Vorschlag (§5–§7) stört?
Antwort: Momentan nicht. das wird dann mal bis mal gemacht.

---

## 11. Entscheidungslog
| Datum | Entscheidung | Begründung |
|---|---|---|
| 07.10.2026 | Ansatz 2 (direkte Zielpose) zuerst, Ansatz 1 als Fallback | BA: Zentrierung = grösste Streuung |
| 07.10.2026 | Ein Repo, zwei State-Machines statt zwei Codebases | weniger Duplikat, trotzdem getrennt testbar |
| 07.10.2026 | Serial mit ACK/DONE + Heartbeat | BA: verlorener Befehl blockierte Ablauf |
