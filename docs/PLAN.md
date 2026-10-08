# Semesterarbeit – Projektkontext & Gameplan

> Lebendes Dokument, Kontext für Claude und mich. Bei jeder Designentscheidung nachführen.
> **Version 2** (07.10.2026) – nach Beantwortung der Fragen F1–F20 (Antworten im Anhang A).
> Abgabe: **31.10.2026** → alles, was nicht direkt die Erfolgsquote erhöht, wird gestrichen.

---

## 1. Ziel

Ein FR3 mit eye-in-hand USB-Kamera und neuem Iris-Gripper soll eine **M6-Mutter** (blank, auf weissem Untergrund) innerhalb eines definierten Bereichs finden, greifen, auf eine fixierte Stiftschraube setzen und aufschrauben.
Erfolgskriterium: **höhere Erfolgsquote als in der BA**, Vergleich mit der BA im Bericht (Englisch).

Kernänderungen gegenüber der BA:
1. **Direkte Zielpose** aus Kamerabild (Ansatz 2) statt iterativer Pixel-Regelung.
2. **Iris-Gripper + rotierender Gripper** statt Backen + Schraubendreher.
3. **Keine schwarze Lackierung** mehr – Detektion von blankem Metall auf Weiss.
4. **Clean Slate:** neue ROS2-Software, aus der BA wird nur das Prinzip der Vision-Pipeline übernommen.

---

## 2. Ausgangslage aus der BA (Kurzfassung)

| Phase | Steady-state Fehler BA [mm] |
|---|---|
| Grasp | 0.16 ± 0.14 |
| Vision centering | 1.57 ± 0.57 |
| Seating | 1.78 ± 0.18 |
| Release | 0.86 ± 0.14 |
| Screwing | 1.35 ± 0.19 |

- 8 Runs, Ø Zykluszeit 62 s, 1 Run abgebrochen (Zentrierung oszillierte bis Timeout).
- Gemessen wurde Tracking (Ist vs. Soll), **nicht** Genauigkeit relativ zum Objekt.
- Schwächen → Antworten in der SA: iterative Zentrierung → Ansatz 2; feste Offsets → definierte TF-Kette; Serial ohne Quittung → ACK/DONE-Protokoll; Steifigkeit nicht variiert → 2 Presets.

---

## 3. Hardware (fix)

```
Workstation ──USB-C (serial)──► Arduino UNO R4 Minima + Adafruit Motor Shield v2.3
     │                               ├─ M2 ┐ Iris (gegensinnig)
     │                               ├─ M4 ┘
     │                               └─ M1   Rotation ganzer Gripper (Schleifring)
     ├──USB-A──► HutoPi 720p USB-Kamera (OV9726), eye-in-hand
     └──Ethernet──► FR3 (franka_ros2, Cartesian Impedance Controller aus BA)
```
Portbelegung ist **vorläufig** → in Firmware nur an einer Stelle definiert (Konstanten).

| Grösse | Wert | Quelle |
|---|---|---|
| **Flansch-Mitte → Kamera (Linsenmitte, unterster Punkt)** | Δx = 0, Δy = +44.4 mm, Δz = 60.223 mm | CAD, Stand 08.10.26 (F21) – Achsenrichtung → F27 |
| Kamera → Gripper-Mitte (alt, nur Plausibilität) | Δx = 0, Δy = −44.505 mm, Δz = 52.1 mm | CAD, 07.10.26 |
| **Flansch-Mitte → Mitte Bodenplatte Gripper (TCP, liegt auf Tisch auf)** | Δx = 0, Δy = 0, Δz = 113.823 mm (CAD: 0.1/0.1 → 0) | CAD, Stand 08.10.26 ✅ |
| Iris-Zähne über Gripper-Unterseite | 3.5 mm | CAD |
| Mutter | M6 (Höhe ≈ 5 mm, SW 10 mm, DIN 934) | F15 |
| Stiftschraube | quadratische Basis 10×10 mm, H 20 mm, Gewinde 19 mm | F15 (Höhen → F22) |
| Kamera-Intrinsics | aus BA-Kalibrierung, falls auffindbar, sonst neu | F11 |

Gripper-Verhalten: **alles zeitgesteuert, kein Feedback**. Halten = Iris-Motoren laufen weiter in Schliessrichtung (in der BA ohne Überhitzung erprobt).

---

## 4. Ablauf (Soll)

Arbeitsbereiche für Mutter (**Area N**) und Schraube (**Area S**) sind fest. Zu jeder Area gibt es eine fest definierte **Beobachtungspose**, von der aus die ganze Area im Bild liegt.

### Phase A – Pick
| # | Schritt | Steifigkeit | Abbruch/Übergang |
|---|---|---|---|
| A1 | Iris öffnen, zur Beobachtungspose N fahren | HIGH | Pose erreicht |
| A2 | Bild aufnehmen (n Frames mitteln), Mutter detektieren | – | Detektion gültig, sonst Retry/Abbruch |
| A3 | Pixel → Punkt im Basis-Frame (§5.1), Ziel = Gripper-Mitte über Mutter | – | – |
| A4 | Anfahrpose über Mutter (z.B. +30 mm) | HIGH | Pose erreicht |
| A5 | Absenken bis Gripper-Unterseite auf Tisch | LOW | Fz < Schwelle **oder** z-Ziel erreicht |
| A6 | Iris schliessen (t_close), danach Halten (Motoren laufen weiter) | LOW | Zeit |
| A7 | Anheben | HIGH | Pose erreicht |

### Phase B – Place & Screw
| # | Schritt | Steifigkeit | Abbruch/Übergang |
|---|---|---|---|
| B1 | Beobachtungspose S | HIGH | Pose erreicht |
| B2 | Schraube detektieren, Pixel → Basis | – | Detektion gültig |
| B3 | Anfahrpose: Mutter koaxial über Gewindespitze (+10 mm) | HIGH | Pose erreicht |
| B4 | Absenken in −z mit leichtem xy-Wiggle bis Mutter aufsitzt | LOW | Fz < Schwelle |
| B5 | Optional: kurz rückwärts drehen (Gewindeanfang finden) | LOW | Zeit |
| B6 | M1 vorwärts drehen, Roboter folgt in −z (niedrige z-Steifigkeit, Soll leicht unter Ist) | LOW | Zeit t_screw |
| B7 | Iris öffnen, wegfahren | HIGH | – |

Pick und Place sind **einzeln startbar** (zum Testen), plus ein kombinierter Ablauf.

---

## 5. Methodik

### 5.1 Pixel → Basis-Frame (Ansatz 2)
Kamera schaut senkrecht nach unten, Objektebene bekannt (Tisch + Objekthöhe):

```
(u, v)        entzerrter Pixel des Objekt-Mittelpunkts
d             = z_cam − z_obj   (Abstand Kamera → Objektebene, aus FK + Offset)
X_c = (u − c_x) · d / f_x
Y_c = (v − c_y) · d / f_y
p_base = T_base_flange · T_flange_cam · [X_c, Y_c, d, 1]ᵀ
Ziel Gripper-Mitte (xy) = p_base(xy),  z aus Phase
```
- Allgemeine Umsetzung als **Strahl-Ebene-Schnitt** (funktioniert auch bei leicht schiefer Kamera, kostet nichts extra).
- `T_flange_cam` vorerst aus **CAD-Offset**. Eine volle Hand-Eye-Kalibrierung lassen wir aus Zeitgründen weg. Stattdessen **Verifikation**: Mutter an bekannter Position → Ziel anfahren → Restfehler messen → konstanten xy-Korrekturterm in YAML.
- Objektebenen: Mutter = Tisch + ~5 mm; Schraube = Oberkante der quadratischen Basis (Kontur), siehe F22.
- Fallback (Ansatz 1): 1–2 Korrekturiterationen mit einem zweiten Bild aus der Anfahrpose. Reiner Code-Pfad im selben Modul, per Parameter an/aus.

### 5.2 Detektion ohne Lackierung (blank auf Weiss)
Ein Vorteil ist, dass der Untergrund jetzt **hell** ist und das Objekt dunkler bzw. strukturiert:
1. Grau → Gauss → **Otsu-Threshold invertiert** (oder adaptiv) → Morphologie.
2. Nur Konturen innerhalb einer **ROI = Area im Bild** (Areas sind ja bekannt).
3. **Mutter:** Kontur mit **innerem Loch** (`RETR_CCOMP`-Hierarchie). Das Loch (weisser Tisch durch die Mutter sichtbar) ist ein sehr robustes Merkmal, und sein Mittelpunkt = Mutter-Mittelpunkt. Filter: Fläche, Verhältnis Loch/Aussen, Kompaktheit.
4. **Schraube:** Quadrat 10×10 (approxPolyDP → 4 Ecken, Seitenverhältnis ≈ 1) mit Kreis (Gewinde) in der Mitte. Wir nehmen den Mittelpunkt des Quadrats.
5. Erwartete Grösse in Pixeln aus d und f berechnen → Flächenfilter **automatisch** statt von Hand getunt.
6. Reflexionen von blankem Metall: Mittelung über n Frames, Median-Mittelpunkt; ggf. diffuses Licht.

Alles als reine Funktionen (`numpy`/`cv2`, kein ROS) → am Laptop mit Fotos testbar.

### 5.3 Regelung
- Bestehender CIC aus der BA, **2 Steifigkeits-Presets** (HIGH für freie Bewegung, LOW für Kontakt/Schrauben) in YAML.
- Kontakt: gefiltertes Fz < Schwelle (BA: −4 N, Tiefpass α = 0.1).
- Wiggle (B4): kleine Kreisbahn in xy (≈ 0.5–1 mm, ~2 Hz) während z sinkt; nur falls ohne Wiggle Fehlversuche auftreten.

---

## 6. Gripper-Firmware & Serial-Protokoll

Zeitsteuerung läuft **auf dem Arduino** (nicht-blockierend mit `millis()`). Der PC schickt nur Befehle und wartet auf DONE.

```
PC → Arduino:  <id> <CMD> [args]\n
Arduino → PC:  <id> ACK\n            sofort nach gültigem Befehl
               <id> DONE\n           Aktion fertig (Zeit abgelaufen)
               <id> ERR <grund>\n
               READY\n               nach Boot
```
| Befehl | Wirkung |
|---|---|
| `OPEN <ms> <speed>` | Iris öffnen für ms, dann Motoren aus |
| `CLOSE <ms> <speed> <hold_speed>` | Iris schliessen für ms → DONE → weiter mit hold_speed (Halten) |
| `ROT <+1/-1> <ms> <speed>` | M1 drehen für ms → DONE → aus. Iris hält weiter |
| `STOP` | alle Motoren aus |
| `PING` | → `<id> DONE` (Lebenszeichen + Watchdog-Reset) |
| `STATE` | → `<id> DONE iris=<open/closing/holding/opening/off> rot=<on/off>` |

- **Watchdog:** kein Befehl/PING für > 2 s → alle Motoren aus. Die Bridge sendet PING mit 2 Hz. (Achtung: beim Halten essentiell, sonst lässt die Iris die Mutter fallen, wenn ROS hängt – das ist gewollt sicherer Zustand.)
- Konstanten (Ports, Richtungen, Default-Zeiten) oben in der Firmware, Zeiten sonst immer vom PC → Tuning nur in YAML, nicht neu flashen.
- ROS2: `gripper_bridge`-Node als **Action Server** `~/command` (`GripperCommand.action`: cmd, duration_ms, speed, hold_speed → success, message).

---

## 7. Software-Architektur

ROS2-Workspace unter `software/` (Clean Slate). Distro-agnostisch (Humble/Jazzy), nur `rclpy`, keine exotischen Abhängigkeiten.

```
software/
├── src/
│   ├── sa_interfaces/      ament_cmake: GripperCommand.action, Detection.msg
│   ├── sa_gripper/         serial_protocol.py (rein), gripper_bridge node
│   ├── sa_vision/          detection.py, geometry.py (rein), vision_node
│   └── sa_tasks/           state machines pick/place/full, robot_interface.py (CIC-Wrapper),
│                           launch/, config/{robot,vision,gripper,areas}.yaml
└── tests/  (pro Paket unter test/, pytest, ohne Hardware)
arduino/iris_gripper/       neue Firmware (sketch_sep24a bleibt als Motortest)
calibration/                Intrinsics (yaml), Verifikationsmessungen
data/                       Logs der Runs (csv/rosbag)
tools/                      Auswertung + Plots für den Bericht
```
Prinzipien:
- **Reine Logik ohne ROS** (Detektion, Geometrie, Protokoll, State-Machine-Übergänge) → Unit-Tests am Laptop.
- **Alle Zahlen in YAML**, keine Magic Numbers im Code.
- Hardware hinter Interfaces (`RobotInterface`, `GripperInterface`, `Camera`) → **Mock-Varianten** für Trockenlauf.
- State Machine: einfache explizite Python-Klasse (Enum + Übergangstabelle), kein SMACH/BT – weniger Overhead.
- Jeder Run schreibt automatisch eine CSV (Zeit, Phase, Soll-/Ist-Pose, Fz, Detektion) → direkt BA-vergleichbare Auswertung.
- Code, Kommentare und Logs **auf Englisch** (Bericht ist Englisch).

---

## 8. Zeitplan (Abgabe 31.10.)

| Woche | Fokus | Ohne Workstation? |
|---|---|---|
| **KW41** (07.–11.10.) | Firmware + Protokoll, Workspace-Gerüst, Geometrie + Tests, Detektion an Fotos, Mocks, Logging | ✅ |
| **KW42** (12.–18.10.) | Workstation: Build, Kamera + Intrinsics, Offset-Verifikation, **Phase A läuft** | ❌ |
| **KW43** (19.–25.10.) | **Phase B läuft**, Zeiten/Fz tunen, Evaluation-Runs (N ≥ 10) | ❌ |
| **KW44** (26.–31.10.) | Auswertung, Plots, Bericht fertig | ✅ |

Bericht parallel schreiben (Methodik-Kapitel kann schon in KW41/42 entstehen).

### Vorbereitung – Checkliste (KW41, mit Claude)
- [ ] `CLAUDE.md` + Workspace-Gerüst (Pakete, package.xml, setup.py, Launch, YAML)
- [ ] Firmware `arduino/iris_gripper` + Test am Board (ohne Roboter)
- [ ] `serial_protocol.py` + `gripper_bridge` + Test-CLI
- [ ] `geometry.py` (Pixel → Basis) + Unit-Tests mit synthetischen Daten
- [ ] `detection.py` + Fotos von Mutter/Schraube auf Weiss (Handy oder die Kamera am Laptop) → `data/images/`
- [ ] Intrinsics: BA-Werte suchen, sonst Kalibrierskript (Schachbrett) am Laptop
- [ ] State Machines + Mock-Robot/-Gripper → Trockenlauf des gesamten Ablaufs
- [ ] Logging + Auswerteskript (BA-Metriken)

---

## 9. Evaluation
- **Erfolgsquote** (Hauptmetrik): N ≥ 10 Runs mit verschiedenen Positionen in Area N / Area S. Erfolg je Phase separat protokollieren (Detektion / Greifen / Aufsetzen / Schrauben).
- **BA-Vergleich:** gleiche Tracking-Metriken je Phase + Zykluszeit.
- **Neu:** Positionsfehler der Zielberechnung relativ zum Objekt (Objekt an bekannter Position, Abweichung Ziel ↔ Wahrheit).

---

## 10. Offene Fragen (Runde 2)

> Hinter `Antwort:` schreiben. Kurz reicht.

**F21.** Kamera-Offset: relativ zu welchem Punkt und in welchem Frame?
Antwort (08.10.): Flansch-Mitte → Kamera-Linsenmitte (unterster Part): Δx = 0, Δy = 44.4 mm, Δz = 60.223 mm. ✅

**F27.** Achsen / Frames.
Antwort (08.10., bestätigt): Blick von der Roboterbasis aus, Gripper schaut senkrecht nach unten:
- Basis: z_robo nach oben, x_robo nach vorne, y_robo nach links.
- Kamera (OpenCV): z_cam nach unten (ins Bild), x_cam nach rechts (+u), y_cam nach hinten (+v).
- ⇒ **z_robo = −z_cam, x_robo = −y_cam, y_robo = −x_cam**, d.h. Δx_robo = −Δv, Δy_robo = −Δu (gleich wie BA „Axis Mapping“).
- EE-Frame (BA-Bild): z nach unten, x wie Basis-x, y = −Basis-y (180° um x). Damit R_EE_cam = Rz(+90°).

**F28.** Auf welcher Seite des Flanschs sitzt die Kamera, von der Basis aus gesehen bei nach unten schauendem Gripper: links (+y_robo), rechts (−y_robo), vorne oder hinten? (Damit ist klar, in welchem Frame die +44.4 mm gelten.)
Antwort: 

**F22.** Stiftschraube: Ist die Gesamthöhe 20 mm Basis + 19 mm Gewinde = 39 mm, oder 20 mm insgesamt? Schaut das Gewinde oben aus der Basis heraus?
Antwort: 

**F23.** CIC-Schnittstelle: Wie hast du in der BA Soll-Pose und Steifigkeit gesetzt (Topic-Name, Msg-Typ, z.B. `PoseStamped` auf `/cartesian_impedance/target`)? Woher kommt die Ist-Pose bzw. Fz (`franka_robot_state`)? Ein Code-Snippet aus der BA reicht.
Antwort: 

**F24.** Ab wann genau hast du Zugriff auf die Workstation?
Antwort: 

**F25.** Gibt es BA-Kalibrierwerte der Kamera (fx, fy, cx, cy, Distortion)? Falls ja: hier einfügen oder als Datei in `calibration/` ablegen.
Antwort: 

**F26.** Wie gross sind Area N und Area S ungefähr (mm), und wie weit liegen sie auseinander? Von welcher Höhe aus siehst du die ganze Area?
Antwort: 

---

## 11. Entscheidungslog
| Datum | Entscheidung | Begründung |
|---|---|---|
| 07.10. | Ansatz 2 zuerst, Ansatz 1 als optionaler Korrekturschritt | BA: Zentrierung = grösste Streuung |
| 07.10. | Ein ROS2-Workspace, Pick/Place separat startbar | weniger Duplikat, trotzdem einzeln testbar |
| 07.10. | Serial mit ACK/DONE + Watchdog, Zeitsteuerung auf dem Arduino | kein Feedback vorhanden, BA: verlorene Befehle |
| 07.10. | Keine Hand-Eye-Kalibrierung, CAD-Offset + Verifikation/Korrekturterm | Zeitbudget (Abgabe 31.10.) |
| 08.10. | Kamera-Pose relativ zum **Flansch** definieren (`T_flange_cam`), TCP separat | Flansch-Pose kommt direkt aus FK, keine Kette über Gripper nötig |
| 08.10. | Achsen-Mapping Kamera↔Basis aus BA übernommen (F27) | bestätigt, Grundlage für geometry.py |
| 08.10. | `T_flange_tcp` = (0, 0, 113.823 mm), Werte < 0.2 mm aus CAD = 0 | CAD; alter Kamera→Gripper-Offset (52.1) ist damit überholt |
| 07.10. | Keine Orientierungsbestimmung der Mutter | Iris zentriert selbst (F14) |
| 07.10. | Mutter über inneres Loch detektieren, ROI = bekannte Area | keine Lackierung mehr, robustes Merkmal |
| 07.10. | 2 Steifigkeits-Presets (HIGH/LOW) | reicht laut F2 |
| 07.10. | Clean Slate, Code auf Englisch | F3, F19 |

---

## Anhang A – Antworten Runde 1 (07.10.2026, gekürzt)
- **F1** Ubuntu, Version unbekannt; ROS2-Distro unbekannt.
- **F2** gleicher CIC wie BA, variabel pro Phase; 2 Stiffness-Stufen reichen.
- **F3** nichts übernehmen, max. Prinzip der Vision-Pipeline, evtl. Bridge → Clean Slate.
- **F4** alles über ROS2, `software/` war Platzhalter.
- **F5** Arduino R4 Minima + Adafruit Motor Shield v2.3; Iris vermutlich M2 + M4, Rotation M1 (änderbar).
- **F6** Iris über Zeit, kein Feedback, wird iterativ getunt.
- **F7** Halten = Motoren drehen weiter, in BA ohne Erwärmungsprobleme.
- **F8** Schleifring vorhanden, Rotation unbegrenzt.
- **F9** Schrauben über Zeit; Torque-Sensor am Arm theoretisch möglich, eher nicht.
- **F10** Übersetzungen unbekannt, egal – wird getestet.
- **F11** HutoPi 720p HD USB (OV9726), keine Specs, BA-Kalibrierung evtl. auffindbar.
- **F12** Offset Δx 0, Δy −44.505 mm, Δz 52.1 mm (Stand 07.10.).
- **F13** keine Lackierung mehr, Mutter auf weissem Untergrund.
- **F14** Orientierung nicht nötig, später ggf. nachrüsten.
- **F15** M6; Stiftschraube mit quadr. Basis 10×10×20, Gewinde 19 mm, fixiert.
- **F16** beliebige Position innerhalb fester Areas (eine Kamera).
- **F17** Gripper-Unterseite auf Tisch, Iris-Zähne 3.5 mm höher – unkritisch.
- **F18** Abgabe 31.10.26, keine Meilensteine, Ziel: höhere Erfolgsquote.
- **F19** Bericht Englisch, BA-Vergleich erwünscht.
- **F20** nichts weiter.
