# Plan: Hotpath, Viertel-IRQ und Hall-Sensorfusion

Stand: 2026-10-18. Alle Zeiten und Zyklen in diesem Dokument sind Schätzungen aus
dem Code, nichts davon ist auf dem Board gemessen (Meilenstein M0 ändert das).

Zeitbasis bei 20 kHz PWM und 170 MHz: eine Halbperiode (ein JEOS-Interrupt) sind
25 µs = 4250 Zyklen, ein Viertel-Zyklus (vier Halbperioden) 100 µs = 17000 Zyklen.

## 1. Ziel

| Kontext | Rate | Inhalt |
|---|---|---|
| Hotpath (JEOS, Zero-Latency) | 40 kHz | nur was vom aktuellen Strom abhängt: Park, PI, Inverse Park, SVM, CCR |
| Viertel-IRQ (TIM8-Update, Zero-Latency) | 10 kHz | alles Vorberechnen für die nächsten 4 Halbperioden, Beobachter, Hall-Fusion |
| Regelthread (Zephyr, normale Priorität) | 1 bis 2 kHz | Drehzahlregler, später Lageregler, MTPA, Feldschwächung, Sollwertbegrenzung |
| Hall-Flanken (EXTI, normale Priorität) | bei Flanke | nur Zeitstempel und Hall-Code erfassen |

Annahme für die Vorberechnung: Drehzahl konstant über die vier Halbperioden
(100 µs). Beschleunigung wird nicht modelliert.

## 2. Ist-Zustand im Code

- `CurrentControlIsr::on_jeoc()` (980 Byte) macht heute alles in einem Zug:
  ADC lesen über Funktionszeiger, Überstromcheck, Bereichsprüfung der Duties
  (liest die CCR über einen weiteren Funktionszeiger zurück), Clarke, Park,
  PI mit Entkopplung aus dem **gemessenen** Strom, Inverse Park, HFI-Spannung,
  Totzeitkompensation, SVM, Schreiben der Duties (nicht inline).
- Der Aufruf von `std::sqrt` im Spannungsbegrenzer wird als libm-Aufruf
  `sqrtf` übersetzt (errno-Behandlung), nicht als `vsqrt`.
- `SlowUpdate::run_once()` (928 Byte) läuft heute im TIM8-Update-Callback,
  pollt `samples_ready`, parkt die vier Proben neu, mittelt, ruft Flussbeobachter,
  Kalman-Prädiktion, HFI und berechnet `sc[0..3]` für den nächsten Zyklus
  (Doppelpuffer mit `active`-Flip).
- `MechanicalObserver` ist ein Kalman-Filter mit den Zuständen ω, θ und
  Lastmoment, nur eine skalare Winkelmessung pro Aufruf von
  `inject_angle_error()`. Dieser führt **jedes Mal auch** die
  Kovarianzprädiktion aus. `PmsmFluxObserver::calculate()` ruft
  `inject_angle_error()` selbst am Ende auf. Die Kommentare nennen Hall-Sensoren,
  im Code gibt es dafür nichts. Der Winkelfehler wird nicht auf (-π, π] gewrappt.
  Prädiktion und Korrektur rechnen `std::sin/cos` (zwei Aufrufe pro Zyklus).
- Phase zwischen Viertel-Zähler des Kerns (`sub_step_`) und TIM8-Update ist nicht
  synchronisiert. Das Update liegt am Peak, die CCR1 bis 3 sind ohne Preload.
- Keine Hall-Erfassung, kein Drehzahlregler im Kern (nur `PositionController`
  in der Lib).

## 3. Entscheidungen, die du treffen musst

**D1 Preload und synchroner Viertel-IRQ (Empfehlung: ja).** Heute schreibt der
Hotpath die CCR ohne Preload in den laufenden Zähler. Dadurch muss er fertig
sein, bevor der Zähler die neue Vergleichsstelle erreicht, bei 95 % Duty etwa
1,25 µs nach dem Peak. Das schafft kein Code, der mehr tut als die CCR
schreiben. Mit RCR=0 und CCR-Preload wirken die neuen Werte erst an der nächsten
Grenze (Peak oder Valley), die Frist ist dann eine Halbperiode (25 µs) und der
Hotpath kann ruhig 2 bis 3 µs brauchen. Der Preis ist eine Halbperiode mehr
Totzeit im Regelkreis. Den Viertel-IRQ setzt dann der Hotpath im Sub-Schritt 3
per `NVIC_SetPendingIRQ` selbst. Er läuft danach direkt, ohne Race zum
Sub-Schritt-Zähler, und das Polling von `samples_ready` entfällt. Das ändert die
frühere Vorgabe "Update-IRQ mit RCR=4" ab, die Phase ist aber deterministisch.

**D2 Änderungen in `lib/motor_control` (brauchen deine Freigabe).**
- `MechanicalObserver`: Kovarianzprädiktion und Messupdate trennen
  (`PredictCovariance(dt)`, `UpdateAngle(ε, R)`). `inject_angle_error()` bleibt
  als Kombination erhalten. Nötig, damit Flussbeobachter und Hall im selben
  Zyklus je eine Messung mit eigenem R einspeisen, ohne die Kovarianz doppelt
  zu propagieren.
- `PmsmFluxObserver::calculate()`: den Winkelfehler zurückgeben statt selbst
  einzuspeisen (alte Variante als dünner Wrapper behalten).
- Optional: `std::sin/cos` im Beobachter durch die vorhandene `SinCos`-Tabelle
  ersetzen.

**D3 Entkopplung aus welchem Strom (Empfehlung: Mittelwert des letzten
Viertel-Zyklus).** Heute geht der gemessene Strom ein. Vorberechnet kommt nur
Strom aus dem letzten Mittel oder aus dem Sollwert in Frage. Im stationären Fall
ist das gleich, bei schnellen Sollwertsprüngen entkoppelt der Sollwert besser,
der Mittelwert ist robuster.

**D4 Eigene Sinus/Cosinus-Tabelle für die Ausgangsseite (Empfehlung: ja).** Der
Inverse Park nutzt heute denselben Winkel wie der Park der Messung. Die Spannung
wirkt aber etwa 1,5 Halbperioden später (Rechenzeit, Preload, Halten). Mit
`sc_out[k]` bei θ + ω·(k + Verzögerung)·dt kostet das nichts im Hotpath.

## 4. Datenfluss

Der Viertel-IRQ füllt pro Zyklus einen Plan (Doppelpuffer wie `SubStepBuffer`,
Flip mit release/acquire) für die nächsten vier Halbperioden:

```
struct HotpathPlan {
  SinCos sc_in[4];          // Park des gemessenen Stroms
  SinCos sc_out[4];         // Inverse Park der Sollspannung (D4)
  Rotor<Voltage> ff;        // Entkopplung, konstant über den Zyklus (ω konstant)
  Stator<Voltage> v_inj[4]; // HFI-Einspeisung pro Sub-Schritt
  Rotor<Current> i_ref;     // Sollwert, hier gelatcht (keine zerrissene d/q-Paarung)
  float inv_vdc;            // 1/V_dc, geglättet, für SVM
  float v_limit_sq;         // (v_max * V_dc)^2 für den Kreisbegrenzer
  float ki_dt_d, ki_dt_q, kb_dt_d, kb_dt_q;  // Integratorfaktoren inkl. dt
  float kp_d, kp_q;
};
```

Der Hotpath liest nur den Plan und schreibt zurück: `i_ab[k]`, `u_dq_last`,
Duties. Der Regelthread schreibt Sollwerte in eine Mailbox, die der Viertel-IRQ
beim Planbau einliest (Seqlock oder Doppelpuffer). Aufgerufen werden aus den
Zero-Latency-Kontexten keine Kernel-Funktionen.

### Hotpath (pro Halbperiode, Ziel 400 Zyklen, 2,4 µs)

1. Zwei Shunt-Kanäle lesen, mit Offset und Gain skalieren, dritte Phase aus
   ia + ib + ic = 0.
2. Clarke und Park verschmolzen mit `plan.sc_in[k]`.
3. Fehler, PI mit `kp`, Integrator mit `ki_dt`, plus `plan.ff`.
4. Kreisbegrenzer über `v_limit_sq`, Wurzel nur bei Sättigung (`__builtin_sqrtf`,
   Build mit `-fno-math-errno`).
5. Inverse Park mit `plan.sc_out[k]`, plus `plan.v_inj[k]`.
6. Totzeitkompensation (Vorzeichen des Stroms, Parameter aus dem Plan), SVM mit
   `plan.inv_vdc` statt Division.
7. Duties begrenzen auf 5 bis 95 %, die CCR direkt schreiben. Kein
   Zurücklesen, keine Funktionszeiger. Das Hardware-Zugriffsobjekt wird über
   CMake gewählt und inline eingebunden.
8. Überstromcheck bleibt vor allem anderen (60 A, `power_stage.Trip()`).
9. Im Sub-Schritt 3: Viertel-IRQ pending setzen.

### Viertel-IRQ (alle 100 µs, Budget etwa 15 µs, Ziel 2500 Zyklen)

1. Proben des letzten Zyklus einlesen (mit `sc_in` des Zyklus erneut parken,
   mitteln wie heute).
2. Mechanische Prädiktion mit dem mittleren Strom (Drehmoment aus i_d, i_q).
3. Kovarianzprädiktion einmal, danach Messupdates: Flussbeobachter, Hall,
   optional HFI. Jedes mit eigenem R und auf (-π, π] gewrapptem Fehler.
4. Plan für den nächsten Zyklus bauen: θ_k = θ + k·ω·dt, `sc_in`, `sc_out`,
   `ff`, `v_inj`, `inv_vdc`, `v_limit_sq`, Integratorfaktoren, `i_ref`.
5. Zustandsüberwachung der Endstufe (Fault, Blocked), Zähler für Zeitmessung.

### Regelthread (1 bis 2 kHz)

Drehzahlregler (PI mit Anti-Windup), später Lageregler (`PositionController`),
MTPA, Feldschwächung, Begrenzung des Stromsollwerts (Betrag auf `motor_i_max`,
50 A). Ausgabe: `i_ref` in die Mailbox. Eingang: ω und θ aus einem Snapshot des
Beobachters.

## 5. Hall-Sensorfusion

Hardware: drei Hall-Signale auf PC13, PC14, PC15 (RC-gefiltert 3k3/2n2,
Zeitkonstante etwa 7 µs). Das sind keine Timer-Eingänge, also kein
Hall-Interface des Timers: GPIO mit EXTI (Leitung 13 bis 15) und Zeitstempel
über den DWT-Zyklenzähler.

**Erfassung.** Der EXTI-Handler (normale Priorität, Zero-Latency-IRQs
unterbrechen ihn) speichert Hall-Code und Zeitstempel in einen kleinen Puffer.
Ungültige Codes (000, 111) werden als Fehler gezählt. Ein 10-kHz-Polling im
Viertel-IRQ wäre zu grob: bei 333 Hz elektrisch sind 100 µs etwa 12° Fehler.

**Messmodell** (beides aus demselben Hall-Code):
- *Flanke:* Der Winkel der Flanke ist aus einer Tabelle mit sechs Winkeln
  (steigend/fallend je Hall) bekannt. Der Viertel-IRQ rechnet den Zeitstempel
  zurück: ε = wrap(θ_Flanke + ω̂·Alter − θ̂). Das Alter enthält die Filterverzögerung.
  R aus der Ausrichtungstoleranz (z. B. ±3° elektrisch).
- *Sektor:* ohne Flanke gilt nur "Winkel liegt im 60°-Sektor". Der Innovationsterm
  ist 0, solange θ̂ im Sektor liegt, sonst der Abstand zur nächsten
  Sektorgrenze. Das verhindert, dass der Sektormittelpunkt den Beobachter
  verzieht, und hält ihn im Stillstand im richtigen Sektor ohne Drift.

**Fusion mit dem Flussbeobachter.** Der Flussbeobachter liefert bei kleiner
Drehzahl kein Signal (Gegen-EMK etwa 0). Sein R wird mit der Drehzahl
skaliert (R_flux = R0 + k/ω², oder hartes Abschalten unterhalb einer
Schwelle). Hall bleibt auch bei hoher Drehzahl aktiv und bremst die
Drift des Flussbeobachters. Im Stillstand und nahe null führt allein Hall.

**Tabelle der Flankenwinkel.** Nominal 60° Schritte mit einem Offset. Exakt
bestimmt man sie einmal mit einem Offener-Regelkreis-Lauf (V/f oder
vorgegebener Winkel) und Mitschrieb der Flanken, abgelegt als Einstellung
(Flash-Backend steht noch aus, bis dahin RAM und Shell).

**Position.** Aus dem fusionierten θ und der Polpaarzahl liefert
`PositionTracker`/`RotorAngle` die mechanische Position. Absolute Referenz
bleibt eine Anwendungsfrage (Homing).

**Simulation.** Für native_sim ein einfaches PMSM-Streckenmodell
(ω, θ aus Drehmoment, Ströme aus Spannungen) in der Sim-Hardware, das Ströme und
Hall-Flanken erzeugt. Damit läuft der Hotpath samt Fusion im twister
geschlossen.

## 6. Meilensteine

Jeder Meilenstein baut für BCC und native_sim und lässt den twister grün.

| M | Inhalt | Nachweis |
|---|---|---|
| M0 | Zeitmessung: DWT-Zyklen um Hotpath und Viertel-IRQ, Min/Mittel/Max in Ringpuffer, Ausgabe per Log/Shell. Baseline des heutigen Codes. | Zahlen vom Board |
| M1 | D1 umsetzen: RCR=0, CCR-Preload, Viertel-IRQ vom Hotpath gepended, `samples_ready`-Polling entfällt. Sim spiegelt das. | Test für Reihenfolge und Kadenz |
| M2 | `HotpathPlan` und Planbau im Viertel-IRQ (`sc_in`, `sc_out`, `ff`, `v_inj`, `inv_vdc`, Begrenzer). | Golden-Test: gleiche Eingänge, Duties wie vorher innerhalb Toleranz |
| M3 | Hotpath verschlanken: verschmolzenes Park, keine Funktionszeiger, `-fno-math-errno`, Sättigung ohne libm-Aufruf. | Zyklen gegen Baseline aus M0 |
| M4 | Beobachter-API nach D2, Wrap des Fehlers, getrenntes R. | gtest mit synthetischen Verläufen (konstantes ω, Rampe, Rauschen) |
| M5 | Hall-Treiber (EXTI, Zeitstempel, Plausibilität) und Sim-Hall aus dem Streckenmodell. | ztest mit gestellten Flanken |
| M6 | Fusion im Viertel-IRQ: Sektor- und Flankenmessung, Geschwindigkeits-Gating. | Tests: Stillstand ohne Drift, Hoch- und Runterlauf, Hall-Ausfall, Wechsel zu Fluss-only |
| M7 | Regelthread mit Drehzahlregler, Sollwertbegrenzung und Mailbox. | Sim-Test: Sprung auf ω, Strombegrenzung greift |
| M8 | Board: ADC-Ausrichtung, Offset, Hall-Tabelle per Offener-Regelkreis-Lauf, dann geschlossener Stromregler mit 50-A-Limit, Messung der ISR-Zeiten. | Messwerte, Oszilloskop |

Reihenfolge: M0 und M1 zuerst, weil D1 entscheidet, wie viel Zeit der Hotpath
hat. M2 bis M3 dann gemeinsam. M4 braucht deine Freigabe für die Lib. M5/M6
können nach M4 parallel zu M7 laufen.

## 7. Risiken und offene Punkte

- **ISR-Zeit.** Die Abschätzung (Hotpath etwa 400 Zyklen, Viertel-IRQ etwa 2500)
  ist nicht gemessen. Wenn Hotpath plus Viertel-IRQ in einer Halbperiode nicht
  fertig werden, verschiebt sich der Hotpath-Start und die Regelung verliert
  eine Halbperiode. M0 klärt das früh. Reicht es nicht: PWM-Frequenz senken
  oder Viertel-Arbeit auf zwei Halbperioden verteilen.
- **Code und Daten im Flash.** Bei 170 MHz hat der Flash Wartezustände (ART-Cache
  mildert das). Falls der Jitter stört: Hotpath und Plan in das CCM-SRAM legen
  (Zephyr `CODE_DATA_RELOCATION`). Erst nach der Messung.
- **FPU in Zero-Latency-ISRs.** Lazy Stacking kostet Zyklen, wenn ein Thread die
  FPU nutzt. Im M0 mitmessen.
- **Hall-Ausrichtung.** Ohne kalibrierte Tabelle sind Winkelfehler bis etwa
  ±10° möglich. Flanken-R muss dazu passen.
- **Anlauf.** Im Stillstand kennt die Fusion nur den Sektor (±30°). Das reicht
  für den Anlauf mit etwa 87 % Drehmoment (cos 30°). Das Kalman-Filter konvergiert
  an den ersten Flanken.
- **Motordaten fehlen.** Polpaarzahl, Hall-Anordnung (120° oder 60°),
  maximale Drehzahl und Strombelastbarkeit sind nötig für R, die Schwellen und
  die Tabelle.
