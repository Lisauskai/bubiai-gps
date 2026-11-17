# BUBIAI GPS // Telemetrijos ir navigacijos PWA (`gps.bubiai.eu`)

Moderni, greita, vieno puslapio (SPA / PWA) GPS telemetrijos ir maršruto fiksavimo web aplikacija, optimizuota mobiliesiems telefonams su **Dark / OLED** didelio kontrasto tema.

**Svetainės adresas:** [https://gps.bubiai.eu](https://gps.bubiai.eu)

---

## Pagrindinės funkcijos

### 1. Greičio modulis (Speedometer)
- **Esamas greitis:** `km/h` (pagrindinis didelio kontrasto indikatorius) ir `m/s` šalia.
- **Dvigubas skaičiavimo mechanizmas:** naudojamas tiesioginis GPS Doplerio greitis (`coords.speed`), o jei įrenginys grąžina `null` – greitis apskaičiuojamas pagal **Haversine** atstumo tarp taškų ir laiko pokytį ($\Delta d / \Delta t$) su 5 taškų slenkančio vidurkio (moving average) išlyginimu.
- **Maksimalus greitis (`Max km/h`)** ir **vidutinis greitis** (tiek bendras, tiek gryno judėjimo – atmetus sustojimus, kai greitis `< 1.5 km/h`).

### 2. Aukščio ir reljefo modulis (Altimeter & Elevation)
- **Aukštis virš jūros lygio (`m`)** iš `coords.altitude`.
- **Sukauptas pakilimas (`Elevation Gain`, `+m`) ir nusileidimas (`Elevation Loss`, `-m`)** su dinaminiu slenkstiniu filtru (`±2.5m` arba pagal `altitudeAccuracy`), apsaugančiu nuo GPS aukščio šokinėjimo stovint vietoje.
- **Momentiškas nuolydis / įkalnė (`Gradient %`)** ir **vertikalus greitis (`m/min`)**.

### 3. Kelionės ir navigacijos statistika
- **Nuvažiuotas / nueitas atstumas (`km` ir `m`)** su mikro-poslinkių filtravimu (ignoruojami triukšmai, kai greitis `< 1 km/h` arba poslinkis mažesnis už GPS paklaidą).
- **Kelionės laikmatis:** bendras laikas ir gryno judėjimo laikas su `Pradėti / Pauzė / Atstatyti` valdikliais bei `Demo` simuliacijos režimu.
- **Kompasas ir kryptis:** judėjimo kursas laipsniais (`0–360°`) bei lietuviškomis pasaulio kryptimis (`Š, ŠR, R, PR, P, PV, V, ŠV`).

### 4. Techniniai ir jutiklių parametrai
- **GPS signalo kokybės indikatorius:** horizontalus (`H: ±m`) ir vertikalus (`V: ±m`) tikslumas su spalvine būsena (žalia / geltona / raudona).
- **Tikslios koordinatės (`Latitude`, `Longitude`)** su greito kopijavimo į iškarpinę mygtuku.
- **Pagreičio ir G-jėgos matuoklis (`DeviceMotionEvent`):** realaus laiko G-jėga, maksimali G-jėga, tiesinis pagreitis (`m/s²`), X/Y/Z ašys ir 2D radaras.
- **Realaus laiko grafikas (HTML5 Canvas):** greičio (`km/h`) ir aukščio (`m`) profilio kreivės.
- **Interaktyvus mini-žemėlapis (Leaflet + OpenStreetMap):** tamsus (OLED) ir standartinis OSM sluoksniai, trajektorijos linija (`polyline`), tikslumo apskritimas ir automatinis vietos centravimas.

### 5. Praktiniai įrankiai
- **Screen Wake Lock API:** automatiškai neleidžia telefono ekranui užgesti fiksavimo metu.
- **Didelio kontrasto (Lauko) režimas:** dar ryškesnis kontrastas tiesioginėje saulės šviesoje.
- **Eksportavimas:** užfiksuoto maršruto atsisiuntimas `.gpx` ir `.json` formatais.
- **Автоmatinis sesijos išsaugojimas (`localStorage`):** apsauga nuo netyčinio puslapio perkrovimo.
