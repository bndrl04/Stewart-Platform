# Platformă Stewart cu Raspberry Pi Pico

**Studenti:** *Pohrib George-Rafael & Canschi Nicodim*
**Platformă hardware:** Raspberry Pi Pico 2 W

---

## 1. Descrierea proiectului

Proiectul constă în construirea unei platforme Stewart (hexapod) cu structură printată 3D și tăiată manual, controlată de un Raspberry Pi Pico. Utilizatorul transmite comenzi de poziție și orientare prin interfața wireless, iar sistemul calculează în timp real cinematica inversă și controlează simultan șase servomotoare MG996R pentru a poziționa platforma superioară în spațiu. Sistemul oferă șase grade de libertate complete: translație pe axele X, Y, Z și rotație *pitch*, *roll*, *yaw*.

## 2. Cerințe funcționale

### 2.1 Control al poziției și orientării

Sistemul acceptă comenzi de poziție *(x, y, z)* și orientare *(pitch, roll, yaw)* și calculează automat unghiurile necesare pentru fiecare dintre cele șase servomotoare prin algoritmi de cinematică inversă. Mișcarea rezultată trebuie să fie fluidă și sincronizată între toate servomotoarele.

### 2.2 Generarea semnalelor PWM pentru servomotoare

Fiecare dintre cele șase servomotoare MG996R este controlat printr-un semnal PWM independent, generat de pinii hardware PWM ai Pico. Rezoluția și frecvența semnalului (50 Hz) trebuie să asigure poziționare precisă și stabilă a platformei.

### 2.3 Afișarea stării în timp real

Sistemul comunică starea curentă conform următoarelor mesaje:

- `Initializing...` — la pornire și calibrare;
- `Ready` și poziția curentă — în așteptarea comenzilor;
- `Moving...` — în timpul execuției unei mișcări;
- `Error: <detaliu>` — la detectarea unei erori de inițializare sau limită depășită.

### 2.4 Calibrare și poziție neutră

La pornire, sistemul aduce automat toate servomotoarele în poziția neutră (platformă orizontală, la înălțime medie), verificând răspunsul corect al fiecărui actuator. Eșecul unui servomotor determină semnalizarea erorii și întreruperea procedurii de inițializare.

### 2.5 Limitarea spațiului de lucru

Fiecare comandă de poziție este limitata conform unor parametri stabiliti in prealabil in cod, pentru a nu lasa utilizatorul sa scoata din spatiul de lucru atat platforma, cat si vreunul dintre servomotoare.
### 2.6 Mișcare interpolată

Deplasările între două poziții succesive sunt interpolate în pași mici, evitându-se mișcările bruște care ar putea deteriora mecanismul sau suprasolicita servomotoarele.

### 2.7 Blocare de urgență

Sistemul permite transmiterea unei comenzi de oprire imediată, care readuce platforma în poziția neutră și suspendă procesarea oricăror alte comenzi până la o resetare manuală.

## 3. Cerințe non-funcționale

### 3.1 Fiabilitate mecanică

Structura printată 3D trebuie să reziste forțelor generate de cele șase servomotoare MG996R în regim normal de funcționare. Articulațiile sferice se verifică periodic pentru semne de uzură, iar toate conexiunile electrice sunt securizate împotriva deconectării în timpul mișcării.

### 3.2 Precizia de poziționare

Eroarea de poziționare a platformei superioare nu trebuie să depășească ±2 mm pe componenta de translație și ±2° pe componenta de rotație, în condiții de încărcare normală.

### 3.3 Consum energetic

Alimentarea servomotoarelor se realizează separat de cea a Pico (sursă dedicată 5 V / 3 A), evitându-se solicitarea pinilor microcontrollerului. În repaus, când nicio comandă nu este procesată, servomotoarele sunt dezactivate pentru reducerea consumului și a încălzirii.

### 3.4 Latența de răspuns

Timpul scurs între primirea unei comenzi wireless și începutul efectiv al mișcării nu trebuie să depășească 100 ms, în condiții normale de rețea locală.

### 3.5 Extensibilitate

Arhitectura software permite adăugarea de moduri noi de operare (urmărire de traiectorie, mod demonstrație, control prin IMU) fără restructurarea completă a codului existent. Parametrii geometrici ai platformei (raze, lungimi de tije) sunt configurabili dintr-un singur fișier de configurare.

### 3.6 Toleranța la erori de comunicație

În cazul întreruperii conexiunii wireless, platforma revine automat în poziția neutră după un *timeout* configurabil și rămâne stabilă până la restabilirea legăturii.

---

