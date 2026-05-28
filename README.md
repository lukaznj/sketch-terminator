# 🤖 sketch_terminator

> Cjeloviti ROS 2 paket za autonomno upravljanje 3-DOF crtačkim manipulatorom, koji ujedinjuje YOLO viziju, napredno planiranje putanja, kalibraciju, fizikalno ograničenu kinematiku i naprednog ROSA AI agenta.

---

## 🚀 Ključne Značajke

* **`Glatka i Sigurna Trajektorija`**: Pretvara diskretne geometrijske točke iz planera u kontinuirane vremenske trajektorije (`trajectory_msgs/JointTrajectory`) poštujući maksimalne brzine ($1.0 \text{ rad/s}$) i ubrzanja ($2.0 \text{ rad/s}^2$) Dynamixel servo motora.
* **`Matematička Kinematika`**: Robustan modul za direktnu i inverznu kinematiku bez mogućnosti dijeljenja s nulom (korištenjem `atan2`).
* **`Koordinacija YOLO Vizije i Kalibracije`**: Učitava intrinzične parametre kamere i pretvara 2D bounding box detekcije u 3D koordinate baze robota $\{R\}$ (pri $Z=0$) koristeći ekstrinzičnu matricu.
* **`ROSA AI Agent`**: Napredni LangChain/ROSA autonomni agent koji interpretira prirodni jezik i pretvara naredbe poput *"Idi od car do traffic light i izbjegni cat"* u direktne robotske operacije.
* **`Cyber-Dark Glassmorphic GUI`**: Streamlit kontrolna ploča visoke estetike za manualno ispitivanje kinematike i praćenje rada autonomnog AI agenta u realnom vremenu.

---

## 📁 Struktura Paketa

```
sketch_terminator/
├── config/
│   ├── .env                           # OpenAI API ključ (zaštićen u .gitignore)
│   ├── camera_calibration_params.yaml # Intrinzični parametri kamere
│   ├── extrinsic_camera_params.yaml  # Ekstrinzična transformacijska matrica
│   ├── camera_params.yaml             # Parametri kamere i usb_cam čvora
│   └── controllers.yaml               # Konfiguracija ros2_control kontrolera
├── gui/
│   └── dashboard.py                   # Streamlit Cyber-Dark Web sučelje
├── launch/
│   └── sketch_terminator.launch.py    # Cjelovito pokretanje sustava
├── sketch_terminator/
│   ├── agent.py                       # ROSA agent tvornica i LLM konfiguracija
│   ├── agent_node.py                  # ROS 2 čvor agenta s podrškom za streaming
│   ├── integration_node.py            # Integracijski koordinator vizije i planiranja
│   ├── kinematics.py                  # Geometrijski DK/IK rješavač
│   ├── tools.py                       # LangChain robotski alati za ROSA agenta
│   └── trajectory_generator.py        # Generator glatkih i limitiranih trajektorija
├── test/
│   └── test_kinematics.py             # Test konzistentnosti kinematike
├── package.xml                        # ROS 2 deklaracija paketa i ovisnosti
├── setup.py                           # ROS 2 Python konfiguracija instalacije
├── setup.cfg                          # Instalacijski parametri
├── .gitignore                         # Git iznimke
└── README.md                          # Dokumentacija paketa
```

---

## 🔧 Brzo Pokretanje

### 1. Konfiguracija OpenAI ključa
Otvorite datoteku `config/.env` u paketu i dodajte svoj OpenAI ključ:
```env
OPENAI_API_KEY=sk-proj-...
```

### 2. Izgradnja paketa
```bash
cd /home/wsl/ros2_ws
colcon build --symlink-install --packages-select sketch_terminator
source install/setup.bash
```

### 3. Pokretanje cjelokupnog sustava
```bash
ros2 launch sketch_terminator sketch_terminator.launch.py
```
*Ova naredba automatski pokreće:*
1. **`path_planner_node`** (2D planer s izbjegavanjem prepreka).
2. **`integration_node`** (koordinator vizije, kalibracije i trajektorija).
3. **`agent_node`** (ROSA LLM agent čvor).
4. **Streamlit Web GUI** na `http://localhost:8501`.

---

## 📐 Manualna Verifikacija Kinematike

Za pokretanje izoliranog matematičkog testa direktne i inverzne kinematike pokrenite:
```bash
PYTHONPATH=/home/wsl/ros2_ws/src/sketch_terminator python3 /home/wsl/ros2_ws/src/sketch_terminator/test/test_kinematics.py
```
*Očekivani izlaz je uspješno podudaranje DK i IK izračuna s odstupanjem $0.000000\text{ m}$.*
