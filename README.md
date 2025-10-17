# 🛰️ UWB Indoor Localization Simulation

This project demonstrates **indoor localization** using **Ultra-Wideband (UWB)** technology through a fully simulated environment in Python.  
It shows how a drone’s position can be estimated using **multilateration** from fixed anchors — without any real hardware connection.

---

## 🎬 Simulation Overview

- **Blue Dot:** True drone position (simulated flight path)  
- **Green Dot:** Estimated position using noisy UWB range data  
- **Red Xs:** Fixed UWB anchors  

The system estimates the drone’s position in real-time and visualizes it as an animation.  
If `imageio` is installed, it saves the simulation as a GIF (`uwb_simulation.gif`).  
Otherwise, it displays a live Matplotlib animation window.

---

## 🧠 Concept

UWB (Ultra-Wideband) positioning relies on **time-of-flight** signals between transmitters (anchors) and receivers (the drone).  
By measuring distances to 3 or more anchors and solving via **least squares**, the drone’s position is estimated in 2D.

---

## ⚙️ How to Run

### 1️⃣ Install the required packages
Make sure you have Python installed (>= 3.8), then run:

```bash
pip install -r requirements.txt
# uwb indoor localisation
Simulated indoor localization using UWB and multilateration (Python)
