# Arduino Simple 2-Axis Robot Hand

An accessible, responsive, dual-axis robotic mechanical hand driven by an Arduino microcontroller. This project utilizes a heavy-duty stepper motor to handle 360-degree shoulder rotation and an agile micro-servo to manage precise vertical (up/down) articulation, all controlled via an analog 2-axis joystick module.

---

## ⚠️ CRITICAL POWER WARNING (Read Before Wiring!)

**DO NOT power the motors directly from the Arduino's onboard 5V pin.** 

The combined current draw of the 28BYJ-48 stepper motor and the SG90 servo motor under load exceeds the current limits of the Arduino's onboard voltage regulator. Attempting to connect all three modules (Stepper, Servo, and Joystick) to the Arduino's 5V line will drop the voltage, causing the Arduino to shut down completely, enter a boot loop, or suffer permanent hardware failure.

### 🔌 The Isolated Power Strategy:
- **Motors (High Current):** Splice open a spare USB power cable and route its raw 5V and GND lines to power rail tracks on a breadboard. Connect both the servo and stepper driver power lines strictly to this external USB supply rail.
- **Joystick (Low Current):** Because the joystick draws negligible current, it can safely remain connected directly to the Arduino's original built-in 5V pin.
- **Common Ground Requirement:** You **MUST** run a jumper wire between the Arduino's `GND` pin and the external USB power source's `GND` line on your breadboard. Without this common ground, the signal circuits will lack a reference point and the motors will jitter uncontrollably.

---

## 📌 Complete Hardware Wiring Matrix

### 1. 28BYJ-48 Stepper Motor (Shoulder Axis)
Connect your stepper motor to its ULN2003 driver board, then wire the driver pins to the Arduino and external power line:


| ULN2003 Driver Pin | Connection Target | Function / Description |
| :--- | :--- | :--- |
| **IN1** | **Arduino D2** | Stepper Phase A Control Input |
| **IN2** | **Arduino D3** | Stepper Phase B Control Input |
| **IN3** | **Arduino D4** | Stepper Phase C Control Input |
| **IN4** | **Arduino D5** | Stepper Phase D Control Input |
| **GND** | **External USB GND** | High-Current Ground Return |
| **5V / VCC** | **External USB 5V** | High-Current Power Input |

### 2. SG90 Servo Motor (Vertical Lift Axis)

| SG90 Servo Wire Color | Connection Target | Function / Description |
| :--- | :--- | :--- |
| **Brown Wire** | **External USB GND** | Ground Return |
| **Red Wire** | **External USB 5V** | 5V Main Power Input |
| **Orange/Yellow Wire** | **Arduino D6** | PWM Position Signal Input |

### 3. Analog Joystick Module (Manual Controller)

| Joystick Pin | Connection Target | Function / Description |
| :--- | :--- | :--- |
| **GND** | **Arduino GND** | Logic Reference Ground |
| **VCC** | **Arduino 5V** | Logic Power Input (Safe for Arduino) |
| **VRx** | **Arduino A0** | Horizontal Analog Input (Shoulder Control) |
| **VRy** | **Arduino A1** | Vertical Analog Input (Lift Control) |

---

## ⚙️ Software Requirements & Dependencies

To load and test the code inside this repository, verify your IDE setup:
1. **Arduino IDE** installed on your workstation.
2. Built-in libraries used:
   - **`Servo.h`** (Handles standard angle calculation pulse generation for the SG90).
   - **`Stepper.h`** (Manages the 4-phase driving sequence for the 28BYJ-48).

---

## 🕹️ Operational Mechanics

* **Shoulder Rotation Loop:** Moving the joystick left or right on the **VRx (A0)** axis instructs the `Stepper` library to step the 28BYJ-48 forward or backward, allowing smooth 360-degree rotation.
* **Vertical Articulation Loop:** Pushing the joystick up or down on the **VRy (A1)** axis maps the analog reading (0 to 1023) down into degrees (0 to 180) to update the absolute angle position of the SG90 servo instantly.
