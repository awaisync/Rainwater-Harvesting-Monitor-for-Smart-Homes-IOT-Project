🌧️ Rainwater Harvesting Monitor for Smart Homes

A low-power, solar-powered IoT monitoring node for domestic rainwater harvesting systems.
The device measures water level, temperature, and humidity and provides both local BLE access and long-range LoRaWAN connectivity, while operating autonomously from harvested solar energy.

📌 Key Features

Ultrasonic water-level sensing (HC-SR04)

Temperature and humidity sensing (SHT40)

Dual wireless communication:

Bluetooth Low Energy (BLE) for local access

LoRaWAN for cloud integration

Solar-powered operation using:

Energy harvester (AEM10941 / BQ25570)

Supercapacitor storage

Interrupt-driven firmware architecture

STOP2 low-power mode for minimal energy consumption

On-demand BLE updates via push-button (PB0)

Periodic LoRaWAN uplinks via RTC scheduling

🧠 System Overview

The firmware runs on an STM32L412 microcontroller and follows a wake–execute–sleep model:

RTC wake-up → sensor readout + LoRaWAN uplink

PB0 interrupt → BLE transmission of cached data

STOP2 mode → between all events

No polling loops are used; all actions are interrupt-driven to minimise power usage.

🧰 Hardware Components
Component	Purpose
STM32L412KBT6	Main microcontroller
SHT40	Temperature & humidity sensor
HC-SR04	Ultrasonic distance sensor
Seeed Wio-E5	LoRaWAN module
Seeed XIAO nRF52840	BLE module
Solar panel	Energy source
Energy harvester	Power management
Supercapacitor	Energy storage
📁 Repository Structure
/Core
 ├── Src/main.c
 ├── Inc/main.h
/Docs
 ├── Figures
 ├── Measurements

🚀 Getting Started

For full setup instructions, flashing, wiring, and testing:

👉 See the Wiki:
Getting Started
Firmware Architecture
Power Management

🔋 Power Management Strategy

STOP2 mode between events

RTC-based scheduling for periodic uplinks

Cached sensor values for BLE

No RTOS or polling

Energy harvesting with supercapacitor storage

🛠️ Future Work

Supercapacitor voltage sensing (ADC)

Context-aware transmission scheduling

Hardware-level power gating

DMA-based UART handling

📄 License

This project is intended for academic and educational use.

📬 Contact

For questions or contributions, open an Issue on GitHub.
