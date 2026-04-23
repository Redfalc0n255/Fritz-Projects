**Fritz Phone**
A WiFi-based voice messaging system built with custom hardware and cloud infrastructure.
Created by Ben Fritz

**Overview**

Fritz Phone is a fully custom voice communication system designed to send and receive short audio messages between dedicated handheld devices. The project combines embedded systems, cloud services, PCB design, and hardware integration into a complete working communication network.

The goal of this project was to create a simple and tangible alternative to digital messaging by building a system where each user has a dedicated device. Messages are recorded locally, packaged on the device, transmitted through a cloud backend, and played back on recipient devices. The system was designed to be functional, reliable, and cleanly integrated across both hardware and software.

**Core Objectives**

Build a working device-to-device voice messaging system using ESP32 microcontrollers and Azure cloud services.
Integrate audio recording, local storage, wireless transfer, and playback into a compact embedded device.
Develop a cloud backend capable of receiving, storing, cataloging, and serving messages between registered devices.
Design and assemble reliable hardware using soldered connections, modular subsystems, and a custom PCB.
Create a polished and usable finished product that demonstrates full-stack hardware and software integration.

**System Architecture**

Device Controller: ESP32-based handheld units serve as independent communication nodes, managing button input, audio capture, playback, SD card interaction, and WiFi communication.

Audio System: INMP441 I2S microphone used for digital voice recording, paired with a MAX98357A I2S amplifier and speaker for message playback.

Local Storage: MicroSD card module used to store temporary recordings, system files, and audio assets required by the device.

Cloud Backend: Azure Function App processes device requests, stores WAV files, catalogs metadata, and handles message routing between devices.

Cloud Storage: Audio files are stored in Azure cloud storage, while message metadata such as sender, recipient, timestamps, and delivery state are managed separately.

User Interface: Four-button interface allows users to select recipients, record messages, and play received audio, with LED indicators providing feedback on system state.

**Component List**

1 ESP32 Dev Module: Central microcontroller responsible for device logic, WiFi connectivity, file handling, and I2S communication.

1 INMP441 Microphone: Digital I2S microphone used to capture voice recordings.

1 MAX98357A Audio Amplifier: Handles digital audio output to the speaker.

1 Speaker: Plays voice messages, prompts, and system tones.

1 MicroSD Card Module: Provides local storage for recordings, configuration files, and system audio.

4 Push Buttons: Form the primary user interface for selecting recipients, recording, and playback.

2 Status LEDs: Indicate boot state, message activity, send/receive events, and notification status.

1 Custom PCB: Integrates the major electrical connections into a cleaner and more permanent hardware layout.

1 Power Input System: Provides power to the handheld device and supports continuous operation during use.

**Firmware**

The device firmware was written in Arduino/C++ and handles the core embedded functionality of the system. This includes recording audio from the microphone, storing and reading WAV files from the SD card, managing button interactions, controlling status LEDs, connecting to WiFi, and communicating with the cloud backend.

The firmware also manages the message flow on the device side, including packaging outgoing messages and downloading incoming messages for playback.

**Cloud Backend**

The backend was developed using Azure Functions with C# and .NET. It is responsible for receiving uploaded audio from each device, storing WAV files in the cloud, maintaining metadata for every message, and serving pending messages back to recipient devices.

This allows the Fritz Phone system to operate as a connected network of dedicated communication devices rather than a standalone audio recorder.

**Project Structure**

fritz-phone/
├── firmware/ ESP32 firmware and device logic
├── cloud/ Azure Functions backend
├── hardware/ PCB files and hardware design assets
├── docs/ Supporting documentation
├── images/ Project photos and media
└── README.md

**Images**

For images of the project hardware and build, see the files in the images/ folder.

Suggested image references:






**Notes**

This project was designed as a small-scale communication system between a limited number of dedicated devices. The architecture prioritizes simplicity, reliability, and clean integration between embedded hardware and cloud infrastructure.

It serves as a complete demonstration of hardware design, firmware development, cloud communication, and end-to-end system engineering in a single project.

Author

Benjamin Fritz

Built as a personal engineering project focused on embedded systems, cloud architecture, PCB integration, and real-world hardware/software communication.Here is the readme
