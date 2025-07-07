<a name="readme-top"></a>

# 📗 Table of Contents

- [📖 About the Project](#about-project)
  - [🛠 Built With](#built-with)
    - [Tech Stack](#tech-stack)
    - [Key Features](#key-features)
- [💻 Getting Started](#getting-started)
  - [Setup](#setup)
  - [Prerequisites](#prerequisites)
  - [Install](#install)
  - [Usage](#usage)
  - [Run tests](#run-tests)
  - [Deployment](#deployment)
- [👥 Authors](#authors)
- [🔭 Future Features](#future-features)
- [🤝 Contributing](#contributing)
- [⭐️ Show your support](#support)
- [🙏 Acknowledgements](#acknowledgements)
- [📝 License](#license)

# 📖 [Prosthetic Hand Mechanical Simulator] <a name="about-project"></a>

**[Prosthetic Hand Mechanical Simulator]**
This project presents the implementation of a dynamic simulator for a prosthetic hand, based on a modular digital twin architecture. The simulation models finger motion using second-order nonlinear differential equations, incorporating tendon torque, gravity effects, and hard-stop joint constraints. The system integrates:

- **MATLAB**, used for real-time numerical simulation (Runge-Kutta + error estimation),
- **Autodesk Inventor**, which provides 3D CAD models and inertia data,
- **C#**, acting as an inter-process bridge through shared memory for IPC.

This platform supports control strategy testing, visualization of kinematic behavior, and extension to intelligent agent-based prosthetic control.

## 🛠 Built With <a name="built-with"></a>

### Tech Stack <a name="tech-stack"></a>

<details>
  <summary>Core Technologies</summary>
  <ul>
    <li><a href="https://www.mathworks.com/products/matlab.html">MATLAB</a></li>
    <li><a href="https://www.autodesk.com/products/inventor/">Autodesk Inventor</a></li>
    <li><a href="https://learn.microsoft.com/en-us/dotnet/csharp/">C# (.NET)</a></li>
  </ul>
</details>

### Key Features <a name="key-features"></a>

- 💡 **Digital Twin Architecture** with modular design
- 🎯 **Dynamic Model Simulation** using Newton-Euler formulation
- 🔄 **MATLAB–Inventor Real-Time Communication** via shared memory & mutex
- 🎥 **Live 3D Visualization** based on dynamic joint data
- 📊 **Torque, Angular Position, and Error Estimation** with RK4 solver

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## 💻 Getting Started <a name="getting-started"></a>

To get a local copy up and running, follow these steps.

### Prerequisites

- MATLAB R2022a or newer (with access to `ode45`, plotting, JSON parsing)
- Autodesk Inventor (Professional version recommended)
- Visual Studio (for C# IPC bridge)
- Windows OS

### Setup

Clone this repository to your desired folder:

```sh
  git clone https://github.com/alexansaa/prostheticHandDigitalTwin
  cd prostheticHandDigitalTwin
```

### Install

Install the MATLAB components:

  - Open MATLAB and add the source code folder to the path using:
addpath(genpath('matlab_src'))
  - Make sure that your MATLAB version supports JSON parsing and custom function scripts. The function runSimulation.m is the main entry point.

Build the C# communication bridge:
  - Open the Visual Studio solution file named InventorBridge.sln located in the cs_ipc_bridge folder.
  - Build the solution in Release mode. This creates an executable called InventorBridge.exe.
  - Ensure Autodesk Inventor is installed and that the correct references are set in the C# project. The Inventor API must be accessible from your system.

Check dependencies
  - Autodesk Inventor should be running with a model loaded before starting the bridge.
  - All coordinate systems and UCS names used in the Inventor API must match those expected by the MATLAB simulation.

### Usage

To run the simulation:
- Open MATLAB and execute the following command:
  runSimulation()
  
This will start the dynamics engine and simulate the movement of the prosthetic fingers based on torque and gravitational models.
- To plot the results after the simulation, use:
  plotResults()
  
To enable the real-time communication and visualization:
- Run the InventorBridge.exe application built from the Visual Studio project.
- This bridge uses shared memory and a named mutex to receive joint angles from MATLAB and send them to Autodesk Inventor.
- The Inventor model will update in real-time based on the simulation output.

### Usage
There is a unit test for torque computation available in the test folder. To run it:
- In MATLAB, execute the following script:
testTorqueComputation.m
- This test verifies that torque at joint-3 of the thumb is computed correctly using the current tendon tension and gravitational vector.


### Deployment

This project is currently designed for academic and simulation environments only.

Future deployment to real embedded platforms (microcontrollers, actuators, sensors) is planned but not included.

For a real-world prototype, you would need:

- Torque-controlled motors with feedback
- Sensor input (e.g., EMG signals)
- Embedded code running on a real-time OS
- Safety mechanisms to deal with over-torque or misalignment

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- AUTHORS -->

## 👥 Authors <a name="authors"></a>

👤 **Alexander**

- GitHub: [GitHub](https://github.com/alexansaa)
- LinkedIn: [LinkedIn](https://www.linkedin.com/in/alexander-saavedra-2803b1b6/)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- FUTURE FEATURES -->

## 🔭 Future Features <a name="future-features"></a>

- [ ] **[Integration of EMG signal acquisition to drive the prosthesis using real muscular input.]**
- [ ] **[Control strategy improvements using PID or reinforcement learning agents.]**
- [ ] **[Dynamic analysis including energy loss, damping, and material deformation.]**
- [ ] **[Export simulation models to embedded C code for real-time hardware execution.]**

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTRIBUTING -->

## 🤝 Contributing <a name="contributing"></a>

Contributions, issues, and feature requests are welcome!

Feel free to check the [issues page](https://github.com/alexansaa/ProsteticHandDigitalTwin/issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## ⭐️ Show your support <a name="support"></a>

If you like this project, please give it a star on GitHub

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## 🙏 Acknowledgments <a name="acknowledgements"></a>

Special thanks to the Alan Turing Laboratory at Escuela Politécnica Nacional
for providing guidance, resources, and inspiration throughout the project.
This simulator was developed as part of a graduation thesis in Computer Science Engineering.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- LICENSE -->

## 📝 License <a name="license"></a>

This project is [GNU](./LICENSE.md) licensed.

<p align="right">(<a href="#readme-top">back to top</a>)</p>
