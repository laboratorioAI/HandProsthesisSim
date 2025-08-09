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
  git clone https://github.com/laboratorioAI/HandProsthesisSim.git
  cd prostheticHandDigitalTwin
```

### Install

Build the C# communication bridge:
  - Open the Visual Studio solution file named BufferPrint.sln located in "..\ProsteticHandDigitalTwin\BufferPrint\Bufferprint.sln" 
  - Two projects may open "BufferManager" and "ProthesisSimulation" in the solution explorer. For each of them, right-click on its "Autodesk.Inventor.Interop" element above the References node and remove it (suprim). Next, right-click the References node and select "Add Reference". This will open a dialog box where you can find the button "Search...". This will open a system explorer window where you can look for the Inventor interoperability library (usually located under C:\Program Files\Autodesk\Inventor 2026\Bin\Autodesk.Inventor.Interop.dll). Finally, accept and add the library to the reference of each project.
  - Right-click at the root of your project and select "rebuild" the solution. This creates an executable called "BufferPrint.exe" under "..\ProsteticHandDigitalTwin\BufferPrint\bin\Debug\Bufferprint.exe". Copy this path as it will be used later.

Check dependencies
  - Autodesk Inventor should be running with an assembly loaded before starting the bridge. You must open the assembly called "RightHandSimpleFinger.iam" located at "../ProsteticHandDigitalTwin/Sim Models". If you are shown a message showing that there is any part missing, then look at the folder "Sim Models" where you will find any parts that may could go missing.
  - All coordinate systems and UCS names used in the Inventor API must match those expected by the MATLAB simulation. In order to get the exact names, the Operating System may be in English so that Inventor opens in English and default names are loaded making it posible for the code to identify joint names.

Open the simulation
  - Start MatLab and open the folder "../ProsteticHandDigitalTwin/Digital Twin" as working path.
  - You may find the file named "HandIntegrator.m". You must change the path in the first line of this code to the one you copied earlier (the path where the compiled library was created)
  - Open the file named "FingerController.m". On line 22 of this file, you must also paste the path where the compiled library was created
  - Open the file "HandIntegrator.m" again and press play to run the default simulation. This will start the dynamics engine and simulate the movement of the prosthetic fingers based on torque and gravitational models.

### Usage
You can now check the Inventor's window to see how the model moves by the action of the default torque hard-coded on the simulator's default configuration.

- To plot the results after the simulation, uncomment on each finger:
  plotTripleFinger()
  plotFingerResults()

- To start your journey with the simulator, you can check the config files for each finger, where you can change most of the parameters considered for the simulation
- If you want to check the dynamics behind the simulation, you can check the files starting with "torque" in their name to see how torques are calculated for each phalanx of each considered finger model: Thumb or Index (generic)

- You can find the folder "Python Inertia Calculation" with the code used to check the inertia calculated using the Inventor API. You may need to export the .STP file, for example, from a planar model from Inventor, and use it to calculate its inertia values with the code you may find in this folder.

- If you want to use your control algorithm (AI agent, i.e), you must start understanding "HandIntegrator.m" file. This file incorporates objects representing the dynamics (Physics) implementation, controllers, and error calculation regarding the finger base models considered in the project. You may then use MatLab language to implement your implementation of control agent. Specifically, you can put special attention on time discretization, variable initialization, "run" method, and values stored.
- Otherwise, if you want to use another programming language, you still can do it, but you may have to load the compiled library (we talked about earlier) into your code to make the digital twin platform available to your code. In this case, you must check the original code on how it handles communication and write your version of this logic.


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

- GitHub: [https://github.com/alexansaa](https://github.com/alexansaa)
- LinkedIn: [https://www.linkedin.com/in/alexander-saavedra-garcia/](https://www.linkedin.com/in/alexander-saavedra-garcia/)

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
