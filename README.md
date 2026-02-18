
# LGR Quasi-Static Lap Time Simulator 

## Purpose of this repo
Validate design decisions, and test parameters of Leeds Gryphon Racing FS25/26 car

## Description of the simulator 

This simulator is a Quasi-static, steady-state lap time sim. Fancy words. What do they mean?
- Quasi-static: Break track into small segments, at each segment, determine what's the max speed, propagate speeds forwards, and backwards, and we get a nice speed profile!
- Steady-state: We assert all forces to be resolved at any point in time. (Especially lateral and yaw balance)

A more in-depth description of the simulator can be found on the [wiki](https://github.com/LGRSimulations/LGR_FullTrackQSLapTimeSim/wiki/Description-of-Simulator)

## Setup
There's a few steps to get setup. Follow them in order and you should be fine.

### Prerequisites
In order to work with the software in this repo as painlessly as possible, the following steps will set up Windows Subsystem for Linux (WSL) so you can run development tools in a Linux environment, and then install packages within that environment.

1. Download your desired IDE (Integrated Development Environment)

I use [Visual Studio Code](https://code.visualstudio.com/), VS Code has everything you need really, and works fantastically with WS

2. In VS code, download the python extension (select 3.13.0 if given the choice)

3. (WINDOWS-ONLY) Install WSL using the [guide](https://learn.microsoft.com/en-us/windows/wsl/install), and reboot your pc.

The Ubuntu option is recommended for ease of use.

4. Download git

Download [git](https://git-scm.com/) for your operating system, which should be as simple as `sudo apt install git`.

4. Download uv
uv is a package manger for python, it simplifies running the code, and the python package dependencies.

run the following to install it: `curl -LsSf https://astral.sh/uv/install.sh | sh`


### Setting up your environment
The following instructions are if you use Visual Studio Code.

1. Clone the repository 

- In VS Code, click on source control on the left hand side of your screen. Then insert the following link to clone it when it prompts you to do so.
```https://github.com/LGRSimulations/LGR_FullTrackQSLapTimeSim.git```
- Navigate to the folder that you desire your code to be located.

OR 

- In your terminal (top bar of VS Code), navigate to where you want your code to be located, then type the following:

``` git clone https://github.com/LGRSimulations/LGR_FullTrackQSLapTimeSim.git```

2. Setup your environment

Run `uv sync` in the terminal.

### Running the lap time sim

- In your terminal (ensuring you're in the desired Virtual Env, which can be done with `uv sync`), type `python src/main.py`. This runs the simulator and you should see some pretty graphs.

#### Configure the simulator 

- Refer to `config.json`, this is where we choose the datasets we want to use for the lap time sim. 
- Refer to `datasets/vehicle/parameters.json`
- Datasets are found in the `datasets` folder. Datasets include Powertrain, Tyre, FSUK track data, etc.

#### Lap time sim structure


## 📬 Questions?

Open an issue or discussion! Ping me (Branson Tay) a Teams or LinkedIn message, am happy to discuss the simulator in more detail.