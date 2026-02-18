
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
1. Download git

Download [git](https://git-scm.com/) for your operating system 
This is the industry-standard Version Control software. There are others but git is the most popular. Ensure you click the option that adds to PATH!!!

2. Download your desired virtual environment manager

I have been recommending using Conda, specifically [miniconda](https://www.anaconda.com/download/success), select the RIGHT option where it says miniconda.

3. Download your desired IDE (Integrated Development Environment)

I use [Visual Studio Code](https://code.visualstudio.com/), VS Code has everything you need really.

4. In VS code, download the python extension (select 3.13.0 if given the choice)

### Setting up your environment
The following instructions are if you use Visual Studio Code, and miniconda.

1. Clone the repository 

- In VS Code, click on source control on the left hand side of your screen. Then insert the following link to clone it when it prompts you to do so.
```https://github.com/LGRSimulations/LGR_FullTrackQSLapTimeSim.git```
- Navigate to the folder that you desire your code to be located.

OR 

- In your terminal (top bar of VS Code), navigate to where you want your code to be located, then type the following:

``` git clone https://github.com/LGRSimulations/LGR_FullTrackQSLapTimeSim.git```

2. Setup your environment

We will use conda to setup your environment. 

- Make sure your VS Code has your desired repo open.
- In your terminal, ensure conda is installed correctly by typing `conda`, you should see some commands from conda appear.
- type ```conda create -n test_env python=3.13.0 anaconda```, this creates a new virtual environment. (Note that test_env is your environment name, up to you to name it)
- To see what envs you have in your machine, type `conda info --envs`, you can also see where the interpreter path here.
- Once the environment is created, type ```conda activate test_env``` to activate the environment.
- In visual studio code, use the shortcut `ctrl + shift + p` and select option "Python: Select Interpreter"
- Select your aforenamed environment
- Voila, you have set up your virtual environment for development!

Note: If at any point in time, VS Code doesn't seem to have what I stated above, close all VS code windows, and try again. You might face the issue where VS Code's terminal isn't recognising Conda. In that case, type `cmd` in your windows search bar and use the command prompt to perform all of the above instead.

### Running the lap time sim

- In your terminal (ensuring you're in the desired Virtual Env), type `pip install -r requirements.txt`. This downloads all the required libraries & packages
- In your explorer (top left of VS Code) find the `src/main.py` file
- Run the main.py file.
- Let the simulator do its magic, and you should get some pretty plots after!

#### Configure the simulator 

- Refer to `config.json`, this is where we choose the datasets we want to use for the lap time sim. 
- Refer to `datasets/vehicle/parameters.json`
- Datasets are found in the `datasets` folder. Datasets include Powertrain, Tyre, FSUK track data, etc.

#### Lap time sim structure

### Running The Parameter Sweeper – sweep.py

Run a lap time simulation while sweeping a single vehicle parameter over a range of values.

#### Usage

From the project root:

``` bash
python src/sweep.py <param> <values> [--steps N]
```

#### Arguments

- `<param>`
  Name of the vehicle parameter to sweep (must exist in vehicle.params).

- `<values>`  
  Comma-separated values:
  - Range → generates evenly spaced values between min,max  
    Example: 250,350

- `--steps N` (optional)  
  Number of points in the sweep (default: 5).


#### Examples

Sweep from 250 to 350 with default steps (5) to data.csv
```bash
python src/sweep.py mass 250,350 --output data.csv
```

Sweep with custom steps
```bash
python src/sweep.py mass 250,350 --steps 10
```

#### Output

For each parameter value:
- Loads track from config.json
- Creates vehicle from config
- Runs run_lap_time_simulation
- Prints lap time results in the format:
```
mass = 250.00, Lap Time = 72.31 s
```
#### Notes

- The parameter must exist in vehicle.params
- Track path and vehicle configuration are read from config.json


## 📬 Questions?

Open an issue or discussion! Ping me (Branson Tay) a Teams or LinkedIn message, am happy to discuss the simulator in more detail.