# Project Description "fg_bayesian_pid"
The ***"fg_bayesian_pid"*** project is a simulation-based implementation of the Bayesian algorithm for optimizing the coefficients of the PID controller for control systems. The project uses the ```FlightGear simulator``` and Python libraries ```scikit-optimize``` and ```flightgear-python``` for optimization and simulator interaction, respectively.

## Project Files
+ ### ```bayesian_pid_optimization_X.py```

This file contains the implementation of the optimization algorithm for the PID controller coefficients based on the Bayesian optimization method. The ```gp_minimize``` function from the ```scikit-optimize``` library is used for optimization, which allows solving optimization problems using Gaussian processes.
<br>

+ ### ```pid_control_hierarchy_X.py```

This file contains the implementation of a hierarchical structure of PID controllers for control systems. The hierarchical structure allows solving control problems at different levels of complexity, while ensuring smooth switching between levels.
