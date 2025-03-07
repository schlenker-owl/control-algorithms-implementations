# Initiate the project
Download the git repo

1. `cd path/to/project/src`
2. `python3 -m venv venv`
3. `source venv/bin/activate`
4. `pip install -r requirements.txt`


# Run the Control algorithm Examples
1. `python src/run_pid_example.py` # Simple Proportional-Integral-Derivative (PID) controller.
2. `python src/run_p_example.py`   # Simple Proportional (P) controller.
3. `python src/run_pd_example.py`   # Simple Proportional-Derivative (PD) controller.
4. `python src/run_pi_example.py`   # Simple Proportional-Integral (PI) controller.
5. `python src/run_zn_example.py`   # Simple Ziegler-Nichols Tuning on PID controller.
6. `python src/run_lead_lag_example.py`   # Simple Lead Lag controller.
7. `python src/run_state_space_example.py`   # Simple State Space controller.