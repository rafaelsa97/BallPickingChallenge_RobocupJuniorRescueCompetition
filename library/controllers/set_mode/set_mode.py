# Add this line to your world file:
# Supervisor { controller "set_mode" }

from controller import Supervisor

s = Supervisor()

s.simulationSetMode(Supervisor.SIMULATION_MODE_REAL_TIME)
