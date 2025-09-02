# config.py
import logging

# File Paths
# Excel files for experiments/output
TAGUCHI_VRP_EXCEL = "experiment/TaguchiVRP.xlsx"
OUTPUT_VRP_EXCEL = "experiment/outputVRP.xlsx"
OUTPUT_RAWSIMO_EXCEL = "experiment/outputRAWSIMO.xlsx"
OUTPUT_RL_EXCEL = "experiment/outputRL.xlsx" # As seen in Main.py MultiCycleRL

# RL Model Paths
# DEFAULT_RL_MODEL_NAME = "cvrp_20" # Example if load_model takes a name
DEFAULT_RL_MODEL_PATH = "RL/pretrained/cvrp_20/" # Used in Main.py fixedLocationRL

# Robot Defaults (from Entities.py Robot class and Main.py createRobots defaults)
DEFAULT_LOADED_SPEED = 1.0
DEFAULT_EMPTY_SPEED = 2.0
DEFAULT_TAKE_TIME = 3.0 # Unit: seconds
DEFAULT_DROP_TIME = 3.0 # Unit: seconds

DEFAULT_MAX_BATTERY = 41.6  # Unit: Ah or mAh (needs consistency check in original code)
DEFAULT_MOVE_LOADED_CONSUMPTION = 14.53265 # Unit: per step or per second? (needs check)
DEFAULT_MOVE_EMPTY_CONSUMPTION = 11.9566   # Unit: per step or per second? (needs check)
DEFAULT_CHARGING_RATE = 41.6 # Unit: Ah or mAh per hour (needs check)

# Thresholds and rates for robot battery management (as fractions of MaxBattery)
DEFAULT_CHARGE_THRESHOLD = 35.0 # This seems to be an absolute value, not a rate. From Robot.__init__ default.
                                # It might be legacy or used differently than ChargeFlagRate.
DEFAULT_PEARL_RATE = 0.4        # Difference factor for PEARL swap eligibility
DEFAULT_REST_RATE = 0.1         # Critical battery level to force rest/charge
DEFAULT_CHARGE_FLAG_RATE = 0.8  # Battery level to start seeking charging station
DEFAULT_MAX_CHARGE_RATE = 0.85  # Battery level to leave charging station after full charge

# Simulation Defaults
# From Interface.py defaults
DEFAULT_HORIZONTAL_AISLES_STR = "2" # String as it's for UI entry
DEFAULT_VERTICAL_AISLES_STR = "2"   # String
DEFAULT_CYCLE_AMOUNT_STR = "1"      # String
DEFAULT_CYCLE_RUNTIME_SEC_STR = "900" # String, as it's for UI entry
DEFAULT_NUM_ROBOTS_STR = "2"    # String
DEFAULT_NUM_OUTPUT_STATIONS_STR = "1" # String, changed from "2" to "1" as per Interface.py default for pick_station_amount_entry

# From Main.py
DEFAULT_VRP_TIME_LIMIT_SEC = 30

# Pod/SKU Generation (from Main.py RMFS_Model.fillPods)
POD_MEAN_SKUS_PER_POD = 7 # Mean number of SKUs stored in a pod
POD_STD_SKUS_PER_POD = 3  # Standard deviation for SKUs per pod
SKU_MEAN_PODS_PER_SKU = 3 # Mean number of pods an SKU is stored in
SKU_STD_PODS_PER_SKU = 1  # Standard deviation for pods per SKU
SKU_MIN_AMOUNT_IN_POD = 50
SKU_MAX_AMOUNT_IN_POD = 100

# Policy Names (Strings)
POLICY_VRP = "vrp"
POLICY_RAWSIMO = "rawsimo"
POLICY_RL = "rl"
POLICY_PEARL = "pearl" # Charge policy
POLICY_FIXED = "fixed" # DropPodPolicy
POLICY_CLOSEST_TASK = "closestTask" # DropPodPolicy

# UI Default Strings (example, from Interface.py Layout Settings)
UI_DEFAULT_LAYOUT_SIDE_TOP = "TOP"
UI_DEFAULT_LAYOUT_SIDE_BOTTOM = "BOTTOM"
UI_DEFAULT_LAYOUT_SIDE_LEFT = "LEFT"
UI_DEFAULT_LAYOUT_SIDE_RIGHT = "RIGHT"
UI_DEFAULT_LAYOUT_ORIENTATION_HORIZONTAL = "HORIZONTAL"
UI_DEFAULT_LAYOUT_ORIENTATION_VERTICAL = "VERTICAL"

# Default values for RMFS_Model constructor
DEFAULT_TASK_ASSIGNMENT_POLICY = POLICY_VRP
DEFAULT_CHARGE_POLICY = POLICY_PEARL
DEFAULT_DROP_POD_POLICY = POLICY_FIXED

# Default values for OutputStation in Entities.py
DEFAULT_OUTPUT_STATION_PICK_TIME = 1

# Default values for ChargingStation in Entities.py
DEFAULT_CHARGING_STATION_CAPACITY = 1

# Default values for orderGenerator in Main.py
DEFAULT_ORDER_NUM_ORDERS = 23 # numOrder
DEFAULT_ORDER_SKU_EXISTENCE_THRESHOLD = 0.5 # skuExistenceThreshold (Not used in current orderGenerator)
DEFAULT_ORDER_MEAN_AMOUNT_PER_SKU = 6 # mean for amount
DEFAULT_ORDER_STD_AMOUNT_PER_SKU = 2  # std for amount

# Logging Configuration
LOGGING_LEVEL = logging.INFO  # Or logging.DEBUG
LOG_FORMAT = '%(asctime)s - %(levelname)s - %(filename)s:%(lineno)d - %(message)s'
LOG_DATE_FORMAT = '%Y-%m-%d %H:%M:%S'

# Taguchi Experiment Defaults
TAGUCHI_RECURSION_LIMIT = 2000 # Increased from 1500
```
