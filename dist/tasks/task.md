# Make a custom task

This package define a simple to implement tasks to be run by the [behavior plan](/plans/behavior). Each task should be an anwser to a certain problem, and may yse a state machine for that. For example, in the Robocup, one task should be made for one challenge only. 

The base template for a task is the following:

```Python
from raubase_ros.plan import BaseTask
from raubase_ros.plan.conditions import StartTaskCondition, StopTaskCondition
from raubase_ros.plan.data import Requirement

class TestTask(BaseTask):
    def __init__(self) -> None:
        super().__init__()

    def requirements(self) -> Requirement:
        pass

    def start_conditions(self) -> StartTaskCondition:
        pass

    def stop_conditions(self) -> StopTaskCondition:
        pass

    def loop(self) -> None:
        pass
```

A task is a new class that extends the **BaseTask** interface. Thus, it should implement some interface methods that will be explained in the following sections. However we can note that these methods are splitted in two categories:

  - **Declarative**: provide informations on the requirements, as well as the start and stop condition,
  - **Working**: provide the actual task computation excecuted at each plan update.

## Task requirements

One of the first things to declare is the requirements, i.e. what components the task will use (e.g. the camera, the ArUco results, the movement of the robot, ...). You can find a complete list of the available requirements [here](/python_api/requirements).

For example, if you task need to read the distance sensors and move the robot accordingly, the methods would be filled like:

```Python
    ...
    def requirements(self) -> Requirement:
        return Requirement.MOVE | Requirement.DISTANCE

    ...
```

## Start and stop conditions

These two methods declare when to start and when to stop a task and go to the next one. Again, you can find a complete list of the possible conditions [here](/python_api/conditions). For example, if you want to start right after the previous task you can use the following method:

```Python
    ...
    def start_conditions(self) -> StartTaskCondition:
        return FollowPreviousTask()
    ...
```

## Runtime loop

### Structure of the function

The function  `loop()` of the task will be called regularly by the [behavior plan](/plans/behavior). It should then be written in the shape of a state or a stateless machine depending on the use-case. Implementing a simple state machine in Python is easy:

```Python
from enum import Enum, auto

class State(Enum):
    STATE_1 = auto()
    STATE_2 = auto()
    STATE_3 = auto()
    ...

def StateTask(BaseTask):
    def __init__(self):
        super().__init__()
        self.state = State.STATE_1
    
    ...

    def loop():
        match self.state:
            case State.STATE_1:
                # Do your state 1 action here

                if c1():
                    self.state = State.STATE_2
            case State.STATE_2:
                # Do your state 2 action here

                if c2():
                    self.state = State.STATE_3
            
            ...
```

### Functions and data available

With the requirements, you declared functionnalities you wanted to have. We will now talk on how to use them. This is working with two categories:

  - **Shared Data**, i.e. all results and sensors data available from the requirements of all tasks. These data can be accessed in the variable `self.data.<data name>`.
  - **Shared Control**, i.e. methods you can use to communicate with some parts of the Raubase Software. These methods are gathered under the field `self.control.<function name>`

For example, if you want to read encoders values and then move, you can write:

```Python
    ...
    def requirements(self) -> Requirement:
        # Need these requirement to work
        return Requirement.ENCODER | Requirement.MOVE 

    ...
    def loop(self) -> None:
        # Read right encoder data
        right_encoder = self.data.encoders.right

        # Move at 1 m/s straight ahead
        self.control.move_v_w(1.0, 0.0)

```

You can find the available data [here](/python_api/shared_data), and commands [here](/python_api/shared_cmd).