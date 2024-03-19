# Task conditions

You will find here a complete list of the conditions that you can use for your tasks. 

The implementation of these conditions can be found in the file `raubase_ros/plan/conditions.py` in the package.

## Interfaces 

All of the conditions inherits from a common interface **TaskCondition**:

```Python
class TaskCondition:
    def test(self) -> bool
```

But since some conditions could be only made starting a task or stopping a task, two other interfaces have been added to organize them:

```Python
class StartTaskCondition(TaskCondition):
    pass
```

```Python
class StopTaskCondition(TaskCondition):
    pass
```

!!! warning 

    These three interfaces should not be used in tasks condition, they only provide a solid baseline for all others ones.

!!! info 

    If you want to implement you own conditions, you can make one extending from one the three interface, depending of the needs.


## General purpose conditions:

### Never

```Python
class Never(TaskCondition)
```

This condition prevent any start or stop, depending on where it is used. This condition can be useful in cases where you want a continous sub routine when couple with a **Parallel** task.


### AND and OR conditions

```Python
class ANDConditions(TaskCondition):
    def __init__(self, List[TaskCondition]) -> None

class ORConditions(TaskCondition):
    def __init__(self, List[TaskCondition]) -> None
```

This two conditions are meta-conditions providing a way to use several conditions at once. Obviously we have:

  - AND condition: all condition should be evaluated as True to be True
  - OR condition: if one of the condition is fulfilled, then evaluate to True

An example of how they can be used: 

```Python
    ...
    def start_conditions(self) -> StartConditions:
        return ANDConditions([
            ConditionA(),
            ConditionB(),
            ...
        ])
    ...
```

## Start conditions specifics

### Follow previous task

```Python
class FollowPreviousTask(StartTaskCondition):
```

This condition will always launch the task when asked. Basically, it's always True. It's the opposite of the **Never** condition.

## Stop conditions specifics

### As soon as possible

```Python
class AsSoonAsPossible(StopTaskCondition):
```

This condition will make the plan stop as soon as possible. In the end it will make the task run only time.

