from typing import List, Callable


class TaskCondition:
    """
    Bare condition for the tasks
    """

    def test(self) -> bool:
        raise RuntimeError("Using abstract task condition")


class StartTaskCondition(TaskCondition):
    pass


class StopTaskCondition(TaskCondition):
    pass


class FlowTaskCondition(TaskCondition):
    pass


# =============================================================================
class Never(FlowTaskCondition):
    """
    Condition that is always False.
    """

    def test(self) -> bool:
        return False


class FollowPreviousTask(StartTaskCondition):
    """
    Condition specialization: execute right after the previous task
    """

    def test(self) -> bool:
        return True


class AsSoonAsPossible(StopTaskCondition):
    """
    Ask to stop as soon as possible
    """

    def test(self) -> bool:
        return True


# =============================================================================
class ANDConditions(FlowTaskCondition):
    def __init__(self, conditions: List[TaskCondition]) -> None:
        super().__init__()
        self.conditions = conditions

    def test(self) -> bool:
        for c in self.conditions:
            if not c:
                return False
        return True


class ORConditions(FlowTaskCondition):
    def __init__(self, conditions: List[TaskCondition]) -> None:
        super().__init__()
        self.conditions = conditions

    def test(self) -> bool:
        for c in self.conditions:
            if c:
                return True
        return False


# =============================================================================
class OnValue(StopTaskCondition):
    """
    Test on a value (bool) condition.

    Params:
        - value_callback: a callback (or lambda) returning a bool

    For example, one can use this condition like this:

    class CustomTask(BaseTask):
        def __init__(self):
            self.stop = False
            self.stop_cond = OnValue(lambda: self.stop)
        def stop_conditions(self) -> StopCondition:
            return self.stop_cond
    """

    def __init__(self, value_callback: Callable[[], bool]) -> None:
        super().__init__()
        self.value = value_callback

    def test(self) -> bool:
        return self.value()
