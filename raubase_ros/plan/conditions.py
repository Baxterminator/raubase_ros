from typing import List


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


# =============================================================================
class Never(TaskCondition):
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
class ANDConditions(TaskCondition):
    def __init__(self, conditions: List[TaskCondition]) -> None:
        super().__init__()
        self.conditions = conditions

    def test(self) -> bool:
        for c in self.conditions:
            if not c:
                return False
        return True


class ORConditions(TaskCondition):
    def __init__(self, conditions: List[TaskCondition]) -> None:
        super().__init__()
        self.conditions = conditions

    def test(self) -> bool:
        for c in self.conditions:
            if c:
                return True
        return False
