import commands2


class SubsystemBase(commands2.SubsystemBase):
    """
    All subsystems should inherit from this class for correct usage in commands.
    """
    def __init__(self):
        super().__init__()