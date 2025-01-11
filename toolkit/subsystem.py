import commands2


class Subsystem(commands2.Subsystem):
    """
    Extendable subsystem class. Needs to be extended by every subsystem.
    """

    def init(self):
        """
        Overridable method for initializing the subsystem. Place motor inits/other physical initialization here,
        for example re-zeroing a gyro.
        """
        ...
