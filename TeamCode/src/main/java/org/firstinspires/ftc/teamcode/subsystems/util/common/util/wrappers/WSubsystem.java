package org.firstinspires.ftc.teamcode.subsystems.util.common.util.wrappers;

import com.arcrobotics.ftclib.command.SubsystemBase;

public abstract class WSubsystem extends SubsystemBase {

    public abstract void periodic();
    public abstract void read();
    public abstract void write();
    public abstract void reset();

}