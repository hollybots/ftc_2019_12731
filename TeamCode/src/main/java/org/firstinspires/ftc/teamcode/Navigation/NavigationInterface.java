package org.firstinspires.ftc.teamcode.Navigation;

import org.firstinspires.ftc.teamcode.Utils.FieldPlacement;
import org.firstinspires.ftc.teamcode.Utils.NavigationTypesEnum;

public interface NavigationInterface {

    public void activate();
    public void stop();
    public FieldPlacement getPlacement();
    public NavigationTypesEnum getType();

    //***//
    public FieldPlacement getTarget(String targetName);

    public boolean isActive();
}
