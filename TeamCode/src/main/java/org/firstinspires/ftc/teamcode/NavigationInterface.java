package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.FieldPlacement;

public interface NavigationInterface {

    public void activate();
    public void stop();
    public FieldPlacement getPlacement();
    public NavigationTypesEnum getType();

    //***//
    public FieldPlacement getSkyStone(String skyStoneTargetName);
}
