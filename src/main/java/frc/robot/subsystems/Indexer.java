// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    public Indexer() {}

    public boolean newBall() {
        return false;
    }

    public boolean getIndexerTopSensor() {
        return false;
    }

    public boolean getIndexerBottomSensor() {
        return false;
    }

    public boolean getIntakeSensor() {
        return false;
    }
}