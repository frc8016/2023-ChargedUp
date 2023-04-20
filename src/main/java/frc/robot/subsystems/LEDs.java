// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase {
  private AddressableLED m_led = new AddressableLED(LEDConstants.LED_PORT);

  private AddressableLEDBuffer m_ledBuffer =
      new AddressableLEDBuffer(LEDConstants.TOTAL_LED_LENGTH);

  private int m_LEDRed;
  private int m_LEDGreen;
  private int m_LEDBlue;

  /** Creates a new LEDs. */
  public LEDs() {
    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  /* This file assumes that the LEDs are wired in this arrangement:
   *        |\      |-------------------|      /|
   *        | \     |                   |     / |
   *        |  \    |                   |    /  |
   *        |   \   |                   |   /   |
   * LEFT   |    \  |                   |  /    |   RIGHT
   *        |     \ |                   | /     |
   *        |     BACK                 BACK     |
   *        |                                   |
   *        |  <---- Wired to RoboRIO           |
   *      FRONT                               FRONT
   *
   * Thank you for admiring my beautiful art
   */

  // Following methods abstract the single aLED chain to 4 physical zones (front left, back left,
  // back right, back right), which take an input of a 2-dimensional integer array. Each row is an
  // individual LED element, and the three columns are Red, Green and Blue, respectivly (from
  // 0 - 255). The elements are numbered from bottom to top.

  private void setFrontLeftSolid(int red, int green, int blue) {
    // buffer is a 21 x 3 array of the LEDs, with each row being a different element and the columns
    // being the R, G, and B values
    for (var i = 0; i < LEDConstants.FRONT_LED_LENGTH; i++) {
      int offset = 0;
      m_ledBuffer.setRGB(i + offset, red, green, blue);
    }
  }

  private void setBackLeftSolid(int red, int green, int blue) {
    // buffer is a 9 x 3 array of the LEDs, with each row being a different element and the columns
    // being the R, G, and B values
    for (var i = 0; i < LEDConstants.BACK_LED_LENGTH; i++) {
      int offset = LEDConstants.FRONT_LED_LENGTH;
      m_ledBuffer.setRGB(i + offset, red, green, blue);
    }
  }

  private void setBackRightSolid(int red, int green, int blue) {
    // buffer is a 9 x 3 array of the LEDs, with each row being a different element and the columns
    // being the R, G, and B values
    for (var i = LEDConstants.BACK_LED_LENGTH; i >= 0; i--) {
      int offset = LEDConstants.BACK_LED_LENGTH + LEDConstants.FRONT_LED_LENGTH;
      m_ledBuffer.setRGB(i + offset, red, green, blue);
    }
  }

  private void setFrontRightSolid(int red, int green, int blue) {
    // buffer is a 21 x 3 array of the LEDs, with each row being a different element and the columns
    // being the R, G, and B values
    for (var i = LEDConstants.FRONT_LED_LENGTH - 1; i >= 0; i--) {
      int offset = LEDConstants.FRONT_LED_LENGTH + 2 * LEDConstants.BACK_LED_LENGTH;
      m_ledBuffer.setRGB(i + offset, red, green, blue);
    }
  }

  public void setAllSolid(int red, int green, int blue) {
    setFrontLeftSolid(red, green, blue);
    setBackLeftSolid(red, green, blue);
    setBackRightSolid(red, green, blue);
    setFrontRightSolid(red, green, blue);
  }

  public void setFrontSolid(int red, int green, int blue) {
    setFrontLeftSolid(red, green, blue);
    setFrontRightSolid(red, green, blue);
  }

  public void setBackSolid(int red, int green, int blue) {
    setBackLeftSolid(red, green, blue);
    setBackRightSolid(red, green, blue);
  }

  public void setAllPurple() {
    setAllSolid(
        LEDConstants.CUBE_PURPLE[0], LEDConstants.CUBE_PURPLE[1], LEDConstants.CUBE_PURPLE[2]);
  }

  public void setAllYellow() {
    setAllSolid(
        LEDConstants.CONE_YELLOW[0], LEDConstants.CONE_YELLOW[1], LEDConstants.CONE_YELLOW[2]);
  }

  public void setAllBlue() {
    setAllSolid(LEDConstants.TEAM_BLUE[0], LEDConstants.TEAM_BLUE[1], LEDConstants.TEAM_BLUE[2]);
  }

  public void setAllGreen() {
    setAllSolid(LEDConstants.TEAM_GREEN[0], LEDConstants.TEAM_GREEN[1], LEDConstants.TEAM_GREEN[2]);
  }

  public void setAllCyan() {
    setAllSolid(LEDConstants.TEAM_CYAN[0], LEDConstants.TEAM_CYAN[1], LEDConstants.TEAM_CYAN[2]);
  }

  public void setLEDToAllianceColor() {
    if (DriverStation.getAlliance() != Alliance.Invalid) {
      if (DriverStation.getAlliance() == Alliance.Red) {
        setAllSolid(255, 0, 0);
      } else {
        setAllSolid(0, 0, 255);
      }
    } else {
      setAllYellow();
    }
  }

  public void updateLEDs() {
    m_led.setData(m_ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateLEDs();
  }
}
