// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase {
  private AddressableLED m_led = new AddressableLED(LEDConstants.LED_PORT);

  private AddressableLEDBuffer m_ledBuffer =
      new AddressableLEDBuffer(
          2 * LEDConstants.FRONT_LED_LENGTH + 2 * LEDConstants.BACK_LED_LENGTH);

  /** Creates a new LEDs. */
  public LEDs() {
    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();

    // Place boot-up sequence here
    this.setFrontSolid(55, 186, 27); // 8016 Green
    this.setBackSolid(38, 73, 253); // 8016 Blue (numbers)
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

  private void setFrontLeftBuffer(int[][] buffer) {
    // buffer is a 21 x 3 array of the LEDs, with each row being a different element and the columns
    // being the R, G, and B values
    for (var i = 0; i < LEDConstants.FRONT_LED_LENGTH; i++) {
      m_ledBuffer.setRGB(i, buffer[i][0], buffer[i][1], buffer[i][2]);
    }
    m_led.setData(m_ledBuffer);
  }

  private void setBackLeftBuffer(int[][] buffer) {
    // buffer is a 9 x 3 array of the LEDs, with each row being a different element and the columns
    // being the R, G, and B values
    for (var i = 0; i < LEDConstants.BACK_LED_LENGTH; i++) {
      m_ledBuffer.setRGB(
          i + LEDConstants.FRONT_LED_LENGTH, buffer[i][0], buffer[i][1], buffer[i][2]);
    }
    m_led.setData(m_ledBuffer);
  }

  private void setBackRightBuffer(int[][] buffer) {
    // buffer is a 9 x 3 array of the LEDs, with each row being a different element and the columns
    // being the R, G, and B values
    for (var i = 0; i < LEDConstants.BACK_LED_LENGTH; i++) {
      m_ledBuffer.setRGB(
          LEDConstants.FRONT_LED_LENGTH + 2 * LEDConstants.BACK_LED_LENGTH - 1 - i,
          buffer[i][0],
          buffer[i][1],
          buffer[i][2]);
    }
    m_led.setData(m_ledBuffer);
  }

  private void setFrontRightBuffer(int[][] buffer) {
    // buffer is a 21 x 3 array of the LEDs, with each row being a different element and the columns
    // being the R, G, and B values
    for (var i = 0; i < LEDConstants.FRONT_LED_LENGTH; i++) {
      m_ledBuffer.setRGB(
          2 * LEDConstants.FRONT_LED_LENGTH + 2 * LEDConstants.BACK_LED_LENGTH - 1 - i,
          buffer[i][0],
          buffer[i][1],
          buffer[i][2]);
    }
    m_led.setData(m_ledBuffer);
  }

  // The following methods allow each zone to be set to a single solid color
  public void setFrontLeftSolid(int red, int green, int blue) {
    int[][] buffer = new int[21][3];
    for (var i = 0; i < LEDConstants.FRONT_LED_LENGTH; i++) {
      buffer[i][0] = red;
      buffer[i][1] = green;
      buffer[i][2] = blue;
    }
    setFrontLeftBuffer(buffer);
  }

  public void setBackLeftSolid(int red, int green, int blue) {
    int[][] buffer = new int[21][3];
    for (var i = 0; i < LEDConstants.BACK_LED_LENGTH; i++) {
      buffer[i][0] = red;
      buffer[i][1] = green;
      buffer[i][2] = blue;
    }
    setBackLeftBuffer(buffer);
  }

  public void setBackRightSolid(int red, int green, int blue) {
    int[][] buffer = new int[21][3];
    for (var i = 0; i < LEDConstants.BACK_LED_LENGTH; i++) {
      buffer[i][0] = red;
      buffer[i][1] = green;
      buffer[i][2] = blue;
    }
    setBackRightBuffer(buffer);
  }

  public void setFrontRightSolid(int red, int green, int blue) {
    int[][] buffer = new int[21][3];
    for (var i = 0; i < LEDConstants.FRONT_LED_LENGTH; i++) {
      buffer[i][0] = red;
      buffer[i][1] = green;
      buffer[i][2] = blue;
    }
    setFrontRightBuffer(buffer);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
