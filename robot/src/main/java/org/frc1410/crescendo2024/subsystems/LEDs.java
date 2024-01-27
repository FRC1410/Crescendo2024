package org.frc1410.crescendo2024.subsystems;

/*
* https://store.ctr-electronics.com/content/api/java/html/classcom_1_1ctre_1_1phoenix_1_1led_1_1_c_a_ndle.html
* */


import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.ctre.phoenix.led.RainbowAnimation;

import static org.frc1410.crescendo2024.util.Constants.*;

public class LEDs implements Subsystem {
	private CANdle LEDs;

	public enum Colors {
		BLOOD_OF_ENEMY_DUE_TO_UNFORTUNATE_OCEAN_OIL_SPILL_TURNED_ARSON_FIRE_RED,
		LIMELIGHT_GREEN,
		OCEAN_BREEZE_BLUE,
		RAINBOW_ANIMATION,
	}


	public LEDs () {
		LEDs =  new CANdle(LED_ID);

		CANdleConfiguration config = new CANdleConfiguration();
		config.stripType = CANdle.LEDStripType.RGB;
		config.brightnessScalar = 1.0;
		LEDs.configLOSBehavior(true);
		LEDs.configAllSettings(config);

		LEDs.clearAnimation()
		LEDs.setLEDs(DEFAULT_LED_COLOR_RGB[0], DEFAULT_LED_COLOR_RGB[1], DEFAULT_LED_COLOR_RGB[2]);
	}

	public void changeLEDsColor(Colors Color) {
		switch (Color) {
			case BLOOD_OF_ENEMY_DUE_TO_UNFORTUNATE_OCEAN_OIL_SPILL_TURNED_ARSON_FIRE_RED ->
				LEDs.setLEDs(255, 0 ,0);
			case LIMELIGHT_GREEN ->
				LEDs.setLEDs(0, 255, 0);
			case OCEAN_BREEZE_BLUE ->
				LEDs.setLEDs(0, 0, 255);
			case RAINBOW_ANIMATION ->
				LEDs.animate(new RainbowAnimation(LED_BRIGHTNESS, LED_ANIMATION_SPEED, NUM_LEDS));
		}
	}

}

