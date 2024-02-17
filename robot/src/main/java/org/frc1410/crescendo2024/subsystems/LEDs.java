package org.frc1410.crescendo2024.subsystems;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static org.frc1410.crescendo2024.util.IDs.*;
import static org.frc1410.crescendo2024.util.Constants.*;

public class LEDs implements Subsystem {
	private CANdle leds = new CANdle(LED_ID);

	public enum Colors {
		BLOOD_OF_ENEMY_DUE_TO_UNFORTUNATE_OCEAN_OIL_SPILL_TURNED_ARSON_FIRE_RED, //Shooting trigger pressed no april tag seen (only speaker april tags)
		LIMELIGHT_GREEN, //Shooting trigger pressed and sees april tag (only speaker april tags)
		PRANCING_PONY_PINK, //Stored (piece is triggering the intake/storage limit switch)
		AMP_ARM_FIRE_ANIMATION, //Amp bar out/unfolded
		VIVACIOUS_VIOLENT_VIOLET, //Auto
		OCEAN_BREEZE, //Idle (robot not in a special state) - sea breeze or blue, teal, green, white in like breeze animation
		CLIMBING_RAINBOW_ANIMATION, //When climbing
	}


	public LEDs () {

		CANdleConfiguration config = new CANdleConfiguration();
		config.stripType = CANdle.LEDStripType.RGB;
		config.brightnessScalar = LED_BRIGHTNESS;

		leds.configLOSBehavior(true);
		leds.configAllSettings(config);

		defaultLEDsState();
		// changeLEDsColor(Colors.NO_APRIL_TAG_SHOOTING_RED);
	}


	public void changeLEDsColor(Colors Color) {
		switch (Color) {
			case PRANCING_PONY_PINK -> leds.setLEDs(247,2,174);
			case LIMELIGHT_GREEN -> leds.setLEDs(10,247,2);
			case BLOOD_OF_ENEMY_DUE_TO_UNFORTUNATE_OCEAN_OIL_SPILL_TURNED_ARSON_FIRE_RED -> leds.animate(new StrobeAnimation(255,18,26));
			case VIVACIOUS_VIOLENT_VIOLET -> leds.setLEDs(146,28,189);
			case AMP_ARM_FIRE_ANIMATION -> leds.animate(new FireAnimation());
			case CLIMBING_RAINBOW_ANIMATION -> leds.animate(new RainbowAnimation());
			case OCEAN_BREEZE -> leds.setLEDs(30, 100, 250);
//			default -> leds.animate(new ColorFlowAnimation(30,100,250));
		}
	}

	public void defaultLEDsState() {
		leds.animate(new TwinkleAnimation(30, 100, 250, 0, 0.05, 150, TwinkleAnimation.TwinklePercent.Percent64));
	}

	private void oceanBreezeAnimation() {
		/*
		Colors in order (top down)
		#1e64fa
		#05e9f5
		#05fcba
		#ffffff
		 */
	}

}

