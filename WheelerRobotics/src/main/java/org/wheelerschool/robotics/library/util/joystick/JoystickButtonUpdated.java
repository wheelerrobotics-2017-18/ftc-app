package org.wheelerschool.robotics.library.util.joystick;

import java.util.concurrent.Callable;

/**
 * This library allows to easily get if a joystick button is pushed and if the state is new.
 *
 * @author luciengaitskell
 * @since 161218
 * @version 1.1
 */

public class JoystickButtonUpdated {
    // Callable to run to get the button state:
    private final Callable<Boolean> getButton;

    // The last button state that was read:
    private boolean lastButtonState;

    // The value of the state flip each *new* button press:
    public boolean lastFlipStateValue;


    public JoystickButtonUpdated(Callable<Boolean> getButton) {
        this(getButton, false);
    }

    public JoystickButtonUpdated(Callable<Boolean> getButton, boolean defaultFlipStateValue) {
        // Set the get button state Callable:
        this.getButton = getButton;
        this.lastFlipStateValue = defaultFlipStateValue;
    }


    public class JoystickButtonData {
        /**
         * Contains the joystick button data.
         */
        public boolean buttonState;
        public boolean isButtonStateNew;
        public boolean flipStateValue;
    }

    public JoystickButtonData getValue() throws Exception {
        // Construct a JoystickButtonData object:
        JoystickButtonData joystickButtonData = new JoystickButtonData();

        // Get button value:
        boolean button = getButton.call();

        // Set button state in a JoystickButtonData object
        joystickButtonData.buttonState = button;

        // Set if the button state it new:
        if (button != lastButtonState) {
            joystickButtonData.isButtonStateNew = true;
        } else {
            joystickButtonData.isButtonStateNew = false;
        }

        // Set last button state:
        lastButtonState = button;

        // If the button has been newly pressed:
        if (joystickButtonData.isButtonStateNew && joystickButtonData.buttonState) {
            // Invert last flip state value:
            this.lastFlipStateValue = !this.lastFlipStateValue;
        }
        // Set flip state value in the JoystickButtonData object
        joystickButtonData.flipStateValue = this.lastFlipStateValue;

        // Return the JoystickButtonData object:
        return joystickButtonData;
    }

    public JoystickButtonData getValueIgnoreException() {
        try {
            return this.getValue();
        } catch (Exception e) {}

        return null;
    }
}
