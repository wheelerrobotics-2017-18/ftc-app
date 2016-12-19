package org.wheelerschool.robotics.library.util.joystick;

import java.util.concurrent.Callable;

/**
 * This library allows to easily get if a joystick button is pushed and if the state is new.
 *
 * @author luciengaitskell
 * @since 161218
 * @version 1.0
 */

public class JoystickButtonUpdated {
    // Callable to run to get the button state:
    private final Callable<Boolean> getButton;

    // The last button state that was read:
    private boolean lastButtonState;

    public JoystickButtonUpdated(Callable<Boolean> getButton) {
        // Set the get button state Callable:
        this.getButton = getButton;
    }


    public class JoystickButtonData {
        /**
         * Contains the joystick button data.
         */
        public boolean buttonState;
        public boolean isButtonStateNew;
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
