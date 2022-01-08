package frc.robot.lib;

public enum WheelColour {
    RED(0, "red"), YELLOW(1, "yellow"), BLUE(2, "blue"), GREEN(3, "green"), UNKNOWN(-1, "unknown");

    public final int id;
    public final String name;
    public static final int NUM_COLOURS = 4;

    WheelColour(int id, String name) {
        this.id = id;
        this.name = name;
    }

    public static WheelColour of(int id) {
        switch (id) {
            case 0:
                return RED;
            case 1:
                return YELLOW;
            case 2:
                return BLUE;
            case 3:
                return GREEN;
            default:
                return UNKNOWN;
        }
    }

    public boolean equals(WheelColour colour) {
        return this.id == colour.id;
    }

    public WheelColour next(double direction) {
        return WheelColour.of((this.id + NUM_COLOURS + (direction < 0 ? 1 : -1)) % NUM_COLOURS);
    }

    public LEDColour convert() {
        switch (this) {
            case RED:
                return LEDColour.RED;
            case BLUE:
                return LEDColour.BLUE;
            case YELLOW:
                return LEDColour.YELLOW;
            case GREEN:
                return LEDColour.GREEN;
            case UNKNOWN:
                return LEDColour.WHITE;
            default:
                return LEDColour.WHITE;
        }
    }

    @Override
    public String toString() {
        return name + "(" + id + ")";
    }
}
