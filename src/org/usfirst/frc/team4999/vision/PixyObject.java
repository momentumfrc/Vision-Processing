package org.usfirst.frc.team4999.vision;

public class PixyObject {

    public static enum BlockType { NORMAL_BLOCK, CC_BLOCK };

    private final int signature;
    private final int x;
    private final int y;
    private final int width;
    private final int height;
    private final int angle;

    public PixyObject(int signature, int x, int y, int width, int height, int angle) {
        this.signature = signature;
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
        this.angle = angle;
    }

    public int getSignature() {
        return signature;
    }

    public int getX() {
        return x;
    }

    public int getY() {
        return y;
    }

    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }

    public int getAngle() {
        return angle;
    }
}