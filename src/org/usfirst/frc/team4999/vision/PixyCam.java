package org.usfirst.frc.team4999.vision;

import java.security.InvalidParameterException;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;


class PixyCamIO {
    private static final byte PIXY_SYNC_BYTE = 0x5a;
    private static final byte PIXY_SYNC_BYTE_DATA = 0x5b;

    private SPI spi;
    private ByteQueue sendQueue;

    private byte[] sendBuffer, receiveBuffer;

    public PixyCamIO() {
        // TODO: Change based on which CS port is wired
        spi = new SPI(SPI.Port.kOnboardCS0);
        spi.setMSBFirst();
        spi.setSampleDataOnRising();

        sendQueue = new ByteQueue(64);
        
        sendBuffer = new byte[2];
        receiveBuffer = new byte[2];
    }

    public int transact() {
        if(!sendQueue.empty()) {
            sendBuffer[0] = PIXY_SYNC_BYTE_DATA;
            sendBuffer[1] = sendQueue.dequeue();
        } else {
            sendBuffer[0] = PIXY_SYNC_BYTE;
            sendBuffer[1] = 0;
        }
        spi.transaction(sendBuffer, receiveBuffer, 2);
        return (receiveBuffer[0] << 8) | receiveBuffer[1];
    }
    
    public void juxtapose() {
        sendBuffer[0] = PIXY_SYNC_BYTE;
        spi.transaction(sendBuffer, receiveBuffer, 1);
    }

    public void send(byte[] data) {
        if(data.length < sendQueue.capacity() - sendQueue.size()) {
            for(int i = 0; i < data.length; i++) {
                sendQueue.enqueue(data[i]);
            }
        }
    }

    
}

class PixyCam {
    
    private static final int PIXY_START_WORD = 0xaa55;
    private static final int PIXY_START_WORD_CC = 0xaa56;
    private static final int PIXY_START_WORD_OUT_OF_SYNC = 0x55aa;

    private static final byte PIXY_SERVO_SYNC = 0xff;
    private static final byte PIXY_CAM_BRIGHTNESS_SYNC = 0xfe;
    private static final byte PIXY_LED_SYNC = 0xfd;

    private PixyCamIO io;

    private boolean skipStart = false;

    private ArrayList<PixyObject> detectedObjects = new ArrayList<PixyObject>();
    private PixyObject.BlockType blocktype;

    public PixyCam() {
        io = new PixyCamIO();
    }

    /**
     * Gets the start of a frame
     * @return boolean if data is available
     */
    private boolean getStart() {
        int word = 0xffff, lastword = 0xffff;
        while(true) {
            word = io.transact();
            if (word == 0 && lastword == 0) {
                // The only way to get four bytes of zeros is if there's no data available
                return false;
            } else if (word == PIXY_START_WORD && lastword == PIXY_START_WORD) {
                blocktype = PixyObject.BlockType.NORMAL_BLOCK;
                return true;
            } else if (word == PIXY_START_WORD_CC && lastword == PIXY_START_WORD) {
                blocktype = PixyObject.BlockType.CC_BLOCK;
                return true;
            } else if(word == PIXY_START_WORD_OUT_OF_SYNC) {
                io.juxtapose();
            }
            lastword = word;
        }
    }

    public PixyObject[] getDetectedObjects() {
        PixyObject[] out = PixyObject[0];
        // If the start of the next frame is detected before this method exits, skip the getStart() method since we've already read the start byte
        if(!skipStart) {
            if(!getStart()) {
                // No data to read, so return an empty array
                return out;
            }
        } else {
            skipStart = false;
        }

        while(true) {
            checksum = io.transact();
            if(checksum == PIXY_START_WORD) {
                // We've reached the next frame, so return all the detected objects for the current frame
                // Also, since we've already read the sync byte that signals we're between frames, set skipStart to true so we don't skip this frame entirely looking for the next frame
                skipStart = true;
                blocktype = PixyObject.blocktype.NORMAL_BLOCK;
                return detectedObjects.toArray(out);
            } else if (checksum == PIXY_START_WORD_CC) {
                // We've reached the next frame, so return all the detected objects for the current frame
                // Also, since we've already read the sync byte that signals we're between frames, set skipStart to true so we don't skip this frame entirely looking for the next frame
                skipStart = true;
                blocktype = PixyObject.blocktype.CC_BLOCK;
                return detectedObjects.toArray(out);
            } else if(checksum == 0) {
                // No more data, return everything we've read so far
                return detectedObjects.toArray(out);
            }

            int signature = 0, x = 0, y = 0, width = 0, height = 0, angle = 0;
            signature = io.transact();
            x = io.transact();
            y = io.transact();
            width = io.transact();
            height = io.transact();
            if(blocktype == PixyObject.blocktype.CC_BLOCK) {
                angle = io.transact();
            }

            if((signature + x + y + width + height + angle) & 0xffff == checksum) {
                detectedObjects.add(new PixyObject(signature, x, y, width, height, angle));
            }

            int sync = io.transact();
            if(sync == PIXY_START_WORD) {
                blocktype = PixyObject.BlockType.NORMAL_BLOCK;
            } else if (sync == PIXY_START_WORD_CC) {
                blocktype = PixyObject.BlockType.CC_BLOCK;
            } else {
                // All data read
                return detectedObjects.toArray(out);
            }
            
        }

    }

    /**
     * Sets the position of any connected servos
     * @param s1 Servo 0 (pan) position, between 0 and 1000
     * @param s2 Servo 1 (tilt) position, between 0 and 1000
     */
    public void setServos(int s1, int s2) {
        if(s1 > 1000 || s1 < 0 || s2 > 1000 || s2 < 0) {
            throw new InvalidParameterException("Servo values must be between 0 and 1000");
        }
        byte[] data = new byte[6];
        data[0] = 0x00;
        data[1] = PIXY_SERVO_SYNC;
        data[2] = (byte)s1;
        data[3] = (byte)(s1 >> 8);
        data[4] = (byte)s2;
        data[5] = (byte)(s2 >> 8);

        io.send(data);
    }

    /**
     * Sets the exposure of the camera
     * @param exposure Exposure value, between 0 and 255
     */
    public void setExposure(int exposure) {
        if(exposure < 0 || exposure > 255) {
            throw new InvalidParameterException("Brightness must be between 0 and 255");
        }
        byte[] data = new byte[3];
        data[0] = 0x00;
        data[1] = PIXY_CAM_BRIGHTNESS_SYNC;
        data[2] = (byte)exposure;

        io.send(data);
    }

    /**
     * Sets the color of the Pixy's LED
     * @param red red value, between 0 and 255
     * @param green green value, between 0 and 255
     * @param blue blue value, between 0 and 255
     */
    public void setLED(int red, int green, int blue) {
        if(red < 0 || red > 255 || green < 0 || green > 255 || blue < 0 || blue > 255) {
            throw new InvalidParameterException("Colors must be between 0 and 255");
        }

        byte[] data = new byte[3];
        data[0] = 0x00;
        data[1] = PIXY_LED_SYNC;
        data[2] = (byte)red;
        data[3] = (byte)green;
        data[4] = (byte)blue;

        io.send(data);
    }
}