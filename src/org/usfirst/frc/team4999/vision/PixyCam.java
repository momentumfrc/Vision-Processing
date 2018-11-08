package org.usfirst.frc.team4999.vision;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.SPI;

private class ByteQueue {
    private byte[] data;
    private int length = 0;
    private int start = 0;
    private int end = 0;

    public ByteQueue(int capacity) {
        data = new byte[capacity];
    }

    public boolean enqueue(byte data) {
        if(length >= this.data.length) {
            return false;
        }
        this.data[end] = data;
        length++;
        end = (end + 1) % this.data.length;
        return true;
    }

    public byte dequeue() {
        if(length > 0) {
            byte value = data[start];
            start = (start + 1) % data.length;
            length--;
            return value;
        }
    }

    public int capacity() {
        return data.length;
    }

    public int size() {
        return length;
    }

    public boolean empty(){
        return length == 0;
    }
}

private class PixyCamIO {
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
}