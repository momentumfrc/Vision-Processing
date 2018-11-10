package org.usfirst.frc.team4999.vision;

class ByteQueue {
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
        } else {
            throw new IndexOutOfBoundsException("Cannot dequeue from empty queue");
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