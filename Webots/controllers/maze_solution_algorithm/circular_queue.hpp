#pragma once
#define MAX_QUEUE_SIZE 17
class CircularQueue {
  private:
    float queue[MAX_QUEUE_SIZE];
    int front, rear, capacity, counter;
  public:
    CircularQueue();
    void enqueue(float item);
    float dequeue();
    bool isEmpty();
    void clear();
    int getCounter();
};