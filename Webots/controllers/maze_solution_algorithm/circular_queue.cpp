  #include "circular_queue.hpp"
  
  CircularQueue::CircularQueue() {
      front = rear = -1;
      capacity = MAX_QUEUE_SIZE;
  } 
 
  void CircularQueue::enqueue(float item) {
      if ((front == 0 && rear == capacity - 1) || ((rear + 1) % capacity == front)) {
          // Overwrite the oldest entry
          front = (front + 1) % capacity;
      }
      else {
          if (front == -1) front = 0;
          rear = (rear + 1) % capacity;
      }
      queue[rear] = item;
      counter++;
  }

  float CircularQueue::dequeue() {
      //check if it is empty
      if (front == -1) {
          return 0;
      }
      float item = queue[front];
      if (front == rear) {
          front = -1;
          rear = -1;
      } else {
          front = (front + 1) % capacity;
      }
      counter--;
      return item;
  }

  bool CircularQueue::isEmpty() {
      return front == -1;
  }

  void CircularQueue::clear(){
    front = rear = -1;
  }
  int CircularQueue::getCounter(){
    return counter;
  }