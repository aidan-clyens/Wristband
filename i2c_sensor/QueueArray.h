/***********************************************************/
/*    QueueArray
/*
/*    This is an implementation using arrays. Currently, there
/*    is no dynamic array resizing.
/*    To be used with Arduino, so this uses no C++ standard
/*    library functions.
/*
/*    Aidan Clyens
/*    May 23, 2018
/***********************************************************/
#ifndef QUEUE_ARRAY_H
#define QUEUE_ARRAY_H

#ifdef ARDUINO_H
#include "Arduino.h"
#else
#include <iostream>
#endif

template <typename Type>
class QueueArray {
  public:
    QueueArray( int = 16 );
    ~QueueArray();

    //  Size
    int size() const;
    bool empty() const;

    //  Element Access
    Type front() const;
    Type back() const;

    //  Modifiers
    void push( Type const & );
    void pop();
    void clear();

    //  Misc.
    void print();

  private:
    Type* queue;
    int queue_size;
    int queue_capacity;
    int front_index;
    int back_index;
};

/***************************************
*           Constructors
****************************************/
/*
 * Default QueueArray Constructor
 */
template <typename Type>
QueueArray<Type>::QueueArray(int n):
queue_capacity( n ),
queue_size( 0 ),
queue( new Type[n] ),
front_index( 0 ),
back_index( -1 )
{
  //  Empty Constructor
}

/*
 * QueueArray Destructor
 */
template <typename Type>
QueueArray<Type>::~QueueArray() {
  delete[] queue;
}

/***************************************
*         Public Size Functions
****************************************/
/*
 * size
 * Return the size of the queue
 */
template <typename Type>
int QueueArray<Type>::size() const {
  return queue_size;
}

/*
 * empty
 * Returns a boolean value stating if the queue is empty
 */
template <typename Type>
bool QueueArray<Type>::empty() const {
  return (queue_size == 0);
}

/***************************************
*     Public Element Access Functions
****************************************/
/*
 * front
 * Return the element at the front of the queue
 */
template <typename Type>
Type QueueArray<Type>::front() const {
  return queue[front_index];
}

/*
 * back
 * Return the element at the back of the queue
 */
template <typename Type>
Type QueueArray<Type>::back() const {
  return queue[back_index];
}

/***************************************
*       Public Modifier Functions
****************************************/
/*
 * push
 * Push a new element to the back of the queue
 */
template <typename Type>
void QueueArray<Type>::push(Type const &obj) {
  if (queue_size == queue_capacity) return;
  //  Move back index forwards, accounting for array wraparound
  back_index = ++back_index % queue_capacity;
  queue[back_index] = obj;
  ++queue_size;
}

/*
 * pop
 * Pop the element at the front of the list
 */
template <typename Type>
void QueueArray<Type>::pop() {
  if (empty()) return;
  //  Move front index forwards, accounting for array wraparound
  front_index = ++front_index % queue_capacity;
  --queue_size;
}

/*
 * clear
 * Clears all the elements in the queue
 */
template <typename Type>
void QueueArray<Type>::clear() {
  delete[] queue;
  queue = new Type[queue_capacity];

  queue_size = 0;
  front_index = 0;
  back_index = -1;
}

/***************************************
*         Public Misc. Functions
****************************************/
/*
 * print
 * Print the queue on the Arduino serial monitor
 */
template <typename Type>
void QueueArray<Type>::print() {
  if (empty()) return;

  int i = front_index;

  while (i != back_index) {
    #ifdef ARDUINO_H
    Serial.print(queue[i]);
    Serial.print(" ");
    #else
    std::cout << queue[i] << " ";
    #endif

    i = ++i % queue_capacity;
  }
  #ifdef ARDUINO_H
  Serial.println(queue[back_index]);
  #else
  std::cout << queue[back_index] << std::endl;
  #endif
}

#endif
