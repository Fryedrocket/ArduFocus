// Queue.h
// By Steven de Salas
// Defines a templated (generic) class for a queue of things.
// Used for Arduino projects, just #include "Queue.h" and add this file via the IDE.

#ifndef QUEUE_H
#define QUEUE_H

#include <Arduino.h>

template<class T>
class Queue {
  private:
    int _front, _back, _count;
    T *_data;
    int _maxitems;
  public:
    Queue(int maxitems = 10) { 
      _front = 0;
      _back = 0;
      _count = 0;
      _maxitems = maxitems;
      _data = new T[maxitems + 1];   
    }
    ~Queue() {
      delete[] _data;  
    }
    Queue operator+ (const Queue &other) {
      Queue result(_maxitems + other._maxitems);
      for (int i = 0; i < _count; ++i) {
        result.push(_data[(_front + i) % _maxitems]);
      }
      for (int i = 0; i < other._count; ++i) {
        result.push(other._data[(other._front + i) % other._maxitems]);
      }
      return result;
    }
    inline int count();
    inline int front();
    inline int back();
    void push(const T &item);
    T peek();
    T pop();
    void clear();
};

template<class T>
inline int Queue<T>::count() 
{
  return _count;
}

template<class T>
inline int Queue<T>::front() 
{
  return _front;
}

template<class T>
inline int Queue<T>::back() 
{
  return _back;
}

template<class T>
void Queue<T>::push(const T &item)
{
  if (_count < _maxitems) { // Drops out when full
    _data[_back++]=item;
    ++_count;
    // Check wrap around
    if (_back > _maxitems)
      _back -= (_maxitems + 1);
  }
}

template<class T>
T Queue<T>::pop() {
  if (_count <= 0) return T(); // Returns empty
  else {
    T result = _data[_front];
    _front++;
    --_count;
    // Check wrap around
    if (_front > _maxitems) 
      _front -= (_maxitems + 1);
    return result; 
  }
}

template<class T>
T Queue<T>::peek() {
  if (_count <= 0) return T(); // Returns empty
  else return _data[_front];
}

template<class T>
void Queue<T>::clear() 
{
  _front = _back;
  _count = 0;
}

#endif
