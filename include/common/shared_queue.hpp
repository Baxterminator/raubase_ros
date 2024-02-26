#ifndef RAUBASE_SHARED_QUEUE
#define RAUBASE_SHARED_QUEUE

/*
Copyright (C) 2017-2024 by DTU
Authors:
  Geoffrey Côte: geoffrey.cote@centraliens-nantes.org

The MIT License (MIT)  https://mit-license.org/

Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the “Software”), to deal in the Software without
restriction, including without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies
or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include <mutex>
#include <queue>

/**
 * @brief Queue implementation that support multithreaded actions.
 *
 * @tparam T the type to store
 */
template <typename T>
class SharedQueue {
 public:
  void lock() { _m.lock(); }
  void unlock() { _m.unlock(); }

  void push(const T &elem) {
    lock();
    _queue.push(elem);
    unlock();
  }

  void pop() {
    lock();
    _queue.pop();
    unlock();
  }

  T &front() { return _queue.front(); }

  bool isEmpty() {
    lock();
    bool empty = _queue.empty();
    unlock();
    return empty;
  }

  unsigned long size() {
    lock();
    unsigned long size = _queue.size();
    unlock();
    return size;
  }

  void flush() {
    lock();
    while (!_queue.empty()) _queue.pop();
    unlock();
  }

 protected:
  std::mutex _m;
  std::queue<T> _queue;
};

#endif