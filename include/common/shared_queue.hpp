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

typedef std::unique_lock<std::mutex> ULockMutex;

/**
 * @brief Queue implementation that support multithreaded actions.
 *
 * @tparam T the type to store
 */
template <typename T>
class SharedQueue {
 public:
  void push(const T &elem) {
    auto lock = getLock();
    lock.lock();
    _queue.push(elem);
    _m.unlock();
  }

  void pop() {
    auto lock = getLock();
    lock.lock();
    _queue.pop();
    lock.unlock();
  }

  T &front() { return _queue.front(); }

  bool isEmpty() {
    auto lock = getLock();
    lock.lock();
    bool empty = _queue.empty();
    lock.unlock();
    return empty;
  }

  unsigned long size() {
    auto lock = getLock();
    lock.lock();
    unsigned long size = _queue.size();
    lock.unlock();
    return size;
  }

  void flush() {
    auto lock = getLock();
    lock.lock();
    while (!_queue.empty()) _queue.pop();
    lock.unlock();
  }

  ULockMutex getLock() { return ULockMutex(_m); }
  void unlock(ULockMutex &lock) { lock.unlock(); }

 private:
  std::mutex _m;
  std::queue<T> _queue;
};

#endif