/**
 * Author: Vamsi Kalagaturu
 * 
 * Description: A fixed size queue implementation extending the std::queue class
 *
 * Copyright (c) [2023]
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#ifndef FIXED_SIZE_QUEUE_HPP
#define FIXED_SIZE_QUEUE_HPP

#include <cstddef>
#include <queue>

/**
 * @brief A class representing a fixed size queue
 * @tparam T the type of the queue
 * @tparam N the size of the queue
 */
template <typename T, size_t N>
class FixedSizeQueue : public std::queue<T>
{
public:
  /**
   * @brief Construct a new Fixed Size Queue object
   */
  FixedSizeQueue() : std::queue<T>() {}

  /**
   * @brief push a value to the queue and pop the oldest value if the queue is full
   * @param value the value to push
   */
  void push(const T& value)
  {
    if (this->size() == N)
    {
      this->pop();
    }

    std::queue<T>::push(value);
  }

  /**
   * @brief method to check if the queue is full
   * @return true if the queue is full
   */
  bool isFull() const
  {
    return this->size() == N;
  }

  /**
   * @brief method to get the value at a specific index
   * @param index the index of the value to get
   * @return the value at the index
   */
  T at(size_t index) const
  {
    return this->c.at(index);
  }

private:
  size_t maxSize;
};

#endif  // FIXED_SIZE_QUEUE_HPP
