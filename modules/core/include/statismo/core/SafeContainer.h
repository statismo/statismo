/*
 * This file is part of the statismo library.
 *
 * Copyright (c) 2019 Laboratory of Medical Information Processing
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the project's author nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __STATIMO_CORE_SAFE_CONTAINER_H_
#define __STATIMO_CORE_SAFE_CONTAINER_H_

#include "statismo/core/CommonTypes.h"
#include "statismo/core/NonCopyable.h"

#include <unordered_map>
#include <queue>
#include <thread>
#include <mutex>
#include <memory>
#include <condition_variable>

namespace statismo
{

/*
 * Thread safe unordered map
 *
 * \note: This is not lock-based not fine-grained implementation
 *        and implement only the current needed interface
 */
template <class Key,
          class T,
          class Hash = std::hash<Key>,
          class KeyEqual = std::equal_to<Key>,
          class Allocator = std::allocator<std::pair<const Key, T>>>
class SafeUnorderedMap : public NonCopyable
{
public:
  using ValueType = std::pair<const Key, T>;

  void
  Insert(const ValueType & value)
  {
    std::lock_guard<std::mutex> l{ m_guard };
    m_map.insert(value);
  }

  std::size_t
  Size() const
  {
    std::lock_guard<std::mutex> l{ m_guard };
    return m_map.size();
  }

  bool
  Empty() const
  {
    std::lock_guard<std::mutex> l{ m_guard };
    return m_map.empty();
  }

  bool
  Find(const Key & key, T & value) const
  {
    std::lock_guard<std::mutex> l{ m_guard };
    auto                        got = m_map.find(key);

    if (got == std::cend(m_map))
    {
      return false;
    }
    else
    {
      value = got->second;
      return true;
    }
  }

private:
  using ContainerType = std::unordered_map<Key, T, Hash, KeyEqual, Allocator>;
  ContainerType      m_map;
  mutable std::mutex m_guard;
};

/*
 * Thread safe queue
 *
 * Implementation taken from "C++ concurrency in action", A. Williams
 */
template <typename T>
class SafeQueue : public NonCopyable
{
public:
  SafeQueue()
    : m_head{ new Node }
    , m_tail{ m_head.get() }
  {}

  std::shared_ptr<T>
  TryPop();
  bool
  TryPop(T & value);
  std::shared_ptr<T>
  WaitPop();
  void
  WaitPop(T & value);
  void
  Push(T new_value);
  bool
  Empty() const;

private:
  struct Node
  {
    std::shared_ptr<T>    data;
    std::unique_ptr<Node> next;
  };

  Node *
  GetTail() const
  {
    std::lock_guard<std::mutex> tailLock{ m_tailMutex };
    return m_tail;
  }

  std::unique_ptr<Node>
  PopHead()
  {
    auto oldHead = std::move(m_head);
    m_head = std::move(oldHead->next);
    return oldHead;
  }

  std::unique_lock<std::mutex>
  WaitForData()
  {
    std::unique_lock<std::mutex> headLock{ m_headMutex };
    m_dataCond.wait(headLock, [&] { return m_head.get() != GetTail(); });
    return headLock;
  }

  std::unique_ptr<Node>
  WaitPopHead()
  {
    std::unique_lock<std::mutex> headLock{ WaitForData() };
    return PopHead();
  }

  std::unique_ptr<Node>
  WaitPopHead(T & value)
  {
    std::unique_lock<std::mutex> headLock{ WaitForData() };
    value = std::move(*m_head->data);
    return PopHead();
  }

  std::unique_ptr<Node>
  TryPopHead()
  {
    std::lock_guard<std::mutex> headLock{ m_headMutex };
    if (m_head.get() == GetTail())
    {
      return std::unique_ptr<Node>();
    }
    return PopHead();
  }

  std::unique_ptr<Node>
  TryPopHead(T & value)
  {
    std::lock_guard<std::mutex> headLock{ m_headMutex };
    if (m_head.get() == GetTail())
    {
      return std::unique_ptr<Node>();
    }
    value = std::move(*m_head->data);
    return PopHead();
  }

  mutable std::mutex      m_headMutex;
  std::unique_ptr<Node>   m_head;
  mutable std::mutex      m_tailMutex;
  Node *                  m_tail;
  std::condition_variable m_dataCond;
};

template <typename T>
void
SafeQueue<T>::Push(T new_value)
{
  std::shared_ptr<T>    new_data{ std::make_shared<T>(std::move(new_value)) };
  std::unique_ptr<Node> p(new Node);
  {
    std::lock_guard<std::mutex> tailLock{ m_tailMutex };
    m_tail->data = new_data;
    auto * const newTail = p.get();
    m_tail->next = std::move(p);
    m_tail = newTail;
  }
  m_dataCond.notify_one();
}

template <typename T>
std::shared_ptr<T>
SafeQueue<T>::WaitPop()
{
  const auto oldHead = WaitPopHead();
  return oldHead->data;
}

template <typename T>
void
SafeQueue<T>::WaitPop(T & value)
{
  const auto oldHead = WaitPopHead(value);
}

template <typename T>
std::shared_ptr<T>
SafeQueue<T>::TryPop()
{
  const auto oldHead = TryPopHead();
  return oldHead ? oldHead->data : std::shared_ptr<T>();
}

template <typename T>
bool
SafeQueue<T>::TryPop(T & value)
{
  const auto old_head = TryPopHead(value);
  return (old_head != nullptr);
}

template <typename T>
bool
SafeQueue<T>::Empty() const
{
  std::lock_guard<std::mutex> headLock{ m_headMutex };
  return (m_head.get() == GetTail());
}

} // namespace statismo

#endif