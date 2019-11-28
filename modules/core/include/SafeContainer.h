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

#ifndef __SAFE_CONTAINER_H_
#define __SAFE_CONTAINER_H_

#include "CommonTypes.h"
#include "NonCopyable.h"

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
template<
    class Key,
    class T,
    class Hash = std::hash<Key>,
    class KeyEqual = std::equal_to<Key>,
    class Allocator = std::allocator< std::pair<const Key, T>>
>
class SafeUnorderedMap : public NonCopyable {
  public:
    using ValueType =	std::pair<const Key, T>;

    void insert(const ValueType& value) {
      std::lock_guard<std::mutex> l{m_guard};
      m_map.insert(value);
    }

    std::size_t size() const {
      std::lock_guard<std::mutex> l{m_guard};
      return m_map.size();
    }

    bool empty() const {
      std::lock_guard<std::mutex> l{m_guard};
      return m_map.empty();
    }

    bool find(const Key& key, T& value) const {
      std::lock_guard<std::mutex> l{m_guard};
      auto got = m_map.find(key);

      if (got == std::cend(m_map)) {
        return false;
      } else {
        value = got->second;
        return true;
      }
    }

  private:
    using ContainerType = std::unordered_map<Key, T, Hash, KeyEqual, Allocator>;
    ContainerType m_map;
    mutable std::mutex m_guard;
};

/* 
 * Thread safe queue
 * 
 * Implementation taken from "C++ concurrency in action", A. Williams
 */
template <typename T>
class SafeQueue : public NonCopyable {
   public:
    SafeQueue() : m_head{new Node}, m_tail{m_head.get()} {}

    std::shared_ptr<T> tryPop();
    bool tryPop(T &value);
    std::shared_ptr<T> waitPop();
    void waitPop(T &value);
    void push(T new_value);
    bool empty() const;

   private:
    struct Node {
        std::shared_ptr<T> data;
        std::unique_ptr<Node> next;
    };

    Node *getTail() const {
        std::lock_guard<std::mutex> tailLock{m_tailMutex};
        return m_tail;
    }

    std::unique_ptr<Node> popHead() {
        auto oldHead = std::move(m_head);
        m_head = std::move(oldHead->next);
        return oldHead;
    }

    std::unique_lock<std::mutex> waitForData() {
        std::unique_lock<std::mutex> headLock{m_headMutex};
        m_dataCond.wait(headLock, [&] { return m_head.get() != getTail(); });
        return headLock;
    }

    std::unique_ptr<Node> waitPopHead() {
        std::unique_lock<std::mutex> headLock{waitForData()};
        return popHead();
    }

    std::unique_ptr<Node> waitPopHead(T &value) {
        std::unique_lock<std::mutex> headLock{waitForData()};
        value = std::move(*m_head->data);
        return popHead();
    }

    std::unique_ptr<Node> tryPopHead() {
        std::lock_guard<std::mutex> headLock{m_headMutex};
        if (m_head.get() == getTail()) {
            return std::unique_ptr<Node>();
        }
        return popHead();
    }

    std::unique_ptr<Node> tryPopHead(T &value) {
        std::lock_guard<std::mutex> headLock{m_headMutex};
        if (m_head.get() == getTail()) {
            return std::unique_ptr<Node>();
        }
        value = std::move(*m_head->data);
        return popHead();
    }

    mutable std::mutex m_headMutex;
    std::unique_ptr<Node> m_head;
    mutable std::mutex m_tailMutex;
    Node *m_tail;
    std::condition_variable m_dataCond;
};

template <typename T>
void SafeQueue<T>::push(T new_value) {
    std::shared_ptr<T> new_data{std::make_shared<T>(std::move(new_value))};
    std::unique_ptr<Node> p(new Node);
    {
        std::lock_guard<std::mutex> tailLock{m_tailMutex};
        m_tail->data = new_data;
        auto *const newTail = p.get();
        m_tail->next = std::move(p);
        m_tail = newTail;
    }
    m_dataCond.notify_one();
}

template <typename T>
std::shared_ptr<T> SafeQueue<T>::waitPop() {
    const auto oldHead = waitPopHead();
    return oldHead->data;
}

template <typename T>
void SafeQueue<T>::waitPop(T &value) {
    const auto oldHead = waitPopHead(value);
}

template <typename T>
std::shared_ptr<T> SafeQueue<T>::tryPop() {
    const auto oldHead = tryPopHead();
    return oldHead ? oldHead->data : std::shared_ptr<T>();
}

template <typename T>
bool SafeQueue<T>::tryPop(T &value) {
    const auto old_head = tryPopHead(value);
    return (old_head != nullptr);
}

template <typename T>
bool SafeQueue<T>::empty() const {
    std::lock_guard<std::mutex> headLock{m_headMutex};
    return (m_head.get() == getTail());
}

}

#endif