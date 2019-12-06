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
#include "statismo/core/ThreadPool.h"
#include "statismo/core/Exceptions.h"

namespace statismo
{

thread_local WorkStealingQueue * ThreadPool::s_localQueue = nullptr;
thread_local std::size_t         ThreadPool::s_tid = 0;

ThreadPool::ThreadPool(unsigned maxThreads)
  : ThreadPool{ maxThreads, WaitingMode::YIELD, 0 }
{}

ThreadPool::ThreadPool(unsigned maxThreads, WaitingMode m, unsigned waitTime)
  : m_waitMode{ m }
  , m_waitTime{ waitTime }
{
  const auto threadCount = std::min(std::thread::hardware_concurrency(), maxThreads);

  for (std::remove_cv_t<decltype(threadCount)> i = 0; i < threadCount; ++i)
  {
    m_queues.push_back(std::make_unique<WorkStealingQueue>());
  }

  try
  {
    for (std::remove_cv_t<decltype(threadCount)> i = 0; i < threadCount; ++i)
    {
      m_threads.push_back(RaiiThread{ std::thread{ &ThreadPool::DoThreadJob, this, i } });
    }
  }
  catch (...)
  {
    m_isDone = true;
    throw StatisticalModelException("Failed to create thread pool");
  }
}

void
ThreadPool::DoThreadJob(std::size_t idx)
{
  s_tid = idx;
  s_localQueue = m_queues[s_tid].get();

  while (!m_isDone)
  {
    RunPendingTask();
  }
}

bool
ThreadPool::PopTaskFromLocalQueue(TaskType & t)
{
  return s_localQueue->TryPop(t);
}

bool
ThreadPool::PopTaskFromPoolQueue(TaskType & t)
{
  return m_poolQueue.TryPop(t);
}

bool
ThreadPool::PopTaskFromOtherLocalQueues(TaskType & t)
{
  for (std::size_t i = 0; i < m_queues.size(); ++i)
  {
    const auto idx = (s_tid + i + 1) % m_queues.size();

    if (m_queues[idx]->TrySteal(t))
    {
      return true;
    }
  }

  return false;
}

void
ThreadPool::RunPendingTask()
{
  TaskType t;

  if (PopTaskFromLocalQueue(t) || PopTaskFromPoolQueue(t) || PopTaskFromOtherLocalQueues(t))
  {
    t();
  }
  else
  {
    if (m_waitMode == WaitingMode::YIELD)
    {
      std::this_thread::yield();
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(m_waitTime));
    }
  }
}
} // namespace statismo