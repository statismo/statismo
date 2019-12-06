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
#include "StatismoUnitTest.h"
#include "statismo/core/Exceptions.h"

#include "statismo/core/RandUtils.h"
#include "statismo/core/Utils.h"
#include "statismo/core/ThreadPool.h"
#include "statismo/core/SafeContainer.h"

using namespace statismo;

namespace
{
int
TestUtils()
{
  rand::RandGen(0);

  STATISMO_ASSERT_EQ("02mw-y6id-9r3k-ltlh.txt", utils::CreateTmpName(".txt"));
  STATISMO_ASSERT_EQ(utils::ToLowerCopy("MyTeST"), "mytest");

  const std::string testStr = "this,is,a comma, separated,string?";
  auto              commaSplitVec = utils::Split<','>(testStr);

  STATISMO_ASSERT_EQ(5U, commaSplitVec.size());
  STATISMO_ASSERT_EQ("this", commaSplitVec[0]);
  STATISMO_ASSERT_EQ("is", commaSplitVec[1]);
  STATISMO_ASSERT_EQ("a comma", commaSplitVec[2]);
  STATISMO_ASSERT_EQ(" separated", commaSplitVec[3]);
  STATISMO_ASSERT_EQ("string?", commaSplitVec[4]);

  auto spaceSplitVec = utils::Split<' '>(testStr);
  STATISMO_ASSERT_EQ(3U, spaceSplitVec.size());
  STATISMO_ASSERT_EQ("this,is,a", spaceSplitVec[0]);
  STATISMO_ASSERT_EQ("comma,", spaceSplitVec[1]);
  STATISMO_ASSERT_EQ("separated,string?", spaceSplitVec[2]);

  return EXIT_SUCCESS;
}

int
TestThreadPool()
{
  ThreadPool t;

  auto boolRes = t.Submit([]() { return true; });
  auto intRes = t.Submit([]() { return 666; });

  boolRes.wait();
  intRes.wait();

  STATISMO_ASSERT_TRUE(boolRes.get());
  STATISMO_ASSERT_EQ(666, intRes.get());

  return EXIT_SUCCESS;
}

int
TestSafeContainerQueue()
{
  SafeQueue<int> q;

  STATISMO_ASSERT_TRUE(q.Empty());
  STATISMO_ASSERT_FALSE(q.TryPop());

  q.Push(1);

  STATISMO_ASSERT_FALSE(q.Empty());

  auto val = q.TryPop();
  STATISMO_ASSERT_NEQ(nullptr, val);
  STATISMO_ASSERT_EQ(1, *val);

  STATISMO_ASSERT_TRUE(q.Empty());

  int val2{ 0 };
  STATISMO_ASSERT_FALSE(q.TryPop(val2));

  q.Push(1);
  q.Push(2);

  STATISMO_ASSERT_TRUE(q.TryPop(val2));
  STATISMO_ASSERT_EQ(1, val2);

  STATISMO_ASSERT_TRUE(q.TryPop(val2));
  STATISMO_ASSERT_EQ(2, val2);

  STATISMO_ASSERT_FALSE(q.TryPop(val2));

  q.Push(1);

  auto val3 = q.WaitPop();
  STATISMO_ASSERT_NEQ(nullptr, val3);
  STATISMO_ASSERT_EQ(1, *val3);

  q.Push(1);

  int val4{ 0 };
  q.WaitPop(val4);
  STATISMO_ASSERT_EQ(1, val4);

  return EXIT_SUCCESS;
}

int
TestSafeContainerMap()
{
  statismo::SafeUnorderedMap<std::string, int> myMap;

  STATISMO_ASSERT_TRUE(myMap.Empty());
  STATISMO_ASSERT_EQ(0U, myMap.Size());

  myMap.Insert(std::make_pair("test1", 1));
  myMap.Insert(std::make_pair("test2", 2));

  STATISMO_ASSERT_EQ(2U, myMap.Size());

  int val;
  STATISMO_ASSERT_TRUE(myMap.Find("test1", val));
  STATISMO_ASSERT_EQ(1, val);

  STATISMO_ASSERT_TRUE(myMap.Find("test2", val));
  STATISMO_ASSERT_EQ(2, val);

  STATISMO_ASSERT_FALSE(myMap.Find("test3", val));

  return EXIT_SUCCESS;
}
} // namespace


/**
 * This basic test case covers the test of different utility classes used in the
 * framework
 */
int utilsStatismoTest([[maybe_unused]] int argc, [[maybe_unused]] char * argv[])
{
  auto res = statismo::Translate([]() {
    return statismo::test::RunAllTests("utilsStatismoTest",
                                       { { "TestUtils", TestUtils },
                                         { "TestThreadPool", TestThreadPool },
                                         { "TestSafeContainerQueue", TestSafeContainerQueue },
                                         { "TestSafeContainerMap", TestSafeContainerMap } });
  });

  return !CheckResultAndAssert(res, EXIT_SUCCESS);
}
