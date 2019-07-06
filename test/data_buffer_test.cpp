/**
 * Copyright (C) 2019  Sergey Morozov <sergey@morozov.ch>
 *
 * Permission is hereby granted, free of charge, to any person 
 * obtaining a copy of this software and associated documentation 
 * files (the "Software"), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, 
 * publish, distribute, sublicense, and/or sell copies of the Software, 
 * and to permit persons to whom the Software is furnished to do so, 
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be 
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH 
 * THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "../src/CircularBuffer.hpp"

#include <catch.hpp>
#include <iostream>


TEST_CASE("CircularBuffer<int, 2>::push_back", "[CircularBuffer]")
{
  CircularBuffer<int, 2> db;

  REQUIRE(db.max_size() == 2);
  REQUIRE(db.size() == 0);
  REQUIRE(db.end() - db.begin() == 0);

  db.push_back(5);
  REQUIRE(db.max_size() == 2);
  REQUIRE(db.size() == 1);
  REQUIRE(*db.begin() == 5);
  REQUIRE(*(db.end()-1) == 5);
  REQUIRE(db.end() - db.begin() == 1);

  db.push_back(1);
  REQUIRE(db.max_size() == 2);
  REQUIRE(db.size() == 2);
  REQUIRE(*db.begin() == 5);
  REQUIRE(*(db.begin()+1) == 1);
  REQUIRE(*(db.end()-1) == 1);
  REQUIRE(db.end() - db.begin() == 2);

  db.push_back(3);
  REQUIRE(db.max_size() == 2);
  REQUIRE(db.size() == 2);
  REQUIRE(*db.begin() == 1);
  REQUIRE(*(db.begin()+1) == 3);
  REQUIRE(*(db.end()-1) == 3);
  REQUIRE(db.end() - db.begin() == 2);
}

TEST_CASE("CircularBuffer<int, 1>::push_back", "[CircularBuffer]")
{
  CircularBuffer<int, 1> db;

  REQUIRE(db.max_size() == 1);
  REQUIRE(db.size() == 0);
  REQUIRE(db.end() - db.begin() == 0);

  db.push_back(5);
  REQUIRE(db.max_size() == 1);
  REQUIRE(db.size() == 1);
  REQUIRE(*db.begin() == 5);
  REQUIRE(*(db.end()-1) == 5);
  REQUIRE(db.end() - db.begin() == 1);

  db.push_back(3);
  REQUIRE(db.max_size() == 1);
  REQUIRE(db.size() == 1);
  REQUIRE(*db.begin() == 3);
  REQUIRE(*(db.end()-1) == 3);
  REQUIRE(db.end() - db.begin() == 1);
}

TEST_CASE("CircularBuffer<int, 2>::max_size", "[CircularBuffer]")
{
  CircularBuffer<int, 2> db;

  REQUIRE(db.max_size() == 2);

  db.push_back(5);
  REQUIRE(db.max_size() == 2);

  db.push_back(1);
  REQUIRE(db.max_size() == 2);

  db.push_back(3);
  REQUIRE(db.max_size() == 2);
}

TEST_CASE("CircularBuffer<int, 2>::size", "[CircularBuffer]")
{
  CircularBuffer<int, 2> db;

  REQUIRE(db.size() == 0);

  db.push_back(5);
  REQUIRE(db.size() == 1);

  db.push_back(1);
  REQUIRE(db.size() == 2);

  db.push_back(3);
  REQUIRE(db.size() == 2);
}

TEST_CASE("CircularBuffer<struct test, 2>::begin / end", "[CircularBuffer]")
{
  struct test
  {
    int foo;
    int bar;

    bool operator==(const test& other) const
    {
      return std::tie(this->foo, this->bar) == std::tie(other.foo, other.bar);
    }
  };

  CircularBuffer<test, 2> db;

  REQUIRE(db.begin() == db.end());

  test t1{-8, 6};
  db.push_back(t1);
  REQUIRE( *db.begin() == t1 );
  REQUIRE( db.begin()->foo == -8 );
  REQUIRE( db.begin()->bar == 6 );
  REQUIRE( db.begin() != db.end() );
  REQUIRE( (db.begin()+1) == db.end() );

  test t2{9, -2};
  db.push_back(t2);
  REQUIRE( *db.begin() == t1 );
  REQUIRE( db.begin()->foo == -8 );
  REQUIRE( db.begin()->bar == 6 );
  REQUIRE( *(db.end()-1) == t2 );
  REQUIRE( (db.end()-1)->foo == 9 );
  REQUIRE( (db.end()-1)->bar == -2 );
  REQUIRE( db.begin() != db.end() );
  REQUIRE( (db.begin()+2) == db.end() );

  test t3{0, -16};
  db.push_back(t3);
  REQUIRE( *db.begin() == t2 );
  REQUIRE( db.begin()->foo == 9 );
  REQUIRE( db.begin()->bar == -2 );
  REQUIRE( *(db.end()-1) == t3 );
  REQUIRE( (db.end()-1)->foo == 0 );
  REQUIRE( (db.end()-1)->bar == -16 );
  REQUIRE( db.begin() != db.end() );
  REQUIRE( (db.begin()+2) == db.end() );
}

TEST_CASE("CircularBuffer<int, 2>::iterator::operator++ / --", "[CircularBuffer]")
{
  CircularBuffer<int, 2> db;
  db.push_back(1);
  db.push_back(2);
  db.push_back(3);
  db.push_back(4);

  auto it = db.begin();

  REQUIRE(*it == 3);
  REQUIRE(*(++it) == 4);
  it++;
  REQUIRE(it == db.end());

  it--;
  REQUIRE(*it == 4);
  REQUIRE(*(--it) == 3);
  REQUIRE(it == db.begin());
}
