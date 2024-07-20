#include "riptide_gyro/serial_library.hpp"
#include "riptide_gyro/testing.hpp"
#include <thread>

#if defined(TESTING_ENABLED)

using namespace std::chrono_literals;

TEST(ArduinoLibTest, TestString)
{
    //test copy constructor
    const char *cs = "hello world!";
    arduino_lib::string s(cs);
    ASSERT_NE(cs, s.c_str());
    ASSERT_TRUE(strcmp(s.c_str(), cs) == 0);
    
    //test length
    ASSERT_EQ(s.length(), 12U);

    //test == and !=
    ASSERT_TRUE(s == cs);
    ASSERT_TRUE(s == "hello world!");
    arduino_lib::string other("bruh");
    ASSERT_TRUE(s != other);
    ASSERT_TRUE(s != "bruh");

    //test append and +
    arduino_lib::string s2("1!");
    string s3 = s + s2;
    ASSERT_EQ(s3, "hello world!1!");
    s3.append(s2);
    ASSERT_EQ(s3, "hello world!1!1!");
    ASSERT_EQ(s, "hello world!");

    ASSERT_EQ(s3.length(), 16u);
}


TEST(ArduinoLibTest, TestMutex)
{
    arduino_lib::mutex m;
    ASSERT_TRUE(m.try_lock());
    ASSERT_FALSE(m.try_lock());
    m.unlock();
    ASSERT_TRUE(m.try_lock());
    m.unlock();

    std::thread t([&m](){
        m.lock();
        SLEEP(1000000);
        m.unlock();
    });

    //record start time
    auto start = std::chrono::system_clock::now();

    //let thread start
    SLEEP(1000);

    //lock it
    m.lock();

    auto finish = std::chrono::system_clock::now();
    
    //it should have taken about one second to do that
    auto duration = finish - start;
    ASSERT_NEAR(std::chrono::duration_cast<std::chrono::milliseconds>(duration).count(), 1000, 50);
    
    t.join();
}

TEST(ArduinoLibTest, TestVector)
{
    //test constructor and access
    arduino_lib::vector<int> v = ARDUINOLIB_VECTOR(int, 1, 2, 7, 4);
    ASSERT_EQ(v.size(), 4u);
    ASSERT_EQ(v.at(0), 1);
    ASSERT_EQ(v[2], 7);
    ASSERT_EQ(v.at(2), 7);

    //test == and !=
    arduino_lib::vector<int>
        other(3, 9, 8, 0),
        same(4, 1, 2, 7, 4);
    
    ASSERT_TRUE(v == same);
    ASSERT_TRUE(v != other);

    //test append and +
    v.append(other);
    arduino_lib::vector<int> expected = ARDUINOLIB_VECTOR(int, 1, 2, 7, 4, 9, 8, 0);
    ASSERT_EQ(v, expected);

    arduino_lib::vector<int> v2 = v + other;
    arduino_lib::vector<int> expected2 = ARDUINOLIB_VECTOR(int, 1, 2, 7, 4, 9, 8, 0, 9, 8, 0);
    ASSERT_EQ(v2, expected2);
    ASSERT_EQ(v2.size(), 10u);
    ASSERT_EQ(v, expected);

    //test push_back
    v.push_back(7); //v now 8 long
    arduino_lib::vector<int> expected3 = ARDUINOLIB_VECTOR(int, 1, 2, 7, 4, 9, 8, 0, 7);
    ASSERT_EQ(v, expected3);

    //test throw on bad position
    ASSERT_THROW(v.at(8), arduino_lib::exception);
    ASSERT_THROW(v.at(-1), arduino_lib::exception);
}

TEST(ArduinoLibTest, TestIterator)
{
    //creation
    arduino_lib::vector<int> v(4, 1, 2, 3, 4);
    arduino_lib::Iterator<arduino_lib::vector<int>, int> it = v.begin();
    
    //access
    ASSERT_EQ(it.idx(), 0u);
    ASSERT_EQ(*it, 1);
    
    //addition
    it = it + 1;
    ASSERT_EQ(it.idx(), 1u);
    ASSERT_EQ(*it, 2);
    
    //comparison
    it = it + 3;
    arduino_lib::Iterator<arduino_lib::vector<int>, int> 
        endIt = v.end(),
        beginIt = v.begin();
    
    ASSERT_TRUE(it == endIt);
    ASSERT_TRUE(it != beginIt);

    //subtraction of another operator
    ASSERT_EQ(it - it, 0);
    ASSERT_EQ(beginIt + 1 - beginIt, 1);
    ASSERT_EQ((size_t) (endIt - beginIt), v.size());

    //increment
    ASSERT_EQ(*beginIt, 1);
    it = beginIt;
    it++;
    ASSERT_EQ(*it, 1);
}

// TEST(ArduinoLibTest, TestMap)
// {
//     //test construction
//     arduino_lib::map<arduino_lib::string, arduino_lib::string> m = ARDUINOLIB_MAP(
//         arduino_lib::string, arduino_lib::string, 
//         arduino_lib::pair("ab", "12"),
//         arduino_lib::pair("cd", "34"),
//         arduino_lib::pair("ef", "56"));

//     //test construction, iterators
//     ASSERT_EQ(m.size(), 3);
//     ASSERT_EQ(m.begin()->first, "ab");
//     ASSERT_EQ(m.begin()->second, "12");
//     ASSERT_EQ(m.end() - m.begin(), m.size());

//     //test find
//     arduino_lib::Iterator 
//         abIt = m.find("ab"),
//         cdIt = m.find("cd"),
//         efIt = m.find("ef"),
//         endIt = m.find("asdf");
    
//     ASSERT_EQ(abIt - m.begin(), 0);
//     ASSERT_EQ(cdIt - m.begin(), 1);
//     ASSERT_EQ(efIt - m.begin(), 2);
//     ASSERT_EQ(endIt, m.end());

//     //at and []
//     ASSERT_EQ(m.at("ab"), "12");
//     ASSERT_EQ(m.at("cd"), "34");
//     ASSERT_EQ(m.at("ef"), "56");
//     ASSERT_EQ(m["ab"], "12");
//     ASSERT_EQ(m["cd"], "34");
//     ASSERT_EQ(m["ef"], "56");

//     //throw
//     ASSERT_THROW(m.at("gh"), arduino_lib::exception);

//     //insert
//     m.insert(arduino_lib::pair<arduino_lib::string, arduino_lib::string>("gh", "78"));
//     ASSERT_EQ(m.size(), 4);
//     ASSERT_EQ(m.at("gh"), "78");
//     ASSERT_EQ(m.find("gh") - m.begin(), 3);
// }

#endif 
