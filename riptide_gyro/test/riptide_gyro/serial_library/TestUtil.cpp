#include "riptide_gyro/serial_library.hpp"
#include "riptide_gyro/testing.hpp"

TEST(UtilTest, testConvertFromCString)
{
    int i;
    const char
        str[5] = {1, 2, 3, 4, 5},
        zero[4] = {0};

    //zero value
    i = uwrt_gyro::convertFromCString<int>(zero, 4);
    ASSERT_EQ(i, 0);

    //string with zero length
    i = uwrt_gyro::convertFromCString<int>(str, 0);
    ASSERT_EQ(i, 0);

    //interesting
    char exC = 'a';
    char c = uwrt_gyro::convertFromCString<char>(&exC, 1);
    ASSERT_EQ(c, exC);
    
    //good test of packing T the same size of string
    i = uwrt_gyro::convertFromCString<int>(str, 4);
    ASSERT_EQ(i, 16909060);

    //good test of packing T smaller than string
    i = uwrt_gyro::convertFromCString<int>(str, 5);
    ASSERT_EQ(i, 16909060);

    //good test of packing string smaller than T
    i = uwrt_gyro::convertFromCString<int>(str, 3);
    ASSERT_EQ(i, 66051);
}

TEST(UtilTest, testConvertToCString)
{
    char str[5];
    const char refStr[5] = {1, 2, 3, 4, 5};

    //zero value
    const char zeroStr[5] = {0, 0, 0, 0, 0};
    memset(str, 0, sizeof(str));
    uwrt_gyro::convertToCString<int>(0, str, sizeof(str));
    ASSERT_TRUE(memcmp(str, zeroStr, sizeof(str)) == 0);

    //string with zero length
    memset(str, 0, sizeof(str));
    uwrt_gyro::convertToCString<int>(6, str, 0);
    ASSERT_TRUE(memcmp(str, zeroStr, sizeof(str)) == 0);

    //interesting
    char c = 0;
    uwrt_gyro::convertToCString<char>('B', &c, 1);
    ASSERT_EQ(c, 'B');

    //good test of unpacking T the same size as string
    memset(str, 0, sizeof(str));
    uwrt_gyro::convertToCString<int>(16909060, str, 4);
    ASSERT_TRUE(memcmp(str, refStr, 4) == 0);

    //good test of unpacking T smaller than string
    memset(str, 0, sizeof(str));
    uwrt_gyro::convertToCString<int>(16909060, str, 5);
    ASSERT_TRUE(memcmp(str, refStr, 4) == 0);
    ASSERT_EQ(str[4], 0);

    //good test of unpacking T bigger than string
    memset(str, 0, sizeof(str));
    uwrt_gyro::convertToCString<int>(16909060, str, 3);
    ASSERT_TRUE(memcmp(str, refStr, 3) == 0);
    ASSERT_EQ(str[3], 0);
    ASSERT_EQ(str[4], 0);
}

TEST_F(Type1SerialProcessorTest, testExtractFieldFromBufferBasic)
{
    const char *testMsg = "Aabcd";
    char dst = 0;

    //1-char extraction of beginning value
    size_t result = uwrt_gyro::extractFieldFromBuffer(testMsg, sizeof(testMsg), frameMap[0], FIELD_SYNC, &dst, 1);
    ASSERT_EQ(result, 1);
    ASSERT_EQ(dst, 'A');

    //1-char extraction of middle value
    result = uwrt_gyro::extractFieldFromBuffer(testMsg, sizeof(testMsg), frameMap[0], TYPE_1_FRAME_1_FIELD_1, &dst, 1);
    ASSERT_EQ(result, 1);
    ASSERT_EQ(dst, 'a');

    //1-char extraction of end value
    result = uwrt_gyro::extractFieldFromBuffer(testMsg, sizeof(testMsg), frameMap[0], TYPE_1_FRAME_1_FIELD_3, &dst, 1);
    ASSERT_EQ(result, 1);
    ASSERT_EQ(dst, 'c');
}



TEST(UtilTest, testInsertFieldToBuffer)
{
    GTEST_SKIP();
}
