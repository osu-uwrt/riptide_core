#include "riptide_gyro/serial_library.hpp"
#include "riptide_gyro/testing.hpp"


TEST(UtilTest, testMemstr)
{
    const char
        str1[] = "abcdefg",
        *str1Needle = strstr(str1, "de"),
        *str1Needle2 = strstr(str1, "g");
    
    EXPECT_EQ(uwrt_gyro::memstr(str1, sizeof(str1), "de", 2), str1Needle);
    EXPECT_EQ(uwrt_gyro::memstr(str1, strlen(str1), "g", 1), str1Needle2);
    EXPECT_EQ(uwrt_gyro::memstr(str1, sizeof(str1), "12", 2), nullptr);

    const char
        str3[] = {'a', 0, 'y', 3, 5, 8, 45, 'u'},
        str3Needle[] = {'y', 3},
        *str3NeedleExpected = &str3[2];
    
    EXPECT_EQ(uwrt_gyro::memstr(str3, sizeof(str3), str3Needle, 2), str3NeedleExpected);

    
}


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

    //had issues with this
    const char ffTest[2] = {0x30, 0x02};
    i = uwrt_gyro::convertFromCString<uint16_t>(ffTest, 2);
    ASSERT_EQ(i, 0x3002);
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


TEST_F(Type2SerialProcessorTest, testExtractFieldFromBufferAdvanced)
{
    const char *testMsg1 = "Aa1bcde";
    char dst[5] = {0};

    //1-char extraction of type 2 frame 1 field 1 into bigger buffer (expect "a")
    size_t result = uwrt_gyro::extractFieldFromBuffer(testMsg1, sizeof(testMsg1), frameMap[0], TYPE_2_FIELD_1, dst, sizeof(dst));
    ASSERT_EQ(result, 1);
    ASSERT_TRUE(memcmp(dst, "A", 1) == 0);

    //3-char extraction of type 2 frame 1 field 2 in sequential order (expect "bcd")
    result = uwrt_gyro::extractFieldFromBuffer(testMsg1, sizeof(testMsg1), frameMap[0], TYPE_2_FIELD_2, dst, sizeof(dst));
    ASSERT_EQ(result, 3);
    ASSERT_TRUE(memcmp(dst, "1bd", 3) == 0);

    const char *testMsg2 = "a2bcdeA";

    //3-char extraction of type 2 frame 1 field 2 (disjointed, expect "abe")
    result = uwrt_gyro::extractFieldFromBuffer(testMsg2, sizeof(testMsg2), frameMap[1], TYPE_2_FIELD_2, dst, sizeof(dst));
    ASSERT_EQ(result, 3);
    ASSERT_TRUE(memcmp(dst, "abA", 3) == 0);

    //3-char extraction of type 2 frame 1 field 2 (disjointed, expect "abe"), into smaller buffer (expect "ab")
    dst[2] = 'M'; //this tests that this character wasnt touched
    result = uwrt_gyro::extractFieldFromBuffer(testMsg2, sizeof(testMsg2), frameMap[1], TYPE_2_FIELD_2, dst, sizeof(dst) - 1);
    ASSERT_EQ(result, 3);
    ASSERT_TRUE(memcmp(dst, "abM", 2) == 0);
}


TEST_F(Type1SerialProcessorTest, testInsertFieldToBufferBasic)
{
    const char c = 'j';
    char dst[5] = "Abcd";

    //1-char insertion of beginning value
    uwrt_gyro::insertFieldToBuffer(dst, sizeof(dst), frameMap[0], FIELD_SYNC, &c, sizeof(c));
    ASSERT_TRUE(memcmp(dst, "jbcd", 4) == 0);

    //1-char insertion of beginning value
    strcpy(dst, "Abcd");
    uwrt_gyro::insertFieldToBuffer(dst, sizeof(dst), frameMap[0], TYPE_1_FRAME_1_FIELD_1, &c, sizeof(c));
    ASSERT_TRUE(memcmp(dst, "Ajcd", 4) == 0);

    //1-char insertion of end value
    strcpy(dst, "Abcd");
    uwrt_gyro::insertFieldToBuffer(dst, sizeof(dst), frameMap[0], TYPE_1_FRAME_1_FIELD_3, &c, sizeof(c));
    ASSERT_TRUE(memcmp(dst, "Abcj", 4) == 0);
}


TEST_F(Type2SerialProcessorTest, testInsertFieldToBufferAdvanced)
{
    const char src[] = "XYZ";
    char dst[] = "Ab1defg";

    //1-char insertion of src from bigger buffer
    uwrt_gyro::insertFieldToBuffer(dst, sizeof(dst), frameMap[0], TYPE_2_FIELD_1, src, sizeof(src));
    ASSERT_TRUE(memcmp(dst, "Xb1defg", 7) == 0);

    //3-char insertion of src
    strcpy(dst, "Ab1defg");
    uwrt_gyro::insertFieldToBuffer(dst, sizeof(dst), frameMap[0], TYPE_2_FIELD_2, src, sizeof(src));
    ASSERT_TRUE(memcmp(dst, "AbXYeZg", 7) == 0);

    //3-char insertion of src
    strcpy(dst, "a1bcdeA");
    uwrt_gyro::insertFieldToBuffer(dst, sizeof(dst), frameMap[1], TYPE_2_FIELD_2, src, sizeof(src));
    ASSERT_TRUE(memcmp(dst, "X1YcdeZ", 7) == 0);
}

TEST(UtilTest, testNormalizeSerialFrame)
{
    SerialFrame frameSyncBeginning = {
        FIELD_SYNC,
        TYPE_1_FRAME_1_FIELD_1,
        TYPE_1_FRAME_1_FIELD_2,
        TYPE_1_FRAME_1_FIELD_3
    };

    SerialFrame normalizedSyncBeginning = uwrt_gyro::normalizeSerialFrame(frameSyncBeginning);

    ASSERT_EQ(frameSyncBeginning, normalizedSyncBeginning);

    SerialFrame frameSyncMiddle = {
        TYPE_1_FRAME_1_FIELD_1,
        FIELD_SYNC,
        TYPE_1_FRAME_1_FIELD_2,
        TYPE_1_FRAME_1_FIELD_3
    };

    SerialFrame normalizedSyncMiddle = uwrt_gyro::normalizeSerialFrame(frameSyncMiddle);
    SerialFrame expectedSyncMiddle = {
        FIELD_SYNC,
        TYPE_1_FRAME_1_FIELD_2,
        TYPE_1_FRAME_1_FIELD_3,
        TYPE_1_FRAME_1_FIELD_1
    };

    ASSERT_EQ(normalizedSyncMiddle, expectedSyncMiddle);

    SerialFrame frameSyncEnd = {
        TYPE_1_FRAME_1_FIELD_1,
        TYPE_1_FRAME_1_FIELD_2,
        TYPE_1_FRAME_1_FIELD_3,
        FIELD_SYNC
    };

    SerialFrame normalizedSyncEnd = uwrt_gyro::normalizeSerialFrame(frameSyncEnd);
    SerialFrame expectedSyncEnd = {
        FIELD_SYNC,
        TYPE_1_FRAME_1_FIELD_1,
        TYPE_1_FRAME_1_FIELD_2,
        TYPE_1_FRAME_1_FIELD_3
    };

    ASSERT_EQ(normalizedSyncEnd, expectedSyncEnd);
}
