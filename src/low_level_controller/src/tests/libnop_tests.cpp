#include <nop/structure.h>
#include <nop/serializer.h>
#include <nop/utility/buffer_reader.h>
#include <nop/utility/buffer_writer.h>

#include <iostream>

#include "CppUTest/TestHarness.h" // must be last


TEST_GROUP(LibnopTests)
{
    char buf[1000];


    void setup(void)
    {
    }
};


typedef struct {
    float data[10];
    int data_len;
} buffer_t;
NOP_EXTERNAL_STRUCTURE(buffer_t, (data, data_len));

typedef struct {
    float data[3];
    int data_len;
} buffer_shorter_t;
NOP_EXTERNAL_STRUCTURE(buffer_shorter_t, (data, data_len));

TEST(LibnopTests, BuffersWithDifferentLength)
{
    buffer_t value_in = {
        {1, 2, 3},
        3
    };
    auto serializer = nop::Serializer<nop::BufferWriter>(buf, sizeof(buf));
    serializer.Write(value_in);
    size_t len = serializer.writer().size();

    auto deserializer = nop::Deserializer<nop::BufferReader>(buf, len);
    buffer_shorter_t value_out = {{0}, 0};
    auto status = deserializer.Read(&value_out);
    // std::cout << "deserializer status: " << status.GetErrorMessage() << std::endl;
    // std::cout << value_out.data_len << std::endl;

    CHECK_FALSE(status.has_error());
}

TEST(LibnopTests, BuffersWithDifferentLengthOverflowReturnsError)
{
    buffer_t value_in = {
        {1, 2, 3, 4},
        4
    };
    auto serializer = nop::Serializer<nop::BufferWriter>(buf, sizeof(buf));
    serializer.Write(value_in);
    size_t len = serializer.writer().size();

    auto deserializer = nop::Deserializer<nop::BufferReader>(buf, len);
    buffer_shorter_t value_out = {{0}, 0};
    auto status = deserializer.Read(&value_out);
    // std::cout << "deserializer status: " << status.GetErrorMessage() << std::endl;
    // std::cout << value_out.data_len << std::endl;

    CHECK_TRUE(status.has_error());
}


