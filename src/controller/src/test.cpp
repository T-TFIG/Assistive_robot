#include <iostream>


int main()
{
    int test = 3;
    int *test_ptr = &test;
    std::cout << *test_ptr << std::endl;
    return 0;
}