#include <iostream>
#include <vector>


std::vector<int> test(std::vector<int>& nums)
{
    return {nums.push_back(nums)};
}
int main()
{
    std::vector<int> shit = {1, 2, 3, 4};

    std::cout << test(shit)[0] << std::endl;
}