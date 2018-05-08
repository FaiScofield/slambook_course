#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

class A {
public:
  A(const int& i ) : index(i) {}
  int index = 0;    // C++11 面向对象增强
};

int main() {
  A a1(3), a2(5), a3(9);
  vector<A> avec{a1, a2, a3};   // C++11 初始化列表
  // C++11 Lambda表达式
  std::sort(avec.begin(), avec.end(), [](const A&a1, const A&a2) {return a1.index < a2.index;});
  for ( auto& a : avec ) cout << a.index << " ";    // C++11 区间迭代for循环
  cout << endl;
  return 0;
}

