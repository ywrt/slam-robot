#include "rangetree.h"

int main() {
  typedef std::vector<int> vector_type;
  std::vector<vector_type> test;
  test.push_back( vector_type{5,2} );
  test.push_back( vector_type{2,3} );
  range_tree<vector_type, 2> tree(test.begin(), test.end());
  std::cout << "Walking dim 2:";
  auto print_node = [](vector_type const& v){ std::cout << "(" << v[0] << "," << v[1] << ")"; };
  tree.walk<2>(print_node);
  std::cout << "\nWalking dim 1:";
  tree.walk<1>(print_node);
  std::cout << "\n";

  std::cout << "Range search from {3,3} to {10,10}\n";
  auto nodes = tree.root->range_search( vector_type{3,3}, vector_type{10,10} );
  for (auto it = nodes.begin(); it != nodes.end(); ++it)
  {
    (*it)->walk<2>(print_node);
  }
  std::cout << "\n";
}
