#ifndef RANGETREE_H_
#define RANGETREE_H_

#include <memory>
#include <cstddef>
#include <vector>
#include <iostream>
#include <algorithm>
#include <string>
#include <sstream>

void Assert(bool test) {
  if (!test)
  {
    std::cout << "Assert failed" << std::endl;
    exit(-1);
  }
}

template<typename... Args>
struct Print {
  static void Do(Args... args) {}
};

template<typename Arg, typename... Tail>
struct Print<Arg, Tail...> {
  static void Do(Arg arg, Tail... args) {
      std::cout << arg;
      Print<Tail...>::Do(args...);
  }
};

template<typename... Args>
void Debug(Args... args) {
    std::cout << "DEBUG:[";
    Print<Args...>::Do(args...);
    std::cout << "]\n";
}

template<typename T>
struct element_properties {
  typedef typename T::value_type value_type;
};

template<>
struct element_properties<int> {
  typedef int value_type;
};

template<size_t d, typename T>
typename element_properties<T>::value_type get(T const& t );

template<size_t d>
typename element_properties<int>::value_type get(int i) { return i; }

template<size_t d, typename U, typename A>
typename element_properties<std::vector<U,A>>::value_type get(std::vector<U,A> const& v) {
  return v[d];
}

// A range tree.
// 'T' is the type of the value inserted into the tree.
// elements are 'dim' dimensional.
template<typename T, size_t dim, typename Order = std::less< typename element_properties<T>::value_type> >
struct range_tree {
  typedef typename element_properties<T>::value_type value_type;
  struct sorter {
    bool operator()( T const& left, T const& right ) const {
      return Order()( get<dim-1>(left), get<dim-1>(right) );
    }
  };

  struct printer {
    std::string operator()( T const& t ) const {
      std::string retval = "[ ";
      retval += print_elements( t );
      retval += "]";
      return retval;
    }
    std::string print_elements( T const& t ) const {
      std::stringstream ss;
      typedef typename range_tree<T, dim-1, Order>::printer next_printer;
      ss << next_printer().print_elements(t);
      ss << get<dim-1>(t) << " ";
      return ss.str();
    }
  };

  template<typename Iterator>
  range_tree(Iterator begin, Iterator end) {
    std::sort(begin, end, sorter());
    root.reset(new tree_node(begin, end));
  }

  template<size_t n, typename Func>
  void walk(Func f) const {
      if (root) root->walk<n>(f);
  }

  template<size_t n, typename Func>
  void walk(Func f) {
      if (root) root->walk<n>(f);
  }

  struct tree_node {
    std::unique_ptr< range_tree<T, dim-1, Order> > subtree;
    T value;

    template<size_t n, typename Func>
    void walk(Func f) const {
      if (n==dim && !left && !right)
        f(value);
      if (left)
        left->walk<n>(f);
      if (right)
        right->walk<n>(f);
      if (subtree)
        subtree->walk<n>(f);
    }

    template<size_t n, typename Func>
    void walk(Func f) {
      if (n==dim && !left && !right)
        f(value);
      if (left)
        left->walk<n>(f);
      if (right)
        right->walk<n>(f);
      if (subtree)
        subtree->walk<n>(f);
    }

    void find_path( T const& t, std::vector< tree_node const* >& vec ) {
      vec.push_back(this);
      if (sorter()(t, value) ) {
        if (left)
          left->find_path(t, vec);
      } else if (sorter()(value, t)) {
        if (right)
          right->find_path(t, vec);
      } else {
        // found it!
        return;
      }
    }

    std::vector<tree_node const*> range_search(T const& left, T const& right) {
      std::vector<tree_node const*> left_path;
      std::vector<tree_node const*> right_path;
      find_path(left, left_path );
      find_path(right, right_path );
      // erase common path:
      {
        auto it1 = left_path.begin();
        auto it2 = right_path.begin();
        for( ; it1 != left_path.end() && it2 != right_path.end(); ++it1, ++it2) {
          if (*it1 != *it2)
          {
            Debug( "Different: ", printer()( (*it1)->value ), ", ", printer()( (*it2)->value ) );
            break;
          }

          Debug( "Identical: ", printer()( (*it1)->value ), ", ", printer()( (*it2)->value ) );
        }
        // remove identical prefixes:
        if (it2 == right_path.end() && it2 != right_path.begin())
            --it2;
        if (it1 == left_path.end() && it1 != left_path.begin())
            --it1;
        right_path.erase( right_path.begin(), it2 );
        left_path.erase( left_path.begin(), it1 );
      }
      for (auto it = left_path.begin(); it != left_path.end(); ++it) {
        if (*it && (*it)->right) {
          Debug( "Has right child: ", printer()( (*it)->value ) );
          *it = (*it)->right.get();
          Debug( "It is: ", printer()( (*it)->value ) );
        } else {
          Debug( "Has no right child: ", printer()( (*it)->value ) );
          if ( sorter()( (*it)->value, left) || sorter()( right, (*it)->value) ) {
            Debug( printer()( (*it)->value ), "<", printer()( left ), " so erased" );
            *it = 0;
          }
        }
      }
      for (auto it = right_path.begin(); it != right_path.end(); ++it) {
        if (*it && (*it)->left) {
          Debug( "Has left child: ", printer()( (*it)->value ) );
          *it = (*it)->left.get();
          Debug( "It is: ", printer()( (*it)->value ) );
        } else {
          Debug( "Has no left child: ", printer()( (*it)->value ) );
          if ( sorter()( (*it)->value, left) || sorter()( right, (*it)->value) ) {
            Debug( printer()( right ), "<", printer()( (*it)->value ), " so erased" );
            *it = 0;
          }
        }
      }
      left_path.insert( left_path.end(), right_path.begin(), right_path.end() );
      // remove duds and duplicates:
      auto highwater = std::remove_if( left_path.begin(), left_path.end(), []( tree_node const* n) { return n==0; } );
      std::sort( left_path.begin(), highwater );
      left_path.erase( std::unique( left_path.begin(), highwater ), left_path.end() );
      return left_path;
    }

    std::unique_ptr<tree_node> left;
    std::unique_ptr<tree_node> right;
    // rounds down:
    template<typename Iterator>
    static Iterator middle( Iterator begin, Iterator end ) {
      return (end-begin-1)/2 + begin ;
    }

    template<typename Iterator>
    tree_node(Iterator begin, Iterator end) : value(*middle(begin,end)) {
      Debug( "Inserted ", get<dim-1>(value), " at level ", dim );
      Iterator mid = middle(begin,end);
      Assert(begin != end);
      if (begin +1 != end) { // not a leaf
        Debug("Not a leaf at level ", dim);
        ++mid; // so *mid was the last element in the left sub tree 
        Assert(mid!=begin);
        Assert(mid!=end);
        left.reset(new tree_node(begin, mid));
        right.reset(new tree_node(mid, end));
      } else {
        Debug( "Leaf at level ", dim );
      }
      if (dim > 0) {
        subtree.reset(new range_tree<T, dim-1, Order>(begin, end) );
      }
    }
  };

  std::unique_ptr<tree_node> root;
};
// makes the code above a tad easier:
template<typename T, typename Order >
struct range_tree< T, 0, Order > {
  typedef typename element_properties<T>::value_type value_type;
  struct printer { template<typename Unused>std::string print_elements(Unused const&) {return std::string();} };
  range_tree(...) {};
  struct tree_node {}; // maybe some stub functions in here
  template<size_t n, typename Func>
  void walk(Func f) {}
};


#endif
