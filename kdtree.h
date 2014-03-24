#ifndef KDTREE_H_
#define KDTREE_H_

#include <memory>

template<typename T>
struct element_properties {
  typedef typename T::value_type value_type;
};

template<>
struct element_properties<int> {
  typedef int value_type;
};

template<>
struct element_properties<double> {
  typedef double value_type;
};

template<size_t d, typename T>
typename element_properties<T>::value_type get(T const& t );

template<size_t d>
typename element_properties<int>::value_type get(int i) { return i; }

template<size_t d>
typename element_properties<double>::value_type get(double i) { return i; }

template<size_t d, typename U, typename A>
typename element_properties<std::vector<U,A>>::value_type get(std::vector<U,A> const& v) {
  return v[d];
}

template<int checkdim, typename T>
bool check<checkdim>(const T& pivot, const T& left, const T& right) {
  return !(get<checkdim>(pivot) < get<checkdim>(left)) &&
         !(get<checkdim>(right) < get<checkdim>(pivot)) &&
         check<checkdim - 1>(pivot, left, right);
}

template<typename T>
bool check<0>(const T& pivot, const T& left, const T& right) {
  return !(get<0>(pivot) < get<0>(left)) &&
         !(get<0>(right) < get<0>(pivot));
}


template<typename T, int num_dims>
class KDTree {
 public:

  KDTree() {}

  template<typename iter>
  KDTree(iter first, iter last) : root_(new Node<0>(first, last)) { }




  template<int dim>
  struct Node {
    template<typename iter>
    Node(iter first, iter last) {
      if (first + 1 == last) {
        // Leaf.
        pivot = *first;
      } else {
        iter mid = (last - first + 1) / 2 + first;
        pivot = *mid;
        if (first != mid)
          left_set.reset(new Node<(dim+1)%num_dims>(first, mid));
        if (mid + 1 != last)
          right_set.reset(new Node<(dim+1)%num_dims>(mid + 1, last));
      }
    }


    template<typename Func>
    void search(const T& left, const T& right, Func f) {
      if (left_set && !(get<dim>(pivot) < get<dim>(left)))
        left_set->search(left, right, f);
      if (right_set && (get<dim>(pivot) < get<dim>(right)))
        right_set->search(left, right, f);

      // Check the pivot.
      //if (!check<num_dims - 1>(pivot, left, right)) return;

      f(pivot);
    }

    T pivot;
    std::unique_ptr<Node<(dim+1)%num_dims>> left_set;
    std::unique_ptr<Node<(dim+1)%num_dims>> right_set;
  };

  template<typename Func>
  void search(T left, T right, Func f) {
    root_->search(left, right, f);
  }


  std::unique_ptr<Node<0>> root_;
};

#endif
