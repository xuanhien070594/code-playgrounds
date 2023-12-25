// Things to remember:
// 1. We should not use smart pointers for implementing graph-like data
// structures because it might cause
//    stack overflows due to recursive destructor calls.

#include <iostream>
#include <memory>
#include <ostream>

struct Node {
  int data;
  Node *next;
  Node(int data) : data(data), next(nullptr) {}
};

class LinkedList {
public:
  Node *head;
  LinkedList() : head(nullptr) {}

  void insertAtBeginning(int data);

  void printList();
};