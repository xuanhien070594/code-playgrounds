// Things to remember:
// 1. We should not use smart pointers for implementing graph-like data
// structures because it might cause
//    stack overflows due to recursive destructor calls.

#include "linked_list.h"
#include <iostream>
#include <memory>
#include <ostream>

void LinkedList::insertAtBeginning(int data) {
  Node *new_node = new Node(data);
  new_node->next = head;
  head = new_node;
}

void LinkedList::printList() {
  Node *cur_node = head;
  while (cur_node != nullptr) {
    std::cout << cur_node->data << " ";
    cur_node = cur_node->next;
  };
  std::cout << std::endl;
}
