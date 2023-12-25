#include "linked_list.h"

// This method will cause memory leak because it does not handle deallocating
// memory of the deleted node
void deleteNodeLinkedListUnsafe(LinkedList &linked_list, int data) {
  Node *cur_node = linked_list.head;

  // special case: we want to delete the head node.
  if (cur_node->data == data) {
    std::cout << "We are deleting the head node with data "
              << linked_list.head->data << std::endl;
    linked_list.head = cur_node->next;
    return;
  }

  while (cur_node->next != nullptr) {
    if (cur_node->next->data == data) {
      cur_node->next = cur_node->next->next;
      return;
    }
    cur_node = cur_node->next;
  }

  if (cur_node->next == nullptr) {
    std::cout << "Could not delete node " << data
              << " because there is no such node in the "
                 "provided linked list"
              << std::endl;
  }
}

void deleteNodeLinkedListSafeVersion1(LinkedList &linked_list, int data) {
  Node *cur_node = linked_list.head;
  Node *target_node = nullptr;

  // special case: we want to delete the head node.
  if (cur_node->data == data) {
    std::cout << "We are deleting the head node with data "
              << linked_list.head->data << std::endl;
    target_node = linked_list.head;
    linked_list.head = cur_node->next;
    delete target_node;
    return;
  }

  while (cur_node->next != nullptr) {
    if (cur_node->next->data == data) {
      std::cout << "Found a node with data " << data << " -> deleting ..."
                << std::endl;
      target_node = cur_node->next;
      cur_node->next = cur_node->next->next;
      delete target_node;
      return;
    }
    cur_node = cur_node->next;
  }

  if (cur_node->next == nullptr) {
    std::cout << "Could not delete node " << data
              << " because there is no such node in the "
                 "provided linked list"
              << std::endl;
  }
}

void deleteNodeLinkedListSafeVersion2(LinkedList &linked_list, int data) {
  Node *current = linked_list.head;
  Node *previous = nullptr;

  // Find the node to delete
  while (current != nullptr && current->data != data) {
    previous = current;
    current = current->next;
  }

  // Node not found
  if (current == nullptr) {
    return;
  }

  // Handle deletion of the head node
  if (current == linked_list.head) {
    linked_list.head = current->next;
  } else {
    // Unlink the node from the list
    previous->next = current->next;
  }

  // Delete the node and its memory
  delete current;
}

int main() {
  LinkedList linked_list;
  for (int i = 0; i < 10; i++) {
    linked_list.insertAtBeginning(i);
  }
  std::cout << "Below is the constructed linked list" << std::endl;
  linked_list.printList();

  // delete node 10
  deleteNodeLinkedListSafeVersion2(linked_list, 0);

  std::cout << "Below is the linked list after deleting" << std::endl;
  linked_list.printList();
}