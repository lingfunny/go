#ifndef MYSTACK_H
#define MYSTACK_H

#include <stdexcept>

template <typename T>
class MyStack {
private:
    struct Node {
        T data;
        Node* next;
        Node(T val) : data(val), next(nullptr) {}
    };

    Node* topNode;
    int size;

public:
    MyStack() : topNode(nullptr), size(0) {}

    ~MyStack() {
        while (!isEmpty()) {
            pop();
        }
    }

    void push(T val) {
        Node* newNode = new Node(val);
        newNode->next = topNode;
        topNode = newNode;
        size++;
    }

    void pop() {
        if (isEmpty()) {
            throw std::underflow_error("Stack is empty");
        }
        Node* temp = topNode;
        topNode = topNode->next;
        delete temp;
        size--;
    }

    T top() const {
        if (isEmpty()) {
            throw std::underflow_error("Stack is empty");
        }
        return topNode->data;
    }

    bool isEmpty() const {
        return topNode == nullptr;
    }

    int getSize() const {
        return size;
    }
};

#endif // MYSTACK_H
