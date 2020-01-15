template <typename T, int CAPACITY>
class CircularBuffer {
  public:
    CircularBuffer() : buffer_{}, size_(0), nextPushIndex_(0) {}

    void push(T value) {
      buffer_[nextPushIndex_] = value;
      nextPushIndex_ = (nextPushIndex_ + 1) % CAPACITY;
      if (size_ < CAPACITY) {
        size_++;
      }
    }

    int size() {
      return size_;
    }

    // Index zero is the most-recently-pushed value,
    // index 1 is the second-most-recently-pushed value, etc.
    T operator[](int index) {
      return buffer_[(CAPACITY + nextPushIndex_ - 1 - index) % CAPACITY];
    }

  private:
    T buffer_[CAPACITY];
    int size_;
    int nextPushIndex_;
};

