//
// Created by User on 2019/7/11.
//

#ifndef SERIAL_PIPELINE_CIRCULAR_BUFFER_HPP
#define SERIAL_PIPELINE_CIRCULAR_BUFFER_HPP


#include <type_traits>
#include <stdexcept>

/**
 * 环形队列
 *
 * @tparam t 元素类型
 */
template<class t>
class circular_buffer {
public:
    explicit circular_buffer(size_t size = 0)
        : _head(new t[size + 1]),
          _tail(_head + size + 1),
          _begin(_head),
          _end(_head + size) {}
    
    ~circular_buffer() {
        delete[] _head;
    }
    
    // =====================================================
    
    template<class other_iterator_t>
    circular_buffer(other_iterator_t begin,
                    other_iterator_t end)
        : circular_buffer(end - begin) {
        auto ptr = _head;
        while (begin < end) *ptr++ = *begin++;
    }
    
    circular_buffer(std::initializer_list<t> &&list)
        : circular_buffer(list.begin(), list.end()) {}
    
    // =====================================================
    
    circular_buffer(const circular_buffer &others)
        : circular_buffer(others.begin(), others.end()) {}
    
    circular_buffer(circular_buffer &&others) noexcept
        : _head(others._head),
          _tail(others._tail),
          _begin(others._begin),
          _end(others._end) {
        others._head  = nullptr;
        others._end   = nullptr;
        others._begin = nullptr;
        others._end   = nullptr;
    }
    
    circular_buffer &operator=(const circular_buffer &others) {
        auto size = others.size();
        if (size > _tail - _head - 1) {
            delete[] _head;
            _head = new t[size + 1];
            _tail = _head + size + 1;
        }
        
        _begin = _head;
        _end   = _head + size;
        
        auto ptr      = _begin;
        auto iterator = others.begin();
        while (iterator < others.end())
            *ptr++ = *iterator++;
        
        return *this;
    }
    
    circular_buffer &operator=(circular_buffer &&others) noexcept {
        _head  = others._head;
        _tail  = others._tail;
        _begin = others._begin;
        _end   = others._end;
        
        others._head  = nullptr;
        others._end   = nullptr;
        others._begin = nullptr;
        others._end   = nullptr;
        
        return *this;
    }

private:
    // =====================================================
    
    [[nodiscard]] inline size_t operator[](t *ptr) const {
        return ptr >= _begin
               ? ptr - _begin
               : _tail - _begin + ptr - _head;
    }
    
    [[nodiscard]] inline t *next_n_to(t *ptr, size_t count) const {
        auto next = ptr + count;
        return count == 0
               ? ptr
               : count > 0
                 ? next < _tail
                   ? next
                   : next - (_tail - _head)
                 : next >= _head
                   ? next
                   : next + (_tail - _head);
    }

public:
    // =====================================================
    
    class const_iterator {
        inline void check_same_container(const const_iterator &others) const {
            if (host != others.host)
                throw std::logic_error("these two iterator is not belong to the same container");
        }
    
    public:
        const_iterator(const circular_buffer *host, t *ptr)
            : host(host), ptr(ptr) {}
        
        const_iterator(const const_iterator &) = default;
        
        const_iterator(const_iterator &&) noexcept = default;
        
        [[nodiscard]] const t &operator*() const { return *ptr; }
        
        const_iterator &operator++() {
            ptr = host->next_n_to(ptr, 1);
            return *this;
        }
        
        [[nodiscard]] const_iterator operator++(int) {
            auto copy = *this;
            operator++();
            return copy;
        }
        
        [[nodiscard]] bool operator<(const const_iterator &others) const {
            check_same_container(others);
            return (*host)[ptr] < (*others.host)[others.ptr];
        }
        
        [[nodiscard]] bool operator==(const const_iterator &others) const {
            check_same_container(others);
            return (*host)[ptr] == (*others.host)[others.ptr];
        }
        
        [[nodiscard]] bool operator!=(const const_iterator &others) const {
            return !operator==(others);
        }
        
        [[nodiscard]] bool operator>(const const_iterator &others) const {
            check_same_containerothers();
            return (*host)[ptr] > (*others.host)[others.ptr];
        }
        
        [[nodiscard]] bool operator<=(const const_iterator &others) const {
            return !operator>(others);
        }
        
        [[nodiscard]] bool operator>=(const const_iterator &others) const {
            return !operator<(others);
        }
    
    private:
        const circular_buffer *host;
        t                     *ptr;
    };
    
    class iterator {
        inline void check_same_container(const iterator &others) const {
            if (host != others.host)
                throw std::logic_error("these two iterator is not belong to the same container");
        }
    
    public:
        iterator(circular_buffer *host, t *ptr)
            : host(host), ptr(ptr) {}
        
        iterator(const iterator &) = default;
        
        iterator(iterator &&) noexcept = default;
        
        iterator &operator=(const iterator &) = default;
        
        iterator &operator=(iterator &&others) noexcept {
            host = others.host;
            ptr  = others.ptr;
            
            others.host = nullptr;
            others.ptr  = nullptr;
        }
        
        [[nodiscard]] t &operator*() {
            return *ptr;
        }
        
        [[nodiscard]] const t &operator*() const {
            return *ptr;
        }
        
        [[nodiscard]] bool operator<(const iterator &others) const {
            check_same_container(others);
            return (*host)[ptr] < (*others.host)[others.ptr];
        }
        
        [[nodiscard]] bool operator==(const iterator &others) const {
            check_same_container(others);
            return (*host)[ptr] == (*others.host)[others.ptr];
        }
        
        [[nodiscard]] bool operator!=(const iterator &others) const {
            return !operator==(others);
        }
        
        [[nodiscard]] bool operator>(const iterator &others) const {
            check_same_container(others);
            return (*host)[ptr] > (*others.host)[others.ptr];
        }
        
        [[nodiscard]] bool operator<=(const iterator &others) const {
            return !operator>(others);
        }
        
        [[nodiscard]] bool operator>=(const iterator &others) const {
            return !operator<(others);
        }
        
        iterator &operator+=(int count) {
            ptr = host->next_n_to(ptr, count);
            return *this;
        }
        
        iterator &operator-=(int count) {
            ptr = host->next_n_to(ptr, -count);
            return *this;
        }
        
        iterator &operator++() { return *this += 1; }
        
        [[nodiscard]] iterator operator++(int) {
            auto copy = *this;
            operator++();
            return copy;
        }
        
        iterator &operator--() { return *this -= 1; }
        
        [[nodiscard]] iterator operator--(int) {
            auto copy = *this;
            operator--();
            return copy;
        }
        
        [[nodiscard]] iterator operator+(int count) const {
            auto copy = *this;
            return copy += count;
        }
        
        [[nodiscard]] iterator operator-(int count) const {
            auto copy = *this;
            return copy -= count;
        }
        
        [[nodiscard]]t &operator[](int count) {
            return *host->next_n_to(ptr, count);
        }
        
        [[nodiscard]]const t &operator[](int count) const {
            return *host->next_n_to(ptr, count);
        }
    
    private:
        circular_buffer *host;
        t               *ptr;
    };

private:
    // =====================================================
    
    inline static void swap_ptr(t **a, t **b) {
        auto temp = *a;
        *a = *b;
        *b = temp;
    }
    
    inline void move_all(size_t new_capacity) {
        auto new_head = new t[new_capacity + 1],
             ptr      = new_head;
        
        if (_begin > _end) {
            while (_begin < _tail) *ptr++ = *_begin++;
            _begin = _head;
        }
        while (_begin < _end) *ptr++ = *_begin++;
        
        delete[] _head;
        _head  = new_head;
        _begin = new_head;
        _tail  = new_head + new_capacity + 1;
        _end   = ptr;
    }
    
    // =====================================================
    // 专有

public:
    [[nodiscard]] size_t capacity() const { return _tail - _head - 1; }
    
    void resize(size_t new_size) {
        if (new_size == size()) return;
        
        if (new_size <= capacity())
            _end = next_n_to(_begin, new_size);
        else {
            move_all(new_size);
            _end = _tail - 1;
        }
    }
    
    void shrink_to_fit() {
        if (capacity() != size()) move_all(size());
    }
    
    // =====================================================
    // 容器 Container
    
    [[nodiscard]] iterator begin() { return iterator(this, _begin); }
    
    [[nodiscard]] iterator end() { return iterator(this, _end); }
    
    [[nodiscard]] const_iterator begin() const { return const_iterator(this, _begin); }
    
    [[nodiscard]] const_iterator end() const { return const_iterator(this, _end); }
    
    [[nodiscard]] const_iterator cbegin() const { return const_iterator(this, _begin); }
    
    [[nodiscard]] const_iterator cend() const { return const_iterator(this, _end); }
    
    [[nodiscard]] size_t size() const { return operator[](_end); }
    
    [[nodiscard]] size_t max_size() const { return std::numeric_limits<size_t>::max(); }
    
    [[nodiscard]] bool empty() const { return _begin == _end; }
    
    void swap(circular_buffer &others) {
        swap_ptr(&_head, &others._head);
        swap_ptr(&_tail, &others._tail);
        swap_ptr(&_begin, &others._begin);
        swap_ptr(&_end, &others._end);
    }
    
    // =====================================================
    // 访问元素
    
    t &operator[](size_t index) {
        return *next_n_to(_begin, index);
    }
    
    const t &operator[](size_t index) const {
        return *next_n_to(_begin, index);
    }
    
    // =====================================================
    // 修改元素
    
    void push_back(const t &value) {
        auto _size = size();
        if (_size >= capacity())
            move_all(_size < 2 ? 4 : 2 * _size);
        
        *_end = value;
        if (++_end >= _tail) _end = _head;
    }
    
    void pop_back() {
        if (size() <= 0)
            throw std::logic_error("container is empty");
        
        if (--_end < _head) _end = _tail - 1;
    }
    
    void push_front(const t &value) {
        auto _size = size();
        if (_size >= capacity())
            move_all(_size < 2 ? 4 : 2 * _size);
        
        if (--_begin < _head) _begin = _tail - 1;
        *_begin = value;
    }
    
    void pop_front() {
        if (size() <= 0)
            throw std::logic_error("container is empty");
        
        if (++_begin >= _tail) _begin = _head;
    }

private:
    t *_head, *_tail, *_begin, *_end;
};


#endif //SERIAL_PIPELINE_CIRCULAR_BUFFER_HPP
