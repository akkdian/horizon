#ifndef SHARED_MEMORY_UTILS_HPP_
#define SHARED_MEMORY_UTILS_HPP_

#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>

class SharedMemoryManager {
public:
    void initialize(const std::string &name, size_t size) {
        shm_fd_ = shm_open(name.c_str(), O_CREAT | O_RDWR, 0666);
        if (shm_fd_ == -1) throw std::runtime_error("Failed to open shared memory");
        ftruncate(shm_fd_, size);
        mem_ = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
        if (mem_ == MAP_FAILED) throw std::runtime_error("Failed to map shared memory");
    }

    void write(const void *data) { memcpy(mem_, data, size_); }
    void *read() const { return mem_; }

private:
    int shm_fd_;
    void *mem_;
    size_t size_;
};