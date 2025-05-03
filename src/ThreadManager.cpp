#include "../include/ThreadManager.h"
#include <stdexcept>

ThreadManager::ThreadManager(size_t numThreads) 
    : numThreads(numThreads), running(false), activeThreads(0) {
    if (numThreads <= 0) {
        throw std::invalid_argument("Number of threads must be positive");
    }
    
    threadLoads.resize(numThreads);
    for (auto& load : threadLoads) {
        load.store(0);
    }
}

ThreadManager::~ThreadManager() {
    stop();
}

void ThreadManager::start() {
    if (running) {
        return;
    }
    
    running = true;
    for (size_t i = 0; i < numThreads; ++i) {
        threads.emplace_back(&ThreadManager::workerThread, this, i);
    }
}

void ThreadManager::stop() {
    if (!running) {
        return;
    }
    
    running = false;
    taskCondition.notify_all();
    
    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    threads.clear();
}

void ThreadManager::addTask(std::function<void()> task) {
    if (!task) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(taskMutex);
    taskQueue.push(task);
    taskCondition.notify_one();
}

bool ThreadManager::isRunning() const {
    return running;
}

size_t ThreadManager::getNumThreads() const {
    return numThreads;
}

void ThreadManager::setNumThreads(size_t newNumThreads) {
    if (newNumThreads <= 0) {
        throw std::invalid_argument("Number of threads must be positive");
    }
    
    if (running) {
        stop();
    }
    
    numThreads = newNumThreads;
    threadLoads.resize(numThreads);
    for (auto& load : threadLoads) {
        load.store(0);
    }
    
    if (running) {
        start();
    }
}

size_t ThreadManager::getTaskCount() const {
    std::lock_guard<std::mutex> lock(taskMutex);
    return taskQueue.size();
}

void ThreadManager::waitForCompletion() {
    std::unique_lock<std::mutex> lock(completionMutex);
    while (true) {
        if (getTaskCount() == 0 && activeThreads == 0) {
            break;
        }
        taskCondition.wait(lock);
    }
}

size_t ThreadManager::getActiveThreadCount() const {
    return activeThreads;
}

void ThreadManager::processNextTask() {
    std::function<void()> task;
    {
        std::unique_lock<std::mutex> lock(taskMutex);
        if (taskQueue.empty()) {
            return;
        }
        task = taskQueue.front();
        taskQueue.pop();
    }
    
    if (task) {
        activeThreads++;
        task();
        activeThreads--;
        taskCondition.notify_all();
    }
}

void ThreadManager::workerThread(size_t threadId) {
    while (running) {
        std::function<void()> task;
        {
            std::unique_lock<std::mutex> lock(taskMutex);
            taskCondition.wait(lock, [this] { 
                return !running || !taskQueue.empty(); 
            });
            
            if (!running && taskQueue.empty()) {
                return;
            }
            
            if (!taskQueue.empty()) {
                task = taskQueue.front();
                taskQueue.pop();
            }
        }
        
        if (task) {
            activeThreads++;
            threadLoads[threadId]++;
            task();
            activeThreads--;
            taskCondition.notify_all();
        }
    }
}
