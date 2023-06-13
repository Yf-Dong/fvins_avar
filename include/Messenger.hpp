#ifndef MESSENGER_HPP
#define MESSENGER_HPP

#include <vector>
#include <queue>
#include <future>
#include <memory>
#include <thread>
#include <condition_variable>

namespace Messenger
{

class ThreadPool
{
 public:

  ThreadPool(std::size_t threadsNum) : mbStop(false)
  {
    for (std::size_t i = 0; i < threadsNum; ++i) {
      mvWorkers.emplace_back ([this] {
        while (true) {
          std::function<void()> task;
          {
            std::unique_lock<std::mutex> lock(this->mQueMtx);
            this->mCondition.wait(lock, [this]{
              return this->mbStop || !this->mqTasks.empty();});
            if (this->mbStop && this->mqTasks.empty()) return;
            task = std::move(this->mqTasks.front());
            this->mqTasks.pop();
          }
          task();
        }
    });
    }
  }

  ~ThreadPool()
  {
    {
      std::unique_lock<std::mutex> lock(mQueMtx);
      mbStop = true;
    }
    mCondition.notify_all();
    for (std::thread &worker : mvWorkers) {
      worker.join();
    }
  }

  template<typename Func, typename... Args>
  auto enqueue(Func&& f, Args&&... args)
    -> std::future<typename std::result_of<Func(Args...)>::type>
  {
    typedef typename std::result_of<Func(Args...)>::type result_type;
    auto task = std::make_shared<std::packaged_task<result_type()>> (
      std::bind(std::forward<Func>(f), std::forward<Args>(args)...));

    std::future<result_type> res = task->get_future();
    {
      std::unique_lock<std::mutex>(mQueMtx);
      if (mbStop)
        throw std::runtime_error("enqueue on stopped ThreadPool");
      mqTasks.emplace([task]{(*task)();});
    }
    mCondition.notify_one();
    return res;
  }
 
 private:
  bool                                mbStop;
  std::mutex                          mQueMtx;
  std::vector <std::thread>           mvWorkers;
  std::queue<std::function<void()>>   mqTasks;
  std::condition_variable             mCondition;
};



}



#endif //MESSENGER_HPP