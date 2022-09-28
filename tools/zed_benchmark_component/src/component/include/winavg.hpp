#ifndef WINAVG_HPP
#define WINAVG_HPP

#include <cstddef> // size_t
#include <deque> // std::dequeue
#include <mutex>

namespace stereolabs 
{

class WinAvg
{
public:
    WinAvg(size_t win_size=15);
    ~WinAvg();

    double setNewSize(size_t win_size);
    double addValue(double val);

    /// @brief Get the current average of the stored values
    /// @return average of the stored values
    double getAvg();

    inline size_t size(){return mVals.size();}

private:
    size_t mWinSize;

    std::deque<double> mVals; // The values in the queue used to calculate the windowed average
    double mSumVals; // The updated sum of the values in the queue

    std::mutex mQueueMux;
};

}

#endif