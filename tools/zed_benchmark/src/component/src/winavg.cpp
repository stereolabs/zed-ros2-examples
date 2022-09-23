#include "winavg.hpp"

namespace stereolabs 
{

WinAvg::WinAvg(size_t win_size)
{
    mWinSize = win_size;
}

WinAvg::~WinAvg()
{
}

double WinAvg::setNewSize(size_t win_size)
{
    std::lock_guard<std::mutex> guard(mQueueMux);

    mWinSize = win_size;
    while(mVals.size()>mWinSize)
    {
        double val = mVals.back();
        mVals.pop_back(); 
        mSumVals-=val;       
    }

    return mSumVals/mVals.size();
}

double WinAvg::addValue(double val)
{
    std::lock_guard<std::mutex> guard(mQueueMux);
    if(mVals.size()==mWinSize)
    {
        double val = mVals.back();
        mVals.pop_back(); 
        mSumVals-=val;  
    }

    mVals.push_front(val);
    mSumVals+=val;

    return mSumVals/mVals.size();
}

double WinAvg::getAvg()
{
    std::lock_guard<std::mutex> guard(mQueueMux);
    
    double avg = mSumVals/mVals.size();

    return avg;
}

}
