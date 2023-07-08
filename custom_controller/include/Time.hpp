//
// Created by tiger on 2021/7/2.
//

#ifndef SERIAL_TIME_HPP
#define SERIAL_TIME_HPP

#define MSPERTICK 10

class Time
{
private:
    int m_msPerTick;
    int m_currentTick;
    Time(int msPerTick, int currentTick) : m_currentTick(currentTick), m_msPerTick(msPerTick) { ; }
    static Time *instance()
    {
        static Time instance(MSPERTICK, 0);
        return &instance;
    }

public:
    static void tick() { ++instance()->m_currentTick; }
    static int getTick() { return instance()->m_currentTick; }
    static int getMsPerTick() { return instance()->m_msPerTick; }
};

#endif // SERIAL_TIME_HPP
